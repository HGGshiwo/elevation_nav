# -*- coding: utf-8 -*-
"""
ROS 状态通信桥接模块 (ROS Bridge)
负责与 ROS 导航系统通信：发布 GridMap 消息、订阅机器人里程计/位姿、订阅路径、发布目标点
"""

import json
import math
import random
import threading
import base64
from array import array
from typing import Dict, Any, Optional, List
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Path as ROSPath, Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String as RosString, Int32MultiArray
from grid_map_msgs.msg import GridMap, GridMapInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
import numpy as np

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """欧拉角转四元数 (移植自 jie_octomap ros_bridge)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def normalize_angle(angle: float) -> float:
    """归一化角度到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ElevationRosBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self.is_initialized = False

        # 最新状态缓存
        self.robot_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.global_path: List[List[float]] = []
        self.local_path: List[List[float]] = []
        self.path_version = 0

        # 流形拓扑图缓存 (点云节点与连通边)
        self.graph_nodes: List[List[float]] = []
        self.graph_nodes_version = 0
        self.graph_edges: List[List[float]] = []
        self.graph_edges_version = 0
        # 局部代价地图缓存
        self.local_costmap: Optional[Dict[str, Any]] = None
        self.local_costmap_version = 0
        # 局部代价地图逐格成因码 (调试图层)
        self.local_costmap_debug: Optional[Dict[str, Any]] = None
        self.local_costmap_debug_version = 0
        # 局部代价地图逐格胜出节点 id (调试图层)
        self.local_costmap_debug_nodes: Optional[Dict[str, Any]] = None
        self.local_costmap_debug_nodes_version = 0

        # ROS 话题发布者与订阅者
        self._grid_map_pub: Optional[rospy.Publisher] = None
        self._goal_pub: Optional[rospy.Publisher] = None
        self._cancel_pub: Optional[rospy.Publisher] = None
        self._pcd_cmd_pub: Optional[rospy.Publisher] = None
        self._debug_query_pub: Optional[rospy.Publisher] = None
        self._debug_event = threading.Event()
        self._last_debug_result = ""

        # 仿真运动学与位姿状态 (publish_fake_tf=True 时启用动态积分, 移植自 jie_octomap ros_bridge)
        self.publish_fake_tf = False
        self.tf_broadcaster: Optional[tf2_ros.TransformBroadcaster] = None
        self.tf_child_frame = "base_link"
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.sim_yaw = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = None
        self.last_sim_time = None
        self.linear_noise_std = 0.008
        self.lateral_noise_std = 0.008
        self.angular_noise_std = 0.006
        self.max_step_height = 0.25

    def init_ros(self, node_name: str = "elevation_web_server"):
        """初始化 ROS 节点与通道"""
        try:
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
            self._grid_map_pub = rospy.Publisher(
                "/grid_map", GridMap, queue_size=1, latch=True
            )
            self._goal_pub = rospy.Publisher(
                "/move_base_simple/goal", PoseStamped, queue_size=1
            )
            self._initial_pose_pub = rospy.Publisher(
                "/initialpose", PoseWithCovarianceStamped, queue_size=1
            )
            self._cancel_pub = rospy.Publisher(
                "/move_base/cancel", GoalID, queue_size=1
            )
            self._pcd_cmd_pub = rospy.Publisher(
                "/pcd_file_cmd", RosString, queue_size=1
            )
            self._debug_query_pub = rospy.Publisher(
                "/elevation_debug_query", RosString, queue_size=5
            )

            # 仿真模式 (伪 TF): 参数、TF 广播器、里程计与速度指令 (移植自 jie_octomap ros_bridge)
            self.publish_fake_tf = rospy.get_param("~publish_fake_tf", False)
            self.tf_child_frame = rospy.get_param("~tf_child_frame", "base_link")
            self.linear_noise_std = rospy.get_param("~linear_noise_std", 0.008)
            self.lateral_noise_std = rospy.get_param("~lateral_noise_std", 0.008)
            self.angular_noise_std = rospy.get_param("~angular_noise_std", 0.006)
            self.sim_x = rospy.get_param("~init_x", 0.0)
            self.sim_y = rospy.get_param("~init_y", 0.0)
            self.sim_z = rospy.get_param("~init_z", 0.0)
            self.sim_yaw = rospy.get_param("~init_yaw", 0.0)
            # 地形跟随的分层带宽度: 与建图 max_step_height 同源 (相邻踏面高差极限, 跨层跳变被禁止)
            self.max_step_height = rospy.get_param(
                "/move_base/ElevationGlobalPlanner/max_step_height", 0.25)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self._odom_pub = rospy.Publisher("/loc_base", Odometry, queue_size=10)
            rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback, queue_size=5)

            if self.publish_fake_tf:
                self.last_sim_time = rospy.Time.now()
                rospy.Timer(rospy.Duration(0.02), self.publish_fake_tf_loop)  # 50Hz 高频广播与积分

            # 订阅机器人位姿（支持 /loc_base 或 /odom）
            rospy.Subscriber("/loc_base", Odometry, self._odom_callback, queue_size=5)
            rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=5)

            # 订阅全局规划路径与局部规划路径 (优先使用 3D 流形全局路径)
            rospy.Subscriber("/elevation_global_plan", ROSPath, self._global_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/plan", ROSPath, self._global_path_callback, queue_size=2)

            # 订阅局部规划路径 (多源兼容: TEB, AStarLocalPlanner 及原生 local_plan)
            rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", ROSPath, self._local_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/AStarLocalPlanner/local_plan", ROSPath, self._local_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/local_plan", ROSPath, self._local_path_callback, queue_size=2)
            rospy.Subscriber("/elevation_local_plan", ROSPath, self._local_path_callback, queue_size=2)

            # 订阅局部代价地图 (1:1 流形局部代价地图与原生 local_costmap)
            rospy.Subscriber("/elevation_local_costmap", OccupancyGrid, self._local_costmap_callback, queue_size=1)
            rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self._local_costmap_callback, queue_size=1)
            # 逐格成因码调试图层 (Web 点击诊断 "看不见的障碍")
            rospy.Subscriber("/elevation_local_costmap_debug", OccupancyGrid, self._local_costmap_debug_callback, queue_size=1)
            # 逐格胜出节点 id 调试图层 ([w, h, id...], id=-1 表示无节点)
            rospy.Subscriber("/elevation_local_costmap_debug_nodes", Int32MultiArray, self._local_costmap_debug_nodes_callback, queue_size=1)

            # 订阅流形图节点与边可视化数据
            rospy.Subscriber("/elevation_graph_nodes", PointCloud2, self._graph_nodes_callback, queue_size=1)
            rospy.Subscriber("/elevation_graph_edges", MarkerArray, self._graph_edges_callback, queue_size=1)
            rospy.Subscriber("/elevation_debug_result", RosString, self._debug_result_callback, queue_size=5)

            self.is_initialized = True
            rospy.loginfo("[ElevationRosBridge] ROS node and topic subscriptions ready")
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] ROS init exception: {e}")

    def _graph_nodes_callback(self, msg: PointCloud2):
        """解析流形踏面节点点云"""
        try:
            pts = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                pts.append([round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3), round(float(p[3]), 2)])
            with self._lock:
                self.graph_nodes = pts
                self.graph_nodes_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 graph nodes 点云异常: {e}")

    def _graph_edges_callback(self, msg: MarkerArray):
        """解析流形连通边 MarkerArray"""
        try:
            lines = []
            for marker in msg.markers:
                pts = marker.points
                for i in range(0, len(pts) - 1, 2):
                    lines.append([
                        round(pts[i].x, 3), round(pts[i].y, 3), round(pts[i].z, 3),
                        round(pts[i+1].x, 3), round(pts[i+1].y, 3), round(pts[i+1].z, 3)
                    ])
            with self._lock:
                self.graph_edges = lines
                self.graph_edges_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 graph edges 异常: {e}")

    def _local_costmap_debug_nodes_callback(self, msg: Int32MultiArray):
        """解析逐格胜出节点 id 调试图层
        新格式 [width, height, 表条数T, id×(w*h), (id, x_mm, y_mm, z_mm, trav_x100)×T]
        整包透传给前端 (含宽高), 前端按相同布局解析"""
        try:
            if len(msg.data) < 3:
                return
            w, h = int(msg.data[0]), int(msg.data[1])
            packed = array('i', msg.data).tobytes()
            b64_str = base64.b64encode(packed).decode('ascii')
            with self._lock:
                self.local_costmap_debug_nodes = {
                    "width": w,
                    "height": h,
                    "data": b64_str
                }
                self.local_costmap_debug_nodes_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 costmap debug nodes 异常: {e}")

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # 四元数提取 Yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            self.robot_pose = {
                "x": round(pos.x, 3),
                "y": round(pos.y, 3),
                "z": round(pos.z, 3),
                "yaw": round(yaw, 3)
            }

    def _global_path_callback(self, msg: ROSPath):
        pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        with self._lock:
            self.global_path = pts
            self.path_version += 1

    def _local_path_callback(self, msg: ROSPath):
        with self._lock:
            gpath = list(self.global_path) if self.global_path else []
            robot_z = self.robot_pose.get("z", 0.0) if self.robot_pose else 0.0

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            z = p.pose.position.z
            # 若局部规划器(如 ROS 原生 2D TEB)输出写死 z=0, 但实际机器人与地表处于 3D 高程下
            if abs(z) < 1e-3 and (gpath or abs(robot_z) > 0.05):
                # 分层带过滤: 优先在机器人当前层 |dz| <= max_step_height 的航点里取 2D 最近。
                # 跨层路径 (楼梯井/坡道) 在 2D 上重叠堆叠 —— 水平 0.4m 内可叠着 0.6m 与
                # 4.5m 两层踏面, 无脑取 2D 最近会把局部轨迹投影到楼上/楼下的航点上,
                # 黄线瞬移到完全错误的踏面高度。
                best_z = robot_z
                best_d2 = 999.0
                for band_only in (True, False):  # 先同层带, 带内无候选再放宽到全部航点
                    found = False
                    for gx, gy, gz in gpath:
                        if band_only and abs(gz - robot_z) > self.max_step_height:
                            continue
                        d2 = (x - gx)**2 + (y - gy)**2
                        if d2 < best_d2:
                            best_d2 = d2
                            best_z = gz
                            found = True
                    if found:
                        break
                z = best_z
            pts.append([round(x, 3), round(y, 3), round(z, 3)])

        with self._lock:
            self.local_path = pts

    def _local_costmap_callback(self, msg: OccupancyGrid):
        """解析 1:1 流形局部代价地图 (发布给 Web 前端渲染局部地毯)"""
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            res = float(msg.info.resolution)
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)
            oz = float(msg.info.origin.position.z)

            # 数据映射: 将 int8 数组转为无符号单字节 (255: 未知, 0: 自由, 1~99: 代价, 100: 致命障碍)
            raw = bytearray(len(msg.data))
            for i, val in enumerate(msg.data):
                raw[i] = val if val >= 0 else 255
            b64_str = base64.b64encode(raw).decode('ascii')

            with self._lock:
                self.local_costmap = {
                    "width": w,
                    "height": h,
                    "resolution": round(res, 3),
                    "origin": {
                        "x": round(ox, 3),
                        "y": round(oy, 3),
                        "z": round(oz, 3)
                    },
                    "data": b64_str
                }
                self.local_costmap_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 local costmap 异常: {e}")

    def _local_costmap_debug_callback(self, msg: OccupancyGrid):
        """解析逐格成因码调试图层 (与地毯同几何, 每格 CellReason 0~7)"""
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            raw = bytearray(len(msg.data))
            for i, val in enumerate(msg.data):
                raw[i] = val if val >= 0 else 255
            b64_str = base64.b64encode(raw).decode('ascii')
            with self._lock:
                self.local_costmap_debug = {
                    "width": w,
                    "height": h,
                    "resolution": round(float(msg.info.resolution), 3),
                    "origin": {
                        "x": round(float(msg.info.origin.position.x), 3),
                        "y": round(float(msg.info.origin.position.y), 3),
                        "z": round(float(msg.info.origin.position.z), 3)
                    },
                    "data": b64_str
                }
                self.local_costmap_debug_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 costmap debug 异常: {e}")

    # ------------------ 仿真运动学积分与 TF/Odom 高频广播 (移植自 jie_octomap) ------------------
    def _cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def get_terrain_z(self, x: float, y: float, fallback_z: float) -> float:
        """基于流形踏面节点缓存 (graph_nodes: [x, y, z, traversability]) 探测 (x, y) 处的地表高度。
        分层带过滤: 优先取 |dz| <= max_step_height 的节点 (同层踏面逐级跟随),
        该带内无节点才放宽窗口 —— 防止行进中经过楼板空洞/边缘时 z 跳到下层"""
        best_z = None
        min_sq = 0.35 * 0.35  # 机体半径范围 (35cm)

        with self._lock:
            nodes = self.graph_nodes

        # 第一带: 机体当前层附近 (|dz| <= max_step_height)
        for p in nodes:
            if p[3] >= 0.95:  # 禁行节点 (顶头净空/侧向阻挡) 不可作为贴地依据
                continue
            dz = p[2] - fallback_z
            if abs(dz) > self.max_step_height:
                continue
            dx = p[0] - x
            dy = p[1] - y
            sq = dx * dx + dy * dy
            if sq < min_sq:
                min_sq = sq
                best_z = p[2]

        if best_z is not None:
            return best_z

        # 第二带 (放宽): 当前层附近无踏面节点时, 扩大到机体上下活动窗口
        min_sq = 0.35 * 0.35
        for p in nodes:
            if p[3] >= 0.95:
                continue
            # 过滤掉远高于机体/天花板或深坑
            if p[2] > fallback_z + 0.5 or p[2] < fallback_z - 1.8:
                continue
            dx = p[0] - x
            dy = p[1] - y
            sq = dx * dx + dy * dy
            if sq < min_sq:
                min_sq = sq
                best_z = p[2]

        if best_z is not None:
            return best_z

        # 次选：局部盲区无踏面节点时，回退参考规划路径的 3D 航路点高度
        with self._lock:
            local_path = list(self.local_path)
            global_path = list(self.global_path)
        for path_points in [local_path, global_path]:
            if path_points:
                min_path_sq = 0.8 * 0.8
                path_z = None
                for pt in path_points:
                    dx = pt[0] - x
                    dy = pt[1] - y
                    sq = dx * dx + dy * dy
                    if sq < min_path_sq:
                        min_path_sq = sq
                        path_z = pt[2]
                if path_z is not None:
                    return path_z

        return fallback_z

    def set_sim_pose(self, x: float, y: float, z: float, yaw: Optional[float] = None):
        """前端修改初始位置时, 直接采用吸附节点的踏面高程 z (前端射线拾取已确定层归属),
        立即同步并立即更新 TF; 不再重新贴地探测 —— 防止多层重叠处重新吸附掉到下层"""
        self.sim_x = float(x)
        self.sim_y = float(y)
        self.sim_z = float(z) if z is not None else self.get_terrain_z(self.sim_x, self.sim_y, 0.0)
        if yaw is not None:
            self.sim_yaw = float(yaw)
        self.cmd_vx, self.cmd_vy, self.cmd_wz = 0.0, 0.0, 0.0
        self.last_sim_time = rospy.Time.now()
        if self.publish_fake_tf:
            self.publish_fake_tf_loop()

    def publish_fake_tf_loop(self, event=None):
        if not self.publish_fake_tf or self.tf_broadcaster is None:
            return
        try:
            now_stamp = rospy.Time.now()

            # 运动学积分更新
            if self.last_sim_time is not None:
                dt = (now_stamp - self.last_sim_time).to_sec()
                if 0.0 < dt < 0.5:
                    if self.last_cmd_time is not None and (now_stamp - self.last_cmd_time).to_sec() <= 0.5:
                        vx, vy, wz = self.cmd_vx, self.cmd_vy, self.cmd_wz
                        is_moving = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
                        if is_moving:
                            noisy_vx = vx + random.gauss(0.0, self.linear_noise_std)
                            noisy_vy = vy + random.gauss(0.0, self.lateral_noise_std)
                            noisy_wz = wz + random.gauss(0.0, self.angular_noise_std)
                        else:
                            noisy_vx, noisy_vy, noisy_wz = 0.0, 0.0, 0.0
                    else:
                        noisy_vx, noisy_vy, noisy_wz = 0.0, 0.0, 0.0

                    cos_y = math.cos(self.sim_yaw)
                    sin_y = math.sin(self.sim_yaw)
                    dx_body = noisy_vx * dt
                    dy_body = noisy_vy * dt
                    dyaw = noisy_wz * dt

                    self.sim_x += dx_body * cos_y - dy_body * sin_y
                    self.sim_y += dx_body * sin_y + dy_body * cos_y
                    self.sim_yaw = normalize_angle(self.sim_yaw + dyaw)

                    # 动态自动跟随 3D 地形与规划路径的 z 高度 (平滑低通滤波过渡)
                    target_z = self.get_terrain_z(self.sim_x, self.sim_y, self.sim_z)
                    self.sim_z = 0.85 * self.sim_z + 0.15 * target_z

            self.last_sim_time = now_stamp
            q = euler_to_quaternion(0.0, 0.0, self.sim_yaw)

            # 1. 广播 TF 变换树: map -> odom -> base_link / base_footprint
            transforms = []
            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = now_stamp
            t_map_odom.header.frame_id = "map"
            t_map_odom.child_frame_id = "odom"
            t_map_odom.transform.rotation.w = 1.0
            transforms.append(t_map_odom)

            for frame_name in dict.fromkeys([self.tf_child_frame, "base_link", "base_footprint"]):
                t_odom = TransformStamped()
                t_odom.header.stamp = now_stamp
                t_odom.header.frame_id = "odom"
                t_odom.child_frame_id = frame_name
                t_odom.transform.translation.x = self.sim_x
                t_odom.transform.translation.y = self.sim_y
                t_odom.transform.translation.z = self.sim_z
                t_odom.transform.rotation = q
                transforms.append(t_odom)

            self.tf_broadcaster.sendTransform(transforms)

            # 2. 发布 /loc_base 里程计
            if self._odom_pub:
                odom_msg = Odometry()
                odom_msg.header.stamp = now_stamp
                odom_msg.header.frame_id = "map"
                odom_msg.child_frame_id = self.tf_child_frame
                odom_msg.pose.pose.position.x = self.sim_x
                odom_msg.pose.pose.position.y = self.sim_y
                odom_msg.pose.pose.position.z = self.sim_z
                odom_msg.pose.pose.orientation = q
                odom_msg.twist.twist.linear.x = self.cmd_vx
                odom_msg.twist.twist.linear.y = self.cmd_vy
                odom_msg.twist.twist.angular.z = self.cmd_wz
                self._odom_pub.publish(odom_msg)
        except Exception:
            pass

    def publish_grid_map(self, grid_map_data: Dict[str, Any], frame_id: str = "map"):
        """将生成的 GridMap 数据发布为标准 ROS grid_map_msgs/GridMap"""
        if not self._grid_map_pub:
            return

        meta = grid_map_data["metadata"]
        layers = grid_map_data["layers"]

        msg = GridMap()
        msg.info.header.stamp = rospy.Time.now()
        msg.info.header.frame_id = frame_id
        msg.info.resolution = float(meta["resolution"])
        msg.info.length_x = float(meta["length_x"])
        msg.info.length_y = float(meta["length_y"])
        msg.info.pose.position.x = float(meta["center_x"])
        msg.info.pose.position.y = float(meta["center_y"])
        msg.info.pose.position.z = 0.0
        msg.info.pose.orientation.w = 1.0

        rows = int(meta["rows"])
        cols = int(meta["cols"])

        msg.layers = []
        msg.basic_layers = ["elevation"]

        dim0 = MultiArrayDimension(label="column_index", size=cols, stride=rows * cols)
        dim1 = MultiArrayDimension(label="row_index", size=rows, stride=rows)
        layout = MultiArrayLayout(dim=[dim0, dim1], data_offset=0)

        for layer_name, array_data in layers.items():
            msg.layers.append(layer_name)
            if isinstance(array_data, np.ndarray):
                data_1d = array_data.flatten().tolist()
            else:
                # 嵌套 list 展平并替换 None 为 nan
                flat = []
                for row in array_data:
                    for val in row:
                        flat.append(float('nan') if val is None else float(val))
                data_1d = flat
            arr = Float32MultiArray(layout=layout, data=data_1d)
            msg.data.append(arr)

        self._grid_map_pub.publish(msg)
        rospy.loginfo(f"[ElevationRosBridge] Published ROS GridMap: {cols}x{rows}, res = {meta['resolution']}m")

    def publish_initial_pose(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0, frame_id: str = "map"):
        """发布起始位姿并直接更新本地机器人模型; 仿真模式下同步伪 TF 位姿"""
        if self._initial_pose_pub:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = frame_id
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            pose.pose.pose.position.z = z
            pose.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)
            pose.pose.covariance[0] = 0.25
            pose.pose.covariance[7] = 0.25
            pose.pose.covariance[35] = 0.068
            self._initial_pose_pub.publish(pose)

        # 仿真模式: Web 设置起点 = 触发伪 TF 变换 (立即生效); 实机模式不动 TF, 仅保留 /initialpose
        if self.publish_fake_tf:
            self.set_sim_pose(x, y, z, yaw)
            return

        with self._lock:
            self.robot_pose = {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3), "yaw": round(yaw, 3)}

    def publish_goal(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0, frame_id: str = "map"):
        """发布导航目标点"""
        if not self._goal_pub:
            return
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)
        self._goal_pub.publish(goal)

    def cancel_navigation(self):
        """取消当前导航目标"""
        if self._cancel_pub:
            self._cancel_pub.publish(GoalID())

    def publish_pcd_cmd(self, pcd_path: str):
        """向 C++ 节点发送 PCD 加载指令"""
        if self._pcd_cmd_pub:
            self._pcd_cmd_pub.publish(RosString(data=pcd_path))
            rospy.loginfo(f"[ElevationRosBridge] Sent PCD path command: {pcd_path}")

    def _debug_result_callback(self, msg: RosString):
        """接收 C++ 规划器节点返回的权威诊断结果"""
        with self._lock:
            self._last_debug_result = msg.data
            self._debug_event.set()

    def diagnose_edge(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float, timeout: float = 1.0) -> Dict[str, Any]:
        """向 C++ 节点请求两踏面方块的权威拓扑邻边关系与物理原因诊断"""
        if not self._debug_query_pub:
            return {"status": "error", "message": "ROS bridge publisher not ready"}
        self._debug_event.clear()
        query_str = json.dumps({"x1": x1, "y1": y1, "z1": z1, "x2": x2, "y2": y2, "z2": z2})
        self._debug_query_pub.publish(RosString(data=query_str))
        return self._wait_debug_result(timeout)

    def diagnose_node(self, x: float, y: float, z: float, timeout: float = 1.0) -> Dict[str, Any]:
        """向 C++ 节点请求单个踏面方块的权威通行状态与禁行原因诊断"""
        if not self._debug_query_pub:
            return {"status": "error", "message": "ROS bridge publisher not ready"}
        self._debug_event.clear()
        query_str = json.dumps({"mode": "node", "x1": x, "y1": y, "z1": z})
        self._debug_query_pub.publish(RosString(data=query_str))
        return self._wait_debug_result(timeout)

    def _wait_debug_result(self, timeout: float) -> Dict[str, Any]:
        """等待 C++ 节点在 /elevation_debug_result 上的权威诊断回包"""
        if self._debug_event.wait(timeout=timeout):
            with self._lock:
                try:
                    return json.loads(self._last_debug_result)
                except Exception as e:
                    return {"status": "error", "message": f"Failed to parse C++ result: {e}"}
        return {"status": "timeout", "message": "C++ planner node did not respond in time"}

    def get_live_state(self) -> Dict[str, Any]:
        """获取当前实时状态快照"""
        with self._lock:
            return {
                "robot_pose": dict(self.robot_pose),
                "global_path": list(self.global_path),
                "local_path": list(self.local_path),
                "path_version": self.path_version,
                "graph_nodes": list(self.graph_nodes),
                "graph_nodes_version": self.graph_nodes_version,
                "graph_edges": list(self.graph_edges),
                "graph_edges_version": self.graph_edges_version,
                "local_costmap": dict(self.local_costmap) if self.local_costmap else None,
                "local_costmap_version": self.local_costmap_version,
                "local_costmap_debug": dict(self.local_costmap_debug) if self.local_costmap_debug else None,
                "local_costmap_debug_version": self.local_costmap_debug_version,
                "local_costmap_debug_nodes": dict(self.local_costmap_debug_nodes) if self.local_costmap_debug_nodes else None,
                "local_costmap_debug_nodes_version": self.local_costmap_debug_nodes_version
            }


ros_bridge = ElevationRosBridge()
