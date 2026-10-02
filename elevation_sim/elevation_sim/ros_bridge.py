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
from collections import defaultdict
from typing import Dict, Any, Optional, List, Tuple
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Path as ROSPath, Odometry
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String as RosString, Header
from grid_map_msgs.msg import GridMap, GridMapInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from elevation_planner_core.srv import DiagnoseQuery

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

        self.graph_nodes: List[List[float]] = []
        self.graph_nodes_version = 0
        self.spatial_nodes: Dict[Tuple[int, int], List[List[float]]] = {}
        self.graph_edges: List[List[float]] = []
        self.graph_edges_version = 0
        self.graph_adj: Dict[Tuple[float, float, float], List[Tuple[float, float, float]]] = {}
        self.current_graph_node: Optional[Tuple[float, float, float]] = None
        # 局部 SFC 走廊诊断数据 (各路径点与扩散节点群)
        self.sfc_corridors_debug: List[Dict[str, Any]] = []
        self.sfc_corridors_debug_version = 0
        # 融合引擎动态改写节点的通行性覆盖层: {(x,y,z): trav}
        # 伪 TF 贴地跟随据此排除被障碍封锁的节点 (静态图不随融合更新)
        self.dynamic_trav: Dict[Tuple[float, float, float], float] = {}
        # 融合引擎动态改写节点的区划类型覆盖层: {(x,y,z): zone} (0=FREE, 1=SOFT, 2=BODY_HARD, 3=FORBIDDEN)
        self.dynamic_zone: Dict[Tuple[float, float, float], int] = {}
        # 注入障碍物 (供前端 3D 可视化)
        self.injected_obstacles: List[Dict[str, Any]] = []
        self.injected_obstacles_version = 0
        # Web 栅格编辑器手动摆放的障碍 (freeze 调试模式用): [(x, y, z_top), ...]
        self.editor_obstacles: List[List[float]] = []
        self.last_goal: Optional[Tuple[float, float, float, float, str]] = None
        # 融合引擎动态标记节点 (封锁/软代价, 实时可视化)
        self.dynamic_nodes: List[List[float]] = []
        self.dynamic_nodes_version = 0
        self.rebound_arrows: List[Dict[str, Any]] = []
        self.rebound_arrows_version = 0
        # 平移锁止/碰撞节点 (供前端高亮显示): [x, y, z, zone] (zone=3物理障碍/2机体禁行环/4顶盖)
        self.collision_node: Optional[List[float]] = None
        self.collision_points: List[List[float]] = []
        self.collision_node_time: float = 0.0

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
        self.cmd_vel_freeze = False
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
            self._cmd_vel_pub = rospy.Publisher(
                "/cmd_vel", Twist, queue_size=1
            )
            self._pcd_cmd_pub = rospy.Publisher(
                "/pcd_file_cmd", RosString, queue_size=1
            )
            self._debug_query_pub = rospy.Publisher(
                "/elevation_debug_query", RosString, queue_size=5
            )

            # 仿真模式 (伪 TF): 参数、TF 广播器、里程计与速度指令 (移植自 jie_octomap ros_bridge)
            self.publish_fake_tf = rospy.get_param("~publish_fake_tf", False)
            self.cmd_vel_freeze = bool(rospy.get_param("~cmd_vel_freeze", False))
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
            # 机体硬半径: 与 planner_common.yaml/代价地图同源, 平移闸的支撑距离上限
            self.body_hard_radius = float(rospy.get_param("body_hard_radius", 0.17))
            # 机体站立高度: 与 planner_common.yaml 同源, 预检 R1 判据的机体高度上界
            self.dog_height = float(rospy.get_param("dog_height", 0.45))
            # 足印半径: 用于脚底大部分踏面方块高度聚类
            self.footprint_radius = float(rospy.get_param("footprint_radius", 0.25))
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self._odom_pub = rospy.Publisher("/loc_base", Odometry, queue_size=10)
            rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback, queue_size=5)

            if self.publish_fake_tf:
                self.last_sim_time = rospy.Time.now()
                import threading
                self._tf_thread = threading.Thread(target=self._fake_tf_thread_worker, daemon=True)
                self._tf_thread.start()

            # 订阅机器人位姿（支持 /loc_base 或 /odom）
            rospy.Subscriber("/loc_base", Odometry, self._odom_callback, queue_size=5)
            rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=5)

            # 订阅全局规划路径与局部规划路径 (优先使用 3D 流形全局路径)
            rospy.Subscriber("/elevation_global_plan", ROSPath, self._global_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/plan", ROSPath, self._global_path_callback, queue_size=2)

            # 订阅局部规划路径 (多源兼容: local_spline_plan, AStarLocalPlanner 及原生 local_plan)
            rospy.Subscriber("/move_base/local_spline_plan", ROSPath, self._local_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/AStarLocalPlanner/local_plan", ROSPath, self._local_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/local_plan", ROSPath, self._local_path_callback, queue_size=2)
            rospy.Subscriber("/elevation_local_plan", ROSPath, self._local_path_callback, queue_size=2)

            # 订阅流形图节点与边可视化数据
            rospy.Subscriber("/elevation_graph_nodes", PointCloud2, self._graph_nodes_callback, queue_size=1)
            rospy.Subscriber("/elevation_graph_edges", MarkerArray, self._graph_edges_callback, queue_size=1)
            rospy.Subscriber("/elevation_local_corridors_debug", RosString, self._sfc_corridors_debug_callback, queue_size=1)
            rospy.Subscriber("/elevation_debug_result", RosString, self._debug_result_callback, queue_size=5)
            rospy.Subscriber("/elevation_injected_obstacles", MarkerArray, self._injected_obstacles_callback, queue_size=1)
            rospy.Subscriber("/elevation_dynamic_nodes", PointCloud2, self._dynamic_nodes_callback, queue_size=1)
            rospy.Subscriber("/elevation_rebound_debug", MarkerArray, self._rebound_callback, queue_size=1)

            # 订阅仿真器发布的碰撞事件
            rospy.Subscriber("/elevation_collision_node", RosString, self._collision_node_callback, queue_size=5)

            # 编辑器障碍点云: 与注入器同路径 /lidar_points (freeze 下注入器已禁用, 无交错;
            # 融合引擎对该话题的摄取链路被注入器时代反复验证过)
            self._editor_cloud_pub = rospy.Publisher("/lidar_points", PointCloud2, queue_size=1)
            rospy.Timer(rospy.Duration(0.5), self._publish_editor_cloud)

            self.is_initialized = True
            rospy.loginfo("[ElevationRosBridge] ROS node and topic subscriptions ready")
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] ROS init exception: {e}")

    def _collision_node_callback(self, msg: RosString):
        """接收独立仿真器发出的碰撞事件与碰撞点云"""
        try:
            if not msg.data:
                with self._lock:
                    self.collision_node = None
                    self.collision_points = []
                return
            data = json.loads(msg.data)
            if data and "x" in data:
                with self._lock:
                    self.collision_node = [float(data["x"]), float(data["y"]), float(data["z"]), int(data.get("type", 3))]
                    self.collision_points = data.get("points", [])
            else:
                with self._lock:
                    self.collision_node = None
                    self.collision_points = []
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 collision_node 异常: {e}")

    def _sfc_corridors_debug_callback(self, msg: RosString):
        """解析局部 SFC 凸多边形走廊调试数据 (包含各路径点坐标、node_id 及 8 邻域扩散节点群)"""
        try:
            data = json.loads(msg.data)
            corridors = data.get("corridors", [])
            with self._lock:
                self.sfc_corridors_debug = corridors
                self.sfc_corridors_debug_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 sfc_corridors_debug 异常: {e}")

    def _dynamic_nodes_callback(self, msg: PointCloud2):
        """接收融合引擎动态改写节点 (x,y,z,intensity=trav,zone), 单次遍历同时维护两条消费链:
        1. dynamic_nodes 列表 → Web 前端实时渲染 (zone 三档配色: 禁行=红/机体硬环=橙/软代价=紫)
        2. dynamic_trav 覆盖层 → _effective_trav/贴地/锚定查询的融合实时值
        覆盖层缺失的节点回退静态图值; 覆盖层存在的节点以融合实时值为准。
        zone 字段缺省 (旧发布端) 时置 None, 前端回退 trav 阈值配色。"""
        try:
            has_zone = any(f.name == "zone" for f in msg.fields)
            nodes = []
            overlay_trav = {}
            overlay_zone = {}
            if has_zone:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "zone"), skip_nans=True):
                    x = round(float(p[0]), 3)
                    y = round(float(p[1]), 3)
                    z = round(float(p[2]), 3)
                    trav = round(float(p[3]), 2)
                    zone = int(p[4])
                    nodes.append([x, y, z, trav, zone])
                    overlay_trav[(x, y, z)] = trav
                    overlay_zone[(x, y, z)] = zone
            else:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                    x = round(float(p[0]), 3)
                    y = round(float(p[1]), 3)
                    z = round(float(p[2]), 3)
                    trav = round(float(p[3]), 2)
                    zone = 3 if trav >= 0.95 else 0
                    nodes.append([x, y, z, trav])
                    overlay_trav[(x, y, z)] = trav
                    overlay_zone[(x, y, z)] = zone
            with self._lock:
                self.dynamic_nodes = nodes
                self.dynamic_nodes_version += 1
                self.dynamic_trav = overlay_trav
                self.dynamic_zone = overlay_zone
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 dynamic_nodes 异常: {e}")

    def _effective_trav(self, x: float, y: float, z: float) -> float:
        """节点有效通行性 = 融合动态覆盖层优先, 回退静态图值; 查不到按可通行 0.0"""
        key = (round(x, 3), round(y, 3), round(z, 3))
        dyn = self.dynamic_trav.get(key)
        if dyn is not None:
            return dyn
        cell = self.spatial_nodes.get((int(round(x / 0.10)), int(round(y / 0.10))))
        if cell:
            for p in cell:
                if abs(p[0] - x) < 0.02 and abs(p[1] - y) < 0.02 and abs(p[2] - z) < 0.02:
                    return p[3]
        return 0.0

    def _effective_zone(self, x: float, y: float, z: float) -> int:
        """节点有效区划类型 (CostZone: 0=FREE, 1=SOFT, 2=BODY_HARD, 3=FORBIDDEN)"""
        key = (round(x, 3), round(y, 3), round(z, 3))
        dyn_z = self.dynamic_zone.get(key)
        if dyn_z is not None:
            return dyn_z
        cell = self.spatial_nodes.get((int(round(x / 0.10)), int(round(y / 0.10))))
        if cell:
            for p in cell:
                if abs(p[0] - x) < 0.02 and abs(p[1] - y) < 0.02 and abs(p[2] - z) < 0.02:
                    return int(p[4]) if len(p) >= 5 else (2 if p[3] >= 0.95 else 0)
        return 0

    def _rebound_callback(self, msg: MarkerArray):
        """接收样条优化 rebound 定向排斥可视化箭头 (障碍面参考点 → 控制点)。
        color 红 = 弹簧受压 (控制点仍在 clearance 内), 绿 = 已弹到安全间距外; 纯透传供前端渲染"""
        try:
            arrows = []
            for m in msg.markers:
                if m.action != Marker.ADD or m.type != Marker.ARROW or len(m.points) < 2:
                    continue
                p0, p1 = m.points[0], m.points[1]
                arrows.append({
                    "x0": round(float(p0.x), 3), "y0": round(float(p0.y), 3), "z0": round(float(p0.z), 3),
                    "x1": round(float(p1.x), 3), "y1": round(float(p1.y), 3), "z1": round(float(p1.z), 3),
                    "pressed": bool(m.color.r > 0.5 and m.color.g < 0.5),
                })
            with self._lock:
                self.rebound_arrows = arrows
                self.rebound_arrows_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 rebound arrows 异常: {e}")

    # ------------------ Web 编辑器障碍 (freeze 调试模式手动摆放) ------------------
    # 与自动注入器同规格的圆柱 (radius 0.25, height 0.5): 高于 max_step_height 成墙, 不可绕胯
    OBSTACLE_RADIUS = 0.25
    OBSTACLE_HEIGHT = 0.5

    def toggle_editor_obstacle(self, x: float, y: float, z: float) -> Dict[str, Any]:
        """在踏面节点 (x, y, z) 上放置/移除一个编辑障碍物 (0.15m 内视为同一格)"""
        changed = False
        for i, (ox, oy, oz) in enumerate(self.editor_obstacles):
            if abs(ox - x) < 0.15 and abs(oy - y) < 0.15 and abs(oz - z) < 0.15:
                removed = self.editor_obstacles.pop(i)
                changed = True
                msg = f"已移除障碍物 ({ox:.2f}, {oy:.2f}, {oz:.2f}), 剩余 {len(self.editor_obstacles)} 个"
                break
        if not changed:
            self.editor_obstacles.append([round(x, 3), round(y, 3), round(z, 3)])
            # 距机体距离反馈: 融合引擎只摄取 crop_radius_xy (3.5m) 内的点云
            dist_txt = ""
            with self._lock:
                rp = self.robot_pose
            if rp:
                d = math.hypot(x - rp.get("x", 0.0), y - rp.get("y", 0.0))
                dist_txt = ", 距机体 {:.1f}m {}".format(
                    d, "✓融合范围内" if d <= 3.5 else "✗超出融合ROI 3.5m, 不会封锁!")
            msg = (f"已添加障碍物 ({x:.2f}, {y:.2f}, {z:.2f}), 共 {len(self.editor_obstacles)} 个{dist_txt}")
        # 版本递增: ws 帧的 injected_obstacles 字段按版本门控, freeze 下注入器停发,
        # 不递增则编辑障碍永不进帧 (红圆柱不显示)
        with self._lock:
            self.injected_obstacles_version += 1
        # 摆放/移除后: 立即补发一帧点云 (不等 2Hz 定时器), 融合摄取 ~0.1s 后即触发重规划
        self._publish_editor_cloud()
        rospy.Timer(rospy.Duration(0.15), self._publish_plan_once, oneshot=True)
        return {"action": "removed" if changed else "added",
                "obstacle": removed if changed else [x, y, z], "message": msg}

    def _publish_plan_once(self, _event=None):
        """编辑障碍变更后重发上个 goal: move_base 完整重跑全局 A* + setPlan + 局部管线"""
        try:
            if not self.last_goal:
                rospy.logwarn("[ElevationRosBridge] 编辑障碍变更, 但尚无 goal 可重发, 跳过重规划")
                return
            x, y, z, yaw, frame_id = self.last_goal
            self.publish_goal(x, y, z, yaw, frame_id)
            rospy.loginfo("[ElevationRosBridge] 编辑障碍变更 -> 已重发 goal 触发完整重规划")
        except Exception:
            pass

    def _publish_editor_cloud(self, _event=None):
        """把编辑障碍圆柱采样为表面点云 (侧表面 + 顶盘, 与注入器 sample_cylinder 同法)
        发布到 /lidar_points (融合引擎逐帧摄取)"""
        if not self.editor_obstacles:
            return
        try:
            pts = []
            R, H = self.OBSTACLE_RADIUS, self.OBSTACLE_HEIGHT
            spacing = 0.04
            for (ox, oy, z_top) in self.editor_obstacles:
                z0 = z_top  # 圆柱底面贴踏面
                pts.append((ox, oy, z0 + H))                                   # 顶面圆心
                for z in self._arange_incl(0.0, H, spacing):                   # 侧表面
                    n = max(12, int(math.ceil(2 * math.pi * R / spacing)))
                    for k in range(n):
                        a = 2 * math.pi * k / n
                        pts.append((ox + R * math.cos(a), oy + R * math.sin(a), z0 + z))
                for r in self._arange_incl(spacing, R, spacing):               # 顶盘
                    n = max(8, int(math.ceil(2 * math.pi * r / spacing)))
                    for k in range(n):
                        a = 2 * math.pi * k / n
                        pts.append((ox + r * math.cos(a), oy + r * math.sin(a), z0 + H))
                # 内部填充: 中心轴竖线 + 半径 R/2 内环 —— 只采侧壁+顶盘时, 圆柱中心列
                # 头顶只有 +0.5m 的顶盘一个面, 净空 0.5 >= dog_height 0.45 被判可通行,
                # A* 会从圆柱正中心穿过去; 填充后整个投影面积净空 ≈ 0, 全面封锁
                for z in self._arange_incl(0.0, H, spacing):
                    pts.append((ox, oy, z0 + z))
                    for k in range(4):
                        a = math.pi / 2 * k + math.pi / 4
                        pts.append((ox + R / 2 * math.cos(a), oy + R / 2 * math.sin(a), z0 + z))
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            self._editor_cloud_pub.publish(pc2.create_cloud_xyz32(header, pts))
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[ElevationRosBridge] 编辑器障碍点云发布失败: {e}")

    @staticmethod
    def _arange_incl(start, stop, step):
        out = []
        v = start
        while v <= stop + 1e-9:
            out.append(v)
            v += step
        return out

    def _injected_obstacles_callback(self, msg: MarkerArray):
        """接收注入障碍物真值 MarkerArray (CUBE/CYLINDER), 转紧凑 JSON 供前端渲染"""
        try:
            obs = []
            for m in msg.markers:
                if m.action == Marker.ADD:
                    obs.append({
                        "id": int(m.id),
                        "type": int(m.type),  # 1=CUBE, 3=CYLINDER
                        "x": round(float(m.pose.position.x), 3),
                        "y": round(float(m.pose.position.y), 3),
                        "z": round(float(m.pose.position.z), 3),
                        "qx": round(float(m.pose.orientation.x), 4),
                        "qy": round(float(m.pose.orientation.y), 4),
                        "qz": round(float(m.pose.orientation.z), 4),
                        "qw": round(float(m.pose.orientation.w), 4),
                        "sx": round(float(m.scale.x), 3),
                        "sy": round(float(m.scale.y), 3),
                        "sz": round(float(m.scale.z), 3),
                        "color": [round(float(m.color.r), 3), round(float(m.color.g), 3),
                                  round(float(m.color.b), 3), round(float(m.color.a), 3)]
                    })
            with self._lock:
                self.injected_obstacles = obs
                self.injected_obstacles_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 injected_obstacles 异常: {e}")

    def _graph_nodes_callback(self, msg: PointCloud2):
        """解析流形踏面节点点云并建立空间栅格哈希。
        点云含 zone 字段 (CostZone: 0=自由/1=软代价/2=机体硬环/3=禁行) 时透传为第 5 元,
        供前端三档配色; 旧格式 4 元照常解析。"""
        try:
            has_zone = any(f.name == "zone" for f in msg.fields)
            pts = []
            spatial = {}
            if has_zone:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "zone"), skip_nans=True):
                    pt = [round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3),
                          round(float(p[3]), 2), int(p[4])]
                    pts.append(pt)
                    r = int(round(pt[0] / 0.10))
                    c = int(round(pt[1] / 0.10))
                    spatial.setdefault((r, c), []).append(pt)
            else:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                    pt = [round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3), round(float(p[3]), 2)]
                    pts.append(pt)
                    r = int(round(pt[0] / 0.10))
                    c = int(round(pt[1] / 0.10))
                    spatial.setdefault((r, c), []).append(pt)
            with self._lock:
                self.graph_nodes = pts
                self.spatial_nodes = spatial
                self.graph_nodes_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 graph nodes 点云异常: {e}")

    def _anchor_graph_node_locked(self, x: float, y: float, z: float) -> bool:
        """仅在 (x, y) 局部栅格邻域内锚定可通行踏面节点, 绝不全图兜底。
        找不到合法锚点时保持原锚点不动并返回 False (调用方原地卡住 + 报错):
        宁可卡在障碍物里, 也绝不吸附到禁行节点/天花板/其它楼层。"""
        r0 = int(round(x / 0.10))
        c0 = int(round(y / 0.10))
        # 注意: 本函数约定调用方已持有 self._lock (后缀 _locked), 此处不可再加锁,
        # threading.Lock 不可重入, 二次加锁会死锁冻结 100Hz TF 线程
        spatial = self.spatial_nodes
        if not spatial:
            return False
        # 层带约束: 跨层只能沿楼梯逐级爬升, 锚定一步至多 2×max_step_height,
        # 半层高差 (楼板/天花板) 的节点不在搜索范围
        z_band = 2.0 * self.max_step_height
        best_node = None
        best_d2 = float("inf")
        for dr in range(-6, 7):
            for dc in range(-6, 7):
                cell = spatial.get((r0 + dr, c0 + dc))
                if not cell:
                    continue
                for p in cell:
                    if abs(p[2] - z) > z_band or p[3] >= 0.95:
                        continue
                    if self.dynamic_trav.get((p[0], p[1], p[2]), 0.0) >= 0.95:
                        continue
                    d2 = (p[0] - x)**2 + (p[1] - y)**2 + 16.0 * (p[2] - z)**2
                    if d2 < best_d2:
                        best_d2 = d2
                        best_node = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
        if best_node is not None:
            self.current_graph_node = best_node
        return best_node is not None

    def _graph_edges_callback(self, msg: MarkerArray):
        """解析流形连通边 MarkerArray 并构建拓扑连通图邻接表"""
        try:
            lines = []
            adj = defaultdict(set)
            for marker in msg.markers:
                pts = marker.points
                for i in range(0, len(pts) - 1, 2):
                    p1 = (round(pts[i].x, 3), round(pts[i].y, 3), round(pts[i].z, 3))
                    p2 = (round(pts[i+1].x, 3), round(pts[i+1].y, 3), round(pts[i+1].z, 3))
                    lines.append([p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]])
                    adj[p1].add(p2)
                    adj[p2].add(p1)
            graph_adj = {k: list(v) for k, v in adj.items()}
            with self._lock:
                self.graph_edges = lines
                self.graph_edges_version += 1
                self.graph_adj = graph_adj
                # 若当前尚未锚定连通图节点，或脱离连通图，立即锚定当前仿真位置
                if self.graph_adj and (self.current_graph_node is None or self.current_graph_node not in self.graph_adj):
                    self._anchor_graph_node_locked(self.sim_x, self.sim_y, self.sim_z)
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 graph edges 异常: {e}")

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

    # ------------------ 仿真运动学积分与 TF/Odom 高频广播 (移植自 jie_octomap) ------------------
    def _cmd_vel_callback(self, msg: Twist):
        # freeze 调试模式: 丢弃一切来源的速度指令 (规划器/recovery/手动), 狗钉在原地
        if self.cmd_vel_freeze:
            return
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def get_terrain_z(self, x: float, y: float, fallback_z: float) -> float:
        """基于流形踏面空间拓扑缓存探测 (x, y) 处的地表高度。
        多层建筑防击穿与台阶平滑策略:
        1. 空间局部邻域: 在 (x, y) 的周围网格邻域内检索候选节点，杜绝全图无界遍历与异层干扰
        2. 第一带: 优先取当前层踏面 (|dz| <= max_step_height, 正常水平或台阶小步跟随)
        3. 第二带 (规划路径连续性引导): 当跨步较大时，优先参考当前行进楼层的 3D 规划路径
        4. 第三带: 小幅单步容限 [-0.35m, +0.40m]
        """
        r0 = int(round(x / 0.10))
        c0 = int(round(y / 0.10))
        local_nodes = []

        with self._lock:
            spatial = self.spatial_nodes
            for dr in range(-3, 4):
                for dc in range(-3, 4):
                    cell = spatial.get((r0 + dr, c0 + dc))
                    if cell:
                        local_nodes.extend(cell)

        if not local_nodes:
            with self._lock:
                local_nodes = self.graph_nodes

        best_z = None
        min_sq = 0.35 * 0.35  # 机体半径范围 (35cm)

        # 第一带: 机体当前层附近 (|dz| <= max_step_height)
        for p in local_nodes:
            if p[3] >= 0.95:  # 禁行节点不可作为贴地依据
                continue
            if self.dynamic_trav.get((p[0], p[1], p[2]), 0.0) >= 0.95:
                continue  # 融合动态封锁 (障碍占用)
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

        # 第二带: 规划路径连续引导 (沿局部/全局 3D 路径走廊平滑过渡，杜绝台阶处因离散化掉层)
        with self._lock:
            local_path = list(self.local_path)
            global_path = list(self.global_path)
        for path_points in [local_path, global_path]:
            if path_points:
                min_path_sq = 0.60 * 0.60
                path_z = None
                for pt in path_points:
                    dx = pt[0] - x
                    dy = pt[1] - y
                    sq = dx * dx + dy * dy
                    if sq < min_path_sq:
                        # 确保路径点属于当前行进楼层/梯段 (|pt.z - fallback_z| <= 0.60m)
                        if abs(pt[2] - fallback_z) <= 0.60:
                            min_path_sq = sq
                            path_z = pt[2]
                if path_z is not None:
                    # 在该 3D 路径点高度附近寻找最匹配的真实踏面节点 (容许阶跃贴合)
                    sub_sq = 0.35 * 0.35
                    node_z = None
                    for p in local_nodes:
                        if p[3] >= 0.95:
                            continue
                        if self.dynamic_trav.get((p[0], p[1], p[2]), 0.0) >= 0.95:
                            continue  # 融合动态封锁
                        if abs(p[2] - path_z) <= self.max_step_height:
                            dx = p[0] - x
                            dy = p[1] - y
                            sq = dx * dx + dy * dy
                            if sq < sub_sq:
                                sub_sq = sq
                                node_z = p[2]
                    return node_z if node_z is not None else path_z

        # 第三带 (放宽但严禁穿透楼板): 仅限小幅单步容限 [-0.35m, +0.40m]
        min_sq = 0.35 * 0.35
        for p in local_nodes:
            if p[3] >= 0.95:
                continue
            if self.dynamic_trav.get((p[0], p[1], p[2]), 0.0) >= 0.95:
                continue  # 融合动态封锁 (障碍占用)
            # 严格限制搜索上下界: 向上最多 0.40m, 向下最多 0.35m (绝不可击穿 1.25m 的上下层楼板)
            if p[2] > fallback_z + 0.40 or p[2] < fallback_z - 0.35:
                continue
            dx = p[0] - x
            dy = p[1] - y
            sq = dx * dx + dy * dy
            if sq < min_sq:
                min_sq = sq
                best_z = p[2]

        if best_z is not None:
            return best_z

        return fallback_z

    def get_clustered_footprint_z(self, x: float, y: float, fallback_z: float, footprint_radius: float = 0.25) -> float:
        """基于机器人足印包络下方大部分方块的高度聚类，自适应决定机体当前权威高度：
        1. 在以 (x, y) 为中心、footprint_radius 半径内收集所有有效踏面节点；
        2. 排除障碍禁行 (zone==3 或 trav>=0.95)；
        3. 按 5cm 高度桶进行加权聚类 (距离质心越近权重越高)；
        4. 选出权重最大 (即脚下大部分方块所处) 的主导高度簇，杜绝因台阶交界滞后导致的高度误判。
        """
        r0 = int(round(x / 0.10))
        c0 = int(round(y / 0.10))
        r_range = int(math.ceil(footprint_radius / 0.10)) + 1
        r_sq = footprint_radius * footprint_radius

        candidates = []
        with self._lock:
            spatial = self.spatial_nodes

        if not spatial:
            return fallback_z

        for dr in range(-r_range, r_range + 1):
            for dc in range(-r_range, r_range + 1):
                cell = spatial.get((r0 + dr, c0 + dc))
                if not cell:
                    continue
                for p in cell:
                    # 排除障碍节点
                    if p[3] >= 0.95 or self.dynamic_trav.get((p[0], p[1], p[2]), 0.0) >= 0.95:
                        continue
                    key = (p[0], p[1], p[2])
                    if self.dynamic_zone.get(key, int(p[4]) if len(p) >= 5 else 0) >= 3:
                        continue

                    # 仅在阶梯单步垂直范围内聚类 (|dz| <= max_step_height + 0.10m)
                    dz = p[2] - fallback_z
                    if abs(dz) > self.max_step_height + 0.10:
                        continue

                    dx = p[0] - x
                    dy = p[1] - y
                    d2 = dx * dx + dy * dy
                    if d2 <= r_sq:
                        # 距离越近权重越高
                        weight = 1.0 / (1.0 + 8.0 * d2)
                        candidates.append((p[2], weight))

        if not candidates:
            return fallback_z

        # 5cm 高度窗口聚类
        clusters: Dict[int, List[float]] = {}  # bin_idx -> [total_weight, sum_weighted_z]
        for pz, w in candidates:
            bin_idx = int(round(pz / 0.05))
            if bin_idx not in clusters:
                clusters[bin_idx] = [0.0, 0.0]
            clusters[bin_idx][0] += w
            clusters[bin_idx][1] += w * pz

        # 选出权重最高 (即大部分方块所在) 的主导高度簇
        best_bin = max(clusters.keys(), key=lambda b: clusters[b][0])
        dominant_weight, dominant_sum = clusters[best_bin]
        if dominant_weight > 1e-4:
            return dominant_sum / dominant_weight

        return fallback_z

    def _has_foothold(self, x: float, y: float, z: float) -> Tuple[bool, str, Optional[List[float]]]:
        """平移闸落脚点预检:
        1. 真实物理障碍 (zone=3 FORBIDDEN / 顶盖侵入): 必须在 body_hard_radius (0.17m) 外;
        2. 机体硬禁行环 (zone=2 BODY_HARD): 是已膨胀 0.17m 的安全区, 机器狗质心不直接踩入 (中心落脚容差 ~0.08m) 即可;
        3. 必须在 0.17m 邻域内存在有效支撑踏面 (|dz| <= max_step_height)。
        返回: (是否放行, 原因描述, 冲突节点[x, y, z, zone]或None)
        """
        body_radius = self.body_hard_radius
        center_foot_tol = 0.08  # 质心落脚网格容差
        with self._lock:
            spatial = self.spatial_nodes
        if not spatial:
            return True, "map_not_ready", None
        r0 = int(round(x / 0.10))
        c0 = int(round(y / 0.10))
        r_body_sq = body_radius * body_radius
        r_center_sq = center_foot_tol * center_foot_tol
        has_support = False
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                cell = spatial.get((r0 + dr, c0 + dc))
                if not cell:
                    continue

                # 1. 顶盖/立面结构空间碰撞 (在 body_hard_radius 0.17m 范围内检查)
                #    仅当处于机身高度区间的节点属于障碍/禁行实体时拦截；正常可通行的上行台阶踏面不构成顶盖
                for p in cell:
                    dx = p[0] - x
                    dy = p[1] - y
                    dist_sq = dx * dx + dy * dy
                    if dist_sq > r_body_sq:
                        continue
                    if z + self.max_step_height < p[2] <= z + self.dog_height + 0.15:
                        key = (p[0], p[1], p[2])
                        effective_zone = self.dynamic_zone.get(key, int(p[4]) if len(p) >= 5 else (2 if p[3] >= 0.95 else 0))
                        if effective_zone >= 2 or p[3] >= 0.8:
                            return False, (f"顶盖碰撞: 邻域障碍节点({p[0]:.2f}, {p[1]:.2f}, z={p[2]:.2f}) "
                                           f"高度差 dz={p[2]-z:.2f} 侵入机身带[+{self.max_step_height:.2f}, +{self.dog_height+0.15:.2f}]m"), [p[0], p[1], p[2], 4]

                # 2. 按水平坐标 (x,y) 归集本层踏面候选 (|Δz| <= max_step_height)
                #    如果同一个 (x,y) 存在多个 0.25m 以内的踏面层 (如楼梯踏面与楼梯下地面)，优先选择可通行的一层
                col_layers: Dict[Tuple[float, float], List[Tuple[Any, float]]] = {}
                for p in cell:
                    dx = p[0] - x
                    dy = p[1] - y
                    dist_sq = dx * dx + dy * dy
                    if dist_sq > r_body_sq:
                        continue
                    if abs(p[2] - z) > self.max_step_height:
                        continue

                    col_key = (round(p[0], 2), round(p[1], 2))
                    if col_key not in col_layers:
                        col_layers[col_key] = []
                    col_layers[col_key].append((p, dist_sq))

                for col_key, cand_list in col_layers.items():
                    # 评估该水平位置所有候选层的有效 zone (0=FREE, 1=SOFT, 2=BODY_HARD, 3=FORBIDDEN)
                    # 只要存在可通行的层 (zone 最小)，直接选定该可通行层
                    best_cand = None
                    best_zone = 999
                    best_dist_sq = 0.0

                    for p, dist_sq in cand_list:
                        key = (p[0], p[1], p[2])
                        effective_zone = self.dynamic_zone.get(key, int(p[4]) if len(p) >= 5 else (2 if p[3] >= 0.95 else 0))
                        if effective_zone < best_zone:
                            best_zone = effective_zone
                            best_cand = p
                            best_dist_sq = dist_sq
                        elif effective_zone == best_zone:
                            if best_cand is None or abs(p[2] - z) < abs(best_cand[2] - z):
                                best_cand = p
                                best_dist_sq = dist_sq

                    if best_cand is None:
                        continue

                    p = best_cand
                    dist_sq = best_dist_sq
                    effective_zone = best_zone

                    # 真实物理障碍 (该 (x,y) 下所有候选层均为 zone == 3，无任何可通行踏面)
                    if effective_zone == 3:
                        return False, (f"触碰真实物理障碍(FORBIDDEN): 节点({p[0]:.2f}, {p[1]:.2f}, z={p[2]:.2f}) "
                                       f"距中心 {math.sqrt(dist_sq):.2f}m <= {body_radius:.2f}m"), [p[0], p[1], p[2], 3]

                    # 机体硬禁行环 (zone == 2 BODY_HARD)
                    if effective_zone == 2 and dist_sq <= r_center_sq:
                        return False, (f"质心踩入机体禁行环(BODY_HARD): 节点({p[0]:.2f}, {p[1]:.2f}, z={p[2]:.2f}) "
                                       f"距中心 {math.sqrt(dist_sq):.2f}m <= 质心容差 {center_foot_tol:.2f}m"), [p[0], p[1], p[2], 2]

                    # 只要选出的一层不是真实物理障碍，即构成本层有效支撑踏面
                    if effective_zone < 3:
                        has_support = True

        if not has_support:
            return False, f"无有效支撑踏面: 半径 {body_radius:.2f}m 内未检索到 |Δz|<={self.max_step_height:.2f}m 的合法支撑节点", None
        return True, "ok", None

    def step_connected_terrain(self, target_x: float, target_y: float, fallback_z: float) -> float:
        """严格沿连通图拓扑邻域推进并吸附到当前踏面流形：
        1. 从当前流形锚点 self.current_graph_node 沿邻接边展开 1 跳与 2 跳可达邻居集合
        2. 仅在该拓扑连通闭包内寻找最逼近 (target_x, target_y) 的候选节点，更新为新锚点
        3. 沿新锚点的连通边进行连续线段投影插值，计算精确地表高度，从数学拓扑上 100% 杜绝多层楼板穿透与下坠
        """
        with self._lock:
            adj = self.graph_adj
            spatial = self.spatial_nodes
            curr = self.current_graph_node

        if not adj and not spatial:
            # 流形图未就绪: 无可跟随地形, 原地保持
            return fallback_z

        # 尚未锚定 / 脱锚 (>1.5m) / 锚点本身已禁行 (历史脏锚点) 时重新锚定:
        # 仅允许局部邻域锚定, 失败则原地保持并报错 —— 宁可卡住, 绝不吸附禁行/异层节点
        need_reanchor = (curr is None
                         or math.hypot(curr[0] - target_x, curr[1] - target_y) > 1.5
                         or self._effective_trav(*curr) >= 0.95)
        if need_reanchor:
            with self._lock:
                ok = self._anchor_graph_node_locked(target_x, target_y, fallback_z)
                curr = self.current_graph_node
            if not ok or curr is None:
                rospy.logerr_throttle(
                    1.0,
                    f"[ElevationRosBridge] 锚定失败: ({target_x:.2f}, {target_y:.2f}) 附近 "
                    f"无可通行踏面, 机体原地保持 (z={fallback_z:.2f})")
                return fallback_z

        # 1. 沿连通图邻接边收集 1-hop 与 2-hop 拓扑邻居 (拓扑闭包内严禁包含任何异层或悬崖节点)
        neighbors_1 = list(adj.get(curr, []))

        # 动态互补: 若图边未覆盖当前区域 (如 Marker 截断), 从全量 19 万踏面网格 8 邻域补充物理连通边
        if not neighbors_1 and spatial:
            r0 = int(round(curr[0] / 0.10))
            c0 = int(round(curr[1] / 0.10))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0: continue
                    for p in spatial.get((r0 + dr, c0 + dc), []):
                        if p[3] < 0.8 and abs(p[2] - curr[2]) <= self.max_step_height:
                            neighbors_1.append((round(p[0], 3), round(p[1], 3), round(p[2], 3)))

        # 多候选收集: 1 跳 + 2 跳拓扑闭包 (邻接边; 边未覆盖时 8 邻域物理连通补充)
        cand_set = set()
        for n1 in neighbors_1:
            cand_set.add((n1[0], n1[1], n1[2]))
            n2_list = adj.get(n1, [])
            if not n2_list and spatial:
                nr0 = int(round(n1[0] / 0.10))
                nc0 = int(round(n1[1] / 0.10))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0: continue
                        for p in spatial.get((nr0 + dr, nc0 + dc), []):
                            if p[3] < 0.8 and abs(p[2] - n1[2]) <= self.max_step_height:
                                cand_set.add((round(p[0], 3), round(p[1], 3), round(p[2], 3)))
            else:
                for n2 in n2_list:
                    cand_set.add((n2[0], n2[1], n2[2]))

        # 吸附择优 (多候选): 可通行过滤 (含融合动态封锁) → 单步限幅 → 分数排序
        # 分数 = XY 距目标 + 4×|Δz| (同层优先, 折返楼梯不换梯段) + 2×通行性 (贴障碍降权)
        best_cand = None
        best_score = None
        for cand in cand_set:
            trav = self._effective_trav(*cand)
            if trav >= 0.95:
                continue                       # 不可通行 (静态禁行或融合动态封锁)
            dz = cand[2] - curr[2]
            if abs(dz) > self.max_step_height:
                continue                       # 单步限幅: 每 tick 至多一级台阶
            d_xy = math.hypot(cand[0] - target_x, cand[1] - target_y)
            score = d_xy + 2.0 * trav
            if best_score is None or score < best_score:
                best_score = score
                best_cand = cand

        if best_cand is None:
            # 无可通行候选: 保持当前锚点与踏面 (绝不吸附到不可行走处)
            return curr[2]

        # 更新当前拓扑锚点
        with self._lock:
            self.current_graph_node = best_cand

        # 2. 沿新锚点的连通邻接边进行连续表面线性插值，消除阶梯离散抖动;
        #    插值结果相对旧锚点限幅在单步之内 (杜绝一帧跨两级/换梯段的瞬移)
        best_edge_z = best_cand[2]
        best_edge_dist2 = (best_cand[0] - target_x)**2 + (best_cand[1] - target_y)**2

        nbrs = list(adj.get(best_cand, []))
        if not nbrs and spatial:
            br0 = int(round(best_cand[0] / 0.10))
            bc0 = int(round(best_cand[1] / 0.10))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0: continue
                    for p in spatial.get((br0 + dr, bc0 + dc), []):
                        if p[3] < 0.8 and abs(p[2] - best_cand[2]) <= self.max_step_height:
                            nbrs.append((round(p[0], 3), round(p[1], 3), round(p[2], 3)))

        for nbr in nbrs:
            dx = nbr[0] - best_cand[0]
            dy = nbr[1] - best_cand[1]
            l2 = dx * dx + dy * dy
            if l2 > 1e-4:
                t = max(0.0, min(1.0, ((target_x - best_cand[0]) * dx + (target_y - best_cand[1]) * dy) / l2))
                proj_x = best_cand[0] + t * dx
                proj_y = best_cand[1] + t * dy
                dist2 = (target_x - proj_x)**2 + (target_y - proj_y)**2
                if dist2 < best_edge_dist2:
                    best_edge_dist2 = dist2
                    best_edge_z = best_cand[2] + t * (nbr[2] - best_cand[2])

        # 单步限幅: 插值 z 相对旧锚点至多一级台阶 (连跳两级/换梯段的瞬移在此被截断)
        best_edge_z = curr[2] + max(-self.max_step_height,
                                    min(self.max_step_height, best_edge_z - curr[2]))
        return best_edge_z

    def set_sim_pose(self, x: float, y: float, z: float, yaw: Optional[float] = None):
        """前端修改初始位置时, 立即锚定连通图节点并更新 TF; 不再重新盲目贴地探测"""
        self.sim_x = float(x)
        self.sim_y = float(y)
        if yaw is not None:
            self.sim_yaw = float(yaw)
        with self._lock:
            self._anchor_graph_node_locked(self.sim_x, self.sim_y, float(z) if z is not None else self.sim_z)
            if self.current_graph_node is not None:
                self.sim_z = float(z) if z is not None else self.current_graph_node[2]
            else:
                self.sim_z = float(z) if z is not None else self.get_terrain_z(self.sim_x, self.sim_y, 0.0)
            self.collision_node = None
            self.collision_points = []
            self.collision_node_time = 0.0
        self.cmd_vx, self.cmd_vy, self.cmd_wz = 0.0, 0.0, 0.0
        self.last_sim_time = rospy.Time.now()
        if self.publish_fake_tf:
            self.publish_fake_tf_loop()

    def _fake_tf_thread_worker(self):
        """100Hz 独立高频广播线程，彻底规避 Python 定时器事件队列阻塞造成的 TF 延迟"""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_fake_tf_loop()
            try:
                rate.sleep()
            except Exception:
                break

    def publish_fake_tf_loop(self, event=None):
        if not self.publish_fake_tf or self.tf_broadcaster is None:
            return
        try:
            now_stamp = rospy.Time.now()
            now_sec = now_stamp.to_sec()

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

                    new_x = self.sim_x + dx_body * cos_y - dy_body * sin_y
                    new_y = self.sim_y + dx_body * sin_y + dy_body * cos_y

                    is_translating = (abs(dx_body) > 1e-9 or abs(dy_body) > 1e-9)

                    # 1. 严格沿拓扑连通图流形预测目标点 (new_x, new_y) 的真实地表高度 (连续线段插值, 无阶跃)
                    expected_z = self.step_connected_terrain(new_x, new_y, self.sim_z) if is_translating else self.sim_z

                    # 2. 落脚点预检: 基于前方 (new_x, new_y) 真实的踏面高度 expected_z 进行 3D 包络与障碍判定
                    foothold_ok, foothold_reason, hit_node = self._has_foothold(new_x, new_y, expected_z) if is_translating else (True, "ok", None)
                    with self._lock:
                        if is_translating and not foothold_ok and hit_node:
                            # 发生碰撞时记录/覆盖冲突节点，持续保留直至下一次碰撞覆盖
                            self.collision_node = hit_node

                    if is_translating and not foothold_ok:
                        rospy.logerr_throttle(
                            1.0,
                            f"[ElevationRosBridge] 落脚点预检失败 (平移锁止于 x={self.sim_x:.2f}, y={self.sim_y:.2f}, z={self.sim_z:.2f}): "
                            f"前方 ({new_x:.2f}, {new_y:.2f}, z={expected_z:.2f}) -> {foothold_reason}")
                    else:
                        self.sim_x = new_x
                        self.sim_y = new_y
                        self.sim_z = expected_z
                    self.sim_yaw = normalize_angle(self.sim_yaw + dyaw)

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
        """发布导航目标点 (记录 last_goal 供编辑障碍变更后重发触发全局+局部完整重规划)"""
        if not self._goal_pub:
            return
        self.last_goal = (x, y, z, yaw, frame_id)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)
        self._goal_pub.publish(goal)

    def cancel_navigation(self):
        """取消当前导航目标并刹停机器人 (保留当前前端场景与路径)"""
        if self._cancel_pub:
            self._cancel_pub.publish(GoalID())
            rospy.loginfo("[ElevationRosBridge] Published GoalID() to /move_base/cancel")
        if self._cmd_vel_pub:
            self._cmd_vel_pub.publish(Twist())

    def publish_pcd_cmd(self, pcd_path: str):
        """向 C++ 节点发送 PCD 加载指令"""
        if self._pcd_cmd_pub:
            self._pcd_cmd_pub.publish(RosString(data=pcd_path))
            rospy.loginfo(f"[ElevationRosBridge] Sent PCD path command: {pcd_path}")

    def _debug_result_callback(self, msg: RosString):
        """接收 C++ 规划器节点返回的权威诊断结果 (兼容 Topic 降级)"""
        with self._lock:
            self._last_debug_result = msg.data
            self._debug_event.set()

    def _call_diagnose_service(self, query_str: str, timeout: float = 1.0) -> Dict[str, Any]:
        """通过 ROS Service 同步请求 C++ 权威诊断 (天然线程安全, 彻底杜绝并发串包)"""
        try:
            rospy.wait_for_service("/elevation_debug_diagnose", timeout=timeout)
            proxy = rospy.ServiceProxy("/elevation_debug_diagnose", DiagnoseQuery)
            resp = proxy(query_str)
            return json.loads(resp.result)
        except rospy.ROSException:
            # 容错降级: 若 service 尚未就绪或不可用，尝试通过旧 Topic 机制
            if self._debug_query_pub:
                self._debug_event.clear()
                self._debug_query_pub.publish(RosString(data=query_str))
                return self._wait_debug_result(timeout)
            return {"status": "error", "message": "Diagnose service unavailable"}
        except Exception as e:
            return {"status": "error", "message": f"Diagnose call failed: {e}"}

    def diagnose_edge(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float, timeout: float = 1.0) -> Dict[str, Any]:
        """向 C++ 节点请求两踏面方块的权威拓扑邻边关系与物理原因诊断"""
        query_str = json.dumps({"x1": x1, "y1": y1, "z1": z1, "x2": x2, "y2": y2, "z2": z2})
        return self._call_diagnose_service(query_str, timeout)

    def diagnose_node(self, x: float, y: float, z: float, timeout: float = 1.0) -> Dict[str, Any]:
        """向 C++ 节点请求单个踏面方块的权威通行状态与禁行原因诊断"""
        query_str = json.dumps({"mode": "node", "x1": x, "y1": y, "z1": z})
        return self._call_diagnose_service(query_str, timeout)

    def _wait_debug_result(self, timeout: float) -> Dict[str, Any]:
        """等待 C++ 节点在 /elevation_debug_result 上的权威诊断回包 (Topic 降级备用)"""
        if self._debug_event.wait(timeout=timeout):
            with self._lock:
                try:
                    return json.loads(self._last_debug_result)
                except Exception as e:
                    return {"status": "error", "message": f"Failed to parse C++ result: {e}"}
        return {"status": "timeout", "message": "C++ planner node did not respond in time"}

    def get_live_state(self) -> Dict[str, Any]:
        """获取当前高频轻量实时状态快照 (静态大地图通过 get_static_graph 按需拉取)"""
        with self._lock:
            return {
                "robot_pose": dict(self.robot_pose),
                "global_path": list(self.global_path),
                "local_path": list(self.local_path),
                "collision_node": list(self.collision_node) if self.collision_node else None,
                "collision_points": list(self.collision_points) if self.collision_points else [],
                "path_version": self.path_version,
                "graph_nodes_version": self.graph_nodes_version,
                "graph_edges_version": self.graph_edges_version,
                "sfc_corridors_debug": list(self.sfc_corridors_debug),
                "sfc_corridors_debug_version": self.sfc_corridors_debug_version,
                "injected_obstacles": list(self.injected_obstacles) + [
                    {"id": 10000 + i, "type": 3, "x": ox, "y": oy, "z": oz + self.OBSTACLE_HEIGHT / 2,
                     "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
                     "sx": self.OBSTACLE_RADIUS * 2, "sy": self.OBSTACLE_RADIUS * 2, "sz": self.OBSTACLE_HEIGHT}
                    for i, (ox, oy, oz) in enumerate(self.editor_obstacles)],
                "injected_obstacles_version": self.injected_obstacles_version,
                "dynamic_nodes": list(self.dynamic_nodes),
                "dynamic_nodes_version": self.dynamic_nodes_version,
                "rebound_arrows": list(self.rebound_arrows),
                "rebound_arrows_version": self.rebound_arrows_version
            }

    def get_static_graph(self) -> Dict[str, Any]:
        """获取全量静态 3D 流形拓扑图 (仅供 HTTP 接口按需拉取，避免高频锁竞争)"""
        with self._lock:
            return {
                "graph_nodes": list(self.graph_nodes),
                "graph_nodes_version": self.graph_nodes_version,
                "graph_edges": list(self.graph_edges),
                "graph_edges_version": self.graph_edges_version
            }


ros_bridge = ElevationRosBridge()
