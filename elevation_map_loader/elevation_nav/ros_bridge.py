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
from nav_msgs.msg import Path as ROSPath, Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String as RosString, Int32MultiArray
from grid_map_msgs.msg import GridMap, GridMapInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
try:
    from costmap_converter.msg import ObstacleArrayMsg
    HAS_COSTMAP_CONVERTER = True
except ImportError:
    HAS_COSTMAP_CONVERTER = False
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

        self.graph_nodes: List[List[float]] = []
        self.graph_nodes_version = 0
        self.spatial_nodes: Dict[Tuple[int, int], List[List[float]]] = {}
        self.graph_edges: List[List[float]] = []
        self.graph_edges_version = 0
        self.graph_adj: Dict[Tuple[float, float, float], List[Tuple[float, float, float]]] = {}
        self.current_graph_node: Optional[Tuple[float, float, float]] = None
        # 局部代价地图缓存
        self.local_costmap: Optional[Dict[str, Any]] = None
        self.local_costmap_version = 0
        # 局部代价地图逐格成因码 (调试图层)
        self.local_costmap_debug: Optional[Dict[str, Any]] = None
        self.local_costmap_debug_version = 0
        # 局部代价地图逐格胜出节点 id (调试图层)
        self.local_costmap_debug_nodes: Optional[Dict[str, Any]] = None
        self.local_costmap_debug_nodes_version = 0
        # 供给 TEB 局部规划器的结构化几何障碍物 (替代稠密 costmap)
        self.teb_obstacles: List[Dict[str, Any]] = []
        self.teb_obstacles_version = 0

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

            # 订阅供给 TEB 局部规划器的结构化几何障碍物 (替代原 2D 稠密 costmap)
            if HAS_COSTMAP_CONVERTER:
                rospy.Subscriber("/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, self._teb_obstacles_callback, queue_size=2)

            # 订阅流形图节点与边可视化数据
            rospy.Subscriber("/elevation_graph_nodes", PointCloud2, self._graph_nodes_callback, queue_size=1)
            rospy.Subscriber("/elevation_graph_edges", MarkerArray, self._graph_edges_callback, queue_size=1)
            rospy.Subscriber("/elevation_debug_result", RosString, self._debug_result_callback, queue_size=5)

            self.is_initialized = True
            rospy.loginfo("[ElevationRosBridge] ROS node and topic subscriptions ready")
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] ROS init exception: {e}")

    def _graph_nodes_callback(self, msg: PointCloud2):
        """解析流形踏面节点点云并建立空间栅格哈希"""
        try:
            pts = []
            spatial = {}
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

    def _anchor_graph_node_locked(self, x: float, y: float, z: float):
        """在连通图中寻找距 (x, y, z) 最近的流形锚点 (3D 加权距离，强化 z 权重避免跨层)"""
        # 优先在局部空间栅格邻域内检索候选点，避免全图 19 万节点无界遍历
        r0 = int(round(x / 0.10))
        c0 = int(round(y / 0.10))
        candidates = []
        spatial = self.spatial_nodes
        if spatial:
            for dr in range(-6, 7):
                for dc in range(-6, 7):
                    cell = spatial.get((r0 + dr, c0 + dc))
                    if cell:
                        for p in cell:
                            if abs(p[2] - z) <= 0.80 and p[3] < 0.8:
                                candidates.append((round(p[0], 3), round(p[1], 3), round(p[2], 3)))
        if not candidates and self.graph_adj:
            candidates = list(self.graph_adj.keys())

        best_node = None
        best_d2 = float("inf")
        for node in candidates:
            d2 = (node[0] - x)**2 + (node[1] - y)**2 + 4.0 * (node[2] - z)**2
            if d2 < best_d2:
                best_d2 = d2
                best_node = node
        if best_node is not None:
            self.current_graph_node = best_node

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

    def _teb_obstacles_callback(self, msg: ObstacleArrayMsg):
        """解析供给 TEB 的结构化几何障碍物 (替代稠密 costmap 像素点)"""
        try:
            obstacles_data = []
            for idx, obs in enumerate(msg.obstacles):
                pts = [[round(float(p.x), 3), round(float(p.y), 3), round(float(p.z), 3)] for p in obs.polygon.points]
                radius = round(float(obs.radius), 3)
                obs_id = int(obs.id) if obs.id > 0 else (idx + 1)

                if radius > 0 and len(pts) >= 1:
                    # 圆形障碍物 (柱体/聚类实体)
                    obstacles_data.append({
                        "id": obs_id,
                        "type": "circle",
                        "x": pts[0][0],
                        "y": pts[0][1],
                        "z": pts[0][2],
                        "radius": radius,
                        "source": "3D 空间正障碍聚类 (Physical Obstacle)",
                        "reason": f"在机器狗垂直净空高度带内检测到激光点云实体（如立柱/障碍物），经欧氏聚类拟合为半径 {radius}m 的圆柱避障原语。",
                        "teb_effect": "作为 CircularObstacle 激活 TEB 柯西积分同伦类规划 (HCP)，使机器狗能够从左侧或右侧探索多条拓扑等价路径并选出最优解。"
                    })
                elif radius == 0 and len(pts) == 2:
                    # 线段障碍物 (楼梯断坎/无边界面边缘)
                    dx = pts[1][0] - pts[0][0]
                    dy = pts[1][1] - pts[0][1]
                    length = round(math.hypot(dx, dy), 3)
                    obstacles_data.append({
                        "id": obs_id,
                        "type": "line",
                        "start": pts[0],
                        "end": pts[1],
                        "length": length,
                        "source": "踏面断坎 / 悬空边缘 (Drop-off Boundary)",
                        "reason": f"该线段外侧相邻网格无连通踏面（断坎或台阶高差跌落）。流形提取器沿台沿建立长 {length}m 的 3D 护栏，防止机体踏空跌落。",
                        "teb_effect": "作为 LineObstacle 注入 TEB 优化图，利用解析线段投影距离施加斥力梯度惩罚，严格禁止局部轨迹穿越台沿边缘。"
                    })
                elif radius == 0 and len(pts) > 2:
                    # 多边形障碍物 (大墙体凸包)
                    obstacles_data.append({
                        "id": obs_id,
                        "type": "polygon",
                        "points": pts,
                        "source": "大型凸多边形墙体 (Convex Wall)",
                        "reason": f"检测到由 {len(pts)} 个顶点围成的连续障碍实体，凸包算法拟合为刚性阻挡区域。",
                        "teb_effect": "作为 PolygonObstacle 施加全足印多边形分离轴间距约束，引导机器狗在墙体外侧平滑绕行。"
                    })
                elif len(pts) == 1:
                    obstacles_data.append({
                        "id": obs_id,
                        "type": "circle",
                        "x": pts[0][0],
                        "y": pts[0][1],
                        "z": pts[0][2],
                        "radius": radius if radius > 0 else 0.15,
                        "source": "3D 空间单点障碍 (Point Obstacle)",
                        "reason": "孤立小体积点云阻挡，拟合为紧凑圆形障碍物。",
                        "teb_effect": "作为点/小圆避障体计算局部欧氏斥力势场。"
                    })

            with self._lock:
                self.teb_obstacles = obstacles_data
                self.teb_obstacles_version += 1
        except Exception as e:
            rospy.logwarn(f"[ElevationRosBridge] 解析 teb obstacles 异常: {e}")

    # ------------------ 仿真运动学积分与 TF/Odom 高频广播 (移植自 jie_octomap) ------------------
    def _cmd_vel_callback(self, msg: Twist):
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
            return self.get_terrain_z(target_x, target_y, fallback_z)

        # 若尚未锚定，或发生外部瞬移 (> 1.5m 且与当前锚点脱节)，重新锚定
        if curr is None or math.hypot(curr[0] - target_x, curr[1] - target_y) > 1.5:
            with self._lock:
                self._anchor_graph_node_locked(target_x, target_y, fallback_z)
                curr = self.current_graph_node
            if curr is None:
                return self.get_terrain_z(target_x, target_y, fallback_z)

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

        best_cand = curr
        best_cand_d2 = (curr[0] - target_x)**2 + (curr[1] - target_y)**2

        for n1 in neighbors_1:
            d1 = (n1[0] - target_x)**2 + (n1[1] - target_y)**2
            if d1 < best_cand_d2:
                best_cand_d2 = d1
                best_cand = n1
            n2_list = adj.get(n1, [])
            if not n2_list and spatial:
                nr0 = int(round(n1[0] / 0.10))
                nc0 = int(round(n1[1] / 0.10))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0: continue
                        for p in spatial.get((nr0 + dr, nc0 + dc), []):
                            if p[3] < 0.8 and abs(p[2] - n1[2]) <= self.max_step_height:
                                d2 = (p[0] - target_x)**2 + (p[1] - target_y)**2
                                if d2 < best_cand_d2:
                                    best_cand_d2 = d2
                                    best_cand = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
            else:
                for n2 in n2_list:
                    d2 = (n2[0] - target_x)**2 + (n2[1] - target_y)**2
                    if d2 < best_cand_d2:
                        best_cand_d2 = d2
                        best_cand = n2

        # 更新当前拓扑锚点
        with self._lock:
            self.current_graph_node = best_cand

        # 2. 沿当前节点与其连通邻接边进行连续表面线性插值，消除阶梯离散抖动
        best_edge_z = best_cand[2]
        best_edge_dist2 = best_cand_d2

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

                    # 严格沿拓扑连通图流形跟随地表高度 (平滑低通滤波过渡)
                    target_z = self.step_connected_terrain(self.sim_x, self.sim_y, self.sim_z)
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
                "local_costmap_debug_nodes_version": self.local_costmap_debug_nodes_version,
                "teb_obstacles": list(self.teb_obstacles) if self.teb_obstacles else [],
                "teb_obstacles_version": self.teb_obstacles_version
            }


ros_bridge = ElevationRosBridge()
