# -*- coding: utf-8 -*-
"""
ROS 状态通信桥接模块 (ROS Bridge)
负责与 ROS 导航系统通信：发布 GridMap 消息、订阅机器人里程计/位姿、订阅路径、发布目标点
"""

import json
import math
import threading
from typing import Dict, Any, Optional, List
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Path as ROSPath, Odometry
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String as RosString
from grid_map_msgs.msg import GridMap, GridMapInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
import numpy as np

def euler_to_quaternion(yaw: float) -> Quaternion:
    """航向角 Yaw 转 ROS 四元数"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


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

        # ROS 话题发布者与订阅者
        self._grid_map_pub: Optional[rospy.Publisher] = None
        self._goal_pub: Optional[rospy.Publisher] = None
        self._cancel_pub: Optional[rospy.Publisher] = None
        self._pcd_cmd_pub: Optional[rospy.Publisher] = None
        self._debug_query_pub: Optional[rospy.Publisher] = None
        self._debug_event = threading.Event()
        self._last_debug_result = ""

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

            # 订阅机器人位姿（支持 /loc_base 或 /odom）
            rospy.Subscriber("/loc_base", Odometry, self._odom_callback, queue_size=5)
            rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=5)

            # 订阅全局规划路径与局部规划路径 (优先使用 3D 流形全局路径)
            rospy.Subscriber("/elevation_global_plan", ROSPath, self._global_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/plan", ROSPath, self._global_path_callback, queue_size=2)
            rospy.Subscriber("/move_base/local_plan", ROSPath, self._local_path_callback, queue_size=2)

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
        pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        with self._lock:
            self.local_path = pts

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
        """发布起始位姿并直接更新本地机器人模型"""
        if self._initial_pose_pub:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = frame_id
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            pose.pose.pose.position.z = z
            pose.pose.pose.orientation = euler_to_quaternion(yaw)
            pose.pose.covariance[0] = 0.25
            pose.pose.covariance[7] = 0.25
            pose.pose.covariance[35] = 0.068
            self._initial_pose_pub.publish(pose)

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
        goal.pose.orientation = euler_to_quaternion(yaw)
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
                "graph_edges_version": self.graph_edges_version
            }


ros_bridge = ElevationRosBridge()
