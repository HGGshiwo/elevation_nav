#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立四足机器人点云物理仿真器 (Simulator Node)
基于点云 KD-Tree 空间索引与刚体足印接触力学模型：
1. 输入：
   - 静态环境点云 (PCD 文件 / /elevation_graph_nodes)
   - 动态障碍物点云 (/lidar_points)
   - 运动控制速度 (/cmd_vel)
   - 起始位姿设定 (/initialpose)
2. 物理与越障机制：
   - 前方台阶/凸起自适应挤压抬升 (0 < dz <= max_step_height: 自动上挤)
   - 正常下楼梯与下坡顺畅贴地 (-max_step_height <= dz <= 0)
   - 超高立面障碍硬截断 (dz > max_step_height: 锁止报错并标记碰撞点)
   - 断崖悬崖与悬空跌落保护 (dz < -max_step_height 或下方悬空: 锁止报错)
   - 顶盖空间净空检验 (机身空间侵入: 锁止报错)
3. 输出：
   - TF 变换树: map -> odom -> base_link
   - 机器人位姿与里程计: /loc_base, /odom
   - 碰撞可视化与事件: /elevation_collision_node, /elevation_collision_marker
"""

import os
import sys
import math
import time
import json
import random
import threading
from pathlib import Path
from typing import Optional, Tuple, List, Dict, Any

import numpy as np
from scipy.spatial import cKDTree

import rospy
import tf2_ros
from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    PointStamped,
    Point,
    Quaternion,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from std_msgs.msg import String as RosString, Header

# 引入 PCDLoader
package_root = Path(__file__).resolve().parent.parent
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from elevation_sim.pcd_loader import PCDLoader


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
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


class QuadrupedPointCloudSimulator:
    def __init__(self):
        rospy.init_node("quadruped_simulator", anonymous=False)

        # 参数配置
        self.pcd_file = rospy.get_param("~pcd_file", "/home/hggshiwo/catkin_ws/src/pcd/map.pcd")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.cmd_vel_freeze = bool(rospy.get_param("~cmd_vel_freeze", False))

        # 物理几何参数
        self.footprint_radius = float(rospy.get_param("~footprint_radius", 0.22))
        self.body_hard_radius = float(rospy.get_param("~body_hard_radius", 0.17))
        self.max_step_height = float(rospy.get_param("~max_step_height", 0.25))
        self.dog_height = float(rospy.get_param("~dog_height", 0.45))

        # 运动学噪声参数
        self.linear_noise_std = float(rospy.get_param("~linear_noise_std", 0.008))
        self.lateral_noise_std = float(rospy.get_param("~lateral_noise_std", 0.008))
        self.angular_noise_std = float(rospy.get_param("~angular_noise_std", 0.006))

        # 初始位姿
        self.sim_x = float(rospy.get_param("~init_x", 0.0))
        self.sim_y = float(rospy.get_param("~init_y", 0.0))
        self.sim_z = float(rospy.get_param("~init_z", 0.0))
        self.sim_yaw = float(rospy.get_param("~init_yaw", 0.0))

        # 运行时控制状态
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = None
        self.last_sim_time = rospy.Time.now()

        # 碰撞记录状态
        self.collision_node: Optional[List[float]] = None
        self.last_collision_str = ""

        # 点云与 KD-Tree 数据结构
        self._lock = threading.Lock()
        self.static_pts: Optional[np.ndarray] = None          # Nx3 float32
        self.static_kdtree_2d: Optional[cKDTree] = None       # 基于 (x, y) 的 KD-Tree
        self.dynamic_pts: Optional[np.ndarray] = None         # Mx3 float32
        self.dynamic_kdtree_2d: Optional[cKDTree] = None      # 动态障碍物 KD-Tree

        # 加载初始静态点云
        self._load_static_pcd(self.pcd_file)

        # Snap 初始高度贴地
        self._snap_to_terrain(self.sim_x, self.sim_y, self.sim_z)

        # 发布器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self._odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self._loc_base_pub = rospy.Publisher("/loc_base", Odometry, queue_size=10)
        self._collision_node_pub = rospy.Publisher("/elevation_collision_node", RosString, queue_size=5)
        self._collision_marker_pub = rospy.Publisher("/elevation_collision_marker", Marker, queue_size=5)

        # 订阅器
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback, queue_size=5)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._initial_pose_callback, queue_size=5)
        rospy.Subscriber("/lidar_points", PointCloud2, self._lidar_points_callback, queue_size=2)
        rospy.Subscriber("/elevation_graph_nodes", PointCloud2, self._graph_nodes_callback, queue_size=1)
        rospy.Subscriber("/pcd_file_cmd", RosString, self._pcd_cmd_callback, queue_size=2)

        # 启动 100Hz 高频物理仿真循环线程
        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop_worker, daemon=True)
        self._sim_thread.start()

        rospy.loginfo(f"[Simulator] 四足点云物理仿真器启动完成! 初始化位姿: ({self.sim_x:.2f}, {self.sim_y:.2f}, {self.sim_z:.2f}, yaw={self.sim_yaw:.2f})")

    # ------------------ 点云加载与空间索引构建 ------------------
    def _load_static_pcd(self, pcd_path: str):
        """从 PCD 文件加载点云并构建 2D KD-Tree"""
        try:
            path = Path(pcd_path).expanduser().resolve()
            if not path.is_file():
                rospy.logwarn(f"[Simulator] PCD 文件不存在: {path}")
                return

            pts = PCDLoader.load_full_points(str(path))
            if len(pts) == 0:
                rospy.logwarn(f"[Simulator] PCD 文件点云为空: {path}")
                return

            with self._lock:
                self.static_pts = pts
                self.static_kdtree_2d = cKDTree(pts[:, :2])

            rospy.loginfo(f"[Simulator] 成功加载环境点云 PCD ({len(pts)} 点), 2D KD-Tree 构建完毕: {path.name}")
        except Exception as e:
            rospy.logerr(f"[Simulator] 加载 PCD 失败: {e}")

    def _pcd_cmd_callback(self, msg: RosString):
        """动态重载 PCD 点云"""
        pcd_path = msg.data.strip()
        if pcd_path:
            self._load_static_pcd(pcd_path)
            self._snap_to_terrain(self.sim_x, self.sim_y, self.sim_z)

    def _graph_nodes_callback(self, msg: PointCloud2):
        """当未指定 PCD 文件或图节点更新时，作为环境点云备用源"""
        if self.static_pts is not None and len(self.static_pts) > 0:
            return
        try:
            pts_list = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pts_list.append([p[0], p[1], p[2]])
            if pts_list:
                pts = np.asarray(pts_list, dtype=np.float32)
                with self._lock:
                    self.static_pts = pts
                    self.static_kdtree_2d = cKDTree(pts[:, :2])
                rospy.loginfo(f"[Simulator] 从 /elevation_graph_nodes 摄取 {len(pts)} 个环境表面点构建 KD-Tree")
        except Exception as e:
            rospy.logwarn(f"[Simulator] 解析 graph nodes 异常: {e}")

    def _lidar_points_callback(self, msg: PointCloud2):
        """接收动态障碍物点云并构建动态 KD-Tree"""
        try:
            pts_list = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pts_list.append([p[0], p[1], p[2]])
            with self._lock:
                if pts_list:
                    self.dynamic_pts = np.asarray(pts_list, dtype=np.float32)
                    self.dynamic_kdtree_2d = cKDTree(self.dynamic_pts[:, :2])
                else:
                    self.dynamic_pts = None
                    self.dynamic_kdtree_2d = None
        except Exception as e:
            rospy.logwarn(f"[Simulator] 解析 lidar_points 异常: {e}")

    # ------------------ 状态与控制回调 ------------------
    def _cmd_vel_callback(self, msg: Twist):
        if self.cmd_vel_freeze:
            return
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """响应起点重置指令: 直接采用输入的 (x, y, z, yaw) 更新仿真位姿"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.sim_x = float(pos.x)
        self.sim_y = float(pos.y)
        self.sim_z = float(pos.z)
        self.sim_yaw = float(yaw)
        self.cmd_vx, self.cmd_vy, self.cmd_wz = 0.0, 0.0, 0.0
        self.collision_node = None
        self._publish_collision_event(None)
        self._publish_tf_and_odom(rospy.Time.now())
        rospy.loginfo(f"[Simulator] 成功重置起点位姿 -> ({self.sim_x:.2f}, {self.sim_y:.2f}, {self.sim_z:.2f}, yaw={self.sim_yaw:.2f})")

    def _snap_to_terrain(self, x: float, y: float, fallback_z: float):
        """将位姿吸附到当前 (x, y) 处最合理的地面高度"""
        with self._lock:
            tree = self.static_kdtree_2d
            pts = self.static_pts
        if tree is None or pts is None or len(pts) == 0:
            self.sim_z = fallback_z
            return

        indices = tree.query_ball_point([x, y], r=0.35)
        if not indices:
            self.sim_z = fallback_z
            return

        near_pts = pts[indices]
        # 寻找与 fallback_z 最接近的一组支撑面
        dzs = np.abs(near_pts[:, 2] - fallback_z)
        min_dz_idx = np.argmin(dzs)
        if dzs[min_dz_idx] <= 1.0:
            self.sim_z = float(near_pts[min_dz_idx, 2])
        else:
            self.sim_z = float(np.median(near_pts[:, 2]))

    # ------------------ 核心物理引擎：点云挤压越障与碰撞检测 ------------------
    def evaluate_step_motion(self, new_x: float, new_y: float, curr_z: float) -> Tuple[bool, float, str, Optional[List[float]]]:
        """
        纯点云物理越障判定 (Squeeze-Up Physics):
        1. 收集目标点 (new_x, new_y) 半径 footprint_radius (0.22m) 柱体内的所有点云;
        2. 识别可触及的高度支撑面;
        3. 决策规则:
           - 0.02m < dz <= 0.25m: 【自动往上挤 (上楼/越障)】放行并抬升高度至新表面 z_target;
           - -0.25m <= dz <= 0.02m: 【平地/正常下楼】放行并贴地降落至 z_target;
           - dz > 0.25m: 【超高立面障碍】阻挡锁止，报警并返回碰撞节点;
           - dz < -0.25m 或 无点: 【悬崖/悬空跌落】阻挡锁止，报警并返回碰撞节点;
           - 机身顶盖带 [z_target + 0.25, z_target + dog_height + 0.10] 存在点云: 【顶盖碰撞】阻挡锁止。
        返回: (是否放行, 目标高度 z_target, 原因描述, 冲突节点[x, y, z, zone])
        """
        with self._lock:
            static_tree = self.static_kdtree_2d
            static_pts = self.static_pts
            dyn_tree = self.dynamic_kdtree_2d
            dyn_pts = self.dynamic_pts

        if static_tree is None or static_pts is None or len(static_pts) == 0:
            # 无地图点云时直接放行
            return True, curr_z, "no_pointcloud", None

        # 1. 动态障碍物碰撞硬拦截 (来自 /lidar_points 的圆柱/方块障碍物, 严禁踏踩或攀爬)
        if dyn_tree is not None and dyn_pts is not None and len(dyn_pts) > 0:
            dyn_idx = dyn_tree.query_ball_point([new_x, new_y], r=self.body_hard_radius)
            if dyn_idx:
                dyn_near = dyn_pts[dyn_idx]
                # 检查与当前机身高度重叠的障碍区间 [-0.10m, +dog_height+0.10m]
                dyn_hit_mask = (dyn_near[:, 2] >= curr_z - 0.10) & (dyn_near[:, 2] <= curr_z + self.dog_height + 0.10)
                if np.any(dyn_hit_mask):
                    hit_dyn = dyn_near[dyn_hit_mask][0]
                    return False, curr_z, f"触碰动态物理障碍(圆柱/实体): 坐标({hit_dyn[0]:.2f}, {hit_dyn[1]:.2f}, z={hit_dyn[2]:.2f})", [float(hit_dyn[0]), float(hit_dyn[1]), float(hit_dyn[2]), 3]

        # 2. 静态地形点云查询 (仅使用静态环境点云计算地面与楼梯踏面)
        idx_static = static_tree.query_ball_point([new_x, new_y], r=self.footprint_radius)
        if not idx_static:
            return False, curr_z, f"前方悬空踏空: 半径 {self.footprint_radius:.2f}m 内无地面点云支撑", [new_x, new_y, curr_z, 3]

        col_pts = static_pts[idx_static]

        # 3. 静态立面高墙碰撞检查 (机身硬半径 0.17m 内存在超过单步极限 >0.25m 的实体障碍)
        idx_body = static_tree.query_ball_point([new_x, new_y], r=self.body_hard_radius)
        if idx_body:
            body_pts = static_pts[idx_body]
            wall_mask = (body_pts[:, 2] > curr_z + self.max_step_height) & (body_pts[:, 2] <= curr_z + self.dog_height + 0.10)
            if np.count_nonzero(wall_mask) >= 3:
                hit_wall = body_pts[wall_mask][0]
                return False, curr_z, f"触碰静态立面高墙: 障碍高度 z={hit_wall[2]:.2f} (高出 {hit_wall[2]-curr_z:.2f}m > {self.max_step_height:.2f}m)", [float(hit_wall[0]), float(hit_wall[1]), float(hit_wall[2]), 3]

        # 4. 提取当前单步可达范围内的踏面候选点 [-0.25m, +0.25m]
        reach_mask = (col_pts[:, 2] >= curr_z - self.max_step_height - 0.05) & (col_pts[:, 2] <= curr_z + self.max_step_height + 0.05)
        reach_pts = col_pts[reach_mask]

        if len(reach_pts) == 0:
            # 范围内无有效踏面：检查是深坑还是高墙
            all_below = np.all(col_pts[:, 2] < curr_z - self.max_step_height)
            if all_below:
                return False, curr_z, f"危险断崖跌落: 前方地面下陷落差超过 {self.max_step_height:.2f}m", [new_x, new_y, curr_z - self.max_step_height, 3]
            else:
                highest_pt = col_pts[np.argmax(col_pts[:, 2])]
                dz_high = highest_pt[2] - curr_z
                return False, curr_z, f"超高立面障碍: 障碍高度 z={highest_pt[2]:.2f} (高出 {dz_high:.2f}m > {self.max_step_height:.2f}m)", [float(highest_pt[0]), float(highest_pt[1]), float(highest_pt[2]), 3]

        # 5. 寻找支撑表面并自适应抬升 (Squeeze-Up)
        # 如果前方足印内有高于当前地面 (0.02m < dz <= 0.25m) 的上行台阶/凸起，机身自动被“往上挤”
        higher_mask = reach_pts[:, 2] > curr_z + 0.02
        higher_pts = reach_pts[higher_mask]

        if len(higher_pts) >= 3:
            # 触发【自动往上挤】：取新台阶表面高度 (90分位数抗噪)
            z_target = float(np.percentile(higher_pts[:, 2], 90))
        else:
            # 平地行进或下台阶：按距离质心加权计算主导着地高程
            dists_sq = (reach_pts[:, 0] - new_x)**2 + (reach_pts[:, 1] - new_y)**2
            weights = 1.0 / (1.0 + 15.0 * dists_sq)
            z_target = float(np.sum(reach_pts[:, 2] * weights) / np.sum(weights))

            dz_down = z_target - curr_z
            if dz_down < -self.max_step_height - 0.05:
                # 超过 0.25m 下落极限 -> 断崖跌落保护
                return False, curr_z, f"危险断崖跌落: 下台阶落差 dz={dz_down:.2f}m 超出安全极限", [new_x, new_y, z_target, 3]

        # 所有检查通过，放行运动
        return True, z_target, "ok", None

    # ------------------ 100Hz 仿真主循环 ------------------
    def _sim_loop_worker(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self._running:
            try:
                self._update_simulation_step()
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"[Simulator] 仿真循环异常: {e}")
            rate.sleep()

    def _update_simulation_step(self):
        now_stamp = rospy.Time.now()
        dt = (now_stamp - self.last_sim_time).to_sec() if self.last_sim_time else 0.01
        self.last_sim_time = now_stamp

        if 0.0 < dt < 0.5:
            # 检查速度指令超时 (0.5s 无指令自动归零)
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

            if is_translating:
                # 执行纯点云越障与碰撞评估
                ok, target_z, reason, hit_node = self.evaluate_step_motion(new_x, new_y, self.sim_z)
                if ok:
                    self.sim_x = new_x
                    self.sim_y = new_y
                    self.sim_z = target_z
                else:
                    # 发生碰撞截断
                    self.collision_node = hit_node
                    self._publish_collision_event(hit_node)
                    rospy.logerr_throttle(
                        1.0,
                        f"[Simulator] 物理截断 (锁止于 x={self.sim_x:.2f}, y={self.sim_y:.2f}, z={self.sim_z:.2f}): {reason}"
                    )
            self.sim_yaw = normalize_angle(self.sim_yaw + dyaw)

        # 广播 TF 与发布里程计
        self._publish_tf_and_odom(now_stamp)

    def _publish_tf_and_odom(self, stamp):
        q = euler_to_quaternion(0.0, 0.0, self.sim_yaw)

        # 1. 广播 TF: map -> odom -> base_link
        if self.publish_tf and self.tf_broadcaster is not None:
            t1 = TransformStamped()
            t1.header.stamp = stamp
            t1.header.frame_id = self.map_frame
            t1.child_frame_id = self.odom_frame
            t1.transform.rotation.w = 1.0

            t2 = TransformStamped()
            t2.header.stamp = stamp
            t2.header.frame_id = self.odom_frame
            t2.child_frame_id = self.base_frame
            t2.transform.translation.x = self.sim_x
            t2.transform.translation.y = self.sim_y
            t2.transform.translation.z = self.sim_z
            t2.transform.rotation = q

            self.tf_broadcaster.sendTransform([t1, t2])

        # 2. 发布 Odometry (/loc_base 与 /odom)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.sim_x
        odom.pose.pose.position.y = self.sim_y
        odom.pose.pose.position.z = self.sim_z
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.cmd_vx
        odom.twist.twist.linear.y = self.cmd_vy
        odom.twist.twist.angular.z = self.cmd_wz

        self._loc_base_pub.publish(odom)
        self._odom_pub.publish(odom)

    def _publish_collision_event(self, hit_node: Optional[List[float]]):
        """发布碰撞事件与 3D Marker (若 hit_node 为 None 则清空标记)"""
        if hit_node is None:
            self._collision_node_pub.publish(RosString(data=""))
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "simulator_collision"
            marker.id = 0
            marker.action = Marker.DELETEALL
            self._collision_marker_pub.publish(marker)
            return

        # 1. 发布 JSON 字符串
        data = json.dumps({"x": hit_node[0], "y": hit_node[1], "z": hit_node[2], "type": hit_node[3]})
        self._collision_node_pub.publish(RosString(data=data))

        # 2. 发布 Marker
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "simulator_collision"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = hit_node[0]
        marker.pose.position.y = hit_node[1]
        marker.pose.position.z = hit_node[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.9
        self._collision_marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        sim = QuadrupedPointCloudSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
