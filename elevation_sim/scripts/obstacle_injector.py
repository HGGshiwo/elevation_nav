#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
障碍物注入器 (obstacle_injector) —— 仿真避障测试专用
====================================================
按参数化规则在 /elevation_global_plan 路径上生成障碍物表面点云,
发布到 /lidar_points 驱动「融合图 → 图上封锁 → isPathBlocked → Kinematic A* 绕障」全链路。

采样规格对齐融合管线判据 (cloud_graph_builder):
  - 面采样间距 0.04m: 覆盖 0.05m 体素降采样后每根 0.10m 柱仍 ≥2 点 (min_cluster_points=2),
    表面点近邻距离均匀, 不会被 SOR (mean_k=16, std=4) 误滤;
  - 底部离地 0.02m: 地面节点 headroom ≈ 0.02 < dog_height → 脚下顶头封锁;
  - 高度 ≥0.45m: 侧面包络 (z_bottom < tread_z+0.25 且 z_top > tread_z+k·0.25) 成立
    → 周围 0.15m 硬阻挡 / 0.26m 软代价, 且高于 max_step_height 不可被踏越;
  - 点云 frame_id=map, 融合引擎直接消费 (无 TF 变换环节)。

注意: 无真实雷达遮挡仿真; 障碍只在注入期间存在 (持续发布 = 替障碍记忆)。
实机 (sim:=false) 下不要启动本节点, 以免伪造障碍进入真实感知。
"""

import math
import threading

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


# ---------------- 几何表面采样 ----------------

def arange_incl(start, stop, step):
    """含端点的等差采样序列"""
    n = max(1, int(round((stop - start) / step)) + 1)
    return np.linspace(start, stop, n)


def sample_box(sx, sy, h, spacing):
    """箱体表面点, 局部系原点在底面中心, z 向上 (不含底面)"""
    xs = arange_incl(-sx / 2, sx / 2, spacing)
    ys = arange_incl(-sy / 2, sy / 2, spacing)
    zs = arange_incl(0.0, h, spacing)
    pts = [[x, y, h] for x in xs for y in ys]              # 顶面
    pts += [[sx / 2, y, z] for y in ys for z in zs]        # 四个侧面
    pts += [[-sx / 2, y, z] for y in ys for z in zs]
    pts += [[x, sy / 2, z] for x in xs for z in zs]
    pts += [[x, -sy / 2, z] for x in xs for z in zs]
    return np.asarray(pts, dtype=np.float32)


def sample_cylinder(radius, h, spacing):
    """圆柱侧表面 + 顶盘, 局部系原点在底面圆心"""
    pts = [[0.0, 0.0, h]]                                  # 顶面圆心
    for z in arange_incl(0.0, h, spacing):                 # 侧面
        n = max(12, int(math.ceil(2 * math.pi * radius / spacing)))
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append([radius * math.cos(a), radius * math.sin(a), z])
    for r in arange_incl(spacing, radius, spacing):        # 顶盘 (由外圈到内)
        n = max(8, int(math.ceil(2 * math.pi * r / spacing)))
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append([r * math.cos(a), r * math.sin(a), h])
    return np.asarray(pts, dtype=np.float32)


# ---------------- 路径弧长锚点 ----------------

class PlanAnchor:
    """从 latched 全局路径建立弧长索引, 提供按弧长的位姿/切向查询 (路径自带踏面高程 z)"""

    def __init__(self):
        self._pts = None
        self._cum = None
        self._lock = threading.Lock()

    def update(self, path_msg):
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in path_msg.poses]
        if len(pts) < 2:
            return
        arr = np.asarray(pts, dtype=np.float64)
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(arr, axis=0), axis=1))])
        with self._lock:
            self._pts, self._cum = arr, cum

    def ready(self):
        return self._cum is not None

    def length(self):
        return float(self._cum[-1])

    def at(self, s):
        """弧长 s 处的 (x, y, z, yaw), yaw 为局部切向朝向"""
        with self._lock:
            pts, cum = self._pts, self._cum
        s = float(np.clip(s, 0.0, cum[-1]))
        i = int(np.searchsorted(cum, s, side='right')) - 1
        i = max(0, min(i, len(pts) - 2))
        seg = max(cum[i + 1] - cum[i], 1e-6)
        t = min(1.0, (s - cum[i]) / seg)
        p0, p1 = pts[i], pts[i + 1]
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        z = p0[2] + t * (p1[2] - p0[2])
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return x, y, z, yaw


# ---------------- 运动规则 ----------------

class RuleBase:
    """规则基类 (等距多实例)。

    锚点语义: 设置导航目标后, 从当时的全局计划沿弧长每隔 interval 米锚定一个
    障碍实例 (首个在 start_offset 处, 距终点 end_margin 之前停止),
    世界坐标冻结 —— 障碍"一次成型", 重规划/机器人推进不移动; 新目标才重新锚定。
    子类实现 compute_poses(t) 在冻结锚点列表上定义运动。
    """

    MAX_INSTANCES = 50  # 单规则实例上限 (marker id 按 规则序号*100 + 实例号 编码)

    def __init__(self, cfg, spacing, default_activation):
        self.name = str(cfg.get("name", "rule"))
        self.shape = str(cfg.get("shape", "box"))
        self.interval = float(cfg.get("interval", 3.0))        # 沿路径等间距 (m)
        self.start_offset = float(cfg.get("start_offset", 1.5))  # 首个障碍距起点 (m)
        self.end_margin = float(cfg.get("end_margin", 1.0))      # 距终点该距离内不放 (防堵目标点)
        self.activation_radius = float(cfg.get("activation_radius", default_activation))
        self.z_clearance = float(cfg.get("z_clearance", 0.02))
        self.anchor_poses = []      # 冻结锚点列表 [(x,y,z,yaw)]; 空 = 未武装
        self.instance_poses = []    # 当前帧各实例位姿 (与锚点列表对齐)
        self.active_instances = []  # 当前帧激活的实例下标
        self.active = False

        if self.shape == "box":
            w = float(cfg.get("size_w", 0.4))
            d = float(cfg.get("size_d", 0.4))
            h = float(cfg.get("height", 0.5))
            self.height = h
            self.marker_scale = (w, d, h)
            self.local_pts = sample_box(w, d, h, spacing)
        elif self.shape == "cylinder":
            r = float(cfg.get("radius", 0.25))
            h = float(cfg.get("height", 0.5))
            self.height = h
            self.marker_scale = (2 * r, 2 * r, h)
            self.local_pts = sample_cylinder(r, h, spacing)
        else:
            raise ValueError("unknown shape: %s" % self.shape)

    def disarm(self):
        """新目标到来: 撤销锚点, 等待下一条计划重新锚定"""
        self.anchor_poses = []
        self.instance_poses = []
        self.active_instances = []
        self.active = False

    def refresh_anchor(self, plan, force=False):
        """捕获锚点 (仅注入器在新目标后的计划到达时调用); 返回是否刚完成捕获"""
        if not force:
            return False
        total = plan.length()
        last_s = max(self.start_offset, total - self.end_margin)
        s = self.start_offset
        self.anchor_poses = []
        while s < last_s and len(self.anchor_poses) < self.MAX_INSTANCES:
            self.anchor_poses.append(plan.at(s))
            s += self.interval
        self.on_anchor_captured()
        return len(self.anchor_poses) > 0

    def on_anchor_captured(self):
        """锚点捕获钩子 (子类重置运动状态用)"""
        pass

    def update(self, t, plan, robot_xy):
        if not self.anchor_poses:
            self.active = False
            self.instance_poses = []
            self.active_instances = []
            return
        self.compute_poses(t)
        self.active_instances = []
        for i, pose in enumerate(self.instance_poses):
            if robot_xy is None or math.hypot(robot_xy[0] - pose[0], robot_xy[1] - pose[1]) <= self.activation_radius:
                self.active_instances.append(i)
        self.active = len(self.active_instances) > 0
        self.post_update(t)

    def compute_poses(self, t):
        self.instance_poses = list(self.anchor_poses)

    def post_update(self, t):
        """激活门之后的规则级后置钩子 (如 blink 周期门控)"""
        pass

    def world_pts(self, noise_std, pose):
        x, y, z0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        pts = self.local_pts.copy()
        wx = pts[:, 0] * c - pts[:, 1] * s + x
        wy = pts[:, 0] * s + pts[:, 1] * c + y
        wz = pts[:, 2] + z0 + self.z_clearance
        out = np.stack([wx, wy, wz], axis=1)
        if noise_std > 0:
            out = out + np.random.normal(0.0, noise_std, out.shape).astype(np.float32)
        return out.astype(np.float32)


class StaticOnPathRule(RuleBase):
    """静止箱子固定在各捕获锚点处, yaw 对齐捕获时的路径切向 (宽面正对行进方向 = 一堵墙)"""
    pass


class BlinkRule(StaticOnPathRule):
    """各实例同步周期出现/消失: 验证融合图无衰减特性与封锁格自动清除"""

    def __init__(self, cfg, spacing, default_activation):
        super().__init__(cfg, spacing, default_activation)
        self.period_on = float(cfg.get("period_on", 8.0))
        self.period_off = float(cfg.get("period_off", 5.0))

    def post_update(self, t):
        if self.active:
            in_window = (t % (self.period_on + self.period_off)) < self.period_on
            if not in_window:
                self.active = False
                self.active_instances = []


class CrossingRule(RuleBase):
    """各实例以冻结锚点为中心, 沿捕获时的路径法向匀速横穿 (同相三角波往返)"""

    def __init__(self, cfg, spacing, default_activation):
        super().__init__(cfg, spacing, default_activation)
        self.lateral_range = float(cfg.get("lateral_range", 1.5))
        self.speed = float(cfg.get("speed", 0.2))

    def compute_poses(self, t):
        u = (t * self.speed) % (2 * self.lateral_range)
        d = u if u <= self.lateral_range else 2 * self.lateral_range - u
        d -= self.lateral_range / 2.0  # 居中往返
        poses = []
        for x0, y0, z0, yaw in self.anchor_poses:
            nx, ny = -math.sin(yaw), math.cos(yaw)
            poses.append((x0 + nx * d, y0 + ny * d, z0, yaw))
        self.instance_poses = poses


class OncomingRule(RuleBase):
    """各实例从冻结锚点沿捕获时的路径反方向迎面移动;
    实例距机器人 <stop_dist 时单独消失, 冷却后从各自锚点重现"""

    def __init__(self, cfg, spacing, default_activation):
        super().__init__(cfg, spacing, default_activation)
        self.speed = float(cfg.get("speed", 0.25))
        self.stop_dist = float(cfg.get("stop_dist", 0.8))
        self.cooldown = float(cfg.get("cooldown", 5.0))
        self._t0 = None
        self._paused_at = None
        self._tangents = []

    def on_anchor_captured(self):
        n = len(self.anchor_poses)
        self._tangents = [(math.cos(yaw), math.sin(yaw)) for (_, _, _, yaw) in self.anchor_poses]
        self._t0 = None
        self._paused_at = [None] * n

    def update(self, t, plan, robot_xy):
        if not self.anchor_poses:
            self.active = False
            self.instance_poses = []
            self.active_instances = []
            return
        n = len(self.anchor_poses)
        if self._t0 is None or len(self._t0) != n:
            self._t0 = [t] * n
        if self._paused_at is None or len(self._paused_at) != n:
            self._paused_at = [None] * n

        self.instance_poses = []
        self.active_instances = []
        for k, (ax, ay, az, ayaw) in enumerate(self.anchor_poses):
            if self._paused_at[k] is not None:
                if t - self._paused_at[k] < self.cooldown:
                    self.instance_poses.append((ax, ay, az, ayaw))  # 冷却中: 保持锚点位姿但不激活
                    continue
                self._t0[k] = t
                self._paused_at[k] = None
            tx, ty = self._tangents[k]
            d = (t - self._t0[k]) * self.speed
            pose = (ax - tx * d, ay - ty * d, az, ayaw)
            self.instance_poses.append(pose)
            if robot_xy is None or math.hypot(robot_xy[0] - pose[0], robot_xy[1] - pose[1]) <= self.activation_radius:
                if robot_xy is not None and math.hypot(robot_xy[0] - pose[0], robot_xy[1] - pose[1]) < self.stop_dist:
                    self._paused_at[k] = t  # 抵近消失, 冷却后从锚点重现
                else:
                    self.active_instances.append(k)
        self.active = len(self.active_instances) > 0


RULE_TYPES = {
    "static_on_path": StaticOnPathRule,
    "blink": BlinkRule,
    "crossing": CrossingRule,
    "oncoming": OncomingRule,
}


# ---------------- 踏面高程吸附 ----------------

class TreadZSnapper:
    """把锚点 z 吸附到最近的真实踏面高程。

    全局路径位姿的 z 理论上是踏面高程, 但插值锚点/异常规划输出可能带 0 或层间值,
    导致障碍物插进楼层之间 (渲染在踏面下方, 融合图上不与任何层冲突)。
    这里用 latched 的 /elevation_graph_nodes (全局流形图节点, 含真实踏面 z) 建立空间索引,
    在锚点 XY 附近取与参考 z 同层最近的节点 z 做修正; 无图数据时回退路径 z。
    """

    CELL = 0.5  # 平面桶格 (m)

    def __init__(self):
        self.buckets = None  # dict[(bi,bj)] -> np.ndarray Nx3
        rospy.Subscriber("/elevation_graph_nodes", PointCloud2, self._on_nodes, queue_size=1)

    def _on_nodes(self, msg):
        try:
            pts = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                           dtype=np.float64)
            if pts.size == 0 or pts.shape[1] < 3:
                return
            b = {}
            keys = np.floor(pts[:, :2] / self.CELL).astype(np.int64)
            for (bi, bj), p in zip(keys, pts):
                b.setdefault((int(bi), int(bj)), []).append((p[0], p[1], p[2]))
            self.buckets = {k: np.array(v) for k, v in b.items()}
            rospy.loginfo("[obstacle_injector] tread z-index built: %d graph nodes", len(pts))
        except Exception as e:
            rospy.logwarn(f"[obstacle_injector] tread z-index build failed: {e}")

    def snap(self, x, y, z_ref, xy_radius=0.6):
        """取 XY 附近踏面节点中与 z_ref 最接近的层高; 找不到则回退 z_ref"""
        if not self.buckets:
            return z_ref
        bi, bj = int(math.floor(x / self.CELL)), int(math.floor(y / self.CELL))
        r = int(math.ceil(xy_radius / self.CELL))
        best_z, best_dz = None, None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                arr = self.buckets.get((bi + di, bj + dj))
                if arr is None:
                    continue
                d_xy = np.hypot(arr[:, 0] - x, arr[:, 1] - y)
                cand = arr[d_xy <= xy_radius]
                if cand.shape[0] == 0:
                    continue
                dz = np.abs(cand[:, 2] - z_ref)
                k = int(np.argmin(dz))
                if best_dz is None or float(dz[k]) < best_dz:
                    best_dz = float(dz[k])
                    best_z = float(cand[k, 2])
        return best_z if best_z is not None else z_ref


# ---------------- 节点 ----------------

class ObstacleInjector:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        self.spacing = float(rospy.get_param("~sample_spacing", 0.04))
        self.noise = float(rospy.get_param("~point_noise", 0.005))
        self.default_activation = float(rospy.get_param("~activation_radius", 6.0))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        plan_topic = rospy.get_param("~plan_topic", "/elevation_global_plan")
        goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        cloud_topic = rospy.get_param("~cloud_topic", "/lidar_points")
        marker_topic = rospy.get_param("~marker_topic", "/elevation_injected_obstacles")

        self.anchor = PlanAnchor()
        self.z_snapper = TreadZSnapper()
        rospy.Subscriber(plan_topic, Path, self._on_plan, queue_size=1)
        rospy.Subscriber(goal_topic, PoseStamped, self._on_goal, queue_size=1)
        self.cloud_pub = rospy.Publisher(cloud_topic, PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # goal 锚定模式的状态机: 新目标 → disarm → 下一条计划到达时重新锚定 (一次成型)
        self.last_goal_xy = None
        self.need_reanchor = False

        self.rules = []
        for cfg in rospy.get_param("~rules", []):
            cls = RULE_TYPES.get(str(cfg.get("type", "")))
            if cls is None:
                rospy.logwarn("[obstacle_injector] unknown rule type: %s", cfg.get("type"))
                continue
            try:
                self.rules.append(cls(cfg, self.spacing, self.default_activation))
            except (ValueError, KeyError) as e:
                rospy.logerr("[obstacle_injector] bad rule %s: %s", cfg.get("name"), e)
        self.marker_ids = {r.name: i * 100 for i, r in enumerate(self.rules)}  # 实例 id = 基号 + 实例号
        rospy.loginfo("[obstacle_injector] %d rule(s) loaded, spacing=%.3fm", len(self.rules), self.spacing)

    def _on_plan(self, msg):
        self.anchor.update(msg)

    def _on_goal(self, msg):
        """新导航目标: 撤销全部障碍锚点, 等待新计划到达后按新路径一次成型"""
        gx, gy = msg.pose.position.x, msg.pose.position.y
        if self.last_goal_xy is None or math.hypot(gx - self.last_goal_xy[0], gy - self.last_goal_xy[1]) > 0.1:
            self.last_goal_xy = (gx, gy)
            for r in self.rules:
                r.disarm()
            self.need_reanchor = True
            rospy.loginfo("[obstacle_injector] new goal (%.2f, %.2f) → re-anchor on next global plan", gx, gy)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                 rospy.Time(0), rospy.Duration(0.05))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def _marker(self, rule, inst_idx, action, pose):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = rospy.get_rostime()
        m.ns = "injected_obstacles"
        m.id = self.marker_ids[rule.name] + inst_idx
        m.action = action
        if action == Marker.ADD:
            x, y, z0, yaw = pose
            m.type = Marker.CUBE if rule.shape == "box" else Marker.CYLINDER
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z0 + rule.z_clearance + rule.height / 2.0
            q = quaternion_from_euler(0.0, 0.0, yaw)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x, m.scale.y, m.scale.z = rule.marker_scale
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.45
        m.lifetime = rospy.Duration(0.3)
        return m

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.anchor.ready():
                # goal 模式: 新目标后等待第一条计划到达, 按新路径一次性锚定全部障碍
                if self.need_reanchor:
                    for r in self.rules:
                        r.refresh_anchor(self.anchor, force=True)
                    self.need_reanchor = False

                t = rospy.get_rostime().to_sec()
                robot_xy = self._robot_xy()

                clouds = []
                markers = []
                for rule in self.rules:
                    rule.update(t, self.anchor, robot_xy)
                    for i, pose in enumerate(rule.instance_poses):
                        if i in rule.active_instances:
                            # 锚点 z 吸附到真实踏面高程 (防障碍插进楼层之间)
                            px, py, pz, pyaw = pose
                            pz = self.z_snapper.snap(px, py, pz)
                            clouds.append(rule.world_pts(self.noise, (px, py, pz, pyaw)))
                            markers.append(self._marker(rule, i, Marker.ADD, (px, py, pz, pyaw)))
                        else:
                            markers.append(self._marker(rule, i, Marker.DELETE, pose))

                # 无激活障碍时发布空帧: 融合引擎按最新帧重建观测柱表,
                # 障碍离开后封锁格随下一帧融合自动清除
                header = Header(stamp=rospy.get_rostime(), frame_id=self.map_frame)
                if clouds:
                    msg = point_cloud2.create_cloud_xyz32(header, np.vstack(clouds))
                else:
                    msg = point_cloud2.create_cloud_xyz32(header, [])
                self.cloud_pub.publish(msg)
                self.marker_pub.publish(MarkerArray(markers=markers))
            rate.sleep()


def main():
    rospy.init_node("obstacle_injector")
    ObstacleInjector().spin()


if __name__ == "__main__":
    main()
