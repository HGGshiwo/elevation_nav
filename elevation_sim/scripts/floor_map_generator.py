#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 节点：3D 流形拓扑踏面 -> 2D 分层地图生成服务
订阅 /elevation_graph_nodes 与 /elevation_graph_edges，响应指令并自动展平生成多层 2D 栅格地图
"""

import os
import sys
from pathlib import Path
from typing import List, Dict, Any, Optional

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String as RosString, Header
import rospkg

package_root = Path(__file__).resolve().parent.parent
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from elevation_sim.floor_map_generator import FloorMapGenerator


class FloorMapGeneratorNode:
    def __init__(self):
        rospy.init_node("floor_map_generator", anonymous=False)

        # 读取 ROS 参数
        self.resolution = rospy.get_param("~resolution", 0.10)
        self.dog_height = rospy.get_param("~dog_height", 0.45)
        self.max_step_height = rospy.get_param("~max_step_height", 0.25)
        self.auto_generate = rospy.get_param("~auto_generate", False)

        try:
            r = rospkg.RosPack()
            default_out = os.path.join(r.get_path("elevation_sim"), "web", "static", "generated_maps")
        except Exception:
            default_out = str(package_root / "web" / "static" / "generated_maps")

        self.output_dir = rospy.get_param("~output_dir", default_out)
        os.makedirs(self.output_dir, exist_ok=True)

        self.generator = FloorMapGenerator(
            resolution=self.resolution,
            dog_height=self.dog_height,
            max_step_height=self.max_step_height
        )

        self.graph_nodes: List[List[float]] = []
        self.graph_edges: List[List[float]] = []
        self.has_new_nodes = False

        # 订阅流形图话题
        rospy.Subscriber("/elevation_graph_nodes", PointCloud2, self._on_nodes, queue_size=1)
        rospy.Subscriber("/elevation_graph_edges", MarkerArray, self._on_edges, queue_size=1)
        # 订阅指令话题
        rospy.Subscriber("/generate_2d_floors_cmd", RosString, self._on_cmd, queue_size=5)

        # 发布建图完成状态话题
        self.status_pub = rospy.Publisher("/elevation_2d_floors_status", RosString, queue_size=1, latch=True)

        rospy.loginfo("[FloorMapGenerator] Node initialized. Ready to generate 2D layered maps to: %s", self.output_dir)

    def _on_nodes(self, msg: PointCloud2):
        try:
            has_zone = any(f.name == "zone" for f in msg.fields)
            pts = []
            if has_zone:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "zone"), skip_nans=True):
                    pts.append([round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3),
                               round(float(p[3]), 2), int(p[4])])
            else:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                    pts.append([round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3),
                               round(float(p[3]), 2), 0])
            self.graph_nodes = pts
            self.has_new_nodes = True
            rospy.loginfo("[FloorMapGenerator] Received %d static graph nodes", len(pts))
            if self.auto_generate:
                self.generate()
        except Exception as e:
            rospy.logwarn("[FloorMapGenerator] Failed to parse nodes point cloud: %s", str(e))

    def _on_edges(self, msg: MarkerArray):
        try:
            lines = []
            for marker in msg.markers:
                pts = marker.points
                for i in range(0, len(pts) - 1, 2):
                    lines.append([
                        pts[i].x, pts[i].y, pts[i].z,
                        pts[i+1].x, pts[i+1].y, pts[i+1].z
                    ])
            self.graph_edges = lines
            rospy.loginfo("[FloorMapGenerator] Received %d graph edges", len(lines))
        except Exception as e:
            rospy.logwarn("[FloorMapGenerator] Failed to parse graph edges: %s", str(e))

    def _on_cmd(self, msg: RosString):
        rospy.loginfo("[FloorMapGenerator] Triggered 2D map generation via command: %s", msg.data)
        self.generate()

    def generate(self) -> List[Dict[str, Any]]:
        if not self.graph_nodes:
            rospy.logwarn("[FloorMapGenerator] No graph nodes available yet to generate maps")
            return []

        rospy.loginfo("[FloorMapGenerator] Starting 2D multi-layer map flattening on %d nodes...", len(self.graph_nodes))
        t0 = rospy.Time.now()
        layers = self.generator.generate_layers(
            nodes=self.graph_nodes,
            edges=self.graph_edges,
            output_dir=self.output_dir
        )
        elapsed = (rospy.Time.now() - t0).toSec()
        rospy.loginfo("[FloorMapGenerator] Successfully generated %d 2D floor maps in %.3f s", len(layers), elapsed)

        # 发布状态通知
        status_msg = RosString()
        status_msg.data = f"OK: {len(layers)} layers generated"
        self.status_pub.publish(status_msg)
        return layers

    def run(self):
        rospy.spin()


def main():
    node = FloorMapGeneratorNode()
    node.run()


if __name__ == "__main__":
    main()
