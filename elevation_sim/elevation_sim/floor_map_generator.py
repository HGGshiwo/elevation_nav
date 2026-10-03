# -*- coding: utf-8 -*-
"""
多层流形高程图 2D 分层踏面展平与建图引擎 (FloorMapGenerator)
基于纯 3D 流形拓扑连通图 (Topological Connectivity Graph) 与多源竞争扩散 (Multi-Source Wavefront) 进行无重叠分层展平。
- 零硬编码高度切片：自适应识别垂直重叠分叉点；
- 纯连通图最短路径驱动：大平层与悬空外挂区域按流形测地距离自洽归属；
- 严格保证单图 (gx, gy) 零重叠；
- 支持 Turbo 高程伪彩色阶、精准 1-to-1 空间节点映射表与 ROS map_server 标准导出。
"""

import os
import io
import json
import math
import heapq
import zipfile
from collections import deque, defaultdict
from typing import List, Dict, Any, Tuple, Optional, Set

import numpy as np
import cv2


class FloorMapGenerator:
    """
    3D 流形拓扑踏面 -> 2D 分层地图生成器 (纯连通图模型)
    """
    def __init__(self,
                 resolution: float = 0.10,
                 dog_height: float = 0.45,
                 max_step_height: float = 0.25,
                 margin_meters: float = 1.0):
        self.resolution = float(resolution)
        self.dog_height = float(dog_height)
        self.max_step_height = float(max_step_height)
        self.margin_meters = float(margin_meters)
        self.layers_meta: List[Dict[str, Any]] = []
        self.layers_lookups: Dict[int, Dict[str, Any]] = {}
        self.layers_images: Dict[int, np.ndarray] = {}
        self.layers_vis_images: Dict[int, np.ndarray] = {}

    def generate_layers(self,
                        nodes: List[List[float]],
                        edges: Optional[List[List[float]]] = None,
                        output_dir: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        基于纯 3D 流形拓扑图的无重叠分层建图核心流程：
        1. 构建流形可通行拓扑邻接图 G = (V, E)；
        2. 扫描 (gx, gy) 空间柱识别重叠分叉点，确定图层数 K 并初始化各层种子集；
        3. 在流形图上执行多源同步 Dijkstra 拓扑扩散，计算各节点到各层种子的最短图路径；
        4. 执行 (gx, gy) 零冲突仲裁分配，将节点赋予拓扑距离最近且无冲突的图层；
        5. 各层渲染 2D 栅格地图 (Free=254, Unknown=205) 与 Turbo 高程伪彩图；
        6. 生成 1-to-1 映射索引、ROS YAML 与 ZIP 打包。
        """
        if not nodes:
            return []

        res = self.resolution
        self.layers_meta.clear()
        self.layers_lookups.clear()
        self.layers_images.clear()
        self.layers_vis_images.clear()

        # ---- 1. 解析节点数据并筛选可通行点 ----
        all_nodes_dict: Dict[Tuple[float, float, float], Dict[str, Any]] = {}
        trav_nodes_dict: Dict[Tuple[float, float, float], Dict[str, Any]] = {}

        for i, n in enumerate(nodes):
            x = float(n[0])
            y = float(n[1])
            z = float(n[2])
            trav = float(n[3]) if len(n) > 3 else 0.0
            zone = int(n[4]) if len(n) > 4 else 0
            is_trav = (trav < 0.95) and (zone != 3)

            k_xyz = (round(x, 3), round(y, 3), round(z, 3))
            node_obj = {
                "id": i,
                "x": x,
                "y": y,
                "z": z,
                "trav": trav,
                "zone": zone,
                "is_trav": is_trav,
                "k_xyz": k_xyz
            }
            all_nodes_dict[k_xyz] = node_obj
            if is_trav:
                trav_nodes_dict[k_xyz] = node_obj

        if not trav_nodes_dict:
            return []

        # ---- 2. 构建 3D 流形拓扑连通图 G = (V, E) ----
        adj: Dict[Tuple[float, float, float], Set[Tuple[float, float, float]]] = defaultdict(set)
        if edges:
            for e in edges:
                if len(e) >= 6:
                    k1 = (round(float(e[0]), 3), round(float(e[1]), 3), round(float(e[2]), 3))
                    k2 = (round(float(e[3]), 3), round(float(e[4]), 3), round(float(e[5]), 3))
                    if k1 in trav_nodes_dict and k2 in trav_nodes_dict:
                        adj[k1].add(k2)
                        adj[k2].add(k1)

        # 若未提供完整边，按 8 邻域与运动学台阶高差构建拓扑边
        spatial_trav = defaultdict(list)
        for k_xyz, node in trav_nodes_dict.items():
            r = int(round(node["x"] / res))
            c = int(round(node["y"] / res))
            spatial_trav[(r, c)].append(node)

        for (r, c), cell_nodes in spatial_trav.items():
            for u in cell_nodes:
                u_key = u["k_xyz"]
                if len(adj[u_key]) > 0:
                    continue
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        for v in spatial_trav.get((r + dr, c + dc), []):
                            if abs(u["z"] - v["z"]) <= self.max_step_height:
                                adj[u_key].add(v["k_xyz"])
                                adj[v["k_xyz"]].add(u_key)

        # ---- 3. 计算对齐的全局 2D 平面包围盒 ----
        all_x = [n["x"] for n in trav_nodes_dict.values()]
        all_y = [n["y"] for n in trav_nodes_dict.values()]
        min_x = min(all_x) - self.margin_meters
        max_x = max(all_x) + self.margin_meters
        min_y = min(all_y) - self.margin_meters
        max_y = max(all_y) + self.margin_meters

        origin_x = math.floor(min_x / res) * res
        origin_y = math.floor(min_y / res) * res
        width = int(math.ceil((max_x - origin_x) / res)) + 1
        height = int(math.ceil((max_y - origin_y) / res)) + 1

        # ---- 4. 识别垂直重叠分叉点并初始化图层种子 ----
        # 将所有可通行点投影到 (gx, gy) 柱状网格
        col_map: Dict[Tuple[int, int], List[Dict[str, Any]]] = defaultdict(list)
        for node in trav_nodes_dict.values():
            gx = int(round((node["x"] - origin_x) / res))
            gy = int(round((node["y"] - origin_y) / res))
            col_map[(gx, gy)].append(node)

        # 对每个柱内的踏面按高度 z 升序排序
        for (gx, gy) in col_map:
            col_map[(gx, gy)].sort(key=lambda n: n["z"])

        # 计算空间最大垂直重叠层数
        max_overlap = max(len(nodes_list) for nodes_list in col_map.values())
        num_layers = max(1, max_overlap)

        # 初始化各图层的种子点集 S_0, S_1, ... S_{num_layers-1}
        seed_sets: Dict[int, Set[Tuple[float, float, float]]] = defaultdict(set)
        for (gx, gy), nodes_list in col_map.items():
            if len(nodes_list) > 1:
                # 属于垂直重叠柱：按高度名次分配为各层种子
                for lvl, node in enumerate(nodes_list):
                    layer_id = min(lvl, num_layers - 1)
                    seed_sets[layer_id].add(node["k_xyz"])

        # 若全图无垂直重叠（例如单层平地或长直大坡道），选择最低点为初始种子
        if num_layers == 1 or not any(len(s) > 0 for s in seed_sets.values()):
            num_layers = 1
            lowest_k = min(trav_nodes_dict.keys(), key=lambda k: trav_nodes_dict[k]["z"])
            seed_sets[0].add(lowest_k)

        # ---- 5. 流形拓扑图多源同步 Dijkstra 扩散与 K-Medoids 迭代 ----
        # 记录每个节点到各图层种子的最短图路径距离: dist_to_layer[k_xyz][layer_id] = distance
        assigned_layers: Dict[int, Dict[Tuple[int, int], Dict[str, Any]]] = defaultdict(dict)
        max_iterations = 3

        for iteration in range(max_iterations):
            dist_to_layer: Dict[Tuple[float, float, float], Dict[int, float]] = defaultdict(dict)
            pq: List[Tuple[float, Tuple[float, float, float], int]] = []

            # 将所有图层的种子推入优先队列 (初始代价 0.0)
            for lid in range(num_layers):
                for s_key in seed_sets[lid]:
                    dist_to_layer[s_key][lid] = 0.0
                    heapq.heappush(pq, (0.0, s_key, lid))

            # 沿流形连通边同步扩散 (纯图路径长度，无任何硬编码高度判断)
            while pq:
                d_cur, u_key, lid = heapq.heappop(pq)
                if d_cur > dist_to_layer[u_key].get(lid, float("inf")):
                    continue

                u_node = trav_nodes_dict[u_key]
                for v_key in adj.get(u_key, []):
                    v_node = trav_nodes_dict[v_key]
                    # 3D 拓扑边欧氏位移
                    edge_w = math.sqrt(
                        (v_node["x"] - u_node["x"])**2 +
                        (v_node["y"] - u_node["y"])**2 +
                        (v_node["z"] - u_node["z"])**2
                    )
                    new_dist = d_cur + edge_w
                    if new_dist < dist_to_layer[v_key].get(lid, float("inf")):
                        dist_to_layer[v_key][lid] = new_dist
                        heapq.heappush(pq, (new_dist, v_key, lid))

            # ---- 6. (gx, gy) 零冲突拓扑仲裁分配 ----
            current_assignment: Dict[int, Dict[Tuple[int, int], Dict[str, Any]]] = defaultdict(dict)
            unassigned_nodes_set = set(trav_nodes_dict.keys())

            # 对每个柱网格进行分配
            for (gx, gy), nodes_list in col_map.items():
                if len(nodes_list) == 1:
                    # 单节点：赋给拓扑距离最近的图层
                    node = nodes_list[0]
                    k_xyz = node["k_xyz"]
                    dist_map = dist_to_layer.get(k_xyz, {})
                    if dist_map:
                        best_lid = min(dist_map.keys(), key=lambda lid: dist_map[lid])
                    else:
                        best_lid = 0
                    current_assignment[best_lid][(gx, gy)] = node
                    unassigned_nodes_set.discard(k_xyz)
                else:
                    # 多重节点（重叠柱）：按各节点到各层的拓扑距离做最优二分匹配（谁近谁得该层）
                    # 构建距离代价矩阵并做升序分配
                    for node in nodes_list:
                        k_xyz = node["k_xyz"]
                        dist_map = dist_to_layer.get(k_xyz, {})
                        # 尝试依次放入最近且未被当前柱占据的层
                        sorted_lids = sorted(range(num_layers), key=lambda lid: dist_map.get(lid, float("inf")))
                        for target_lid in sorted_lids:
                            if (gx, gy) not in current_assignment[target_lid]:
                                current_assignment[target_lid][(gx, gy)] = node
                                unassigned_nodes_set.discard(k_xyz)
                                break

            # 处理由于图断连未被波前覆盖的孤立节点（放入无冲突的层）
            for un_k in list(unassigned_nodes_set):
                node = trav_nodes_dict[un_k]
                gx = int(round((node["x"] - origin_x) / res))
                gy = int(round((node["y"] - origin_y) / res))
                for target_lid in range(num_layers):
                    if (gx, gy) not in current_assignment[target_lid]:
                        current_assignment[target_lid][(gx, gy)] = node
                        break

            assigned_layers = current_assignment

            # 迭代更新各层中心种子 (Medoid Refinement)
            if iteration < max_iterations - 1 and num_layers > 1:
                new_seed_sets = defaultdict(set)
                for lid in range(num_layers):
                    layer_nodes_list = list(assigned_layers[lid].values())
                    if not layer_nodes_list:
                        continue
                    # 选取该层内部与重叠柱相连的核心节点作为下一轮种子
                    for n in layer_nodes_list:
                        gx = int(round((n["x"] - origin_x) / res))
                        gy = int(round((n["y"] - origin_y) / res))
                        if len(col_map.get((gx, gy), [])) > 1:
                            new_seed_sets[lid].add(n["k_xyz"])
                    if not new_seed_sets[lid]:
                        # 若无重叠柱点，选该层中位数节点作为种子
                        mid_node = sorted(layer_nodes_list, key=lambda nd: nd["z"])[len(layer_nodes_list) // 2]
                        new_seed_sets[lid].add(mid_node["k_xyz"])
                seed_sets = new_seed_sets

        # ---- 7. 过滤空层并按高程特征排序 ----
        valid_layers = []
        for lid, l_nodes_dict in assigned_layers.items():
            if len(l_nodes_dict) > 0:
                mean_z = sum(n["z"] for n in l_nodes_dict.values()) / len(l_nodes_dict)
                valid_layers.append((mean_z, l_nodes_dict))

        # 按平均高程升序排列，使层号与物理直觉一致 (Layer 0=底层, Layer 1=高层)
        valid_layers.sort(key=lambda item: item[0])

        # ---- 8. 栅格化生成 2D 地图、Turbo 色阶与 1-to-1 查找表 ----
        lut_base = np.arange(256, dtype=np.uint8).reshape(256, 1)
        turbo_lut = cv2.applyColorMap(lut_base, cv2.COLORMAP_TURBO)

        for final_lid, (layer_mean_z, layer_nodes) in enumerate(valid_layers):
            z_vals = [n["z"] for n in layer_nodes.values()]
            z_min = min(z_vals)
            z_max = max(z_vals)
            z_mean = sum(z_vals) / len(z_vals)

            img_gray = np.full((height, width), 205, dtype=np.uint8)  # ROS 205=Unknown
            img_color = np.full((height, width, 3), [22, 25, 30], dtype=np.uint8)  # 深色背景
            pixel_lookup: Dict[str, Any] = {}

            dz = max(z_max - z_min, 1e-4)

            for (gx, gy), node in layer_nodes.items():
                if 0 <= gx < width and 0 <= gy < height:
                    col = gx
                    row = height - 1 - gy  # 图像第 0 行为最北(+Y)

                    img_gray[row, col] = 254  # Free

                    # 连续 Turbo 伪彩色彩映射
                    t = np.clip((node["z"] - z_min) / dz, 0.0, 1.0)
                    color_idx = int(round(t * 255.0))
                    bgr = turbo_lut[color_idx, 0].tolist()
                    img_color[row, col] = bgr

                    hex_color = f"#{int(bgr[2]):02x}{int(bgr[1]):02x}{int(bgr[0]):02x}"

                    pixel_lookup[f"{col},{row}"] = {
                        "node_id": int(node["id"]),
                        "x": round(float(node["x"]), 3),
                        "y": round(float(node["y"]), 3),
                        "z": round(float(node["z"]), 3),
                        "traversability": round(float(node["trav"]), 3),
                        "cost_zone": int(node["zone"]),
                        "color": hex_color,
                        "layer_id": final_lid
                    }

            layer_meta = {
                "layer_id": final_lid,
                "name": f"第 {final_lid + 1} 层 (Z: {z_min:.2f}m ~ {z_max:.2f}m)",
                "z_min": round(float(z_min), 3),
                "z_max": round(float(z_max), 3),
                "z_mean": round(float(z_mean), 3),
                "node_count": len(layer_nodes),
                "resolution": res,
                "width": width,
                "height": height,
                "origin": [round(origin_x, 3), round(origin_y, 3), 0.0],
                "image_file": f"layer_{final_lid}.png",
                "vis_image_file": f"layer_{final_lid}_vis.png",
                "yaml_file": f"layer_{final_lid}.yaml",
                "nodes_file": f"layer_{final_lid}_nodes.json"
            }

            self.layers_meta.append(layer_meta)
            self.layers_lookups[final_lid] = pixel_lookup
            self.layers_images[final_lid] = img_gray
            self.layers_vis_images[final_lid] = img_color

            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
                cv2.imwrite(os.path.join(output_dir, layer_meta["image_file"]), img_gray)
                cv2.imwrite(os.path.join(output_dir, layer_meta["vis_image_file"]), img_color)

                yaml_content = (
                    f"image: {layer_meta['image_file']}\n"
                    f"resolution: {res}\n"
                    f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0.000]\n"
                    f"negate: 0\n"
                    f"occupied_thresh: 0.65\n"
                    f"free_thresh: 0.196\n"
                    f"mode: trinary\n"
                )
                with open(os.path.join(output_dir, layer_meta["yaml_file"]), "w", encoding="utf-8") as f:
                    f.write(yaml_content)

                with open(os.path.join(output_dir, layer_meta["nodes_file"]), "w", encoding="utf-8") as f:
                    json.dump(pixel_lookup, f)

        if output_dir:
            manifest = {
                "total_layers": len(self.layers_meta),
                "resolution": res,
                "global_origin": [origin_x, origin_y, 0.0],
                "global_width": width,
                "global_height": height,
                "layers": self.layers_meta
            }
            with open(os.path.join(output_dir, "manifest.json"), "w", encoding="utf-8") as f:
                json.dump(manifest, f, indent=2, ensure_ascii=False)

        return self.layers_meta

    def get_layer_lookup(self, layer_id: int) -> Dict[str, Any]:
        """获取指定图层的 1-to-1 像素到 3D 节点查找表"""
        return self.layers_lookups.get(layer_id, {})

    def generate_zip_archive(self) -> io.BytesIO:
        """打包所有分层地图 PNG、ROS YAML、JSON 索引与说明文件为 ZIP 字节流"""
        zip_buf = io.BytesIO()
        with zipfile.ZipFile(zip_buf, mode="w", compression=zipfile.ZIP_DEFLATED) as zf:
            for meta in self.layers_meta:
                lid = meta["layer_id"]
                if lid in self.layers_images:
                    _, buf_gray = cv2.imencode(".png", self.layers_images[lid])
                    zf.writestr(meta["image_file"], buf_gray.tobytes())

                if lid in self.layers_vis_images:
                    _, buf_vis = cv2.imencode(".png", self.layers_vis_images[lid])
                    zf.writestr(meta["vis_image_file"], buf_vis.tobytes())

                yaml_content = (
                    f"image: {meta['image_file']}\n"
                    f"resolution: {meta['resolution']}\n"
                    f"origin: [{meta['origin'][0]:.3f}, {meta['origin'][1]:.3f}, 0.000]\n"
                    f"negate: 0\n"
                    f"occupied_thresh: 0.65\n"
                    f"free_thresh: 0.196\n"
                    f"mode: trinary\n"
                )
                zf.writestr(meta["yaml_file"], yaml_content)

                if lid in self.layers_lookups:
                    zf.writestr(meta["nodes_file"], json.dumps(self.layers_lookups[lid], ensure_ascii=False))

            manifest = {
                "total_layers": len(self.layers_meta),
                "resolution": self.resolution,
                "layers": self.layers_meta
            }
            zf.writestr("manifest.json", json.dumps(manifest, indent=2, ensure_ascii=False))

            readme = (
                "==========================================================\n"
                "  Elevation Nav: 3D Manifold Flattened 2D Floor Maps\n"
                "  基于 3D 流形拓扑连通图的纯拓扑无重叠展平地图包\n"
                "==========================================================\n"
                "1. ROS 标准地图加载方法 (以 Layer 0 为例):\n"
                "   rosrun map_server map_server layer_0.yaml\n\n"
                "2. 栅格数值标准:\n"
                "   - 254 (白色/彩色): 可通行踏面 (Free Space)\n"
                "   - 205 (深暗底色): 未知/悬空背景 (Unknown Space)\n\n"
                "3. 3D 节点 1-to-1 精确定位:\n"
                "   参考 layer_X_nodes.json 中以 'col,row' 为键的详细空间节点坐标 (X, Y, Z, NodeID)。\n"
            )
            zf.writestr("README.txt", readme)

        zip_buf.seek(0)
        return zip_buf
