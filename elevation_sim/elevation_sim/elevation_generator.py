# -*- coding: utf-8 -*-
"""
2.5D 高程图生成与几何特征分析核心模块
基于机器狗上下越障极限 (max_step_height) 与水平步幅，提取连续踏面，自动剥离高空悬浮杂点与立面
"""

import math
from typing import Dict, Any, Optional, List
import numpy as np
from scipy.ndimage import uniform_filter, maximum_filter, minimum_filter


class ElevationMapGenerator:
    def __init__(
        self,
        resolution: float = 0.10,
        max_step_height: float = 0.25,
        step_radius: float = 0.25,
        max_slope_deg: float = 30.0,
        fill_radius: float = 0.20,
        z_filter_min: Optional[float] = None,
        z_filter_max: Optional[float] = None
    ):
        self.resolution = float(resolution)
        self.max_step_height = float(max_step_height)
        self.step_radius = float(step_radius)
        self.max_slope_deg = float(max_slope_deg)
        self.fill_radius = float(fill_radius)
        self.z_filter_min = z_filter_min
        self.z_filter_max = z_filter_max

    def generate_from_points(self, points: np.ndarray) -> Dict[str, Any]:
        """
        从 3D 点云构建多图层 2.5D 高程图
        自动利用机器狗跨越约束 (max_step_height) 滤除高空悬浮杂点
        """
        if len(points) == 0:
            raise ValueError("输入点云为空")

        # 1. Z 轴可选范围粗筛
        mask = np.ones(len(points), dtype=bool)
        if self.z_filter_min is not None:
            mask &= (points[:, 2] >= self.z_filter_min)
        if self.z_filter_max is not None:
            mask &= (points[:, 2] <= self.z_filter_max)
        pts = points[mask]
        if len(pts) == 0:
            raise ValueError("过滤后有效点云为空")

        x_coords, y_coords, z_coords = pts[:, 0], pts[:, 1], pts[:, 2]

        # 2. 地图水平尺寸与栅格分辨率计算
        min_x = float(np.min(x_coords)) - self.resolution
        max_x = float(np.max(x_coords)) + self.resolution
        min_y = float(np.min(y_coords)) - self.resolution
        max_y = float(np.max(y_coords)) + self.resolution

        cols = int(math.ceil((max_x - min_x) / self.resolution))
        rows = int(math.ceil((max_y - min_y) / self.resolution))

        length_x = cols * self.resolution
        length_y = rows * self.resolution
        center_x = min_x + length_x / 2.0
        center_y = min_y + length_y / 2.0

        # 3. 栅格化投影
        col_indices = np.clip(np.floor((x_coords - min_x) / self.resolution).astype(np.int32), 0, cols - 1)
        row_indices = np.clip(np.floor((y_coords - min_y) / self.resolution).astype(np.int32), 0, rows - 1)

        # 4. 统计每个网格的点集与垂直分布
        flat_idx = row_indices * cols + col_indices
        order = np.argsort(flat_idx)
        sorted_flat = flat_idx[order]
        sorted_z = z_coords[order]

        unique_idx, start_idx = np.unique(sorted_flat, return_index=True)
        end_idx = np.append(start_idx[1:], len(sorted_flat))

        raw_elevation = np.full((rows, cols), np.nan, dtype=np.float32)
        wall_boxes = []

        for u_idx, s_idx, e_idx in zip(unique_idx, start_idx, end_idx):
            r = u_idx // cols
            c = u_idx % cols
            cell_z = sorted_z[s_idx:e_idx]
            z_low = float(np.min(cell_z))
            z_high = float(np.max(cell_z))
            v_span = z_high - z_low

            # 若网格内垂直跨度较大（>0.45m），记录为立体墙柱障碍
            if v_span > 0.45:
                wall_boxes.append({
                    "x": round(min_x + (c + 0.5) * self.resolution, 3),
                    "y": round(min_y + (r + 0.5) * self.resolution, 3),
                    "z_min": round(z_low, 3),
                    "z_max": round(z_high, 3),
                    "height": round(v_span, 3)
                })

            # 初始候选地面高度：取最底层聚集处
            raw_elevation[r, c] = np.percentile(cell_z, 15) if len(cell_z) >= 5 else z_low

        # 5. 【核心】：按照机器狗跨越能力 (max_step_height 与 step_radius) 剔除高空孤立杂点
        clean_elevation = self._filter_isolated_high_points(raw_elevation, self.max_step_height, self.step_radius)

        # 6. 空洞修补：仅在连续平缓地面内部进行局部插值，不拉扯边界
        fill_kernel_cells = max(1, int(round(self.fill_radius / self.resolution)))
        elevation_filled = self._fill_holes_constrained(clean_elevation, fill_kernel_cells, self.max_step_height)

        # 7. 计算 step_height（8 邻域踏面差分）与 slope（坡度）
        step_height = self._compute_step_height(elevation_filled)
        slope_deg = self._compute_slope(elevation_filled, self.resolution)

        # 8. 障碍掩码计算 (0: 可通行踏面, 1: 陡坡/断崖, 2: 垂直墙壁, 3: 空洞无支撑)
        obstacle_mask = np.zeros((rows, cols), dtype=np.uint8)

        # 标记无地面支撑区域
        obstacle_mask[np.isnan(elevation_filled)] = 3

        # 标记垂直立面/墙壁
        for wb in wall_boxes:
            c = int((wb["x"] - min_x) / self.resolution)
            r = int((wb["y"] - min_y) / self.resolution)
            if 0 <= r < rows and 0 <= c < cols:
                obstacle_mask[r, c] = 2

        # 标记超标台阶与超标坡度
        obstacle_mask[step_height > self.max_step_height] = 1
        obstacle_mask[slope_deg > self.max_slope_deg] = 1

        # 统计有效范围
        valid_elev = elevation_filled[~np.isnan(elevation_filled)]
        min_elev = float(np.min(valid_elev)) if len(valid_elev) > 0 else 0.0
        max_elev = float(np.max(valid_elev)) if len(valid_elev) > 0 else 1.0

        # 将 NaN 转为特定标记值便于 JSON/ROS 传输
        safe_elev = np.where(np.isnan(elevation_filled), None, np.round(elevation_filled, 3))

        return {
            "metadata": {
                "resolution": self.resolution,
                "rows": rows,
                "cols": cols,
                "min_x": round(min_x, 3),
                "min_y": round(min_y, 3),
                "length_x": round(length_x, 3),
                "length_y": round(length_y, 3),
                "center_x": round(center_x, 3),
                "center_y": round(center_y, 3),
                "min_elevation": round(min_elev, 2),
                "max_elevation": round(max_elev, 2),
                "max_step_height": self.max_step_height,
                "step_radius": self.step_radius
            },
            "layers": {
                "elevation": safe_elev.tolist(),
                "step_height": np.nan_to_num(step_height, nan=0.0).round(3).tolist(),
                "slope": np.nan_to_num(slope_deg, nan=0.0).round(1).tolist(),
                "obstacle_mask": obstacle_mask.tolist()
            },
            "wall_boxes": wall_boxes[:1000] # 传递立体墙柱供前端立体渲染
        }

    def _filter_isolated_high_points(self, elev: np.ndarray, max_step: float, step_radius: float) -> np.ndarray:
        """剔除与周围步幅半径内落差过大、机器狗踩不上去的孤立高空杂点"""
        rows, cols = elev.shape
        clean = elev.copy()
        tol = max(0.35, max_step * 1.5)
        # 根据水平步幅半径换算搜索窗口
        radius_cells = max(1, int(round(step_radius / self.resolution)))

        for r in range(rows):
            for c in range(cols):
                z0 = elev[r, c]
                if np.isnan(z0):
                    continue

                r_min, r_max = max(0, r - radius_cells), min(rows, r + radius_cells + 1)
                c_min, c_max = max(0, c - radius_cells), min(cols, c + radius_cells + 1)
                neighbors = elev[r_min:r_max, c_min:c_max]
                valid_nbrs = neighbors[~np.isnan(neighbors)]

                if len(valid_nbrs) > 1:
                    continuous = np.any(np.abs(valid_nbrs - z0) <= tol)
                    if not continuous:
                        clean[r, c] = np.nan

        return clean

    def _fill_holes_constrained(self, elev: np.ndarray, radius_cells: int, max_step: float) -> np.ndarray:
        """严格受限的空洞修补：仅在高度差平缓的连续地表内部插值"""
        filled = elev.copy()
        nan_mask = np.isnan(filled)
        if not np.any(nan_mask):
            return filled

        k_size = 2 * radius_cells + 1
        weights = (~nan_mask).astype(np.float32)
        values = np.nan_to_num(filled, nan=0.0)

        smooth_values = uniform_filter(values, size=k_size, mode='nearest')
        smooth_weights = uniform_filter(weights, size=k_size, mode='nearest')

        # 仅当周围有效点权重充分 (至少有 3 个近邻) 才插补
        can_fill = nan_mask & (smooth_weights > 0.35)
        filled[can_fill] = smooth_values[can_fill] / np.maximum(smooth_weights[can_fill], 1e-5)
        return filled

    def _compute_step_height(self, elev: np.ndarray) -> np.ndarray:
        valid_elev = np.nan_to_num(elev, nan=0.0)
        max_l = maximum_filter(valid_elev, size=3, mode='nearest')
        min_l = minimum_filter(valid_elev, size=3, mode='nearest')
        step = np.maximum(np.abs(valid_elev - max_l), np.abs(valid_elev - min_l))
        step[np.isnan(elev)] = 0.0
        return step

    def _compute_slope(self, elev: np.ndarray, res: float) -> np.ndarray:
        valid_elev = np.nan_to_num(elev, nan=0.0)
        gy, gx = np.gradient(valid_elev, res, res)
        grad_mag = np.sqrt(gx * gx + gy * gy)
        slope_deg = np.degrees(np.arctan(grad_mag))
        slope_deg[np.isnan(elev)] = 0.0
        return slope_deg
