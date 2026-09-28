# -*- coding: utf-8 -*-
"""
PCD 点云高效加载与下采样预览模块
基于 Open3D 与 NumPy，实现点云统计分析、包围盒提取与轻量化预览采样
"""

import os
from pathlib import Path
from typing import Dict, Any, Optional
import numpy as np
import open3d as o3d


class PCDLoader:
    @staticmethod
    def inspect_and_preview(
        pcd_path: str,
        target_preview_points: int = 15000,
        z_min: Optional[float] = None,
        z_max: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        快速分析 PCD 文件并提取下采样的点云数据供前端 3D 预览
        """
        path = Path(pcd_path).expanduser().resolve()
        if not path.is_file():
            return {
                "success": False,
                "error": f"找不到点云文件: {path}"
            }

        try:
            # 1. 使用 Open3D 读取点云
            cloud = o3d.io.read_point_cloud(str(path))
            if cloud.is_empty():
                return {
                    "success": False,
                    "error": "点云文件为空或格式不受支持"
                }

            pts = np.asarray(cloud.points, dtype=np.float32)
            total_points = len(pts)
            if total_points == 0:
                return {
                    "success": False,
                    "error": "未读取到有效点"
                }

            # 2. 高度 Z 轴初步过滤（若指定）
            if z_min is not None:
                pts = pts[pts[:, 2] >= z_min]
            if z_max is not None:
                pts = pts[pts[:, 2] <= z_max]

            # 3. 统计包围盒
            min_bound = np.min(pts, axis=0).tolist()
            max_bound = np.max(pts, axis=0).tolist()
            span = [
                round(max_bound[0] - min_bound[0], 2),
                round(max_bound[1] - min_bound[1], 2),
                round(max_bound[2] - min_bound[2], 2)
            ]

            # 4. 下采样至 target_preview_points 以便前端轻量流畅渲染
            if len(pts) > target_preview_points:
                step = max(1, len(pts) // target_preview_points)
                sampled_pts = pts[::step][:target_preview_points]
            else:
                sampled_pts = pts

            # 展平为一维数组传递 [x0, y0, z0, x1, y1, z1, ...]
            flat_pts = sampled_pts.flatten().round(3).tolist()

            file_size_mb = round(path.stat().st_size / (1024 * 1024), 2)

            return {
                "success": True,
                "file_path": str(path),
                "file_name": path.name,
                "file_size_mb": file_size_mb,
                "total_points": total_points,
                "preview_points_count": len(sampled_pts),
                "min_bound": [round(v, 2) for v in min_bound],
                "max_bound": [round(v, 2) for v in max_bound],
                "span": span,
                "points": flat_pts
            }

        except Exception as e:
            return {
                "success": False,
                "error": f"读取 PCD 异常: {str(e)}"
            }

    @staticmethod
    def load_full_points(pcd_path: str) -> np.ndarray:
        """加载完整点云为 Nx3 float32 数组"""
        path = Path(pcd_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"PCD 文件不存在: {path}")

        cloud = o3d.io.read_point_cloud(str(path))
        pts = np.asarray(cloud.points, dtype=np.float32)
        return pts
