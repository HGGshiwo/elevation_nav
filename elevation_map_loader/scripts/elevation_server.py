#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2.5D 高程图 Web 可视化管理服务端
基于 FastAPI 提供 HTTP / REST 接口与 WebSocket 全双工长连接，连接 Three.js 前端与 ROS 导航
"""

import os
import sys
import json
import asyncio
from pathlib import Path
from typing import Dict, Any, Optional
from concurrent.futures import ThreadPoolExecutor

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from pydantic import BaseModel

# 确保本包 python 模块在 sys.path 中
package_root = Path(__file__).resolve().parent.parent
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from elevation_nav.pcd_loader import PCDLoader
from elevation_nav.elevation_generator import ElevationMapGenerator
from elevation_nav.ros_bridge import ros_bridge

app = FastAPI(title="Elevation Map Navigation Web Server")
app.add_middleware(GZipMiddleware, minimum_size=1000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

executor = ThreadPoolExecutor(max_workers=4)

async def run_in_thread(func, *args, **kwargs):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(executor, lambda: func(*args, **kwargs))

# 静态资源挂载
static_dir = package_root / "web" / "static"
app.mount("/static", StaticFiles(directory=static_dir, html=True), name="static")

# 全局高程图缓存
latest_grid_map: Optional[Dict[str, Any]] = None
latest_grid_map_json: Optional[str] = None


@app.get("/")
def read_index():
    return FileResponse(static_dir / "index.html")


class PCDPreviewRequest(BaseModel):
    pcd_path: str
    target_points: int = 15000
    z_min: Optional[float] = None
    z_max: Optional[float] = None


class PCDGenerateRequest(BaseModel):
    pcd_path: str
    resolution: float = 0.10
    max_step_height: float = 0.25
    step_radius: float = 0.25
    max_slope_deg: float = 30.0
    fill_radius: float = 0.20
    z_min: Optional[float] = None
    z_max: Optional[float] = None


class NavGoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


class LoadPcdRequest(BaseModel):
    pcd_path: str
    map_config_path: Optional[str] = None


@app.on_event("startup")
def startup_event():
    try:
        ros_bridge.init_ros()
    except Exception as e:
        print(f"[Startup] ROS 初始化提示: {e}")


@app.post("/api/pcd/preview")
async def preview_pcd(req: PCDPreviewRequest):
    """分析 PCD 文件并返回轻量采样点供前端弹窗 3D 预览"""
    def do_preview():
        return PCDLoader.inspect_and_preview(
            pcd_path=req.pcd_path,
            target_preview_points=req.target_points,
            z_min=req.z_min,
            z_max=req.z_max
        )
    res = await run_in_thread(do_preview)
    if not res.get("success", False):
        raise HTTPException(status_code=400, detail=res.get("error", "预览点云失败"))
    return JSONResponse(content=res)


@app.post("/api/pcd/generate")
async def generate_grid_map(req: PCDGenerateRequest):
    """从 PCD 生成 2.5D 高程图，发布到 ROS 并下发给前端"""
    global latest_grid_map, latest_grid_map_json

    def process():
        pts = PCDLoader.load_full_points(req.pcd_path)
        gen = ElevationMapGenerator(
            resolution=req.resolution,
            max_step_height=req.max_step_height,
            step_radius=req.step_radius,
            max_slope_deg=req.max_slope_deg,
            fill_radius=req.fill_radius,
            z_filter_min=req.z_min,
            z_filter_max=req.z_max
        )
        return gen.generate_from_points(pts)

    try:
        grid_map_result = await run_in_thread(process)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"生成高程图失败: {str(e)}")

    response_payload = {
        "metadata": grid_map_result["metadata"],
        "layers": grid_map_result["layers"],
        "wall_boxes": grid_map_result.get("wall_boxes", [])
    }

    latest_grid_map = response_payload
    latest_grid_map_json = json.dumps(response_payload)

    # 发布到 ROS 话题
    ros_bridge.publish_grid_map(grid_map_result)

    return JSONResponse(content=response_payload)


@app.get("/api/grid_map/current")
async def get_current_grid_map():
    """获取当前已生成的高程图"""
    if latest_grid_map is None:
        return JSONResponse(content={"has_map": False})
    return JSONResponse(content={"has_map": True, "data": latest_grid_map})


@app.post("/api/nav/set_start")
@app.post("/api/set_start")
def set_nav_start(req: NavGoalRequest):
    """设置机器人起点/吸附位姿"""
    ros_bridge.publish_initial_pose(req.x, req.y, req.z, req.yaw)
    data = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    return {"status": "ok", "start": data}


@app.post("/api/nav/set_goal")
@app.post("/api/set_goal")
def set_nav_goal(req: NavGoalRequest):
    """设置导航目标点"""
    ros_bridge.publish_goal(req.x, req.y, req.z, req.yaw)
    data = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    return {"status": "ok", "goal": data}


@app.post("/api/nav/cancel_goal")
def cancel_nav_goal():
    """取消当前导航"""
    ros_bridge.cancel_navigation()
    return {"status": "cancelled"}


@app.get("/api/nav/robot_model")
def get_robot_model():
    """返回理论建模的机器人几何尺寸 (与 planner_common.yaml 同源, 供前端按真实包络渲染胶囊体+足印圆盘)"""
    def read_param(names, default):
        for name in names:
            try:
                value = rospy.get_param(name)
                if value is not None:
                    return float(value)
            except Exception:
                continue
        return default

    return {
        "dog_height": read_param(
            ["/move_base/ElevationGlobalPlanner/dog_height"], 0.45),
        "body_hard_radius": read_param(
            ["/move_base/ElevationGlobalPlanner/body_hard_radius"], 0.15),
        "footprint_radius": read_param(
            ["/move_base/ElevationGlobalPlanner/footprint_radius"], 0.30),
        "max_step_height": read_param(
            ["/move_base/ElevationGlobalPlanner/max_step_height"], 0.25),
    }


@app.get("/api/nav/diagnose_edge")
def diagnose_edge(x1: float, y1: float, z1: float, x2: float, y2: float, z2: float):
    """请求 C++ 规划器节点权威诊断两踏面方块的拓扑邻边关系与物理原因"""
    res = ros_bridge.diagnose_edge(x1, y1, z1, x2, y2, z2)
    return JSONResponse(content=res)


@app.get("/api/nav/diagnose_node")
def diagnose_node(x: float, y: float, z: float):
    """请求 C++ 规划器节点权威诊断单个踏面方块的通行状态与禁行原因"""
    res = ros_bridge.diagnose_node(x, y, z)
    return JSONResponse(content=res)


@app.post("/api/load_pcd")
def load_pcd(req: LoadPcdRequest):
    """向 C++ 节点发送加载 PCD 文件指令并自动提取高程图"""
    if not os.path.exists(req.pcd_path):
        raise HTTPException(status_code=400, detail=f"PCD 文件不存在: {req.pcd_path}")
    cmd = req.pcd_path
    if req.map_config_path:
        if not os.path.exists(req.map_config_path):
            raise HTTPException(status_code=400, detail=f"地图配置文件不存在: {req.map_config_path}")
        cmd = f"{req.pcd_path};{req.map_config_path}"
    ros_bridge.publish_pcd_cmd(cmd)
    return {"status": "ok", "message": f"已向 C++ 节点发送 PCD 加载指令: {cmd}"}


@app.websocket("/ws/live")
async def websocket_live(websocket: WebSocket):
    """实时推送机器人位姿、导航规划路径、流形拓扑图与 C++ 节点发布的 GridMap"""
    await websocket.accept()
    last_path_v = -1
    last_nodes_v = -1
    last_edges_v = -1
    last_corridor_v = -1
    last_teb_obstacles_v = -1
    last_costmap_v = -1
    last_debug_v = -1
    last_debug_nodes_v = -1
    try:
        while True:
            state = ros_bridge.get_live_state()
            frame = {
                "robot_pose": state["robot_pose"],
                "local_path": state["local_path"]
            }
            if state["path_version"] != last_path_v:
                frame["global_path"] = state["global_path"]
                frame["path_version"] = state["path_version"]
                last_path_v = state["path_version"]

            if state.get("graph_nodes_version", 0) != last_nodes_v and state.get("graph_nodes"):
                frame["graph_nodes"] = state["graph_nodes"]
                frame["graph_nodes_version"] = state["graph_nodes_version"]
                last_nodes_v = state["graph_nodes_version"]

            if state.get("graph_edges_version", 0) != last_edges_v and state.get("graph_edges"):
                frame["graph_edges"] = state["graph_edges"]
                frame["graph_edges_version"] = state["graph_edges_version"]
                last_edges_v = state["graph_edges_version"]

            if state.get("corridor_nodes_version", 0) != last_corridor_v:
                frame["corridor_nodes"] = state.get("corridor_nodes", [])
                frame["corridor_lines"] = state.get("corridor_lines", [])
                frame["corridor_walls"] = state.get("corridor_walls", [])
                frame["corridor_nodes_version"] = state["corridor_nodes_version"]
                last_corridor_v = state["corridor_nodes_version"]

            if state.get("teb_obstacles_version", 0) != last_teb_obstacles_v:
                frame["teb_obstacles"] = state.get("teb_obstacles", [])
                frame["teb_obstacles_version"] = state["teb_obstacles_version"]
                last_teb_obstacles_v = state["teb_obstacles_version"]

            if state.get("local_costmap_version", 0) != last_costmap_v and state.get("local_costmap"):
                frame["local_costmap"] = state["local_costmap"]
                last_costmap_v = state["local_costmap_version"]

            if state.get("local_costmap_debug_version", 0) != last_debug_v and state.get("local_costmap_debug"):
                frame["local_costmap_debug"] = state["local_costmap_debug"]
                last_debug_v = state["local_costmap_debug_version"]

            if state.get("local_costmap_debug_nodes_version", 0) != last_debug_nodes_v and state.get("local_costmap_debug_nodes"):
                frame["local_costmap_debug_nodes"] = state["local_costmap_debug_nodes"]
                last_debug_nodes_v = state["local_costmap_debug_nodes_version"]

            await websocket.send_text(json.dumps(frame))
            await asyncio.sleep(0.1)  # 10Hz 稳定推送
    except (WebSocketDisconnect, Exception):
        pass


def main():
    import rospy
    ros_bridge.init_ros()
    port = rospy.get_param("~web_port", 8080)
    print(f"==================================================")
    print(f"  Elevation Nav Web Server running on : {port}")
    print(f"  Open in browser: http://localhost:{port}")
    print(f"==================================================")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
