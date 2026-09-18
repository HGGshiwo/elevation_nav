import * as THREE from 'three';
import { initScene } from '../scene.js';
import { LayerManager } from '../layer.js';
import { GraphVisualizer } from './graph_visualizer.js';
import { CorridorVisualizer } from './corridor_visualizer.js';
import { initEditor } from './editor.js';
import { initRobotTracker } from './robot_tracker.js';
import { initMapStorage } from './map_storage.js';
import { initWsStream } from './ws_stream.js';

// ---- 1. 场景初始化 ----
const container = document.getElementById('canvas-container');
const { scene, camera, renderer, controls, roamController, editPlane, setFrameCallback } = initScene(container);
renderer.domElement.addEventListener('contextmenu', e => e.preventDefault());

// ---- 2. 高性能图层管理器初始化 ----
const layers = {
    occupied: new LayerManager(scene, "occupied", [0.4, 0.4, 0.4]),
    preblocked: new LayerManager(scene, "preblocked", [0.4, 0.4, 0.4]),
    traversable: new LayerManager(scene, "traversable", [0.4, 0.4, 0.4]),
    risk_cost: new LayerManager(scene, "risk_cost", [0.4, 0.4, 0.4]),
    local_octomap: new LayerManager(scene, "local_octomap", [0.2, 0.2, 0.2]),
    fused_octomap: new LayerManager(scene, "fused_octomap", [0.2, 0.2, 0.2]),
    emergency_stop_free: new LayerManager(scene, "emergency_stop_free", [0.06, 0.06, 0.06]),
    emergency_stop_occupied: new LayerManager(scene, "emergency_stop_occupied", [0.10, 0.10, 0.10])
};

// 默认仅显示禁行与可通行图层
layers.preblocked.mesh.visible = true;
layers.traversable.mesh.visible = true;
layers.occupied.mesh.visible = true;
layers.risk_cost.mesh.visible = false;
layers.local_octomap.mesh.visible = false;
layers.fused_octomap.mesh.visible = false;
layers.emergency_stop_free.mesh.visible = false;
layers.emergency_stop_occupied.mesh.visible = false;

// 3D 流形拓扑图渲染器 (点云直通踏面与连通网格)
const graphVisualizer = new GraphVisualizer(scene, camera, controls);

// 3D 拓扑流形管道渲染器 (基于 A* 路径与连通图的 3m 运动安全走廊)
const corridorVisualizer = new CorridorVisualizer(scene);

// 获取当前已勾选需流式同步的图层列表
function getActiveRequestedLayers() {
    const requested = [];
    if (document.getElementById('show-preblocked')?.checked) requested.push('preblocked');
    if (document.getElementById('show-traversable')?.checked) requested.push('traversable');
    if (document.getElementById('show-risk-cost')?.checked) requested.push('risk_cost');
    if (document.getElementById('show-octomap-local')?.checked) requested.push('local_octomap');
    if (document.getElementById('show-octomap-fused')?.checked) requested.push('fused_octomap');
    if (document.getElementById('show-emergency-stop')?.checked) {
        requested.push('emergency_stop_free');
        requested.push('emergency_stop_occupied');
    }
    return requested;
}

// ---- 3. 核心功能模块组装 ----
// 3.1 机器狗追踪与路径模块 (跟随视角驱动挂入渲染循环)
const robotTracker = initRobotTracker(scene, controls);
if (setFrameCallback) setFrameCallback(() => robotTracker.updateFollow());

// 3.2 地图持久化存储与 ROS 同步模块
const mapStorage = initMapStorage(layers, () => {
    if (wsStream) wsStream.sendSubscription();
});

// 3.3 栅格与交互编辑模块 (接入 3D 流形吸附)
const editor = initEditor(scene, camera, renderer, controls, layers, editPlane, (layerName) => {
    mapStorage.markDirty(layerName);
}, graphVisualizer, roamController);

// 3.4 WebSocket 全双工流式同步模块 (支持流形图、3D拓扑管道与点云直通数据)
const wsStream = initWsStream(layers, robotTracker, getActiveRequestedLayers, graphVisualizer, corridorVisualizer);

// ---- 4. 图层显隐开关与 WebSocket 订阅联动 ----
// 4.0 3D 拓扑流形管道与内部几何障碍物显隐控制
document.getElementById('show-corridor')?.addEventListener('change', (e) => {
    corridorVisualizer.setVisible(e.target.checked);
});

document.getElementById('show-teb-obstacles')?.addEventListener('change', (e) => {
    corridorVisualizer.setObstaclesVisible(e.target.checked);
});

// ---- 4. 图层显隐开关与 WebSocket 订阅联动 ----
function bindLayerToggle(id, layerObj) {
    document.getElementById(id)?.addEventListener('change', (e) => {
        if (layerObj.mesh) layerObj.mesh.visible = e.target.checked;
        if (layerObj.setVisible) layerObj.setVisible(e.target.checked);
        if (wsStream) wsStream.sendSubscription();
    });
}

// 4.1 禁行(红)控制：联动控制人工禁行体素与 3D 流形低净空/阻挡踏面节点
document.getElementById('show-preblocked')?.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    if (layers.preblocked?.mesh) layers.preblocked.mesh.visible = isChecked;
    graphVisualizer.setBlockedVisible(isChecked);
    if (wsStream) wsStream.sendSubscription();
});

// 4.2 可通行(绿)控制：联动控制可通行体素与 3D 流形踏面节点及步态连通网格
document.getElementById('show-traversable')?.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    if (layers.traversable?.mesh) layers.traversable.mesh.visible = isChecked;
    graphVisualizer.setTraversableVisible(isChecked);
    if (wsStream) wsStream.sendSubscription();
});

bindLayerToggle('show-risk-cost', layers.risk_cost);
bindLayerToggle('show-octomap-local', layers.local_octomap);
bindLayerToggle('show-octomap-fused', layers.fused_octomap);

document.getElementById('show-emergency-stop')?.addEventListener('change', (e) => {
    layers.emergency_stop_free.mesh.visible = e.target.checked;
    layers.emergency_stop_occupied.mesh.visible = e.target.checked;
    if (wsStream) wsStream.sendSubscription();
});
