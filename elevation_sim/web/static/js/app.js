import * as THREE from 'three';
import { initScene } from '../scene.js';
import { LayerManager } from '../layer.js';
import { GraphVisualizer } from './graph_visualizer.js';
import { CorridorVisualizer } from './corridor_visualizer.js';
import { SfcDebugVisualizer } from './sfc_debug_visualizer.js';
import { InjectedObstacleVisualizer } from './obstacle_visualizer.js';
import { DynamicNodeVisualizer } from './dynamic_nodes_visualizer.js';
import { ReboundVisualizer } from './rebound_visualizer.js';
import { FloorMapVisualizer } from './floor_map_visualizer.js';
import { initEditor } from './editor.js';
import { initRobotTracker } from './robot_tracker.js';
import { initWsStream } from './ws_stream.js';
import { makeDraggable } from './draggable.js';

// ---- 1. 场景初始化 ----
const container = document.getElementById('canvas-container');
const { scene, camera, renderer, controls, roamController, editPlane, setFrameCallback } = initScene(container);
renderer.domElement.addEventListener('contextmenu', e => e.preventDefault());

// 面板支持鼠标拖拽
makeDraggable(document.getElementById('ui-panel'), document.getElementById('ui-panel-header'));
makeDraggable(document.getElementById('debug-panel'), document.querySelector('#debug-panel .debug-title'));
makeDraggable(document.getElementById('sfc-debug-panel'), document.querySelector('#sfc-debug-panel .debug-title'));

// ---- 2. 高性能图层管理器初始化 ----
const layers = {
    occupied: new LayerManager(scene, "occupied", [0.4, 0.4, 0.4]),
    preblocked: new LayerManager(scene, "preblocked", [0.4, 0.4, 0.4]),
    traversable: new LayerManager(scene, "traversable", [0.4, 0.4, 0.4]),
    risk_cost: new LayerManager(scene, "risk_cost", [0.4, 0.4, 0.4])
};

// 默认仅显示禁行与可通行图层
layers.preblocked.mesh.visible = true;
layers.traversable.mesh.visible = true;
layers.occupied.mesh.visible = true;
layers.risk_cost.mesh.visible = false;

// 注入障碍物真值渲染器 + 融合动态标记节点渲染器
const obstacleVisualizer = new InjectedObstacleVisualizer(scene);
const dynamicNodeVisualizer = new DynamicNodeVisualizer(scene);
const reboundVisualizer = new ReboundVisualizer(scene);

// 3D 流形拓扑图渲染器 (点云直通踏面与连通网格)
const graphVisualizer = new GraphVisualizer(scene, camera, controls);

// 2D 分层踏面地图与精准 1-to-1 空间定位可视化器
const floorMapVisualizer = new FloorMapVisualizer(graphVisualizer);

// 3D 拓扑管道渲染器 (SFC 凸走廊边界线框, 默认显示)
const corridorVisualizer = new CorridorVisualizer(scene);

// 3D 可通行走廊诊断与单步扩散调试渲染器
const sfcDebugVisualizer = new SfcDebugVisualizer(scene, camera, controls);

// 获取当前已勾选需流式同步的图层列表
function getActiveRequestedLayers() {
    const requested = [];
    if (document.getElementById('show-preblocked')?.checked) requested.push('preblocked');
    if (document.getElementById('show-traversable')?.checked) requested.push('traversable');
    return requested;
}

// ---- 3. 核心功能模块组装 ----
// 3.1 机器狗追踪与路径模块
const robotTracker = initRobotTracker(scene, controls);
if (setFrameCallback) setFrameCallback(() => {
    robotTracker.updateFollow();
    if (robotTracker.updateCollisionAnimation) robotTracker.updateCollisionAnimation();
});

// 3.2 交互与调试控制模块 (接入 3D 流形吸附)
const editor = initEditor(scene, camera, renderer, controls, layers, editPlane, null, graphVisualizer, roamController);

// ---- 3.3 SFC 可通行走廊诊断 UI 控制与数据绑定 ----
let currentSfcCorridors = [];

function updateSfcDebugPanel(corridors) {
    currentSfcCorridors = corridors || [];
    const listEl = document.getElementById('sfc-points-list');
    const totalCountEl = document.getElementById('sfc-total-count');
    if (!listEl) return;

    if (totalCountEl) totalCountEl.textContent = `${currentSfcCorridors.length} 个点`;

    if (currentSfcCorridors.length === 0) {
        listEl.innerHTML = '<div style="color: #888; font-size: 12px; text-align: center; padding: 20px 0;">等待局部规划走廊数据...</div>';
        return;
    }

    listEl.innerHTML = '';
    currentSfcCorridors.forEach((corr, idx) => {
        const card = document.createElement('div');
        card.className = 'sfc-card';
        card.dataset.idx = idx;

        const isViolated = corr.is_violated;
        const crossLayer = corr.is_cross_layer;
        const colorHex = isViolated ? '#ff5252' : (crossLayer ? '#ffab40' : '#69f0ae');

        card.innerHTML = `
            <div style="display:flex; justify-content:space-between; align-items:center;">
                <span style="font-weight:bold; font-size:12px; color:#fff;">#${idx} (${corr.ref_x.toFixed(2)}, ${corr.ref_y.toFixed(2)})</span>
                <span class="sfc-badge" style="background:${isViolated ? '#b71c1c' : '#1b5e20'}; color:#fff;">
                    ${isViolated ? '违约' : '合规'}
                </span>
            </div>
            <div style="font-size:11px; color:#aaa; margin-top:2px;">
                扩散节点: <span style="color:#00e5ff;">${corr.total_nodes}</span> | 高程: <span style="color:${colorHex};">${corr.ref_z.toFixed(2)}m</span>
            </div>
        `;

        card.addEventListener('click', () => {
            document.querySelectorAll('.sfc-card').forEach(c => c.classList.remove('active'));
            card.classList.add('active');
            sfcDebugVisualizer.highlightPoint(idx);
            showSfcPointDetail(idx);
        });

        listEl.appendChild(card);
    });
}

function showSfcPointDetail(idx) {
    const corr = currentSfcCorridors[idx];
    if (!corr) return;

    const detailBox = document.getElementById('sfc-detail-box');
    if (detailBox) detailBox.style.display = 'block';

    document.getElementById('sfc-sel-idx').textContent = idx;
    document.getElementById('sfc-sel-id').textContent = corr.root_id || '-';
    document.getElementById('sfc-sel-xyz').textContent = `${corr.ref_x.toFixed(2)}, ${corr.ref_y.toFixed(2)}, ${corr.ref_z.toFixed(2)}`;
    document.getElementById('sfc-sel-opt-p').textContent = `${corr.opt_x.toFixed(2)}, ${corr.opt_y.toFixed(2)}, ${corr.opt_z.toFixed(2)}`;

    const violEl = document.getElementById('sfc-sel-viol');
    if (violEl) {
        violEl.textContent = corr.is_violated ? '严重违约 (穿透走廊)' : '合规约束';
        violEl.style.color = corr.is_violated ? '#ff1744' : '#69f0ae';
    }

    document.getElementById('sfc-sel-node-count').textContent = corr.total_nodes;

    const crossWarn = document.getElementById('sfc-sel-cross-warning');
    if (crossWarn) crossWarn.style.display = corr.is_cross_layer ? 'block' : 'none';

    const tableEl = document.getElementById('sfc-node-table');
    if (tableEl && corr.nodes) {
        tableEl.innerHTML = corr.nodes.map(n => `
            <div style="display:flex; justify-content:space-between; padding:2px 0; border-bottom:1px solid #333;">
                <span>#${n.id} (${n.x.toFixed(2)}, ${n.y.toFixed(2)}, ${n.z.toFixed(2)})</span>
                <span style="color:${n.trav > 0.05 ? '#ffab40' : '#69f0ae'};">d=${n.dist.toFixed(2)}m</span>
            </div>
        `).join('');
    }
}

// 3.4 WebSocket 全双工流式同步模块
const wsStream = initWsStream(
    layers,
    robotTracker,
    getActiveRequestedLayers,
    graphVisualizer,
    corridorVisualizer,
    sfcDebugVisualizer,
    (corridors) => updateSfcDebugPanel(corridors),
    obstacleVisualizer,
    dynamicNodeVisualizer,
    reboundVisualizer
);

// ---- 4. 图层显隐开关与 WebSocket 订阅联动 ----
document.getElementById('show-corridor')?.addEventListener('change', (e) => {
    corridorVisualizer.setVisible(e.target.checked);
});

document.getElementById('show-sfc-debug')?.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    sfcDebugVisualizer.setVisible(isChecked);
    const panel = document.getElementById('sfc-debug-panel');
    if (panel) panel.style.display = isChecked ? 'flex' : 'none';
});

document.getElementById('show-preblocked')?.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    if (layers.preblocked?.mesh) layers.preblocked.mesh.visible = isChecked;
    graphVisualizer.setBlockedVisible(isChecked);
    if (wsStream) wsStream.sendSubscription();
});

document.getElementById('show-traversable')?.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    if (layers.traversable?.mesh) layers.traversable.mesh.visible = isChecked;
    graphVisualizer.setTraversableVisible(isChecked);
    if (wsStream) wsStream.sendSubscription();
});

document.getElementById('show-injected-obstacles')?.addEventListener('change', (e) => {
    obstacleVisualizer.setVisible(e.target.checked);
});

document.getElementById('show-rebound')?.addEventListener('change', (e) => {
    reboundVisualizer.setVisible(e.target.checked);
});
