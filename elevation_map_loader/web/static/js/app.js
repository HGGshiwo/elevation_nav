import * as THREE from 'three';
import { initScene } from '../scene.js';
import { LayerManager } from '../layer.js';
import { GraphVisualizer } from './graph_visualizer.js';
import { CorridorVisualizer } from './corridor_visualizer.js';
import { SfcDebugVisualizer } from './sfc_debug_visualizer.js';
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

// 3D 可通行走廊诊断与单步扩散调试渲染器
const sfcDebugVisualizer = new SfcDebugVisualizer(scene, camera, controls);

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

// ---- 3.4 SFC 可通行走廊诊断 UI 控制与数据绑定 ----
let currentSfcCorridors = [];

function updateSfcDebugPanel(corridors) {
    currentSfcCorridors = corridors || [];
    const listEl = document.getElementById('sfc-points-list');
    const totalCountEl = document.getElementById('sfc-total-count');
    if (!listEl) return;

    if (totalCountEl) totalCountEl.textContent = `${currentSfcCorridors.length} 个点`;

    if (currentSfcCorridors.length === 0) {
        listEl.innerHTML = '<div style="color: #888; font-size: 12px; text-align: center; padding: 20px 0;">等待局部规划走廊数据...</div>';
        const detailBox = document.getElementById('sfc-detail-box');
        if (detailBox) detailBox.style.display = 'none';
        return;
    }

    listEl.innerHTML = '';
    currentSfcCorridors.forEach((c, idx) => {
        const rootLayer = c.layer !== undefined ? c.layer : 0;
        const nodes = c.nodes || [];
        const hasCrossLayer = nodes.some(n => n.l !== undefined && n.l !== rootLayer);

        const card = document.createElement('div');
        card.className = `sfc-card ${idx === sfcDebugVisualizer.selectedIndex ? 'active' : ''}`;
        card.id = `sfc-card-${idx}`;
        
        let warningBadge = hasCrossLayer ? `<span class="sfc-badge" style="background:#d50000;color:#fff;">⚠ 跨层扩散</span>` : '';

        card.innerHTML = `
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:3px;">
                <span style="font-weight:bold; color:${idx === sfcDebugVisualizer.selectedIndex ? '#ffd600' : '#00e5ff'}; font-size:12px;">
                    #${idx} <span style="color:#aaa;">(nid: ${c.root_id})</span>
                </span>
                <div>
                    <span class="sfc-badge" style="background:#37474f; color:#80d8ff;">L${rootLayer}</span>
                    ${warningBadge}
                </div>
            </div>
            <div style="font-size:11px; color:#bbb; display:flex; justify-content:space-between;">
                <span>(${c.x.toFixed(2)}, ${c.y.toFixed(2)}, ${c.z.toFixed(2)})</span>
                <span style="color:#69f0ae;">扩散: ${nodes.length} 个</span>
            </div>
        `;

        card.addEventListener('click', () => {
            sfcDebugVisualizer.selectCorridor(idx);
        });

        listEl.appendChild(card);
    });
}

function updateSfcDetailBox(idx, c) {
    const detailBox = document.getElementById('sfc-detail-box');
    if (!detailBox) return;

    if (!c) {
        detailBox.style.display = 'none';
        return;
    }

    detailBox.style.display = 'block';
    
    // 更新列表项高亮态
    document.querySelectorAll('.sfc-card').forEach((el, i) => {
        if (i === idx) el.classList.add('active');
        else el.classList.remove('active');
    });

    document.getElementById('sfc-sel-idx').textContent = idx;
    document.getElementById('sfc-sel-id').textContent = c.root_id;
    document.getElementById('sfc-sel-layer').textContent = `Layer ${c.layer !== undefined ? c.layer : 0}`;
    document.getElementById('sfc-sel-xyz').textContent = `(${c.x.toFixed(3)}, ${c.y.toFixed(3)}, ${c.z.toFixed(3)})`;

    const optPEl = document.getElementById('sfc-sel-opt-p');
    const violEl = document.getElementById('sfc-sel-viol');
    if (optPEl) {
        if (c.opt_p) {
            optPEl.textContent = `(${c.opt_p[0].toFixed(3)}, ${c.opt_p[1].toFixed(3)}, ${c.opt_p[2].toFixed(3)})`;
        } else {
            optPEl.textContent = `-`;
        }
    }
    if (violEl) {
        if (c.viol !== undefined) {
            if (c.viol <= 0.001) {
                violEl.innerHTML = `<span style="color:#00e676;">0.0000m (✓ 严格在走廊内)</span>`;
            } else {
                violEl.innerHTML = `<span style="color:#ff1744;">+${c.viol.toFixed(4)}m (⚠ 穿透走廊约束)</span>`;
            }
        } else {
            violEl.textContent = `-`;
        }
    }

    const nodes = c.nodes || [];
    document.getElementById('sfc-sel-node-count').textContent = nodes.length;

    const rootLayer = c.layer !== undefined ? c.layer : 0;
    const crossNodes = nodes.filter(n => n.l !== undefined && n.l !== rootLayer);
    const crossWarn = document.getElementById('sfc-sel-cross-warning');
    if (crossWarn) {
        if (crossNodes.length > 0) {
            crossWarn.style.display = 'block';
            crossWarn.textContent = `⚠ 警告：存在 ${crossNodes.length} 个跨层扩散节点 (蔓延至 Layer ${[...new Set(crossNodes.map(n => n.l))].join(',')})！`;
        } else {
            crossWarn.style.display = 'none';
        }
    }

    // 渲染节点明细表
    const tableEl = document.getElementById('sfc-node-table');
    if (tableEl) {
        let rowsHtml = `<table style="width:100%; border-collapse:collapse; text-align:left;">
            <tr style="color:#888; border-bottom:1px solid #444;">
                <th style="padding:2px 4px;">NodeID</th>
                <th style="padding:2px 4px;">(X, Y, Z)</th>
                <th style="padding:2px 4px;">Layer</th>
                <th style="padding:2px 4px;">(Row, Col)</th>
            </tr>`;
        nodes.forEach(n => {
            const isCross = (n.l !== undefined && n.l !== rootLayer);
            const rowStyle = isCross ? 'background:rgba(255,23,68,0.25); color:#ff8a80; font-weight:bold;' : 'color:#ccc;';
            rowsHtml += `
                <tr style="${rowStyle} border-bottom: 1px solid #333;">
                    <td style="padding:2px 4px;">${n.id}</td>
                    <td style="padding:2px 4px;">${n.x.toFixed(2)}, ${n.y.toFixed(2)}, ${n.z.toFixed(2)}</td>
                    <td style="padding:2px 4px;">${n.l !== undefined ? n.l : '-'} ${isCross ? '⚠' : ''}</td>
                    <td style="padding:2px 4px;">(${n.r}, ${n.c})</td>
                </tr>`;
        });
        rowsHtml += `</table>`;
        tableEl.innerHTML = rowsHtml;
    }
}

sfcDebugVisualizer.setOnSelectCallback((idx, c) => {
    updateSfcDetailBox(idx, c);
});

// 聚焦视角
document.getElementById('btn-sfc-focus')?.addEventListener('click', () => {
    sfcDebugVisualizer.focusOnSelected();
});

// 复制单点 JSON
document.getElementById('btn-sfc-copy-sel')?.addEventListener('click', () => {
    const idx = sfcDebugVisualizer.selectedIndex;
    if (idx >= 0 && idx < currentSfcCorridors.length) {
        navigator.clipboard.writeText(JSON.stringify(currentSfcCorridors[idx], null, 2))
            .then(() => alert(`已成功复制点 #${idx} 的完整走廊数据 JSON！`));
    }
});

// 复制全部 JSON
document.getElementById('btn-sfc-copy-all')?.addEventListener('click', () => {
    if (currentSfcCorridors.length > 0) {
        navigator.clipboard.writeText(JSON.stringify(currentSfcCorridors, null, 2))
            .then(() => alert(`已成功复制全部 ${currentSfcCorridors.length} 个走廊数据 JSON！`));
    }
});

// 3.4 WebSocket 全双工流式同步模块 (支持流形图、3D拓扑管道、点云直通与 SFC 走廊诊断)
const wsStream = initWsStream(
    layers,
    robotTracker,
    getActiveRequestedLayers,
    graphVisualizer,
    corridorVisualizer,
    sfcDebugVisualizer,
    (corridors) => updateSfcDebugPanel(corridors)
);

// ---- 4. 图层显隐开关与 WebSocket 订阅联动 ----
// 4.0 3D 拓扑流形管道与内部几何障碍物显隐控制
document.getElementById('show-corridor')?.addEventListener('change', (e) => {
    corridorVisualizer.setVisible(e.target.checked);
});

// 4.0.1 调试可通行走廊显隐与侧边栏控制
document.getElementById('show-sfc-debug')?.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    sfcDebugVisualizer.setVisible(isChecked);
    const panel = document.getElementById('sfc-debug-panel');
    if (panel) panel.style.display = isChecked ? 'flex' : 'none';
});

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

function bindLayerToggle(id, layerObj) {
    document.getElementById(id)?.addEventListener('change', (e) => {
        if (layerObj.mesh) layerObj.mesh.visible = e.target.checked;
        if (layerObj.setVisible) layerObj.setVisible(e.target.checked);
        if (wsStream) wsStream.sendSubscription();
    });
}

bindLayerToggle('show-risk-cost', layers.risk_cost);
bindLayerToggle('show-octomap-local', layers.local_octomap);
bindLayerToggle('show-octomap-fused', layers.fused_octomap);

document.getElementById('show-emergency-stop')?.addEventListener('change', (e) => {
    layers.emergency_stop_free.mesh.visible = e.target.checked;
    layers.emergency_stop_occupied.mesh.visible = e.target.checked;
    if (wsStream) wsStream.sendSubscription();
});

