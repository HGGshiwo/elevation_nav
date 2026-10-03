import * as THREE from 'three';
import { initScene } from '../scene.js';
import { LayerManager } from '../layer.js';
import { GraphVisualizer } from './graph_visualizer.js';
import { CorridorVisualizer } from './corridor_visualizer.js';
import { SfcDebugVisualizer } from './sfc_debug_visualizer.js';
import { InjectedObstacleVisualizer } from './obstacle_visualizer.js';
import { DynamicNodeVisualizer } from './dynamic_nodes_visualizer.js';
import { ReboundVisualizer } from './rebound_visualizer.js';
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
    emergency_stop_free: new LayerManager(scene, "emergency_stop_free", [0.06, 0.06, 0.06]),
    emergency_stop_occupied: new LayerManager(scene, "emergency_stop_occupied", [0.10, 0.10, 0.10])
};

// 默认仅显示禁行与可通行图层
layers.preblocked.mesh.visible = true;
layers.traversable.mesh.visible = true;
layers.occupied.mesh.visible = true;
layers.risk_cost.mesh.visible = false;
layers.emergency_stop_free.mesh.visible = false;
layers.emergency_stop_occupied.mesh.visible = false;

// 注入障碍物真值渲染器 + 融合动态标记节点渲染器
const obstacleVisualizer = new InjectedObstacleVisualizer(scene);
const dynamicNodeVisualizer = new DynamicNodeVisualizer(scene);
const reboundVisualizer = new ReboundVisualizer(scene);

// 3D 流形拓扑图渲染器 (点云直通踏面与连通网格)
const graphVisualizer = new GraphVisualizer(scene, camera, controls);

// 3D 拓扑管道渲染器 (SFC 凸走廊边界线框, 默认显示)
const corridorVisualizer = new CorridorVisualizer(scene);

// 3D 可通行走廊诊断与单步扩散调试渲染器
const sfcDebugVisualizer = new SfcDebugVisualizer(scene, camera, controls);

// 获取当前已勾选需流式同步的图层列表
function getActiveRequestedLayers() {
    const requested = [];
    if (document.getElementById('show-preblocked')?.checked) requested.push('preblocked');
    if (document.getElementById('show-traversable')?.checked) requested.push('traversable');
    if (document.getElementById('show-emergency-stop')?.checked) {
        requested.push('emergency_stop_free');
        requested.push('emergency_stop_occupied');
    }
    return requested;
}

// ---- 3. 核心功能模块组装 ----
// 3.1 机器狗追踪与路径模块 (跟随视角驱动与碰撞脉冲挂入渲染循环)
const robotTracker = initRobotTracker(scene, controls);
if (setFrameCallback) setFrameCallback(() => {
    robotTracker.updateFollow();
    if (robotTracker.updateCollisionAnimation) robotTracker.updateCollisionAnimation();
});

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

// 走廊点快速查找定位
function performSfcSearch() {
    const inputEl = document.getElementById('sfc-search-input');
    if (!inputEl || currentSfcCorridors.length === 0) return;
    const q = inputEl.value.trim();
    if (!q) return;

    let targetIdx = -1;

    // 1. 尝试作为纯数字匹配 (先当索引，再当 root_id)
    if (/^\d+$/.test(q)) {
        const num = parseInt(q, 10);
        if (num >= 0 && num < currentSfcCorridors.length) {
            targetIdx = num;
        } else {
            targetIdx = currentSfcCorridors.findIndex(c => c.root_id === num);
        }
    }

    // 2. 尝试解析坐标 (x,y 或 x,y,z)
    if (targetIdx < 0) {
        const matches = q.match(/[-+]?[0-9]*\.?[0-9]+/g);
        if (matches && matches.length >= 2) {
            const qx = parseFloat(matches[0]);
            const qy = parseFloat(matches[1]);
            const qz = matches.length >= 3 ? parseFloat(matches[2]) : null;

            let bestDist = Infinity;
            currentSfcCorridors.forEach((c, i) => {
                let d = (c.x - qx) ** 2 + (c.y - qy) ** 2;
                if (qz !== null) d += ((c.z - qz) * 2) ** 2;
                if (d < bestDist) {
                    bestDist = d;
                    targetIdx = i;
                }
            });
        }
    }

    if (targetIdx >= 0 && targetIdx < currentSfcCorridors.length) {
        sfcDebugVisualizer.selectCorridor(targetIdx);
        sfcDebugVisualizer.focusOnSelected();
        const card = document.getElementById(`sfc-card-${targetIdx}`);
        if (card) card.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    } else {
        alert(`未找到匹配的局部规划走廊点: "${q}"`);
    }
}

document.getElementById('btn-sfc-search')?.addEventListener('click', performSfcSearch);
document.getElementById('sfc-search-input')?.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        e.preventDefault();
        performSfcSearch();
    }
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

// 3.4 WebSocket 全双工流式同步模块 (支持流形图、注入障碍、拓扑管道、点云直通与 SFC 走廊诊断)
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
// 4.0.0 拓扑管道(走廊边界线框)显隐
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

// 注入障碍物真值显隐
document.getElementById('show-injected-obstacles')?.addEventListener('change', (e) => {
    obstacleVisualizer.setVisible(e.target.checked);
});
document.getElementById('show-dynamic-nodes')?.addEventListener('change', (e) => {
    dynamicNodeVisualizer.setVisible(e.target.checked);
});

document.getElementById('show-rebound')?.addEventListener('change', (e) => {
    reboundVisualizer.setVisible(e.target.checked);
});

document.getElementById('show-emergency-stop')?.addEventListener('change', (e) => {
    layers.emergency_stop_free.mesh.visible = e.target.checked;
    layers.emergency_stop_occupied.mesh.visible = e.target.checked;
    if (wsStream) wsStream.sendSubscription();
});

