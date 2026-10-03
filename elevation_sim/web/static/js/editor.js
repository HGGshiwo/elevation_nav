import * as THREE from 'three';

/**
 * 3D 流形拓扑与导航交互模块 (Editor / Interaction Controller)
 * 支持：视角漫游、设起点、设终点、3D 踏面诊断调试、动态障碍物交互
 */
export function initEditor(scene, camera, renderer, controls, layers, editPlane, onDirty, graphVisualizer = null, roamController = null) {
    const statusEl = document.getElementById('status');
    const debugPanelDiv = document.getElementById('debug-panel');
    const btnCopyDebug = document.getElementById('btn-copy-debug');

    let currentTool = 'view';
    const checkedTool = document.querySelector('input[name="tool"]:checked');
    if (checkedTool) currentTool = checkedTool.value;

    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();

    // 监听工具切换
    document.querySelectorAll('input[name="tool"]').forEach(radio => {
        radio.addEventListener('change', (e) => {
            currentTool = e.target.value;
            if (currentTool === 'view') {
                controls.enabled = true;
                if (controls.mouseButtons) controls.mouseButtons.LEFT = null;
            } else { // start, goal, debug, obstacle: 左键用于交互拾取，右键保留视角旋转
                controls.enabled = true;
                if (controls.mouseButtons) controls.mouseButtons.LEFT = null;
            }

            // MC 式视角旋转仅在拖动视角工具下启用
            if (roamController) roamController.cameraLookEnabled = (currentTool === 'view');

            if (graphVisualizer) graphVisualizer.clearHoveredNode();
            if (debugPanelDiv) {
                debugPanelDiv.style.display = (currentTool === 'debug') ? 'block' : 'none';
            }
            renderer.domElement.style.cursor = 'default';
        });
    });

    // 调试诊断状态缓存 (两方块连通性对比)
    let debugNodeA = null;
    let debugNodeB = null;

    // 射线拾取 3D 流形踏面方块
    function getInteractionTarget() {
        if (!graphVisualizer) return null;
        raycaster.setFromCamera(mouse, camera);

        const snappedNode = graphVisualizer.findSnappedNode(raycaster, true);
        if (snappedNode) {
            return {
                type: 'snapped_node',
                voxel: { x: snappedNode.x, y: snappedNode.y, z: snappedNode.z, renderZ: snappedNode.renderZ },
                scale: [0.15, 0.15, 0.15],
                node: snappedNode
            };
        }
        return null;
    }

    // 请求 C++ 规划器单节点权威诊断
    function applyBackendStatus(statusEl, data) {
        if (!statusEl || !data || data.status !== 'ok') return;
        const nd = data.node || {};
        const trav = (typeof nd.traversability === 'number') ? nd.traversability
                   : (typeof data.traversability === 'number') ? data.traversability : 0;
        let text, color;
        if (data.reason_code === 'dynamic_blocked') { text = '动态禁行'; color = '#e040fb'; }
        else if (data.blocked) { text = '禁行/障碍'; color = '#ff5252'; }
        else if (data.reason_code === 'dynamic_inflation' || trav > 0.05) { text = '软代价区'; color = '#ffab40'; }
        else { text = '可通行'; color = '#69f0ae'; }
        statusEl.innerText = `${text} (trav=${trav.toFixed(2)})`;
        statusEl.style.color = color;
    }

    function fetchNodeReason(node, reasonElId, statusElId) {
        const reasonEl = document.getElementById(reasonElId);
        if (!reasonEl) return;
        reasonEl.innerText = '查询中...';
        reasonEl.style.color = '#aaa';
        fetch(`/api/nav/diagnose_node?x=${node.x}&y=${node.y}&z=${node.z}`)
            .then(r => r.json())
            .then(data => {
                if (data.status !== 'ok') {
                    reasonEl.innerText = (data.status === 'timeout') ? '后端诊断超时' : '诊断不可用';
                    reasonEl.style.color = '#ffab40';
                    return;
                }
                reasonEl.innerText = data.reason || '-';
                const colors = {
                    free: '#69f0ae',
                    soft_inflation: '#ffab40',
                    dynamic_inflation: '#ffab40',
                    body_hard_inflation: '#fb8c00',
                    blocked_headroom: '#ff5252',
                    blocked_lateral: '#ff5252',
                    dynamic_blocked: '#e040fb',
                    unknown: '#aaa'
                };
                reasonEl.style.color = colors[data.reason_code] || '#fff';
                const sEl = document.getElementById(statusElId);
                if (sEl) applyBackendStatus(sEl, data);
            })
            .catch(() => {
                reasonEl.innerText = '查询异常';
                reasonEl.style.color = '#ff5252';
            });
    }

    function fetchEdgeReason(nodeA, nodeB) {
        const reasonEl = document.getElementById('debug-rel-reason');
        if (!reasonEl) return;
        reasonEl.innerText = '分析中...';
        reasonEl.style.color = '#aaa';
        fetch(`/api/nav/diagnose_edge?x1=${nodeA.x}&y1=${nodeA.y}&z1=${nodeA.z}&x2=${nodeB.x}&y2=${nodeB.y}&z2=${nodeB.z}`)
            .then(r => r.json())
            .then(data => {
                if (data.status !== 'ok') {
                    reasonEl.innerText = (data.status === 'timeout') ? '后端诊断超时' : '诊断不可用';
                    reasonEl.style.color = '#ffab40';
                    return;
                }
                const connectedEl = document.getElementById('debug-rel-connected');
                if (connectedEl) {
                    connectedEl.innerText = data.connected ? '已连通 (可通行边)' : '未连通 (无法跨步)';
                    connectedEl.style.color = data.connected ? '#69f0ae' : '#ff5252';
                }
                reasonEl.innerText = data.reason || '-';
                reasonEl.style.color = data.connected ? '#69f0ae' : '#ff5252';
            })
            .catch(() => {
                reasonEl.innerText = '分析异常';
                reasonEl.style.color = '#ff5252';
            });
    }

    // 更新单个方块的面板信息
    function updateBlockCard(prefix, node) {
        document.getElementById(`debug-node-${prefix}-xy`).innerText = `${node.x.toFixed(2)}, ${node.y.toFixed(2)}`;
        document.getElementById(`debug-node-${prefix}-z`).innerText = `${node.z.toFixed(2)} m`;
        document.getElementById(`debug-node-${prefix}-edges`).innerText = `${node.edge_count || node.edges?.length || 0} 条`;

        const statusEl = document.getElementById(`debug-node-${prefix}-status`);
        if (node.isBlocked) {
            statusEl.innerText = '禁行 / 障碍物';
            statusEl.style.color = '#ff5252';
        } else if (node.traversability > 0.05) {
            statusEl.innerText = `软代价带 (trav=${node.traversability.toFixed(2)})`;
            statusEl.style.color = '#ffab40';
        } else {
            statusEl.innerText = '可通行 (自由区)';
            statusEl.style.color = '#69f0ae';
        }

        fetchNodeReason(node, `debug-node-${prefix}-reason`, `debug-node-${prefix}-status`);
    }

    // 更新 A-B 拓扑对比与连通性分析
    function updateRelationCard(nodeA, nodeB) {
        const dx = nodeB.x - nodeA.x;
        const dy = nodeB.y - nodeA.y;
        const dz = nodeB.z - nodeA.z;
        const dxy = Math.hypot(dx, dy);
        const slopeDeg = (dxy > 1e-4) ? (Math.atan2(Math.abs(dz), dxy) * 180 / Math.PI) : (Math.abs(dz) > 0.01 ? 90 : 0);

        document.getElementById('debug-rel-dxy').innerText = `${dxy.toFixed(2)} m`;
        document.getElementById('debug-rel-dz').innerText = `${dz >= 0 ? '+' : ''}${dz.toFixed(2)} m`;
        document.getElementById('debug-rel-slope').innerText = `${slopeDeg.toFixed(1)}°`;

        let localConnected = false;
        if (graphVisualizer) {
            localConnected = graphVisualizer.checkEdgeConnected(nodeA, nodeB);
        }

        const connEl = document.getElementById('debug-rel-connected');
        connEl.innerText = localConnected ? '已连通 (拓扑直达)' : '未连通 (无直达拓扑边)';
        connEl.style.color = localConnected ? '#69f0ae' : '#ff5252';

        fetchEdgeReason(nodeA, nodeB);
    }

    // 交互点击触发调试检查
    function handleDebugInspection(node) {
        if (!debugNodeA || (debugNodeA && debugNodeB)) {
            debugNodeA = node;
            debugNodeB = null;
            if (graphVisualizer) graphVisualizer.setDebugMarkerA(node);
            updateBlockCard('a', node);

            ['debug-node-b-xy', 'debug-node-b-z', 'debug-node-b-status', 'debug-node-b-reason', 'debug-node-b-edges',
             'debug-rel-dxy', 'debug-rel-dz', 'debug-rel-slope', 'debug-rel-connected', 'debug-rel-reason'].forEach(id => {
                const el = document.getElementById(id);
                if (el) el.innerText = '-';
            });
            const reasonEl = document.getElementById('debug-rel-reason');
            if (reasonEl) {
                reasonEl.innerText = '请点击第二个方块以进行两点连通性对比';
                reasonEl.style.color = '#ffab40';
            }
        } else {
            debugNodeB = node;
            if (graphVisualizer) graphVisualizer.setDebugMarkerB(node);
            updateBlockCard('b', node);
            updateRelationCard(debugNodeA, debugNodeB);
        }
    }

    // 鼠标移动与悬浮拾取
    renderer.domElement.addEventListener('mousemove', (e) => {
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        if (currentTool === 'view') {
            if (graphVisualizer) graphVisualizer.clearHoveredNode();
            return;
        }

        const target = getInteractionTarget();
        if (target && target.type === 'snapped_node') {
            if (graphVisualizer) graphVisualizer.highlightHoveredNode(target.node);
        } else {
            if (graphVisualizer) graphVisualizer.clearHoveredNode();
        }
    });

    renderer.domElement.addEventListener('mouseleave', () => {
        if (graphVisualizer) graphVisualizer.clearHoveredNode();
    });

    // 鼠标点击事件
    renderer.domElement.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return; // 仅响应左键

        const target = getInteractionTarget();
        if (!target) return;

        if (currentTool === 'start') {
            if (target.node && target.node.isBlocked) {
                if (statusEl) statusEl.innerText = `该踏面属于禁行/障碍区域，无法设为起点！`;
                return;
            }
            const pos = target.voxel;
            if (graphVisualizer) {
                graphVisualizer.clearHoveredNode();
                graphVisualizer.setStartMarker(pos);
            }
            if (statusEl) statusEl.innerText = `已精确吸附起点: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`;
            fetch('/api/nav/set_start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x: pos.x, y: pos.y, z: pos.z, yaw: 0.0 })
            }).catch(() => {});
        } else if (currentTool === 'goal') {
            if (target.node && target.node.isBlocked) {
                if (statusEl) statusEl.innerText = `该踏面属于禁行/障碍区域，无法设为终点！`;
                return;
            }
            const pos = target.voxel;
            if (graphVisualizer) {
                graphVisualizer.clearHoveredNode();
                graphVisualizer.setGoalMarker(pos);
            }
            if (statusEl) statusEl.innerText = `已精确吸附终点: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`;
            fetch('/api/nav/set_goal', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x: pos.x, y: pos.y, z: pos.z, yaw: 0.0 })
            }).catch(() => {});
        } else if (currentTool === 'debug') {
            handleDebugInspection(target.node);
        } else if (currentTool === 'obstacle') {
            const pos = target.voxel;
            fetch('/api/obstacle/toggle', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x: pos.x, y: pos.y, z: pos.z })
            }).then(r => r.json()).then(j => {
                if (statusEl) statusEl.innerText = j.message || JSON.stringify(j);
            }).catch(() => {});
        }
    });

    document.getElementById('btn-cancel-goal')?.addEventListener('click', () => {
        fetch('/api/nav/cancel_goal', { method: 'POST' }).catch((e) => console.error("取消导航异常:", e));
    });

    document.getElementById('btn-clear-debug')?.addEventListener('click', () => {
        debugNodeA = null;
        debugNodeB = null;
        if (graphVisualizer) graphVisualizer.clearDebugMarkers();
        ['debug-node-a-xy', 'debug-node-a-z', 'debug-node-a-status', 'debug-node-a-reason', 'debug-node-a-edges',
         'debug-node-b-xy', 'debug-node-b-z', 'debug-node-b-status', 'debug-node-b-reason', 'debug-node-b-edges',
         'debug-rel-dxy', 'debug-rel-dz', 'debug-rel-slope', 'debug-rel-connected', 'debug-rel-reason'].forEach(id => {
            const el = document.getElementById(id);
            if (el) el.innerText = '-';
        });
    });

    if (btnCopyDebug) {
        btnCopyDebug.addEventListener('click', () => {
            const debugText = debugPanelDiv?.innerText || '';
            navigator.clipboard.writeText(debugText).then(() => {
                alert("已复制调试诊断信息到剪贴板");
            }).catch(() => {});
        });
    }

    return {
        getCurrentTool: () => currentTool,
        setTool: (t) => { currentTool = t; }
    };
}
