import * as THREE from 'three';

/**
 * 栅格地图交互与编辑模块 (Editor)
 * 完整实现：画笔、橡皮擦、设起点、设终点、Z轴编辑平面、游标拾取与体素增删
 */
export function initEditor(scene, camera, renderer, controls, layers, editPlane, onDirty, graphVisualizer = null, costmapVisualizer = null, roamController = null) {
    const statusEl = document.getElementById('status');
    const brushSizeInput = document.getElementById('brush-size');
    const editLayerSelect = document.getElementById('edit-layer');
    const debugPanelDiv = document.getElementById('debug-panel');
    const btnCopyDebug = document.getElementById('btn-copy-debug');
    const obsDebugPanelDiv = document.getElementById('obstacle-debug-panel');
    const btnCopyObsDebug = document.getElementById('btn-copy-obs-debug');
    const btnClearObsDebug = document.getElementById('btn-clear-obs-debug');

    let currentTool = 'view';
    let currentLayer = 'occupied';
    let brushSize = 1;
    let zHeight = 0.0;
    let isPainting = false;

    if (brushSizeInput) brushSize = parseInt(brushSizeInput.value) || 1;
    if (editLayerSelect) currentLayer = editLayerSelect.value;
    const checkedTool = document.querySelector('input[name="tool"]:checked');
    if (checkedTool) currentTool = checkedTool.value;

    // 游标提示框
    const cursorGeo = new THREE.BoxGeometry(1.0, 1.0, 1.0);
    const cursorMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
    const cursor = new THREE.Mesh(cursorGeo, cursorMat);
    cursor.visible = false;
    scene.add(cursor);

    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();

    // 监听工具切换
    document.querySelectorAll('input[name="tool"]').forEach(radio => {
        radio.addEventListener('change', (e) => {
            currentTool = e.target.value;
            if (currentTool === 'view') {
                controls.enabled = true;
                // 左键 = MC 式第一人称视角旋转 (RoamController 接管), 不再用 OrbitControls 环绕
                if (controls.mouseButtons) controls.mouseButtons.LEFT = null;
            } else if (currentTool === 'brush' || currentTool === 'eraser') {
                controls.enabled = false;
            } else { // start, goal, debug, obstacle_debug: 左键用于交互拾取，右键保留视角旋转
                controls.enabled = true;
                if (controls.mouseButtons) controls.mouseButtons.LEFT = null;
            }

            // MC 式视角旋转仅在拖动视角工具下启用, 其余工具左键留给拾取/绘制
            if (roamController) roamController.cameraLookEnabled = (currentTool === 'view');

            if (currentTool === 'view') {
                cursor.visible = false;
            }
            if (graphVisualizer) graphVisualizer.clearHoveredNode();
            if (debugPanelDiv) {
                debugPanelDiv.style.display = (currentTool === 'debug') ? 'block' : 'none';
            }
            if (obsDebugPanelDiv) {
                // 点击后才显示调试信息，未点击选中时不显示
                if (currentTool !== 'obstacle_debug' || !costmapVisualizer?.selectedObstacleId) {
                    obsDebugPanelDiv.style.display = 'none';
                }
            }
            if (currentTool !== 'obstacle_debug' && costmapVisualizer) {
                costmapVisualizer.clearSelection();
                costmapVisualizer.clearHoveredObstacle();
            }
            renderer.domElement.style.cursor = 'default';
        });
    });

    if (editLayerSelect) {
        editLayerSelect.addEventListener('change', (e) => { currentLayer = e.target.value; });
    }
    if (brushSizeInput) {
        brushSizeInput.addEventListener('input', (e) => { brushSize = parseInt(e.target.value) || 1; });
    }

    // 调试诊断状态缓存 (两方块连通性对比)
    let debugNodeA = null;
    let debugNodeB = null;

    // 射线拾取交互目标
    function getInteractionTarget() {
        raycaster.setFromCamera(mouse, camera);

        // 0. 若当前为设起点、设终点或调试方块，交互目标严格限制在 3D 流形踏面方块上！
        if (currentTool === 'start' || currentTool === 'goal' || currentTool === 'debug') {
            if (!graphVisualizer) return null;
            // 均允许拾取踏面方块 (含障碍方块，以便在起终点模式下给出阻挡提示，在调试模式下诊断)
            const snappedNode = graphVisualizer.findSnappedNode(raycaster, true);
            if (snappedNode) {
                return {
                    type: 'snapped_node',
                    layerName: 'manifold_graph',
                    voxel: { x: snappedNode.x, y: snappedNode.y, z: snappedNode.z, renderZ: snappedNode.renderZ },
                    scale: [0.15, 0.15, 0.15],
                    node: snappedNode
                };
            }
            // 未吸附到踏面方块时直接返回 null，绝不降级击穿到底部体素编辑平面！
            return null;
        }

        const targetLayer = layers[currentLayer];
        if (!targetLayer) return null;
        const scale = targetLayer.scale || [0.2, 0.2, 0.2];

        // 1. 与显示的体素图层求交
        const meshes = [];
        Object.keys(layers).forEach(name => {
            if (layers[name]?.mesh?.visible) meshes.push(layers[name].mesh);
        });

        const voxelHits = raycaster.intersectObjects(meshes);
        if (voxelHits.length > 0) {
            const hit = voxelHits[0];
            let hitLayerName = null;
            Object.keys(layers).forEach(name => {
                if (layers[name]?.mesh === hit.object) hitLayerName = name;
            });

            if (hitLayerName) {
                const voxel = layers[hitLayerName].getVoxelByInstanceId(hit.instanceId);
                if (voxel) {
                    return {
                        type: 'voxel',
                        layerName: hitLayerName,
                        voxel: voxel,
                        normal: hit.face.normal.clone(),
                        scale: voxel.scale || layers[hitLayerName].scale
                    };
                }
            }
        }

        // 2. 与参考平面求交
        if (editPlane && currentTool !== 'eraser') {
            const planeHits = raycaster.intersectObject(editPlane);
            if (planeHits.length > 0) {
                const p = planeHits[0].point;
                const snapX = Math.floor(p.x / scale[0] + 0.5) * scale[0];
                const snapY = Math.floor(p.y / scale[1] + 0.5) * scale[1];
                return {
                    type: 'plane',
                    layerName: currentLayer,
                    voxel: { x: snapX, y: snapY, z: zHeight },
                    normal: new THREE.Vector3(0, 0, 1),
                    scale: scale
                };
            }
        }
        return null;
    }

    // 执行笔刷绘制与擦除
    function applyBrush(target) {
        if (!target) return;
        const targetLayer = layers[currentLayer];
        if (!targetLayer) return;
        const scale = target.scale || targetLayer.scale;
        const center = target.type === 'voxel' && currentTool === 'brush'
            ? {
                x: target.voxel.x + target.normal.x * scale[0],
                y: target.voxel.y + target.normal.y * scale[1],
                z: target.voxel.z + target.normal.z * scale[2]
            }
            : target.voxel;

        const half = Math.floor(brushSize / 2);
        let changed = false;

        for (let dx = -half; dx <= half; dx++) {
            for (let dy = -half; dy <= half; dy++) {
                const gx = center.x + dx * scale[0];
                const gy = center.y + dy * scale[1];
                const gz = center.z;

                if (currentTool === 'brush') {
                    if (targetLayer.addVoxel(gx, gy, gz)) changed = true;
                } else if (currentTool === 'eraser') {
                    const eraseLayer = layers[target.layerName] || targetLayer;
                    if (eraseLayer.removeVoxel(gx, gy, gz)) changed = true;
                }
            }
        }

        if (changed && onDirty) {
            onDirty(currentLayer);
        }
    }

    // 请求 C++ 规划器单节点权威诊断, 将禁行原因写入面板行 (queryEl 不存在或查询失败时静默跳过)
    function fetchNodeReason(node, reasonElId) {
        const reasonEl = document.getElementById(reasonElId);
        if (!reasonEl) return;
        // 可通行且非软代价区无需查询原因, 显示确定文案
        if (!node.isBlocked && node.traversability <= 0.05) {
            reasonEl.innerText = '可安全通行';
            reasonEl.style.color = '#69f0ae';
            return;
        }
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
                    lateral_body: '#ff5252',
                    headroom: '#ff5252',
                    blocked: '#ff5252'
                };
                reasonEl.style.color = colors[data.reason_code] || '#ff5252';
            })
            .catch(() => {
                reasonEl.innerText = '诊断请求失败';
                reasonEl.style.color = '#ffab40';
            });
    }

    // 踏面方块深度调试与两点邻边关系诊断
    function handleDebugInspection(node) {
        if (!node) return;

        // 若尚未选择 A，或 A与B 均已选择过：重置并选择 A
        if (!debugNodeA || (debugNodeA && debugNodeB)) {
            debugNodeA = node;
            debugNodeB = null;

            if (graphVisualizer) graphVisualizer.setDebugMarkerA(node);

            const xyEl = document.getElementById('debug-node-a-xy');
            const zEl = document.getElementById('debug-node-a-z');
            const statusElA = document.getElementById('debug-node-a-status');
            const edgesElA = document.getElementById('debug-node-a-edges');

            if (xyEl) xyEl.innerText = `(${node.x.toFixed(2)}, ${node.y.toFixed(2)})`;
            if (zEl) zEl.innerText = `${node.z.toFixed(3)} m`;
            if (statusElA) {
                statusElA.innerText = node.isBlocked ? '禁行/障碍' : (node.traversability >= 0.3 ? '复杂台阶' : '可通行');
                statusElA.style.color = node.isBlocked ? '#ff5252' : (node.traversability >= 0.3 ? '#ffab40' : '#69f0ae');
            }
            if (edgesElA) {
                const cnt = graphVisualizer ? graphVisualizer.getNodeEdgeCount(node) : 0;
                edgesElA.innerText = `${cnt} 条连通边`;
            }
            fetchNodeReason(node, 'debug-node-a-reason');

            // 清空 B 与 对比关系
            document.getElementById('debug-node-b-xy') && (document.getElementById('debug-node-b-xy').innerText = '-');
            document.getElementById('debug-node-b-z') && (document.getElementById('debug-node-b-z').innerText = '-');
            document.getElementById('debug-node-b-status') && (document.getElementById('debug-node-b-status').innerText = '-');
            document.getElementById('debug-node-b-reason') && (document.getElementById('debug-node-b-reason').innerText = '-');
            document.getElementById('debug-node-b-edges') && (document.getElementById('debug-node-b-edges').innerText = '-');
            document.getElementById('debug-rel-dxy') && (document.getElementById('debug-rel-dxy').innerText = '-');
            document.getElementById('debug-rel-dz') && (document.getElementById('debug-rel-dz').innerText = '-');
            document.getElementById('debug-rel-slope') && (document.getElementById('debug-rel-slope').innerText = '-');
            document.getElementById('debug-rel-connected') && (document.getElementById('debug-rel-connected').innerText = '等待选择方块 B...');
            document.getElementById('debug-rel-reason') && (document.getElementById('debug-rel-reason').innerText = '请点击第二个方块以进行拓扑分析');

            console.log(`[Debug] 选中基准方块 A: (${node.x.toFixed(2)}, ${node.y.toFixed(2)}, ${node.z.toFixed(2)})`);
        } else {
            // 已有 A，当前点击作为 B 进行两点比对
            if (Math.hypot(node.x - debugNodeA.x, node.y - debugNodeA.y) < 0.03 && Math.abs(node.z - debugNodeA.z) < 0.05) {
                return;
            }
            debugNodeB = node;

            const isConn = graphVisualizer ? graphVisualizer.areNodesConnected(debugNodeA, debugNodeB) : false;
            if (graphVisualizer) graphVisualizer.setDebugMarkerB(node, isConn);

            const xyEl = document.getElementById('debug-node-b-xy');
            const zEl = document.getElementById('debug-node-b-z');
            const statusElB = document.getElementById('debug-node-b-status');
            const edgesElB = document.getElementById('debug-node-b-edges');

            if (xyEl) xyEl.innerText = `(${node.x.toFixed(2)}, ${node.y.toFixed(2)})`;
            if (zEl) zEl.innerText = `${node.z.toFixed(3)} m`;
            if (statusElB) {
                statusElB.innerText = node.isBlocked ? '禁行/障碍' : (node.traversability >= 0.3 ? '复杂台阶' : '可通行');
                statusElB.style.color = node.isBlocked ? '#ff5252' : (node.traversability >= 0.3 ? '#ffab40' : '#69f0ae');
            }
            if (edgesElB) {
                const cnt = graphVisualizer ? graphVisualizer.getNodeEdgeCount(node) : 0;
                edgesElB.innerText = `${cnt} 条连通边`;
            }
            fetchNodeReason(node, 'debug-node-b-reason');

            // 计算相对几何量
            const dxy = Math.hypot(debugNodeB.x - debugNodeA.x, debugNodeB.y - debugNodeA.y);
            const dz = Math.abs(debugNodeB.z - debugNodeA.z);
            const slopeDeg = (Math.atan2(dz, Math.max(dxy, 1e-4)) * 180.0 / Math.PI);

            const relDxyEl = document.getElementById('debug-rel-dxy');
            const relDzEl = document.getElementById('debug-rel-dz');
            const relSlopeEl = document.getElementById('debug-rel-slope');
            const relConnEl = document.getElementById('debug-rel-connected');
            const relReasonEl = document.getElementById('debug-rel-reason');

            if (relDxyEl) {
                relDxyEl.innerText = `${dxy.toFixed(3)} m ${dxy > 0.35 ? '(>0.35m 超限)' : '(≤0.35m 正常)'}`;
                relDxyEl.style.color = dxy > 0.35 ? '#ff5252' : '#ddd';
            }
            if (relDzEl) {
                relDzEl.innerText = `${dz.toFixed(3)} m ${dz > 0.25 ? '(>0.25m 超限)' : '(≤0.25m 正常)'}`;
                relDzEl.style.color = dz > 0.25 ? '#ff5252' : '#ddd';
            }
            if (relSlopeEl) {
                relSlopeEl.innerText = `${slopeDeg.toFixed(1)}° (无角度限制)`;
                relSlopeEl.style.color = '#ddd';
            }

            if (relConnEl) {
                relConnEl.innerText = isConn ? '✔ 已成功建立连通边' : '❌ 未建立拓扑临边';
                relConnEl.style.color = isConn ? '#69f0ae' : '#ff5252';
            }

            if (relReasonEl) {
                if (isConn) {
                    relReasonEl.innerText = '满足动力学通行约束，已生成连通边';
                    relReasonEl.style.color = '#69f0ae';
                } else {
                    const fails = [];
                    if (debugNodeA.isBlocked) fails.push('方块 A 属于禁行/低净空');
                    if (debugNodeB.isBlocked) fails.push('方块 B 属于禁行/低净空');
                    if (dz > 0.25) fails.push(`台阶高差过大 (Δz=${dz.toFixed(2)}m > 0.25m)`);
                    if (dxy > 0.35) fails.push(`跨步距离过远 (Δxy=${dxy.toFixed(2)}m > 0.35m)`);
                    if (slopeDeg > 30.0) fails.push(`爬坡坡度超标 (${slopeDeg.toFixed(1)}° > 30.0°)`);
                    if (dxy > 0.16 && fails.length === 0) fails.push('非8-连通相邻网格');
                    if (fails.length === 0) fails.push('未在同层或未在搜索窗口内');
                    relReasonEl.innerText = fails.join('；');
                    relReasonEl.style.color = '#ff5252';
                }
            }

            // 请求 C++ 规划器后端进行权威物理诊断 (单源真值)
            fetch(`/api/nav/diagnose_edge?x1=${debugNodeA.x}&y1=${debugNodeA.y}&z1=${debugNodeA.z}&x2=${debugNodeB.x}&y2=${debugNodeB.y}&z2=${debugNodeB.z}`)
                .then(r => r.json())
                .then(data => {
                    if (data.status === 'ok') {
                        if (relConnEl) {
                            relConnEl.innerText = data.connected ? '✔ 拓扑临边已连通 (C++权威)' : '❌ 未建立拓扑临边 (C++权威)';
                            relConnEl.style.color = data.connected ? '#69f0ae' : '#ff5252';
                        }
                        if (relReasonEl) {
                            relReasonEl.innerText = data.reason || (data.connected ? '满足全部动力学通行约束' : '未成临边');
                            relReasonEl.style.color = data.connected ? '#69f0ae' : '#ff5252';
                        }
                        if (data.metrics && relDxyEl && relDzEl && relSlopeEl) {
                            const lim = data.limits || {};
                            relDxyEl.innerText = `${data.metrics.dxy.toFixed(3)} m (上限 ${lim.max_stride_length || 0.35}m)`;
                            relDzEl.innerText = `${data.metrics.dz.toFixed(3)} m (上限 ${lim.max_step_height || 0.25}m)`;
                            relSlopeEl.innerText = `${data.metrics.slope_deg.toFixed(1)}° (上限 ${lim.max_slope_deg || 30.0}°)`;
                        }
                        if (graphVisualizer) {
                            graphVisualizer.setDebugMarkerB(node, data.connected);
                        }
                        console.log('[Debug] 收到 C++ 后端权威诊断结果:', data);
                    }
                })
                .catch(() => {});

            console.log(`[Debug] A-B 对比: dxy=${dxy.toFixed(3)}, dz=${dz.toFixed(3)}, slope=${slopeDeg.toFixed(1)}°, connected=${isConn}`);
        }
    }

    // 障碍物排查诊断面板信息渲染
    function showObstacleDebug(obs) {
        if (!obs) return;
        if (obsDebugPanelDiv) obsDebugPanelDiv.style.display = 'block';

        const idEl = document.getElementById('obs-id');
        const typeEl = document.getElementById('obs-type');
        const srcEl = document.getElementById('obs-source');
        const zEl = document.getElementById('obs-z');
        const geoEl = document.getElementById('obs-geometry');
        const reasonEl = document.getElementById('obs-reason');
        const tebEl = document.getElementById('obs-teb-effect');

        if (idEl) idEl.innerText = obs.id !== undefined ? obs.id : '-';
        if (typeEl) {
            const typeMap = {
                circle: '圆柱体 (CircularObstacle)',
                line: '线段/护栏 (LineObstacle)',
                polygon: '多边形 (PolygonObstacle)'
            };
            typeEl.innerText = typeMap[obs.type] || obs.type;
        }
        if (srcEl) srcEl.innerText = obs.source || '3D 结构化障碍物';

        let zVal = '-';
        let geoInfo = '-';

        if (obs.type === 'circle') {
            zVal = obs.z !== undefined ? `${Number(obs.z).toFixed(3)} m` : '-';
            geoInfo = `圆心: (${Number(obs.x ?? 0).toFixed(2)}, ${Number(obs.y ?? 0).toFixed(2)})\n半径 R: ${Number(obs.radius ?? 0.15).toFixed(2)} m`;
        } else if (obs.type === 'line') {
            if (obs.start && obs.end) {
                const zAvg = ((obs.start[2] + obs.end[2]) * 0.5).toFixed(3);
                zVal = `${zAvg} m`;
                geoInfo = `起点: (${obs.start[0].toFixed(2)}, ${obs.start[1].toFixed(2)})\n终点: (${obs.end[0].toFixed(2)}, ${obs.end[1].toFixed(2)})\n长度: ${obs.length !== undefined ? Number(obs.length).toFixed(2) : '-'} m`;
            }
        } else if (obs.type === 'polygon' && obs.points) {
            if (obs.points.length > 0) {
                const zAvg = (obs.points.reduce((acc, p) => acc + p[2], 0) / obs.points.length).toFixed(3);
                zVal = `${zAvg} m`;
                geoInfo = `顶点数: ${obs.points.length} 个\n首点: (${obs.points[0][0].toFixed(2)}, ${obs.points[0][1].toFixed(2)})`;
            }
        }

        if (zEl) zEl.innerText = zVal;
        if (geoEl) geoEl.innerText = geoInfo;
        if (reasonEl) reasonEl.innerText = obs.reason || '由局部流形提取器根据点云或连通性边界生成。';
        if (tebEl) tebEl.innerText = obs.teb_effect || '注入 TEB 局部规划器进行避障轨迹非线性求解。';
    }

    function clearObstacleDebugUI() {
        ['obs-id', 'obs-type', 'obs-source', 'obs-z', 'obs-geometry', 'obs-reason', 'obs-teb-effect'].forEach(id => {
            const el = document.getElementById(id);
            if (el) el.innerText = '-';
        });
    }

    // 事件监听
    renderer.domElement.addEventListener('mousemove', (e) => {
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        if (currentTool === 'view') {
            cursor.visible = false;
            return;
        }

        if (currentTool === 'obstacle_debug') {
            cursor.visible = false;
            if (graphVisualizer) graphVisualizer.clearHoveredNode();

            if (costmapVisualizer) {
                camera.updateMatrixWorld();
                raycaster.setFromCamera(mouse, camera);
                const obs = costmapVisualizer.pickObstacle(raycaster);
                if (obs) {
                    costmapVisualizer.setHoveredObstacle(obs.id);
                    renderer.domElement.style.cursor = 'pointer';
                } else {
                    costmapVisualizer.clearHoveredObstacle();
                    renderer.domElement.style.cursor = 'default';
                }
            }
            return;
        }

        // 在设起点/设终点/调试方块模式下，严格仅吸附流形方块，画笔游标立方体永久隐藏！
        if (currentTool === 'start' || currentTool === 'goal' || currentTool === 'debug') {
            cursor.visible = false;
            const target = getInteractionTarget();
            if (target && target.type === 'snapped_node') {
                if (graphVisualizer) graphVisualizer.highlightHoveredNode(target.node);
            } else {
                if (graphVisualizer) graphVisualizer.clearHoveredNode();
            }
            return;
        }

        const target = getInteractionTarget();
        if (target) {
            if (graphVisualizer) graphVisualizer.clearHoveredNode();
            const sc = target.scale || [0.2, 0.2, 0.2];
            cursor.visible = (currentTool === 'brush' || currentTool === 'eraser');
            cursor.scale.set(sc[0] * brushSize, sc[1] * brushSize, sc[2]);
            cursor.position.set(target.voxel.x, target.voxel.y, target.voxel.z);
            cursorMat.color.setHex(0xffffff);

            if (isPainting && (currentTool === 'brush' || currentTool === 'eraser')) {
                applyBrush(target);
            }
        } else {
            cursor.visible = false;
            if (graphVisualizer) graphVisualizer.clearHoveredNode();
        }
    });

    renderer.domElement.addEventListener('mouseleave', () => {
        cursor.visible = false;
        isPainting = false;
        if (graphVisualizer) graphVisualizer.clearHoveredNode();
        if (costmapVisualizer) {
            costmapVisualizer.clearHoveredObstacle();
            renderer.domElement.style.cursor = 'default';
        }
    });

    renderer.domElement.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return; // 仅响应左键点击

        if (currentTool === 'obstacle_debug') {
            if (!costmapVisualizer) return;
            camera.updateMatrixWorld();
            raycaster.setFromCamera(mouse, camera);
            const obs = costmapVisualizer.pickObstacle(raycaster);
            if (obs) {
                if (costmapVisualizer.selectedObstacleId === obs.id) {
                    // 再次点击已选中的障碍物：取消选中并隐藏调试面板
                    costmapVisualizer.clearSelection();
                    clearObstacleDebugUI();
                    if (obsDebugPanelDiv) obsDebugPanelDiv.style.display = 'none';
                    if (statusEl) statusEl.innerText = `已取消选中障碍物 [ID: ${obs.id}]`;
                } else {
                    // 点击后才锁定选中，并显示详细调试诊断信息！
                    costmapVisualizer.selectObstacle(obs.id);
                    showObstacleDebug(obs);
                    if (statusEl) statusEl.innerText = `已选中障碍物 [ID: ${obs.id}] (${obs.type})`;
                }
            } else {
                costmapVisualizer.clearSelection();
                clearObstacleDebugUI();
                if (obsDebugPanelDiv) obsDebugPanelDiv.style.display = 'none';
                if (statusEl) statusEl.innerText = `未点中障碍物，已清除选中`;
            }
            return;
        }

        const target = getInteractionTarget();
        if (!target) return;

        if (currentTool === 'brush' || currentTool === 'eraser') {
            isPainting = true;
            applyBrush(target);
        } else if (currentTool === 'start') {
            if (target.type !== 'snapped_node') return;
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
            if (target.type !== 'snapped_node') return;
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
            if (target.type === 'snapped_node') {
                handleDebugInspection(target.node);
            }
        }
    });

    window.addEventListener('mouseup', () => { isPainting = false; });

    document.getElementById('btn-cancel-goal')?.addEventListener('click', () => {
        fetch('/api/cancel_goal', { method: 'POST' }).catch(() => {});
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

    if (btnClearObsDebug) {
        btnClearObsDebug.addEventListener('click', () => {
            if (costmapVisualizer) costmapVisualizer.clearSelection();
            clearObstacleDebugUI();
            if (obsDebugPanelDiv) obsDebugPanelDiv.style.display = 'none';
            if (statusEl) statusEl.innerText = '已清除障碍物高亮选择';
        });
    }

    if (btnCopyObsDebug) {
        btnCopyObsDebug.addEventListener('click', () => {
            const text = obsDebugPanelDiv ? obsDebugPanelDiv.innerText : '';
            navigator.clipboard.writeText(text).then(() => {
                alert("已复制障碍物排查诊断信息到剪贴板");
            }).catch(() => {});
        });
    }

    return {
        getCurrentTool: () => currentTool,
        setTool: (t) => { currentTool = t; }
    };
}
