import * as THREE from 'three';

// 踏面网格常量：与栅格地图 0.10m 分辨率相匹配，留出微小缝隙增强台阶砖面辨识度
const CELL_SIZE = 0.092;
const HALF_SIZE = CELL_SIZE / 2; // 0.046m

/**
 * 创建紧贴踏面的高清晰度白色边缘方框线 (水平 XY 世界平面，与踏面尺寸 100% 严丝合缝)
 */
function createSquareOutline(halfSize = HALF_SIZE, color = 0xffffff, linewidth = 2) {
    const s = halfSize;
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array([
        -s, -s, 0,   s, -s, 0,
         s, -s, 0,   s,  s, 0,
         s,  s, 0,  -s,  s, 0,
        -s,  s, 0,  -s, -s, 0
    ]);
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const mat = new THREE.LineBasicMaterial({
        color: color,
        linewidth: linewidth,
        transparent: true,
        opacity: 0.98,
        depthTest: true,
        depthWrite: false
    });
    const line = new THREE.LineSegments(geo, mat);
    line.renderOrder = 999;
    return line;
}

/**
 * 创建通用三维吸附标记柱 (方框 + 指示针)
 */
function createMarkerPin(color, sphereR = 0.05, poleH = 0.18, outlineColor = 0xffffff) {
    const group = new THREE.Group();
    const border = createSquareOutline(HALF_SIZE, outlineColor, 3);
    border.position.set(0, 0, 0.002);
    group.add(border);

    const sphereGeo = new THREE.SphereGeometry(sphereR, 16, 16);
    const sphereMat = new THREE.MeshStandardMaterial({ color: color, emissive: color, roughness: 0.3 });
    const sphere = new THREE.Mesh(sphereGeo, sphereMat);
    sphere.position.set(0, 0, poleH);
    group.add(sphere);

    const poleGeo = new THREE.CylinderGeometry(0.01, 0.01, poleH, 8);
    const poleMat = new THREE.MeshBasicMaterial({ color: color });
    const pole = new THREE.Mesh(poleGeo, poleMat);
    pole.rotation.x = Math.PI / 2;
    pole.position.set(0, 0, poleH * 0.5);
    group.add(pole);
    return group;
}

/**
 * 3D 流形拓扑图原生 Three.js 可视化器
 * 负责渲染可通行水平踏面网格 (InstancedMesh)、三维动力学连通边 (Edges) 以及物理级射线表面吸附
 */
export class GraphVisualizer {
    constructor(scene, camera = null, controls = null) {
        this.scene = scene;
        this.camera = camera;
        this.controls = controls;

        this.traversableNodesObject = null;
        this.blockedNodesObject = null;
        this.edgesObject = null;

        this.traversableVisible = true;
        this.blockedVisible = true;
        this.hasAdjustedView = false;

        // 踏面单元几何体 (水平 XY 平面，法线朝上 +Z，与真实台阶完全平行)
        this.stepGeometry = new THREE.PlaneGeometry(CELL_SIZE, CELL_SIZE);

        // 缓存节点与边数据供高精度吸附与邻边拓扑查询
        this.traversableNodeList = [];
        this.blockedNodeList = [];
        this.allNodeList = [];
        this.rawEdges = [];

        // 鼠标悬停白色高亮边缘方框 (尺寸与踏面严格一致，无缝贴合)
        this.hoverOutline = createSquareOutline(HALF_SIZE, 0xffffff, 3);
        this.hoverOutline.visible = false;
        this.scene.add(this.hoverOutline);

        // 起点与终点 3D 吸附标识 Marker
        this.startMarker = null;
        this.goalMarker = null;

        // 调试诊断专属标记 (方块A、方块B及其连通线)
        this.debugMarkerA = null;
        this.debugMarkerB = null;
        this.debugLinkLine = null;

        // 连通边材质 (半透明明亮青绿)
        this.edgeMaterial = new THREE.LineBasicMaterial({
            color: 0x00e676,
            transparent: true,
            opacity: 0.55,
            linewidth: 2
        });
    }

    disposeMesh(mesh) {
        if (!mesh) return;
        this.scene.remove(mesh);
        if (mesh.geometry && mesh.geometry !== this.stepGeometry) mesh.geometry.dispose();
        if (mesh.material) mesh.material.dispose();
    }

    updateNodes(nodes) {
        this.disposeMesh(this.traversableNodesObject);
        this.traversableNodesObject = null;
        this.disposeMesh(this.blockedNodesObject);
        this.blockedNodesObject = null;

        this.traversableNodeList = [];
        this.blockedNodeList = [];
        this.allNodeList = [];
        this.hoverOutline.visible = false;

        if (!nodes || nodes.length === 0) return;

        const count = nodes.length;
        const travNodes = [];
        const blockNodes = [];

        const colorOk = new THREE.Color(0x00e676);   // 自由区 (zone=0, 绿)
        const colorWarn = new THREE.Color(0xffb300); // (旧格式回退用) 复杂台阶 (橙黄)
        const colorBlock = new THREE.Color(0xf44336);// 障碍禁行 (zone=3, 红)
        const colorRing = new THREE.Color(0xfb8c00); // 机体硬禁行环 (zone=2, 橙, 不可入)
        const costNear = new THREE.Color(0x8e24aa);  // 软代价带内侧 (深紫, 代价高)
        const costFar = new THREE.Color(0xd1c4e9);   // 软代价带外侧 (淡紫, 代价低)

        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        let minZ = Infinity, maxZ = -Infinity;

        for (let i = 0; i < count; i++) {
            const n = nodes[i];
            const px = n[0];
            const py = n[1];
            const pz = n[2] + 0.015; // 物理踏面微抬高 1.5cm，浮于原始点云之上避免深度闪烁

            if (px < minX) minX = px; if (px > maxX) maxX = px;
            if (py < minY) minY = py; if (py > maxY) maxY = py;
            if (pz < minZ) minZ = pz; if (pz > maxZ) maxZ = pz;

            const trav = (n[3] !== undefined) ? Math.max(0, Math.min(1, n[3])) : 0;
            const zone = (n.length >= 5 && Number.isFinite(n[4])) ? n[4] : null;
            const isBlocked = (zone !== null) ? (zone >= 2) : (trav >= 0.8);

            const c = new THREE.Color();
            if (zone !== null) {
                // 新格式: 按 CostZone 三档显式配色 (与动态节点渲染器同一色板)
                if (zone >= 3) {
                    c.copy(colorBlock);
                } else if (zone === 2) {
                    c.copy(colorRing);
                } else if (zone === 1) {
                    // 软代价带: trav 0.8→0.05 映射 深紫→淡紫 渐变
                    const t = Math.max(0, Math.min(1, (trav - 0.05) / 0.75));
                    c.lerpColors(costNear, costFar, t);
                } else {
                    c.copy(colorOk);
                }
            } else if (isBlocked) {
                c.copy(colorBlock);
            } else if (trav >= 0.3) {
                c.lerpColors(colorWarn, colorBlock, (trav - 0.3) / 0.5);
            } else {
                c.lerpColors(colorOk, colorWarn, trav / 0.3);
            }

            const item = {
                id: i,
                x: px,
                y: py,
                z: n[2],       // 真实踏面高度 (供 A* 规划与高度差计算)
                renderZ: pz,   // 3D 渲染高度
                traversability: trav,
                zone: zone,
                isBlocked: isBlocked,
                color: c
            };

            if (isBlocked) {
                blockNodes.push(item);
            } else {
                travNodes.push(item);
            }
        }

        this.traversableNodeList = travNodes;
        this.blockedNodeList = blockNodes;
        this.allNodeList = [...travNodes, ...blockNodes];

        // 1. 创建可通行水平踏面矩阵 (InstancedMesh: 水平世界平面，永不随视角旋转)
        if (travNodes.length > 0) {
            const travCount = travNodes.length;
            const travMat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, transparent: true, opacity: 0.88, depthWrite: true });
            this.traversableNodesObject = new THREE.InstancedMesh(this.stepGeometry, travMat, travCount);
            this.traversableNodesObject.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
            this.traversableNodesObject.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(travCount * 3), 3);

            const dummy = new THREE.Object3D();
            for (let i = 0; i < travCount; i++) {
                const node = travNodes[i];
                dummy.position.set(node.x, node.y, node.renderZ);
                dummy.updateMatrix();
                this.traversableNodesObject.setMatrixAt(i, dummy.matrix);
                this.traversableNodesObject.setColorAt(i, node.color);
            }
            this.traversableNodesObject.instanceMatrix.needsUpdate = true;
            if (this.traversableNodesObject.instanceColor) this.traversableNodesObject.instanceColor.needsUpdate = true;
            this.traversableNodesObject.visible = this.traversableVisible;
            this.scene.add(this.traversableNodesObject);
        }

        // 2. 创建阻挡水平踏面矩阵 (逐实例配色: 障碍禁行红 / 机体硬禁行环橙)
        if (blockNodes.length > 0) {
            const blockCount = blockNodes.length;
            const blockMat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, transparent: true, opacity: 0.85, depthWrite: true });
            this.blockedNodesObject = new THREE.InstancedMesh(this.stepGeometry, blockMat, blockCount);
            this.blockedNodesObject.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
            this.blockedNodesObject.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(blockCount * 3), 3);

            const dummy = new THREE.Object3D();
            for (let i = 0; i < blockCount; i++) {
                const node = blockNodes[i];
                dummy.position.set(node.x, node.y, node.renderZ);
                dummy.updateMatrix();
                this.blockedNodesObject.setMatrixAt(i, dummy.matrix);
                this.blockedNodesObject.setColorAt(i, node.color);
            }
            this.blockedNodesObject.instanceMatrix.needsUpdate = true;
            if (this.blockedNodesObject.instanceColor) this.blockedNodesObject.instanceColor.needsUpdate = true;
            this.blockedNodesObject.visible = this.blockedVisible;
            this.scene.add(this.blockedNodesObject);
        }

        // 首次加载自动聚焦视野中心
        if (!this.hasAdjustedView && this.controls && this.camera && count > 0) {
            const cx = (minX + maxX) * 0.5;
            const cy = (minY + maxY) * 0.5;
            const cz = (minZ + maxZ) * 0.5;
            const span = Math.max(maxX - minX, maxY - minY, 6.0);
            this.controls.target.set(cx, cy, cz);
            this.camera.position.set(cx, cy - span * 1.4, cz + span * 1.1);
            this.controls.update();
            this.hasAdjustedView = true;
        }

        console.log(`[GraphVisualizer] 渲染 3D 踏面: 可通行 ${travNodes.length} 个, 禁行/机体环 ${blockNodes.length} 个`);
    }

    updateEdges(edges) {
        if (this.edgesObject) {
            this.scene.remove(this.edgesObject);
            this.edgesObject.geometry.dispose();
            this.edgesObject = null;
        }

        this.rawEdges = edges || [];
        if (!edges || edges.length === 0) return;

        const positions = [];
        for (let i = 0; i < edges.length; i++) {
            const e = edges[i];
            positions.push(e[0], e[1], e[2] + 0.015);
            positions.push(e[3], e[4], e[5] + 0.015);
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        this.edgesObject = new THREE.LineSegments(geo, this.edgeMaterial);
        this.edgesObject.visible = this.traversableVisible;
        this.scene.add(this.edgesObject);

        console.log(`[GraphVisualizer] 渲染 3D 步态连通边: ${edges.length} 条`);
    }

    /**
     * 判断两节点是否在拓扑图中存在连通边
     */
    areNodesConnected(nodeA, nodeB) {
        if (!nodeA || !nodeB || !this.rawEdges || this.rawEdges.length === 0) return false;
        for (let i = 0; i < this.rawEdges.length; i++) {
            const e = this.rawEdges[i];
            const mA1 = Math.hypot(e[0] - nodeA.x, e[1] - nodeA.y) < 0.045 && Math.abs(e[2] - nodeA.z) < 0.15;
            const mB2 = Math.hypot(e[3] - nodeB.x, e[4] - nodeB.y) < 0.045 && Math.abs(e[5] - nodeB.z) < 0.15;
            if (mA1 && mB2) return true;

            const mA2 = Math.hypot(e[3] - nodeA.x, e[4] - nodeA.y) < 0.045 && Math.abs(e[5] - nodeA.z) < 0.15;
            const mB1 = Math.hypot(e[0] - nodeB.x, e[1] - nodeB.y) < 0.045 && Math.abs(e[2] - nodeB.z) < 0.15;
            if (mA2 && mB1) return true;
        }
        return false;
    }

    /**
     * 获取节点的连通邻边总数
     */
    getNodeEdgeCount(node) {
        if (!node || !this.rawEdges || this.rawEdges.length === 0) return 0;
        let count = 0;
        for (let i = 0; i < this.rawEdges.length; i++) {
            const e = this.rawEdges[i];
            if ((Math.hypot(e[0] - node.x, e[1] - node.y) < 0.045 && Math.abs(e[2] - node.z) < 0.15) ||
                (Math.hypot(e[3] - node.x, e[4] - node.y) < 0.045 && Math.abs(e[5] - node.z) < 0.15)) {
                count++;
            }
        }
        return count;
    }

    /**
     * 高精度射线求交与踏面吸附
     * @param {THREE.Raycaster} raycaster
     * @param {boolean} allowBlocked 是否允许拾取禁行/障碍节点 (调试模式时为 true)
     */
    findSnappedNode(raycaster, allowBlocked = false) {
        const list = allowBlocked ? this.allNodeList : this.traversableNodeList;
        if (!list || list.length === 0) return null;
        if (!allowBlocked && !this.traversableVisible) return null;

        const ray = raycaster.ray;
        const origin = ray.origin;
        const dir = ray.direction;

        if (Math.abs(dir.z) < 1e-5) return null;

        const s = HALF_SIZE; // 0.046m
        let bestHitNode = null;
        let bestHitT = Infinity;

        let bestNearNode = null;
        let bestNearDistSq = Infinity;
        let bestNearT = Infinity;
        const maxSnapMargin = 0.10; // 平滑吸附容差 10cm
        // 层偏好阈值: xy 距离差在 5cm 以内的候选视为"同距段",
        // 同距段内取 t 最小 (最靠近相机 = 用户正视的上层表面),
        // 防止俯视多层重叠时吸附到夹缝下方的低层踏面
        const layerPrefDistSq = 0.0025;

        for (let i = 0; i < list.length; i++) {
            const n = list[i];
            const t = (n.renderZ - origin.z) / dir.z;
            if (t <= 0.1) continue;

            const px = origin.x + t * dir.x;
            const py = origin.y + t * dir.y;
            const dx = Math.abs(px - n.x);
            const dy = Math.abs(py - n.y);

            if (dx <= s && dy <= s) {
                if (t < bestHitT) {
                    bestHitT = t;
                    bestHitNode = n;
                }
            } else if (bestHitNode === null) {
                const ex = Math.max(0, dx - s);
                const ey = Math.max(0, dy - s);
                const distSq = ex * ex + ey * ey;
                if (distSq <= maxSnapMargin * maxSnapMargin) {
                    if (distSq < bestNearDistSq - layerPrefDistSq ||
                        (distSq <= bestNearDistSq + layerPrefDistSq && t < bestNearT)) {
                        bestNearDistSq = distSq;
                        bestNearT = t;
                        bestNearNode = n;
                    }
                }
            }
        }

        return bestHitNode || bestNearNode;
    }

    highlightHoveredNode(node) {
        if (!node || !this.hoverOutline) return;
        this.hoverOutline.position.set(node.x, node.y, node.renderZ + 0.002);
        this.hoverOutline.material.color.setHex(node.isBlocked ? 0xff5252 : 0xffffff);
        this.hoverOutline.visible = true;
    }

    clearHoveredNode() {
        if (this.hoverOutline) this.hoverOutline.visible = false;
    }

    setStartMarker(pos) {
        if (this.startMarker) {
            this.scene.remove(this.startMarker);
            this.startMarker = null;
        }
        if (!pos) return;
        this.startMarker = createMarkerPin(0x00e676, 0.05, 0.18);
        const z = pos.renderZ !== undefined ? pos.renderZ : (pos.z + 0.015);
        this.startMarker.position.set(pos.x, pos.y, z);
        this.scene.add(this.startMarker);
    }

    setGoalMarker(pos) {
        if (this.goalMarker) {
            this.scene.remove(this.goalMarker);
            this.goalMarker = null;
        }
        if (!pos) return;
        this.goalMarker = createMarkerPin(0xff1744, 0.05, 0.20);
        const z = pos.renderZ !== undefined ? pos.renderZ : (pos.z + 0.015);
        this.goalMarker.position.set(pos.x, pos.y, z);
        this.scene.add(this.goalMarker);
    }

    setDebugMarkerA(node) {
        this.clearDebugMarkers();
        if (!node) return;
        this.debugMarkerA = createMarkerPin(0x00e5ff, 0.04, 0.14, 0x00e5ff); // 亮青色
        this.debugMarkerA.position.set(node.x, node.y, node.renderZ);
        this.scene.add(this.debugMarkerA);
    }

    setDebugMarkerB(node, isConnected = false) {
        if (this.debugMarkerB) {
            this.scene.remove(this.debugMarkerB);
            this.debugMarkerB = null;
        }
        if (this.debugLinkLine) {
            this.scene.remove(this.debugLinkLine);
            this.debugLinkLine = null;
        }
        if (!node) return;
        this.debugMarkerB = createMarkerPin(0xffab40, 0.04, 0.14, 0xffab40); // 亮橙色
        this.debugMarkerB.position.set(node.x, node.y, node.renderZ);
        this.scene.add(this.debugMarkerB);

        if (this.debugMarkerA) {
            const pA = this.debugMarkerA.position;
            const pB = this.debugMarkerB.position;
            const geo = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(pA.x, pA.y, pA.z + 0.01),
                new THREE.Vector3(pB.x, pB.y, pB.z + 0.01)
            ]);
            const mat = new THREE.LineBasicMaterial({
                color: isConnected ? 0x00e676 : 0xff1744,
                linewidth: 3,
                depthTest: true
            });
            this.debugLinkLine = new THREE.Line(geo, mat);
            this.debugLinkLine.renderOrder = 999;
            this.scene.add(this.debugLinkLine);
        }
    }

    clearDebugMarkers() {
        if (this.debugMarkerA) {
            this.scene.remove(this.debugMarkerA);
            this.debugMarkerA = null;
        }
        if (this.debugMarkerB) {
            this.scene.remove(this.debugMarkerB);
            this.debugMarkerB = null;
        }
        if (this.debugLinkLine) {
            this.scene.remove(this.debugLinkLine);
            this.debugLinkLine = null;
        }
    }

    /**
     * 根据输入文本 (支持 "x,y", "x,y,z" 浮点坐标或 "node_id" 数字) 在全图踏面中检索匹配方块
     * @param {string|number} query
     * @returns {Object|null} 匹配到的踏面节点对象 {id, x, y, z, renderZ, ...}
     */
    findNodeByCoordOrId(query) {
        if (!this.allNodeList || this.allNodeList.length === 0) return null;
        if (query === null || query === undefined) return null;

        const str = String(query).trim();
        if (!str) return null;

        // 1. 若为纯整数 (Node ID)
        if (/^\d+$/.test(str)) {
            const targetId = parseInt(str, 10);
            if (targetId >= 0 && targetId < this.allNodeList.length) {
                const direct = this.allNodeList[targetId];
                if (direct && direct.id === targetId) return direct;
            }
            const found = this.allNodeList.find(n => n.id === targetId);
            if (found) return found;
        }

        // 2. 坐标解析 (提取所有浮点数，支持正负数)
        const matches = str.match(/[-+]?[0-9]*\.?[0-9]+/g);
        if (matches && matches.length >= 2) {
            const qx = parseFloat(matches[0]);
            const qy = parseFloat(matches[1]);
            const qz = matches.length >= 3 ? parseFloat(matches[2]) : null;

            let bestNode = null;
            let bestScore = Infinity;

            for (let i = 0; i < this.allNodeList.length; i++) {
                const n = this.allNodeList[i];
                const dxy2 = (n.x - qx) ** 2 + (n.y - qy) ** 2;
                if (dxy2 > 0.8 * 0.8) continue; // 仅在 0.8m 水平邻域检索

                let score = dxy2;
                if (qz !== null) {
                    const dz = Math.abs(n.z - qz);
                    score += dz * 4.0; // 优先匹配同高程层
                }
                if (score < bestScore) {
                    bestScore = score;
                    bestNode = n;
                }
            }
            if (bestNode) return bestNode;
        }

        return null;
    }

    /**
     * 将相机视角平滑移至指定踏面方块并居中对准
     */
    focusOnNode(node) {
        if (!node || !this.controls || !this.camera) return;
        const rz = node.renderZ !== undefined ? node.renderZ : node.z;
        this.controls.target.set(node.x, node.y, rz);
        this.camera.position.set(node.x - 1.5, node.y - 1.5, rz + 1.8);
        this.controls.update();
    }

    setTraversableVisible(visible) {
        this.traversableVisible = visible;
        if (this.traversableNodesObject) this.traversableNodesObject.visible = visible;
        if (this.edgesObject) this.edgesObject.visible = visible;
        if (!visible && this.hoverOutline) this.hoverOutline.visible = false;
    }

    setBlockedVisible(visible) {
        this.blockedVisible = visible;
        if (this.blockedNodesObject) this.blockedNodesObject.visible = visible;
    }

    setVisible(visible) {
        this.setTraversableVisible(visible);
        this.setBlockedVisible(visible);
        if (this.hoverOutline) this.hoverOutline.visible = false;
        if (this.startMarker) this.startMarker.visible = visible;
        if (this.goalMarker) this.goalMarker.visible = visible;
    }
}
