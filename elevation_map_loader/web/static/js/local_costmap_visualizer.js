import * as THREE from 'three';

/**
 * TEB 结构化几何障碍物 3D 可视化器 (TEB Obstacle Visualizer)
 * 渲染供给 TEB 局部规划器的实体 3D 障碍物 (替代原 2D 栅格地毯)
 *
 * 特性:
 * 1. 默认深色边框: 未选中状态下，边缘采用深暗色 (0x3f1212 / 深暗红)，沉稳真实不抢眼;
 * 2. 交互选中高亮: 选中目标后，边缘切换为高亮纯白 (0xffffff)，本体高亮发光，突出焦点;
 * 3. 完整追溯元数据: 包含障碍物类型、物理来源 (断坎台沿/点云聚类)、生成成因排查与 TEB 优化机制;
 * 4. 射线精确拾取 (Raycasting): 支持在调试模式下点击三维实体障碍物进行诊断。
 */

export const CELL_REASON = {};
export const CELL_REASON_TEXT = {};

export class LocalCostmapVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.visible = true;
        this.obstaclesVisible = true;

        this.group = new THREE.Group();
        this.scene.add(this.group);

        this.obstaclesGroup = new THREE.Group();
        this.group.add(this.obstaclesGroup);

        this.lastTebObstacles = null;
        this.selectedObstacleId = null;
        this.hoveredObstacleId = null;
        this.obstacleMeshMap = new Map(); // id -> { bodyMesh, edgeMesh, groundMesh, data }

        // ---- 默认状态材质 (深色边缘) ----
        this.bodyMat = new THREE.MeshLambertMaterial({
            color: 0xef4444,        // 经典致命红
            transparent: true,
            opacity: 0.60,
            depthWrite: true,
            side: THREE.DoubleSide
        });

        this.edgeMat = new THREE.LineBasicMaterial({
            color: 0x3f0a0a,        // 现在的边缘改成深暗红褐色
            linewidth: 2,
            transparent: true,
            opacity: 0.95
        });

        this.groundStripMat = new THREE.MeshBasicMaterial({
            color: 0x991b1b,        // 深暗红地面警示衬底
            transparent: true,
            opacity: 0.75,
            depthWrite: false,
            side: THREE.DoubleSide
        });

        // ---- 选中高亮状态材质 (纯白边缘 + 高亮发光本体) ----
        this.selectedBodyMat = new THREE.MeshLambertMaterial({
            color: 0xff3b30,
            emissive: 0x330000,     // 自发光强调
            transparent: true,
            opacity: 0.90,
            depthWrite: true,
            side: THREE.DoubleSide
        });

        this.selectedEdgeMat = new THREE.LineBasicMaterial({
            color: 0xffffff,        // 选中时边缘变成耀眼纯白
            linewidth: 3,
            transparent: true,
            opacity: 1.0
        });

        this.selectedGroundStripMat = new THREE.MeshBasicMaterial({
            color: 0xffffff,        // 地面衬白光环
            transparent: true,
            opacity: 0.85,
            depthWrite: false,
            side: THREE.DoubleSide
        });
    }

    // 兼容原接口 (已弃用 2D 地毯)
    update(costmapMsg) {}
    updateDebug(debugMsg) {}
    updateDebugNodes(debugNodesMsg) {}
    getWinnerNode(nodeId) { return null; }
    setDebugMode(on) {}
    getCellInfo(worldX, worldY) { return null; }
    highlightCell(worldX, worldY) { return null; }
    clearHighlight() {}

    setVisible(visible) {
        this.visible = visible;
        this.group.visible = visible;
    }

    setObstaclesVisible(visible) {
        this.obstaclesVisible = visible;
        this.obstaclesGroup.visible = visible && this.visible;
    }

    clearObstacles() {
        this.obstacleMeshMap.clear();
        while (this.obstaclesGroup.children.length > 0) {
            const child = this.obstaclesGroup.children[0];
            this.obstaclesGroup.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
    }

    /**
     * 射线精确拾取鼠标点击或悬停处的障碍物
     * 采用两级高精度拾取机制：
     * 1. 严格仅对实体表面 (bodyMesh / groundMesh) 进行三角面光线求交，
     *    完全排除 LineSegments (edgeMesh) 避免其 1.0m 巨大默认容差导致的隔空误选；
     * 2. 若光线在实体边缘缝隙处未直击表面，在极小的贴身容差 (15cm) 内寻找距离光线最近的几何实体，
     *    实现贴近吸附，绝对杜绝位置错位与隔空跳选。
     * @param {THREE.Raycaster} raycaster
     * @returns {Object|null} obstacle data
     */
    pickObstacle(raycaster) {
        if (!this.obstaclesVisible || !this.visible) return null;

        // 1. 第一阶段：对所有实体三维表面 (Mesh) 进行三角形精确光线求交
        const solidMeshes = [];
        for (const item of this.obstacleMeshMap.values()) {
            if (item.bodyMesh) solidMeshes.push(item.bodyMesh);
            if (item.groundMesh) solidMeshes.push(item.groundMesh);
        }

        if (solidMeshes.length > 0) {
            const hits = raycaster.intersectObjects(solidMeshes, false);
            for (const hit of hits) {
                if (hit.object && hit.object.userData && hit.object.userData.isObstacle) {
                    return hit.object.userData.obstacle;
                }
            }
        }

        // 2. 第二阶段：贴身微小容差 (15cm) 空间近场吸附 (仅对直接指向边缘缝隙时辅助捕获)
        let bestObs = null;
        let minRayDist = 0.15; // 严格限制在 15cm 容差内，严禁 1.0m 隔空误触
        let bestCamDist = Infinity;

        const ray = raycaster.ray;
        const ptRay = new THREE.Vector3();
        const ptSeg = new THREE.Vector3();

        for (const item of this.obstacleMeshMap.values()) {
            const obs = item.data;
            if (!obs) continue;

            let distToRay = Infinity;
            let camDist = Infinity;

            if (obs.type === 'circle') {
                const cx = obs.x !== undefined ? obs.x : 0;
                const cy = obs.y !== undefined ? obs.y : 0;
                const cz = (obs.z !== undefined ? obs.z : 0) + 0.20;
                const center = new THREE.Vector3(cx, cy, cz);

                // 确保在相机前方
                const toCenter = new THREE.Vector3().subVectors(center, ray.origin);
                const projT = toCenter.dot(ray.direction);
                if (projT > 0.2) {
                    const r = Math.max(0.10, obs.radius || 0.15);
                    distToRay = Math.max(0, ray.distanceToPoint(center) - r);
                    camDist = projT;
                }
            } else if (obs.type === 'line' && obs.start && obs.end) {
                const zAvg = ((obs.start[2] + obs.end[2]) * 0.5) + 0.20;
                const p1 = new THREE.Vector3(obs.start[0], obs.start[1], zAvg);
                const p2 = new THREE.Vector3(obs.end[0], obs.end[1], zAvg);

                const dSq = ray.distanceSqToSegment(p1, p2, ptRay, ptSeg);
                const projT = new THREE.Vector3().subVectors(ptRay, ray.origin).dot(ray.direction);
                if (projT > 0.2) {
                    distToRay = Math.sqrt(Math.max(0, dSq));
                    camDist = projT;
                }
            } else if (obs.type === 'polygon' && obs.points && obs.points.length >= 3) {
                let sx = 0, sy = 0, sz = 0;
                for (const p of obs.points) {
                    sx += p[0]; sy += p[1]; sz += p[2];
                }
                const n = obs.points.length;
                const center = new THREE.Vector3(sx / n, sy / n, sz / n + 0.20);
                const projT = new THREE.Vector3().subVectors(center, ray.origin).dot(ray.direction);
                if (projT > 0.2) {
                    distToRay = ray.distanceToPoint(center);
                    camDist = projT;
                }
            }

            if (distToRay < minRayDist) {
                minRayDist = distToRay;
                bestCamDist = camDist;
                bestObs = obs;
            }
        }

        return bestObs;
    }

    /**
     * 悬停高亮指定 ID 的障碍物 (边缘变白)
     * @param {number|string} obsId
     */
    setHoveredObstacle(obsId) {
        if (this.hoveredObstacleId === obsId) return;
        this.hoveredObstacleId = obsId;
        this._applyHighlight();
    }

    /** 清除悬停高亮 */
    clearHoveredObstacle() {
        if (this.hoveredObstacleId === null) return;
        this.hoveredObstacleId = null;
        this._applyHighlight();
    }

    /**
     * 选中锁定指定 ID 的障碍物 (边缘变白，本体发亮)
     * @param {number|string} obsId
     */
    selectObstacle(obsId) {
        this.selectedObstacleId = obsId;
        this._applyHighlight();
    }

    /** 清除当前高亮选择与悬停 */
    clearSelection() {
        this.selectedObstacleId = null;
        this.hoveredObstacleId = null;
        this._applyHighlight();
    }

    getSelectedObstacle() {
        if (!this.selectedObstacleId || !this.obstacleMeshMap.has(this.selectedObstacleId)) return null;
        return this.obstacleMeshMap.get(this.selectedObstacleId).data;
    }

    getHoveredObstacle() {
        if (!this.hoveredObstacleId || !this.obstacleMeshMap.has(this.hoveredObstacleId)) return null;
        return this.obstacleMeshMap.get(this.hoveredObstacleId).data;
    }

    _applyHighlight() {
        for (const [id, item] of this.obstacleMeshMap.entries()) {
            const isHighlighted = (id === this.selectedObstacleId || id === this.hoveredObstacleId);
            if (item.bodyMesh) {
                item.bodyMesh.material = isHighlighted ? this.selectedBodyMat : this.bodyMat;
                item.bodyMesh.renderOrder = isHighlighted ? 15 : 10;
            }
            if (item.edgeMesh) {
                item.edgeMesh.material = isHighlighted ? this.selectedEdgeMat : this.edgeMat;
                item.edgeMesh.renderOrder = isHighlighted ? 16 : 11;
            }
            if (item.groundMesh) {
                item.groundMesh.material = isHighlighted ? this.selectedGroundStripMat : this.groundStripMat;
                item.groundMesh.renderOrder = isHighlighted ? 14 : 9;
            }
        }
    }

    /**
     * 更新并渲染供给 TEB 局部规划器的结构化几何障碍物
     * @param {Array} obstacles [{id, type: 'circle'|'line'|'polygon', source, reason, ...}]
     */
    updateTebObstacles(obstacles) {
        this.lastTebObstacles = obstacles;
        this.clearObstacles();

        if (!obstacles || obstacles.length === 0) {
            return;
        }

        const wallH = 0.40;  // 障碍物物理高度 (40cm，匹配机器狗高度)
        const wallW = 0.14;  // 线段障碍物实体厚度 (14cm，全视角实体与适度拾取靶面)

        for (const obs of obstacles) {
            if (!obs) continue;
            const obsId = obs.id !== undefined ? obs.id : `${obs.type}_${Math.random()}`;
            const isHighlighted = (obsId === this.selectedObstacleId || obsId === this.hoveredObstacleId);

            const curBodyMat = isHighlighted ? this.selectedBodyMat : this.bodyMat;
            const curEdgeMat = isHighlighted ? this.selectedEdgeMat : this.edgeMat;
            const curGroundMat = isHighlighted ? this.selectedGroundStripMat : this.groundStripMat;
            const bodyOrder = isHighlighted ? 15 : 10;
            const edgeOrder = isHighlighted ? 16 : 11;
            const groundOrder = isHighlighted ? 14 : 9;

            let bodyMesh = null, edgeMesh = null, groundMesh = null;

            if (obs.type === 'line' && obs.start && obs.end) {
                const [x1, y1, z1] = obs.start;
                const [x2, y2, z2] = obs.end;
                const z = (z1 + z2) * 0.5 + 0.005;

                const dx = x2 - x1;
                const dy = y2 - y1;
                const len = Math.hypot(dx, dy);
                if (len < 1e-4) continue;

                const mx = (x1 + x2) * 0.5;
                const my = (y1 + y2) * 0.5;
                const mz = z + wallH * 0.5;
                const yaw = Math.atan2(dy, dx);

                // 1. 实体 3D 长方体防护墙 (带 6 个面)
                const boxGeo = new THREE.BoxGeometry(len, wallW, wallH);
                bodyMesh = new THREE.Mesh(boxGeo, curBodyMat);
                bodyMesh.position.set(mx, my, mz);
                bodyMesh.rotation.z = yaw;
                bodyMesh.renderOrder = bodyOrder;
                bodyMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(bodyMesh);

                // 2. 3D 棱边线框 (未选中为深色，选中为纯白)
                const edgesGeo = new THREE.EdgesGeometry(boxGeo);
                edgeMesh = new THREE.LineSegments(edgesGeo, curEdgeMat);
                edgeMesh.position.set(mx, my, mz);
                edgeMesh.rotation.z = yaw;
                edgeMesh.renderOrder = edgeOrder;
                edgeMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(edgeMesh);

                // 3. 贴地警示底带
                const stripW = 0.22;
                const stripGeo = new THREE.PlaneGeometry(len, stripW);
                groundMesh = new THREE.Mesh(stripGeo, curGroundMat);
                groundMesh.position.set(mx, my, z + 0.005);
                groundMesh.rotation.z = yaw;
                groundMesh.renderOrder = groundOrder;
                groundMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(groundMesh);

            } else if (obs.type === 'circle') {
                const { x, y, z: rawZ, radius: r } = obs;
                const z = (rawZ !== undefined ? rawZ : 0.0) + 0.005;
                const radius = Math.max(0.12, r || 0.16);

                // 1. 完整封闭的 3D 圆柱体 (带上下实心盖板)
                const cylGeo = new THREE.CylinderGeometry(radius, radius, wallH, 32, 1, false);
                bodyMesh = new THREE.Mesh(cylGeo, curBodyMat);
                bodyMesh.rotation.x = Math.PI * 0.5;
                bodyMesh.position.set(x, y, z + wallH * 0.5);
                bodyMesh.renderOrder = bodyOrder;
                bodyMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(bodyMesh);

                // 2. 顶盖/底盘圆环 + 4 条垂直母线框线
                const edgesGeo = new THREE.EdgesGeometry(cylGeo, 25);
                edgeMesh = new THREE.LineSegments(edgesGeo, curEdgeMat);
                edgeMesh.rotation.x = Math.PI * 0.5;
                edgeMesh.position.set(x, y, z + wallH * 0.5);
                edgeMesh.renderOrder = edgeOrder;
                edgeMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(edgeMesh);

                // 3. 地面警示底盘
                const diskGeo = new THREE.CircleGeometry(radius + 0.03, 32);
                groundMesh = new THREE.Mesh(diskGeo, curGroundMat);
                groundMesh.position.set(x, y, z + 0.005);
                groundMesh.renderOrder = groundOrder;
                groundMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(groundMesh);

            } else if (obs.type === 'polygon' && obs.points && obs.points.length >= 3) {
                const pts = obs.points;
                const z = pts[0][2] + 0.005;

                // 1. 挤出 3D 棱柱
                const shape = new THREE.Shape();
                shape.moveTo(pts[0][0], pts[0][1]);
                for (let i = 1; i < pts.length; i++) {
                    shape.lineTo(pts[i][0], pts[i][1]);
                }
                shape.closePath();

                const polyGeo = new THREE.ExtrudeGeometry(shape, {
                    depth: wallH,
                    bevelEnabled: false
                });
                bodyMesh = new THREE.Mesh(polyGeo, curBodyMat);
                bodyMesh.position.set(0, 0, z);
                bodyMesh.renderOrder = bodyOrder;
                bodyMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(bodyMesh);

                // 2. 多边形外棱线
                const edgesGeo = new THREE.EdgesGeometry(polyGeo);
                edgeMesh = new THREE.LineSegments(edgesGeo, curEdgeMat);
                edgeMesh.position.set(0, 0, z);
                edgeMesh.renderOrder = edgeOrder;
                edgeMesh.userData = { isObstacle: true, obstacle: obs, id: obsId };
                this.obstaclesGroup.add(edgeMesh);
            }

            if (bodyMesh) {
                this.obstacleMeshMap.set(obsId, {
                    bodyMesh,
                    edgeMesh,
                    groundMesh,
                    data: obs
                });
            }
        }

        this.obstaclesGroup.visible = this.obstaclesVisible && this.visible;
    }
}
