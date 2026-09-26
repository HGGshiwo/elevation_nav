import * as THREE from 'three';

/**
 * 可通行走廊单步扩散与凸包约束 3D 调试可视化器 (SFC Debug Visualizer)
 * 专用于排查局部路径点、8方向单步邻域扩散范围、层级蔓延以及 2D 凸包约束生成。
 */
export class SfcDebugVisualizer {
    constructor(scene, camera, controls) {
        this.scene = scene;
        this.camera = camera;
        this.controls = controls;
        this.visible = false;

        this.corridors = [];
        this.selectedIndex = -1;
        this.onSelectCallback = null;

        // 根显示容器
        this.group = new THREE.Group();
        this.group.name = "SfcDebugGroup";
        this.group.visible = false;
        this.scene.add(this.group);

        // 1. 所有路径点中心标记组 (原始输入点 W_i)
        this.waypointGroup = new THREE.Group();
        this.group.add(this.waypointGroup);

        // 2. 全局所有优化物理点标记组 (优化物理点 p_i)
        this.allOptPointGroup = new THREE.Group();
        this.group.add(this.allOptPointGroup);

        // 3. 选中走廊的凸多边形包络组
        this.polygonGroup = new THREE.Group();
        this.group.add(this.polygonGroup);

        // 4. 选中走廊的所有扩散节点与层级提示组
        this.nodesGroup = new THREE.Group();
        this.group.add(this.nodesGroup);

        // 5. 连通拓扑射线组 (从中心指向各扩散节点)
        this.raysGroup = new THREE.Group();
        this.group.add(this.raysGroup);

        // 6. 选中项专用的配对指示线、控制点与光环组
        this.optPointGroup = new THREE.Group();
        this.group.add(this.optPointGroup);

        // --- 材质定义 ---
        // 原始航点材质 (未选中: 青蓝，选中: 亮黄)
        this.wpGeometry = new THREE.SphereGeometry(0.08, 16, 16);
        this.wpMaterial = new THREE.MeshStandardMaterial({
            color: 0x00e5ff,
            emissive: 0x005577,
            roughness: 0.2,
            metalness: 0.2
        });
        this.selectedWpMaterial = new THREE.MeshStandardMaterial({
            color: 0xffeb3b,
            emissive: 0xff6f00,
            roughness: 0.1,
            metalness: 0.3
        });

        // 优化物理点材质 (未选中: 典雅浅紫 / 违约浅红)
        this.optPGeometry = new THREE.SphereGeometry(0.065, 16, 16);
        this.optPMaterial = new THREE.MeshStandardMaterial({
            color: 0xba68c8,
            emissive: 0x4a148c,
            roughness: 0.2,
            metalness: 0.2
        });
        this.optPBreachMaterial = new THREE.MeshStandardMaterial({
            color: 0xef5350,
            emissive: 0xb71c1c,
            roughness: 0.2,
            metalness: 0.2
        });

        // 优化物理点选中高亮材质 (同步变亮黄！违约则为警示红)
        this.selectedOptPMaterial = new THREE.MeshStandardMaterial({
            color: 0xffea00,
            emissive: 0xff6f00,
            roughness: 0.1,
            metalness: 0.4
        });
        this.selectedOptPBreachMaterial = new THREE.MeshStandardMaterial({
            color: 0xff1744,
            emissive: 0xd50000,
            roughness: 0.1,
            metalness: 0.4
        });

        // 凸包边框线材质
        this.hullLineMat = new THREE.LineBasicMaterial({
            color: 0xffd600,
            linewidth: 3,
            transparent: true,
            opacity: 0.95,
            depthTest: true
        });

        // 凸包半透明填充材质
        this.hullFillMat = new THREE.MeshStandardMaterial({
            color: 0xffea00,
            emissive: 0x554400,
            transparent: true,
            opacity: 0.30,
            side: THREE.DoubleSide,
            depthWrite: false
        });

        // 同层扩散节点材质 (翡翠绿)
        this.sameLayerNodeMat = new THREE.MeshStandardMaterial({
            color: 0x00e676,
            emissive: 0x004d20,
            transparent: true,
            opacity: 0.85
        });

        // 跨层/蔓延节点材质 (荧光红/紫，极度醒目)
        this.crossLayerNodeMat = new THREE.MeshStandardMaterial({
            color: 0xff1744,
            emissive: 0xd50000,
            transparent: true,
            opacity: 0.95
        });

        // 扩散射线材质
        this.rayLineMat = new THREE.LineBasicMaterial({
            color: 0x80d8ff,
            transparent: true,
            opacity: 0.45
        });

        this.crossLayerRayMat = new THREE.LineBasicMaterial({
            color: 0xff5252,
            transparent: true,
            opacity: 0.85
        });

        this.nodeBoxGeo = new THREE.BoxGeometry(0.08, 0.08, 0.03);

        // 3D 视口点击交互：支持直接在 3D 视图中点击原始航点或优化物理点切换走廊
        const domEl = controls?.domElement;
        if (domEl) {
            let startX = 0, startY = 0;
            domEl.addEventListener('pointerdown', (e) => {
                startX = e.clientX;
                startY = e.clientY;
            });
            domEl.addEventListener('pointerup', (e) => {
                if (e.button !== 0 || !this.visible) return;
                // 仅响应轻击，排除拖拽视角
                if (Math.hypot(e.clientX - startX, e.clientY - startY) > 5) return;

                const rect = domEl.getBoundingClientRect();
                const mouse = new THREE.Vector2(
                    ((e.clientX - rect.left) / rect.width) * 2 - 1,
                    -((e.clientY - rect.top) / rect.height) * 2 + 1
                );
                const rc = new THREE.Raycaster();
                rc.setFromCamera(mouse, camera);
                const candidates = [...this.waypointGroup.children, ...this.allOptPointGroup.children];
                const hits = rc.intersectObjects(candidates, false);
                if (hits.length > 0) {
                    const hitObj = hits[0].object;
                    if (hitObj.userData && hitObj.userData.corridorIndex !== undefined) {
                        this.selectCorridor(hitObj.userData.corridorIndex);
                    }
                }
            });
        }
    }

    setVisible(visible) {
        this.visible = visible;
        this.group.visible = visible;
    }

    setOnSelectCallback(cb) {
        this.onSelectCallback = cb;
    }

    /**
     * 更新来自后端的走廊调试 JSON 数据
     * @param {Array} corridors - [{idx, root_id, x, y, z, layer, polygon: [[x,y,z],...], nodes: [{id,x,y,z,l,r,c},...], opt_p: [x,y,z], opt_q: [x,y], viol: 0.0}]
     */
    updateData(corridors) {
        this.corridors = corridors || [];
        this._renderAllPoints();

        // 若之前选中的索引仍然有效，刷新高亮；否则默认选中第 0 个点
        if (this.selectedIndex >= 0 && this.selectedIndex < this.corridors.length) {
            this.selectCorridor(this.selectedIndex, false);
        } else if (this.corridors.length > 0) {
            this.selectCorridor(0, false);
        } else {
            this.clearHighlight();
        }
    }

    /**
     * 选择并高亮特定序号的局部路径点走廊
     */
    selectCorridor(index, triggerCallback = true) {
        if (index < 0 || index >= this.corridors.length) {
            this.selectedIndex = -1;
            this.clearHighlight();
            return;
        }

        this.selectedIndex = index;
        const c = this.corridors[index];

        // 1. 刷新原始路径点小球材质与缩放 (选中点变亮黄色)
        this.waypointGroup.children.forEach((child, i) => {
            if (child.isMesh) {
                child.material = (i === index) ? this.selectedWpMaterial : this.wpMaterial;
                child.scale.setScalar(i === index ? 1.4 : 1.0);
            }
        });

        // 2. 刷新对应优化物理点小球材质与缩放 (同步变亮黄色！违约变红)
        this.allOptPointGroup.children.forEach((child, i) => {
            if (child.isMesh) {
                const corr = this.corridors[i];
                const isBreached = (corr && corr.viol !== undefined && corr.viol > 0.001);
                if (i === index) {
                    child.material = isBreached ? this.selectedOptPBreachMaterial : this.selectedOptPMaterial;
                    child.scale.setScalar(1.6);
                } else {
                    child.material = isBreached ? this.optPBreachMaterial : this.optPMaterial;
                    child.scale.setScalar(1.0);
                }
            }
        });

        // 3. 渲染选中的凸包多边形、扩散节点与 1:1 配对连线
        this._renderSelectedCorridor(c);

        if (triggerCallback && this.onSelectCallback) {
            this.onSelectCallback(index, c);
        }
    }

    /**
     * 将相机视角平滑移至当前选中的走廊中心
     */
    focusOnSelected() {
        if (this.selectedIndex < 0 || this.selectedIndex >= this.corridors.length) return;
        const c = this.corridors[this.selectedIndex];
        if (!this.controls || !this.camera) return;

        const targetPos = new THREE.Vector3(c.x, c.y, c.z);
        this.controls.target.copy(targetPos);
        
        // 将相机摆在斜上方 2.5m 处
        this.camera.position.set(c.x - 1.8, c.y - 1.8, c.z + 2.2);
        this.controls.update();
    }

    _renderAllPoints() {
        // 清空所有原始航点与优化物理点
        this._clearGroup(this.waypointGroup);
        this._clearGroup(this.allOptPointGroup);

        if (!this.corridors || this.corridors.length === 0) return;

        this.corridors.forEach((c, idx) => {
            const isSelected = (idx === this.selectedIndex);

            // 1. 原始规划航点小球 W_i
            const wpMesh = new THREE.Mesh(
                this.wpGeometry,
                isSelected ? this.selectedWpMaterial : this.wpMaterial
            );
            wpMesh.position.set(c.x, c.y, c.z + 0.05);
            wpMesh.scale.setScalar(isSelected ? 1.4 : 1.0);
            wpMesh.userData = { corridorIndex: idx, rootId: c.root_id, type: 'waypoint' };
            this.waypointGroup.add(wpMesh);

            // 2. 对应优化物理点小球 p_i
            if (c.opt_p) {
                const [px, py, pz] = c.opt_p;
                const isBreached = (c.viol !== undefined && c.viol > 0.001);

                let optMat = this.optPMaterial;
                if (isSelected) {
                    optMat = isBreached ? this.selectedOptPBreachMaterial : this.selectedOptPMaterial;
                } else if (isBreached) {
                    optMat = this.optPBreachMaterial;
                }

                const optMesh = new THREE.Mesh(this.optPGeometry, optMat);
                optMesh.position.set(px, py, pz + 0.06);
                optMesh.scale.setScalar(isSelected ? 1.6 : 1.0);
                optMesh.userData = { corridorIndex: idx, rootId: c.root_id, type: 'opt_p' };
                this.allOptPointGroup.add(optMesh);
            }
        });
    }

    _renderSelectedCorridor(c) {
        // 清理旧选中物
        this._clearGroup(this.polygonGroup);
        this._clearGroup(this.nodesGroup);
        this._clearGroup(this.raysGroup);
        this._clearGroup(this.optPointGroup);

        if (!c) return;

        const rootLayer = c.layer !== undefined ? c.layer : 0;
        const rootPos = new THREE.Vector3(c.x, c.y, c.z + 0.04);

        // 1. 渲染凸包轮廓线与半透明面片
        const poly = c.polygon || [];
        if (poly.length >= 3) {
            // 凸包边界线
            const linePts = [];
            for (let i = 0; i < poly.length; ++i) {
                const p1 = poly[i];
                const p2 = poly[(i + 1) % poly.length];
                linePts.push(p1[0], p1[1], (p1[2] !== undefined ? p1[2] : c.z) + 0.05);
                linePts.push(p2[0], p2[1], (p2[2] !== undefined ? p2[2] : c.z) + 0.05);
            }
            const lineGeo = new THREE.BufferGeometry();
            lineGeo.setAttribute('position', new THREE.Float32BufferAttribute(linePts, 3));
            const lineSegs = new THREE.LineSegments(lineGeo, this.hullLineMat);
            this.polygonGroup.add(lineSegs);

            // 凸包扇形三角面
            const triPts = [];
            const centerZ = c.z + 0.045;
            for (let i = 0; i < poly.length; ++i) {
                const p1 = poly[i];
                const p2 = poly[(i + 1) % poly.length];
                triPts.push(c.x, c.y, centerZ);
                triPts.push(p1[0], p1[1], (p1[2] !== undefined ? p1[2] : c.z) + 0.045);
                triPts.push(p2[0], p2[1], (p2[2] !== undefined ? p2[2] : c.z) + 0.045);
            }
            const triGeo = new THREE.BufferGeometry();
            triGeo.setAttribute('position', new THREE.Float32BufferAttribute(triPts, 3));
            triGeo.computeVertexNormals();
            const fillMesh = new THREE.Mesh(triGeo, this.hullFillMat);
            this.polygonGroup.add(fillMesh);
        }

        // 2. 渲染所有扩散节点 (同层为绿色，跨层/楼上扩散为红色闪耀提示)
        const nodes = c.nodes || [];
        const rayPts = [];
        const crossRayPts = [];

        nodes.forEach(nd => {
            const isCrossLayer = (nd.l !== undefined && nd.l !== rootLayer);
            const mat = isCrossLayer ? this.crossLayerNodeMat : this.sameLayerNodeMat;
            const nodeMesh = new THREE.Mesh(this.nodeBoxGeo, mat);
            nodeMesh.position.set(nd.x, nd.y, nd.z + 0.015);
            this.nodesGroup.add(nodeMesh);

            // 连线
            if (isCrossLayer) {
                crossRayPts.push(rootPos.x, rootPos.y, rootPos.z);
                crossRayPts.push(nd.x, nd.y, nd.z + 0.02);
            } else {
                rayPts.push(rootPos.x, rootPos.y, rootPos.z);
                rayPts.push(nd.x, nd.y, nd.z + 0.02);
            }
        });

        // 3. 渲染连接射线
        if (rayPts.length > 0) {
            const rayGeo = new THREE.BufferGeometry();
            rayGeo.setAttribute('position', new THREE.Float32BufferAttribute(rayPts, 3));
            this.raysGroup.add(new THREE.LineSegments(rayGeo, this.rayLineMat));
        }

        if (crossRayPts.length > 0) {
            const crossRayGeo = new THREE.BufferGeometry();
            crossRayGeo.setAttribute('position', new THREE.Float32BufferAttribute(crossRayPts, 3));
            this.raysGroup.add(new THREE.LineSegments(crossRayGeo, this.crossLayerRayMat));
        }

        // 4. 渲染优化后 1:1 物理点与控制点的专属高亮细节
        if (c.opt_p) {
            const [px, py, pz] = c.opt_p;
            const isBreached = (c.viol !== undefined && c.viol > 0.001);

            // 物理点贴地光环 (选中为亮黄色/金色光环，违约为红色光环)
            const ringGeo = new THREE.RingGeometry(0.08, 0.12, 24);
            const ringMat = new THREE.MeshBasicMaterial({
                color: isBreached ? 0xff1744 : 0xffea00,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.90
            });
            const ringMesh = new THREE.Mesh(ringGeo, ringMat);
            ringMesh.position.set(px, py, pz + 0.02);
            this.optPointGroup.add(ringMesh);

            // 原始黄色航点 -> 优化黄色物理点的 1:1 强配对位移指示线 (亮黄色虚线，直观指示从原始点到优化点的映射)
            const dispPts = [c.x, c.y, c.z + 0.05, px, py, pz + 0.06];
            const dispGeo = new THREE.BufferGeometry();
            dispGeo.setAttribute('position', new THREE.Float32BufferAttribute(dispPts, 3));
            const dispLineMat = new THREE.LineDashedMaterial({
                color: isBreached ? 0xff5252 : 0xffea00,
                dashSize: 0.04,
                gapSize: 0.02,
                linewidth: 3
            });
            const dispLine = new THREE.Line(dispGeo, dispLineMat);
            dispLine.computeLineDistances();
            this.optPointGroup.add(dispLine);

            // 若存在控制点 q_i，渲染控制点菱形标记与控制引力虚线
            if (c.opt_q) {
                const [qx, qy] = c.opt_q;
                const qMat = new THREE.MeshStandardMaterial({
                    color: 0xffab00,
                    emissive: 0xff6d00,
                    roughness: 0.2
                });
                const qMesh = new THREE.Mesh(new THREE.OctahedronGeometry(0.055), qMat);
                qMesh.position.set(qx, qy, pz + 0.06);
                this.optPointGroup.add(qMesh);

                const qLinePts = [qx, qy, pz + 0.06, px, py, pz + 0.06];
                const qLineGeo = new THREE.BufferGeometry();
                qLineGeo.setAttribute('position', new THREE.Float32BufferAttribute(qLinePts, 3));
                const qLineMat = new THREE.LineDashedMaterial({
                    color: 0xffab00,
                    dashSize: 0.03,
                    gapSize: 0.02
                });
                const qLine = new THREE.Line(qLineGeo, qLineMat);
                qLine.computeLineDistances();
                this.optPointGroup.add(qLine);
            }
        }
    }

    clearHighlight() {
        this._clearGroup(this.polygonGroup);
        this._clearGroup(this.nodesGroup);
        this._clearGroup(this.raysGroup);
        this._clearGroup(this.optPointGroup);
    }

    clear() {
        this.corridors = [];
        this.selectedIndex = -1;
        this._clearGroup(this.waypointGroup);
        this._clearGroup(this.allOptPointGroup);
        this._clearGroup(this.polygonGroup);
        this._clearGroup(this.nodesGroup);
        this._clearGroup(this.raysGroup);
        this._clearGroup(this.optPointGroup);
    }

    _clearGroup(grp) {
        while (grp.children.length > 0) {
            const ch = grp.children[0];
            grp.remove(ch);
            if (ch.geometry) ch.geometry.dispose();
        }
    }
}
