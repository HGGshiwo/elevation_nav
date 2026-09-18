import * as THREE from 'three';

/**
 * 拓扑流形管道 3D 可视化器 (Topological Corridor Visualizer)
 * 渲染沿 A* 全局路径生成的 3D 拓扑流形管道 (踏面网格、立体防护包络与边界轮廓)
 * 替代旧版 2D 局部代价地图，呈现 TEB 局部规划器的三维流形运动包络
 */
export class CorridorVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.visible = true;

        this.group = new THREE.Group();
        this.scene.add(this.group);

        // 管道踏面实例网格 (InstancedMesh)
        this.surfaceMesh = null;

        // 管道 3D 边界轮廓线与防护墙
        this.boundaryGroup = new THREE.Group();
        this.group.add(this.boundaryGroup);

        // 材质定义
        // 踏面材质: 半透明青绿/翡翠绿荧光踏面
        this.surfaceMaterial = new THREE.MeshStandardMaterial({
            color: 0x00e676,
            emissive: 0x004d40,
            transparent: true,
            opacity: 0.45,
            roughness: 0.3,
            metalness: 0.1,
            side: THREE.DoubleSide,
            depthWrite: false
        });

        // 管道侧壁边界线材质 (亮青色高反差荧光线)
        this.boundaryLineMaterial = new THREE.LineBasicMaterial({
            color: 0x00f5ff,
            linewidth: 2.5,
            transparent: true,
            opacity: 0.95
        });

        // 管道连续曲面与立体防护墙材质 (高透发光曲面)
        this.wallMaterial = new THREE.MeshStandardMaterial({
            color: 0x00e5ff,
            emissive: 0x003344,
            transparent: true,
            opacity: 0.32,
            roughness: 0.2,
            metalness: 0.1,
            side: THREE.DoubleSide,
            depthWrite: false
        });

        // 管道内部障碍物组 (TEB 3D 几何障碍物)
        this.obstaclesGroup = new THREE.Group();
        this.obstaclesVisible = false;
        this.obstaclesGroup.visible = false;
        this.group.add(this.obstaclesGroup);

        this.obsBodyMat = new THREE.MeshStandardMaterial({
            color: 0xff3b30,
            roughness: 0.4,
            metalness: 0.1,
            transparent: true,
            opacity: 0.75
        });
        this.obsEdgeMat = new THREE.LineBasicMaterial({
            color: 0xff9500,
            linewidth: 2
        });

        this.lastNodeCount = 0;
    }

    setVisible(visible) {
        this.visible = visible;
        this.group.visible = visible;
    }

    /**
     * 更新拓扑管道 3D 呈现
     * @param {Array<Array<number>>} nodes - 管道内的三维节点数组 [[x, y, z], ...]
     * @param {Array<Array<number>>} corridorLines - 后端 C++ 精确计算的 3D 边界线端点 [[x,y,z], ...]
     * @param {Array<Array<number>>} corridorWalls - 后端 C++ 精确计算的立体防护墙顶点 [[x,y,z], ...]
     */
    update(nodes, corridorLines = null, corridorWalls = null) {
        const hasLines = corridorLines && corridorLines.length > 0;
        const hasWalls = corridorWalls && corridorWalls.length > 0;
        const hasNodes = nodes && nodes.length > 0;

        if (!hasLines && !hasWalls && !hasNodes) {
            this.clear();
            return;
        }

        // 1. 清理上一帧的网格与边界线
        this._cleanupMeshes();

        // 2. 只有在无连续曲面数据时才回退至离散方块网格 (避免方块遮挡视线)
        if (!hasWalls && hasNodes) {
            const count = nodes.length;
            const cellGeo = new THREE.BoxGeometry(0.092, 0.092, 0.015);
            this.surfaceMesh = new THREE.InstancedMesh(cellGeo, this.surfaceMaterial, count);
            this.surfaceMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);

            const dummy = new THREE.Object3D();
            for (let i = 0; i < count; ++i) {
                const [x, y, z] = nodes[i];
                dummy.position.set(x, y, z);
                dummy.rotation.set(0, 0, 0);
                dummy.updateMatrix();
                this.surfaceMesh.setMatrixAt(i, dummy.matrix);
            }

            this.surfaceMesh.instanceMatrix.needsUpdate = true;
            this.group.add(this.surfaceMesh);
        }

        // 3. 渲染后端 C++ 直接传入的真 3D 拓扑连续管道缎带曲面与边界发光轮廓
        if (hasLines) {
            const linePositions = new Float32Array(corridorLines.flat());
            const lineGeo = new THREE.BufferGeometry();
            lineGeo.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
            const lineSegments = new THREE.LineSegments(lineGeo, this.boundaryLineMaterial);
            this.boundaryGroup.add(lineSegments);
        }

        if (hasWalls) {
            const wallVertices = new Float32Array(corridorWalls.flat());
            const wallGeo = new THREE.BufferGeometry();
            wallGeo.setAttribute('position', new THREE.BufferAttribute(wallVertices, 3));
            wallGeo.computeVertexNormals();
            const wallMesh = new THREE.Mesh(wallGeo, this.wallMaterial);
            this.boundaryGroup.add(wallMesh);
        }

        this.lastNodeCount = hasNodes ? nodes.length : 0;
        this.group.visible = this.visible;
    }

    _cleanupMeshes() {
        if (this.surfaceMesh) {
            this.group.remove(this.surfaceMesh);
            if (this.surfaceMesh.geometry) this.surfaceMesh.geometry.dispose();
            this.surfaceMesh = null;
        }

        while (this.boundaryGroup.children.length > 0) {
            const child = this.boundaryGroup.children[0];
            this.boundaryGroup.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
    }

    setObstaclesVisible(visible) {
        this.obstaclesVisible = visible;
        this.obstaclesGroup.visible = visible;
    }

    clearObstacles() {
        while (this.obstaclesGroup.children.length > 0) {
            const child = this.obstaclesGroup.children[0];
            this.obstaclesGroup.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
    }

    updateTebObstacles(obstacles) {
        this.clearObstacles();
        if (!obstacles || obstacles.length === 0) return;

        const wallH = 0.40;
        const wallW = 0.14;

        for (const obs of obstacles) {
            if (!obs) continue;
            if (obs.type === 'line' && obs.start && obs.end) {
                const [x1, y1, z1] = obs.start;
                const [x2, y2, z2] = obs.end;
                const z = (z1 + z2) * 0.5 + 0.005;
                const dx = x2 - x1, dy = y2 - y1;
                const len = Math.hypot(dx, dy);
                if (len < 1e-4) continue;

                const boxGeo = new THREE.BoxGeometry(len, wallW, wallH);
                const bodyMesh = new THREE.Mesh(boxGeo, this.obsBodyMat);
                bodyMesh.position.set((x1 + x2) * 0.5, (y1 + y2) * 0.5, z + wallH * 0.5);
                bodyMesh.rotation.z = Math.atan2(dy, dx);
                this.obstaclesGroup.add(bodyMesh);

                const edgesGeo = new THREE.EdgesGeometry(boxGeo);
                const edgeMesh = new THREE.LineSegments(edgesGeo, this.obsEdgeMat);
                edgeMesh.position.copy(bodyMesh.position);
                edgeMesh.rotation.copy(bodyMesh.rotation);
                this.obstaclesGroup.add(edgeMesh);
            } else if (obs.type === 'circle') {
                const { x, y, z: rawZ, radius: r } = obs;
                const z = (rawZ !== undefined ? rawZ : 0.0) + 0.005;
                const radius = Math.max(0.12, r || 0.16);

                const cylGeo = new THREE.CylinderGeometry(radius, radius, wallH, 24, 1, false);
                const bodyMesh = new THREE.Mesh(cylGeo, this.obsBodyMat);
                bodyMesh.rotation.x = Math.PI * 0.5;
                bodyMesh.position.set(x, y, z + wallH * 0.5);
                this.obstaclesGroup.add(bodyMesh);

                const edgesGeo = new THREE.EdgesGeometry(cylGeo, 25);
                const edgeMesh = new THREE.LineSegments(edgesGeo, this.obsEdgeMat);
                edgeMesh.position.copy(bodyMesh.position);
                edgeMesh.rotation.copy(bodyMesh.rotation);
                this.obstaclesGroup.add(edgeMesh);
            }
        }
        this.obstaclesGroup.visible = this.obstaclesVisible;
    }

    clear() {
        this._cleanupMeshes();
        this.clearObstacles();
        this.lastNodeCount = 0;
    }
}
