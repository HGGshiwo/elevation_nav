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
     */
    update(nodes, corridorLines = null) {
        const hasLines = corridorLines && corridorLines.length > 0;
        const hasNodes = nodes && nodes.length > 0;

        if (!hasLines && !hasNodes) {
            this.clear();
            return;
        }

        // 1. 清理上一帧的网格与边界线
        this._cleanupMeshes();

        // 2. 管道底面踏面方块 (若有)
        if (hasNodes) {
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

        // 3. 渲染后端 C++ 直接传入的真 3D 拓扑连续管道发光边界轮廓线
        if (hasLines) {
            const linePositions = new Float32Array(corridorLines.flat());
            const lineGeo = new THREE.BufferGeometry();
            lineGeo.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
            const lineSegments = new THREE.LineSegments(lineGeo, this.boundaryLineMaterial);
            this.boundaryGroup.add(lineSegments);
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

    clear() {
        this._cleanupMeshes();
        this.lastNodeCount = 0;
    }
}
