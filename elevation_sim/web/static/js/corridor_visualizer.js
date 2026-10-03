import * as THREE from 'three';

/**
 * 拓扑管道 3D 可视化器 (Topological Corridor Visualizer)
 * 渲染局部 SFC 凸走廊的 3D 边界发光线框 (数据源: /move_base/local_sfc_corridor
 * 的 sfc_corridor_wireframe LINE_LIST, 经 ros_bridge 转发为 ws 帧 corridor_lines)
 */
export class CorridorVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.visible = true;

        this.group = new THREE.Group();
        this.scene.add(this.group);

        // 管道边界轮廓线与防护墙
        this.boundaryGroup = new THREE.Group();
        this.group.add(this.boundaryGroup);

        // 管道侧壁边界线材质 (深橙色高反差荧光线, 与青蓝色全局路径区分)
        this.boundaryLineMaterial = new THREE.LineBasicMaterial({
            color: 0xff6d00,
            linewidth: 2.5,
            transparent: true,
            opacity: 0.95
        });
    }

    setVisible(visible) {
        this.visible = visible;
        this.group.visible = visible;
    }

    /**
     * 更新拓扑管道 3D 呈现
     * @param {Array<Array<number>>}|null nodes - 保留旧接口兼容; 管道节点数据源已随 TEB 移除, 传 null
     * @param {Array<Array<number>>} corridorLines - 后端 C++ 精确计算的 3D 边界线端点 [[x,y,z], ...]
     */
    update(nodes, corridorLines = null) {
        const hasLines = corridorLines && corridorLines.length > 0;

        if (!hasLines) {
            this.clear();
            return;
        }

        // 清理上一帧边界线
        this._cleanupMeshes();

        // 渲染后端 C++ 直接传入的真 3D 拓扑连续管道发光边界轮廓线
        const linePositions = new Float32Array(corridorLines.flat());
        const lineGeo = new THREE.BufferGeometry();
        lineGeo.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
        const lineSegments = new THREE.LineSegments(lineGeo, this.boundaryLineMaterial);
        this.boundaryGroup.add(lineSegments);

        this.group.visible = this.visible;
    }

    _cleanupMeshes() {
        while (this.boundaryGroup.children.length > 0) {
            const child = this.boundaryGroup.children[0];
            this.boundaryGroup.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
    }

    clear() {
        this._cleanupMeshes();
    }
}
