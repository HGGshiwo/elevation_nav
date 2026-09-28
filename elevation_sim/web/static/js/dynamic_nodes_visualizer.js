import * as THREE from 'three';

/**
 * 融合引擎动态标记节点实时渲染器 (/elevation_dynamic_nodes)
 * 数据: [x, y, z, traversability] —— 融合引擎原位改写属性的全局图节点
 * 渲染: 与踏面同尺寸 (0.092m) 的方块实例, 直接叠在对应节点踏面上,
 *       视觉效果即"障碍物把对应节点变成了红色":
 *       封锁 (trav≥0.95) = 红 (与静态禁行同色 0xf44336), 软代价 (0~1) = 橙
 */
export class DynamicNodeVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.group = new THREE.Group();
        this.group.visible = true;
        scene.add(this.group);
        this.material = new THREE.MeshBasicMaterial({
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.85,
            depthWrite: true
        });
        // 场景为 Z-up (camera.up = +Z): PlaneGeometry 默认法线朝 +Z 即水平平放,
        // 与踏面方块同口径; 不可加 rotateX(-PI/2) —— 那是 Y-up 项目的写法, 会把方块立起来
        this.geometry = new THREE.PlaneGeometry(0.092, 0.092);
        this.mesh = null;
    }

    setVisible(v) {
        this.group.visible = v;
    }

    update(list) {
        if (!Array.isArray(list)) return;
        // 重建实例网格 (数量 = 当前被动态标记的节点数, 数百级, 10Hz 重建开销可忽略)
        if (this.mesh) {
            this.group.remove(this.mesh);
            this.mesh.dispose();
            this.mesh = null;
        }
        const n = list.length;
        if (n === 0) return;

        this.mesh = new THREE.InstancedMesh(this.geometry, this.material, n);
        this.mesh.renderOrder = 6; // 绘制在踏面板之上

        const mtx = new THREE.Matrix4();
        const blocked = new THREE.Color(0xf44336); // 与静态禁行踏面同色
        const soft = new THREE.Color(0xff9800);
        for (let i = 0; i < n; ++i) {
            const p = list[i];
            mtx.makeTranslation(p[0], p[1], p[2] + 0.02);
            this.mesh.setMatrixAt(i, mtx);
            this.mesh.setColorAt(i, p[3] >= 0.95 ? blocked : soft);
        }
        this.mesh.instanceMatrix.needsUpdate = true;
        if (this.mesh.instanceColor) this.mesh.instanceColor.needsUpdate = true;
        this.group.add(this.mesh);
    }
}
