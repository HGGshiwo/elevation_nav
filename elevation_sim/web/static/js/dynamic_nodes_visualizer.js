import * as THREE from 'three';

/**
 * 融合引擎动态标记节点实时渲染器 (/elevation_dynamic_nodes)
 * 数据: [x, y, z, traversability, zone?] —— 融合引擎原位改写属性的全局图节点
 *       zone = CostZone 枚举: 0=自由 / 1=软代价带 / 2=机体硬禁行环 / 3=障碍禁行
 * 渲染: 与踏面同尺寸 (0.092m) 的方块实例, 直接叠在对应节点踏面上, 三档配色:
 *       禁行 (zone=3, 或旧格式 trav≥0.95) = 红 0xf44336
 *       机体硬禁行环 (zone=2, 或旧格式 0.8~0.95) = 橙黄 0xfb8c00 (不可入)
 *       软代价带 (zone=1, 或旧格式 <0.8) = 紫色渐变 (深紫→淡紫, 代价随距离衰减)
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
        const n = list.length;

        // 数量不变时原位更新 (避免逐帧 remove/add 网格造成的视觉闪烁); 否则重建
        if (this.mesh && this.mesh.count !== n) {
            this.group.remove(this.mesh);
            this.mesh.dispose();
            this.mesh = null;
        }
        if (n === 0) {
            if (this.mesh) {
                this.group.remove(this.mesh);
                this.mesh.dispose();
                this.mesh = null;
            }
            return;
        }
        if (!this.mesh) {
            this.mesh = new THREE.InstancedMesh(this.geometry, this.material, n);
            this.mesh.renderOrder = 6; // 绘制在踏面板之上
            this.group.add(this.mesh);
        }

        const mtx = new THREE.Matrix4();
        const blocked = new THREE.Color(0xf44336); // 障碍禁行 (zone=3)
        const ring = new THREE.Color(0xfb8c00);    // 机体硬禁行环 (zone=2, 不可入)
        const costNear = new THREE.Color(0x8e24aa); // 软代价带内侧 (深紫, 代价高)
        const costFar = new THREE.Color(0xd1c4e9);  // 软代价带外侧 (淡紫, 代价低)
        for (let i = 0; i < n; ++i) {
            const p = list[i];
            mtx.makeTranslation(p[0], p[1], p[2] + 0.02);
            this.mesh.setMatrixAt(i, mtx);
            const trav = p[3];
            const zone = (p.length >= 5 && Number.isFinite(p[4])) ? p[4] : null;
            if (zone !== null) {
                // 新格式: 按 CostZone 显式三档
                if (zone >= 3) {
                    this.mesh.setColorAt(i, blocked);
                } else if (zone === 2) {
                    this.mesh.setColorAt(i, ring);
                } else {
                    // 软代价带: trav 0.8→0.05 映射 深紫→淡紫 渐变, 与机体圈/禁行红明确区分
                    const t = Math.max(0, Math.min(1, (trav - 0.05) / 0.75));
                    this.mesh.setColorAt(i, costNear.clone().lerp(costFar, t));
                }
            } else if (trav >= 0.95) {
                // 旧格式回退: trav 阈值近似分档
                this.mesh.setColorAt(i, blocked);
            } else if (trav >= 0.8) {
                this.mesh.setColorAt(i, ring);
            } else {
                const t = Math.max(0, Math.min(1, (trav - 0.05) / 0.75));
                this.mesh.setColorAt(i, costNear.clone().lerp(costFar, t));
            }
        }
        this.mesh.instanceMatrix.needsUpdate = true;
        if (this.mesh.instanceColor) this.mesh.instanceColor.needsUpdate = true;
    }
}
