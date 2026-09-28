import * as THREE from 'three';

/**
 * 样条优化 rebound 定向排斥渲染器 (最简版: 平放方块, 渲染路径与 dynamic_nodes 完全同款)
 * 数据: [{x0,y0,z0, x1,y1,z1, pressed}] —— 障碍面参考点 → 控制点的定向弹簧
 * 每支弹簧画两个方块: 障碍点(红) + 控制点(红=受压中 / 绿=已弹开); 纯排查用极简实现
 */
export class ReboundVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.group = new THREE.Group();
        this.group.visible = true;
        scene.add(this.group);

        // 障碍面参考点小红方块 (标示弹簧推离的对象)
        this.baseGeometry = new THREE.PlaneGeometry(0.09, 0.09);  // Z-up: 默认法线朝 +Z 即水平平放
        this.baseMaterial = new THREE.MeshBasicMaterial({
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.9,
            depthWrite: true
        });
        this.arrows = [];
        // 驻留显示: 碰撞只存在于单个优化周期 (~0.1s), 空帧立即清除会一闪而过看不见;
        // 最后一组非空弹簧驻留 2s, 期间空帧不清屏
        this.lastData = null;
        this.lastTime = 0;
        this.PERSIST_MS = 2000;
    }

    setVisible(v) {
        this.group.visible = v;
    }

    update(list) {
        const now = performance.now();
        if (Array.isArray(list) && list.length > 0) {
            this.lastData = list;
            this.lastTime = now;
        } else {
            if (!this.lastData || now - this.lastTime > this.PERSIST_MS) {
                this._cleanup();
                this.lastData = null;
                return;
            }
            list = this.lastData;
        }

        console.log(`[Rebound] 绘制 ${list.length} 支弹簧:`, JSON.stringify(list[0]));

        this._cleanup();

        // 每支弹簧 = 控制点处一支箭头 (方向 = 推离障碍的径向, 长度固定 0.3m)
        // + 障碍面参考点一个小红方块; 红箭头 = 受压中, 绿箭头 = 已弹到安全间距外
        const RED = 0xff1744, GREEN = 0x00e676;
        for (const a of list) {
            const dir = new THREE.Vector3(a.x1 - a.x0, a.y1 - a.y0, 0);
            if (dir.lengthSq() < 1e-8) continue;
            dir.normalize();

            const arrow = new THREE.ArrowHelper(
                dir,
                new THREE.Vector3(a.x1, a.y1, a.z1 + 0.04),
                0.30,                       // 长度
                a.pressed ? RED : GREEN,
                0.12, 0.07);                // 箭头头部尺寸
            this.group.add(arrow);
            this.arrows.push(arrow);

            const base = new THREE.Mesh(this.baseGeometry, this.baseMaterial);
            base.position.set(a.x0, a.y0, a.z0 + 0.01);
            base.renderOrder = 8;
            this.group.add(base);
            this.arrows.push(base);
        }
    }

    _cleanup() {
        for (const obj of this.arrows) {
            this.group.remove(obj);
            if (obj.dispose) obj.dispose();  // ArrowHelper 自带 dispose; Mesh 共享几何体/材质无需释放
        }
        this.arrows = [];
    }
}
