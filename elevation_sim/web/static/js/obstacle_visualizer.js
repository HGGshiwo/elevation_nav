import * as THREE from 'three';

const MARKER_TYPE = { CUBE: 1, CYLINDER: 3 };

/**
 * 注入障碍物真值渲染器 (obstacle_injector 发布的 MarkerArray)
 * 每个障碍一个 Mesh (CUBE/CYLINDER), 按后端转发的紧凑 JSON 增删改;
 * 与规划路径、局部轨迹叠加显示, 用于"注入即所见"的对照验证
 */
export class InjectedObstacleVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.group = new THREE.Group();
        this.group.visible = true;
        scene.add(this.group);
        this.meshes = new Map(); // marker id -> Mesh
    }

    setVisible(v) {
        this.group.visible = v;
    }

    update(list) {
        if (!Array.isArray(list)) return;
        const seen = new Set();

        for (const o of list) {
            seen.add(o.id);
            const isCyl = o.type === MARKER_TYPE.CYLINDER;
            const geoKey = `${isCyl ? 'cyl' : 'box'}:${o.sx}:${o.sy}:${o.sz}`;

            let mesh = this.meshes.get(o.id);
            if (mesh && mesh.userData.geoKey !== geoKey) {
                this.group.remove(mesh);
                mesh.geometry.dispose();
                mesh.material.dispose();
                mesh = null;
            }
            if (!mesh) {
                const geo = isCyl
                    ? new THREE.CylinderGeometry(o.sx / 2, o.sx / 2, o.sz, 24).rotateX(Math.PI / 2)
                    : new THREE.BoxGeometry(o.sx, o.sy, o.sz);
                mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
                    color: 0xff1744,
                    transparent: true,
                    opacity: 0.5,
                    depthWrite: false
                }));
                mesh.userData.geoKey = geoKey;
                // 半透明障碍置于踏面板 (renderOrder 0, depthWrite true) 之后绘制:
                // 否则透明排序会把箱子画在踏面之下, 视觉上"沉进踏板"
                mesh.renderOrder = 6;
                this.group.add(mesh);
                this.meshes.set(o.id, mesh);
            }
            mesh.position.set(o.x, o.y, o.z);
            mesh.quaternion.set(o.qx, o.qy, o.qz, o.qw);
            if (o.color) {
                mesh.material.color.setRGB(o.color[0], o.color[1], o.color[2]);
                mesh.material.opacity = o.color[3];
            }
        }

        // 删除已消失/失活的障碍 (后端 DELETE 动作不转发, 以缺席表示)
        for (const [id, mesh] of this.meshes) {
            if (!seen.has(id)) {
                this.group.remove(mesh);
                mesh.geometry.dispose();
                mesh.material.dispose();
                this.meshes.delete(id);
            }
        }
    }
}
