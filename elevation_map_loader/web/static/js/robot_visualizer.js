import * as THREE from 'three';

/**
 * 机器狗 3D 姿态与航向可视化器
 */
export class RobotVisualizer {
    constructor(scene, controls) {
        this.scene = scene;
        this.controls = controls;

        this.robotGroup = new THREE.Group();
        this.currentPose = { x: 0, y: 0, z: 0, yaw: 0 };
        this.visible = true;

        this._buildRobotModel();
        this.scene.add(this.robotGroup);
    }

    _buildRobotModel() {
        // 机身刚体 (深灰蓝色)
        const bodyGeo = new THREE.BoxGeometry(0.55, 0.28, 0.18);
        const bodyMat = new THREE.MeshStandardMaterial({
            color: 0x2563eb,
            roughness: 0.4,
            metalness: 0.6
        });
        const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
        bodyMesh.position.set(0, 0, 0.15);
        this.robotGroup.add(bodyMesh);

        // 头部前端标识 (亮橙色)
        const headGeo = new THREE.BoxGeometry(0.12, 0.22, 0.12);
        const headMat = new THREE.MeshStandardMaterial({
            color: 0xf97316,
            roughness: 0.3
        });
        const headMesh = new THREE.Mesh(headGeo, headMat);
        headMesh.position.set(0.30, 0, 0.17);
        this.robotGroup.add(headMesh);

        // 航向指示箭头 (沿 X 轴正向)
        const arrowDir = new THREE.Vector3(1, 0, 0);
        const arrowOrigin = new THREE.Vector3(0.35, 0, 0.17);
        this.arrowHelper = new THREE.ArrowHelper(arrowDir, arrowOrigin, 0.5, 0x10b981, 0.15, 0.08);
        this.robotGroup.add(this.arrowHelper);
    }

    updatePose(pose) {
        if (!pose) return;
        this.currentPose = pose;

        // 平滑更新位置与航向 (ROS: X 前, Y 左, Z 上)
        this.robotGroup.position.set(pose.x, pose.y, pose.z);
        this.robotGroup.rotation.z = pose.yaw;
    }

    focusRobot() {
        if (!this.controls) return;
        this.controls.target.set(this.currentPose.x, this.currentPose.y, this.currentPose.z);
    }

    setVisible(visible) {
        this.visible = visible;
        this.robotGroup.visible = visible;
    }
}
