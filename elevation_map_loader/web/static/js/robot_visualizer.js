import * as THREE from 'three';

/**
 * 机器狗 3D 姿态与航向可视化器
 *
 * 机体模型与理论建模一致 (尺寸经 /api/nav/robot_model 从 planner_common.yaml 读取):
 *   - 胶囊体 (机身): 半径 = body_hard_radius, 总高 = dog_height, 中心离地 dog_height/2
 *   - 足印圆盘 (底盘): 半径 = footprint_radius, 贴地薄盘
 *   - 前向锥 (航向): 沿机体 +X 方向
 * 模型原点位于足底触地点, updatePose 的 pose.z 即当前踏面高程。
 */
export class RobotVisualizer {
    constructor(scene, controls) {
        this.scene = scene;
        this.controls = controls;

        this.robotGroup = new THREE.Group();
        this.currentPose = { x: 0, y: 0, z: 0, yaw: 0 };
        this.visible = true;

        // 理论建模尺寸 (api 未就绪时的兜底默认值, 与 planner_common.yaml 一致)
        this.modelParams = {
            dog_height: 0.45,
            body_hard_radius: 0.15,
            footprint_radius: 0.30,
            max_step_height: 0.25
        };

        // 跟随视角状态
        this.followEnabled = false;
        this._followLerp = 0;            // 开启瞬间的相机过渡进度 [0,1]
        this._followStartCamPos = null;  // 过渡起点
        this._lastRobotPos = new THREE.Vector3();
        this._desiredTmp = new THREE.Vector3();
        // 相机侧位姿平滑状态 (双重低通防抖)
        this._smoothCenter = new THREE.Vector3();
        this._smoothYaw = 0;
        this._smoothInit = false;

        this._buildRobotModel();
        this.scene.add(this.robotGroup);

        this._fetchModelParams();
    }

    async _fetchModelParams() {
        try {
            const res = await fetch('/api/nav/robot_model');
            if (!res.ok) return;
            const params = await res.json();
            let changed = false;
            for (const key of Object.keys(this.modelParams)) {
                if (typeof params[key] === 'number' && params[key] > 0
                    && Math.abs(params[key] - this.modelParams[key]) > 1e-4) {
                    this.modelParams[key] = params[key];
                    changed = true;
                }
            }
            if (changed) this._buildRobotModel();
        } catch (e) { /* 后端未启动时沿用默认尺寸 */ }
    }

    _buildRobotModel() {
        // 清空旧模型重建
        while (this.robotGroup.children.length > 0) {
            const child = this.robotGroup.children.pop();
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }

        const { dog_height, body_hard_radius, footprint_radius, max_step_height } = this.modelParams;

        // 1. 足印圆盘 (机体外接圆, 贴地水平): Cylinder 轴沿本地 +Y, 转竖直使盘面水平
        const discGeo = new THREE.CylinderGeometry(footprint_radius, footprint_radius, 0.015, 40);
        const discMat = new THREE.MeshStandardMaterial({
            color: 0xf97316,
            roughness: 0.5,
            metalness: 0.1,
            transparent: true,
            opacity: 0.45,
            side: THREE.DoubleSide
        });
        const discMesh = new THREE.Mesh(discGeo, discMat);
        discMesh.rotation.x = Math.PI / 2;
        discMesh.position.set(0, 0, 0.008);
        this.robotGroup.add(discMesh);

        // 圆盘边界环: Torus 天然位于本地 XY 面, 无需旋转即水平
        const ringGeo = new THREE.TorusGeometry(footprint_radius, 0.008, 8, 64);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0xfdba74 });
        const ringMesh = new THREE.Mesh(ringGeo, ringMat);
        ringMesh.position.set(0, 0, 0.016);
        this.robotGroup.add(ringMesh);

        // 2. 机身胶囊体 (水平躺放, 轴沿机体 +X): 半径 = body_hard_radius,
        //    总长 = 2*footprint_radius, 底部离地 = max_step_height
        const cylLength = Math.max(0.01, 2 * footprint_radius - 2 * body_hard_radius);
        const capsuleGeo = new THREE.CapsuleGeometry(body_hard_radius, cylLength, 8, 24);
        const capsuleMat = new THREE.MeshStandardMaterial({
            color: 0x2563eb,
            roughness: 0.35,
            metalness: 0.55
        });
        const capsuleMesh = new THREE.Mesh(capsuleGeo, capsuleMat);
        capsuleMesh.rotation.z = -Math.PI / 2;
        capsuleMesh.position.set(0, 0, max_step_height + body_hard_radius);
        this.robotGroup.add(capsuleMesh);

        // 3. 前向锥 (航向指示, 沿机体 +X)
        const noseGeo = new THREE.ConeGeometry(0.05, 0.16, 12);
        const noseMat = new THREE.MeshStandardMaterial({
            color: 0x10b981,
            roughness: 0.3
        });
        const noseMesh = new THREE.Mesh(noseGeo, noseMat);
        noseMesh.rotation.z = -Math.PI / 2;
        noseMesh.position.set(footprint_radius + 0.08, 0, max_step_height + body_hard_radius);
        this.robotGroup.add(noseMesh);
    }

    updatePose(pose) {
        if (!pose) return;
        this.currentPose = pose;

        // 平滑更新位置与航向 (ROS: X 前, Y 左, Z 上); 原点 = 足底触地点
        this.robotGroup.position.set(pose.x, pose.y, pose.z);
        this.robotGroup.rotation.z = pose.yaw;
    }

    focusRobot() {
        if (!this.controls) return;
        this.controls.target.set(this.currentPose.x, this.currentPose.y, this.currentPose.z);
    }

    /**
     * 跟随视角开关: 开启后相机平滑过渡到狗正后方近距视角,
     * 相机朝向始终与狗的航向一致 (追逐视角)
     */
    setFollowEnabled(enabled) {
        this.followEnabled = enabled;
        if (enabled) {
            this._followLerp = 0;
            this._smoothInit = false; // 从当前真实位姿重新起算平滑状态
            this._followStartCamPos = this.controls.object.position.clone();
            // 立即对准一次, 避免过渡期间目标点在远处
            this.controls.target.set(
                this.currentPose.x, this.currentPose.y,
                this.currentPose.z + this.modelParams.dog_height * 0.4);
        }
    }

    _normalizeAngle(a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    /**
     * 追逐视角期望相机位: 狗正后方 dist 处、上方 height 处, 随平滑航向旋转
     */
    _chaseCamDesired(out) {
        const dist = 1.4, height = 1.0;
        out.set(
            this._smoothCenter.x - Math.cos(this._smoothYaw) * dist,
            this._smoothCenter.y - Math.sin(this._smoothYaw) * dist,
            this._smoothCenter.z + height);
        return out;
    }

    /** 每帧调用: 跟随模式下驱动相机 (位置与朝向均与狗一致, 双重低通防抖) */
    updateFollow() {
        if (!this.followEnabled || !this.controls) return;

        const rawCenter = new THREE.Vector3(
            this.currentPose.x, this.currentPose.y,
            this.currentPose.z + this.modelParams.dog_height * 0.4);

        if (!this._smoothInit) {
            this._smoothCenter.copy(rawCenter);
            this._smoothYaw = this.currentPose.yaw;
            this._smoothInit = true;
        }

        // 1. 位姿低通: 中心 (含 z) 与航向分别平滑, 航向按最短角差连续化
        this._smoothCenter.lerp(rawCenter, 0.10);
        this._smoothYaw += this._normalizeAngle(this.currentPose.yaw - this._smoothYaw) * 0.10;

        // 2. 相机趋近期望追逐位 (阻尼低于位姿平滑系数, 转向走弧线不突变)
        if (this._followLerp < 1) {
            this._followLerp = Math.min(1, this._followLerp + 0.02);
            const k = this._easeInOut(this._followLerp);
            this.controls.object.position.lerpVectors(this._followStartCamPos, this._chaseCamDesired(this._desiredTmp), k);
        } else {
            this.controls.object.position.lerp(this._chaseCamDesired(this._desiredTmp), 0.12);
        }

        this.controls.target.copy(this._smoothCenter);
        this._lastRobotPos.copy(this._smoothCenter);
    }

    _easeInOut(t) {
        return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    }

    setVisible(visible) {
        this.visible = visible;
        this.robotGroup.visible = visible;
    }
}
