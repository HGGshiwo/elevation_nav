import * as THREE from 'three';

/**
 * 机器狗模型渲染、位姿追踪、跟随视角与导航路径可视化模块
 *
 * 机体模型与理论建模一致 (尺寸经 /api/nav/robot_model 从 planner_common.yaml 读取):
 *   - 胶囊体 (机身): 半径 = body_hard_radius, 总高 = dog_height, 中心离地 dog_height/2
 *   - 足印圆盘 (底盘): 半径 = footprint_radius, 贴地薄盘
 *   - 前向锥 (航向): 沿机体 +X 方向
 * 模型原点位于足底触地点, updatePose 的 pose.z 即当前踏面高程。
 */
export function initRobotTracker(scene, controls) {
    const dogMeshGroup = new THREE.Group();
    let currentPose = { x: 0, y: 0, z: 0, yaw: 0 };

    // 理论建模尺寸 (api 未就绪时的兜底默认值, 与 planner_common.yaml 一致)
    let modelParams = {
        dog_height: 0.45,
        body_hard_radius: 0.15,
        footprint_radius: 0.30,
        max_step_height: 0.25
    };

    // ---- 跟随视角状态 ----
    let followEnabled = false;
    let followLerp = 0;
    let followStartCamPos = null;
    const lastRobotCenter = new THREE.Vector3();
    const _desiredTmp = new THREE.Vector3();
    // 相机侧位姿平滑状态 (对 50Hz 位姿量化与地形 z 波动双重低通, 相机比狗更稳)
    let smoothCenter = new THREE.Vector3();
    let smoothYaw = 0;
    let smoothInit = false;

    /**
     * 构建机器狗刚体模型: 足印圆盘 + 机身胶囊体 + 前向锥
     */
    function buildDogModel() {
        // 清空旧模型重建
        while (dogMeshGroup.children.length > 0) {
            const child = dogMeshGroup.children.pop();
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }

        const { dog_height, body_hard_radius, footprint_radius, max_step_height } = modelParams;

        // 1. 足印圆盘 (机体外接圆, 贴地水平): 理论建模的 footprint_radius 边界
        //    Cylinder 轴沿本地 +Y, 转 rotation.x=+90° 使轴向竖直 (Z), 盘面水平贴地
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
        dogMeshGroup.add(discMesh);

        // 圆盘边界环 (提高俯视辨识度): Torus 天然位于本地 XY 面, 无需旋转即水平
        const ringGeo = new THREE.TorusGeometry(footprint_radius, 0.008, 8, 64);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0xfdba74 });
        const ringMesh = new THREE.Mesh(ringGeo, ringMat);
        ringMesh.position.set(0, 0, 0.016);
        dogMeshGroup.add(ringMesh);

        // 2. 机身胶囊体 (机体硬包络, 水平躺放): 轴沿机体 +X (前进方向)
        //    半径 = body_hard_radius, 总长 = 2*footprint_radius (端点与足印圆边界相切),
        //    底部离地 = max_step_height (机体行进离地包络), 中心高 = max_step_height + body_hard_radius
        //    CapsuleGeometry(radius, length) 轴沿本地 +Y, 转 rotation.z=-90° 使轴指向机体 +X
        const cylLength = Math.max(0.01, 2 * footprint_radius - 2 * body_hard_radius);
        const capsuleGeo = new THREE.CapsuleGeometry(body_hard_radius, cylLength, 8, 24);
        const capsuleMat = new THREE.MeshStandardMaterial({
            color: 0x2196f3,
            roughness: 0.35,
            metalness: 0.55
        });
        const capsuleMesh = new THREE.Mesh(capsuleGeo, capsuleMat);
        capsuleMesh.rotation.z = -Math.PI / 2;
        capsuleMesh.position.set(0, 0, max_step_height + body_hard_radius);
        dogMeshGroup.add(capsuleMesh);

        // 3. 前向锥 (航向指示, 沿机体 +X)
        const noseGeo = new THREE.ConeGeometry(0.05, 0.16, 12);
        const noseMat = new THREE.MeshStandardMaterial({ color: 0x00e676, roughness: 0.3 });
        const noseMesh = new THREE.Mesh(noseGeo, noseMat);
        noseMesh.rotation.z = -Math.PI / 2; // Cone +Y 轴转到 +X
        noseMesh.position.set(footprint_radius + 0.08, 0, max_step_height + body_hard_radius);
        dogMeshGroup.add(noseMesh);
    }

    buildDogModel();
    scene.add(dogMeshGroup);

    // 从后端参数服务读取理论建模尺寸, 与配置不一致时重建模型
    (async function fetchModelParams() {
        try {
            const res = await fetch('/api/nav/robot_model');
            if (!res.ok) return;
            const params = await res.json();
            let changed = false;
            for (const key of Object.keys(modelParams)) {
                if (typeof params[key] === 'number' && params[key] > 0
                    && Math.abs(params[key] - modelParams[key]) > 1e-4) {
                    modelParams[key] = params[key];
                    changed = true;
                }
            }
            if (changed) buildDogModel();
        } catch (e) { /* 后端未启动时沿用默认尺寸 */ }
    })();

    // ---- 导航路径线条对象 ----
    let pathLine = null;
    let localPathLine = null;
    let localAStarPathLine = null;

    const pathMat = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 3 });
    const localPathMat = new THREE.LineBasicMaterial({ color: 0xfacc15, linewidth: 4 }); // 亮金黄 (TEB/局部规划轨迹)
    const localAStarMat = new THREE.LineBasicMaterial({ color: 0xffeb3b, linewidth: 3 });

    function updatePath(points) {
        if (pathLine) {
            scene.remove(pathLine);
            pathLine.geometry.dispose();
            pathLine = null;
        }
        if (!points || points.length < 2) return;

        const coords = [];
        points.forEach(p => coords.push(p[0], p[1], p[2] + 0.05));
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(coords, 3));
        pathLine = new THREE.Line(geo, pathMat);
        scene.add(pathLine);
    }

    function updateLocalPath(points) {
        if (localPathLine) {
            scene.remove(localPathLine);
            localPathLine.geometry.dispose();
            localPathLine = null;
        }
        if (!points || points.length < 2) return;

        const coords = [];
        points.forEach(p => coords.push(p[0], p[1], p[2] + 0.08));
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(coords, 3));
        localPathLine = new THREE.Line(geo, localPathMat);
        const showLocal = (document.getElementById('show-local-path')?.checked ??
                           document.getElementById('show-local-astar')?.checked ?? true);
        localPathLine.visible = showLocal;
        scene.add(localPathLine);
    }

    function updateLocalAStarPath(points) {
        if (localAStarPathLine) {
            scene.remove(localAStarPathLine);
            localAStarPathLine.geometry.dispose();
            localAStarPathLine = null;
        }
        if (!points || points.length < 2) return;

        const coords = [];
        points.forEach(p => coords.push(p[0], p[1], p[2] + 0.06));
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(coords, 3));
        localAStarPathLine = new THREE.Line(geo, localAStarMat);
        const showLocal = document.getElementById('show-local-astar')?.checked ?? true;
        localAStarPathLine.visible = showLocal;
        scene.add(localAStarPathLine);
    }

    // ---- 碰撞/锁止格子高亮显示 (3D 标记柱: 悬浮警示柱 + 顶部圆球，不加底部方格遮挡踏面) ----
    const collisionGroup = new THREE.Group();
    collisionGroup.visible = false;
    scene.add(collisionGroup);

    // 1. 立体竖线标杆 (Cylinder)
    const poleH = 0.28;
    const colPoleGeo = new THREE.CylinderGeometry(0.012, 0.012, poleH, 12);
    const colPoleMat = new THREE.MeshBasicMaterial({ color: 0xff1744 });
    const colPoleMesh = new THREE.Mesh(colPoleGeo, colPoleMat);
    colPoleMesh.rotation.x = Math.PI / 2;
    colPoleMesh.position.set(0, 0, poleH * 0.5);
    colPoleMesh.renderOrder = 21;
    collisionGroup.add(colPoleMesh);

    // 2. 顶部高亮警示圆点 (Sphere)
    const colSphereGeo = new THREE.SphereGeometry(0.065, 20, 20);
    const colSphereMat = new THREE.MeshStandardMaterial({
        color: 0xff1744,
        emissive: 0xd50000,
        roughness: 0.2,
        metalness: 0.3
    });
    const colSphereMesh = new THREE.Mesh(colSphereGeo, colSphereMat);
    colSphereMesh.position.set(0, 0, poleH);
    colSphereMesh.renderOrder = 22;
    collisionGroup.add(colSphereMesh);

    // 3. 顶部环绕发光光环 (Torus)
    const colHaloGeo = new THREE.TorusGeometry(0.095, 0.008, 8, 32);
    const colHaloMat = new THREE.MeshBasicMaterial({
        color: 0xff8a80,
        transparent: true,
        opacity: 0.90
    });
    const colHaloMesh = new THREE.Mesh(colHaloGeo, colHaloMat);
    colHaloMesh.position.set(0, 0, poleH);
    colHaloMesh.renderOrder = 23;
    collisionGroup.add(colHaloMesh);

    // 4. 触碰碰撞点云高亮实例网格 (InstancedMesh: 发光粒子球)
    const maxColPts = 1000;
    const colPtGeo = new THREE.SphereGeometry(0.025, 10, 10);
    const colPtMat = new THREE.MeshStandardMaterial({
        color: 0xff1744,
        emissive: 0xd50000,
        emissiveIntensity: 0.9,
        roughness: 0.2,
        metalness: 0.3
    });
    const colPtsMesh = new THREE.InstancedMesh(colPtGeo, colPtMat, maxColPts);
    colPtsMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    colPtsMesh.visible = false;
    colPtsMesh.renderOrder = 25;
    scene.add(colPtsMesh);
    const colPtsDummy = new THREE.Object3D();

    function updateCollisionNode(node, hitPoints = null) {
        if (!node || !Array.isArray(node) || node.length < 3) {
            collisionGroup.visible = false;
            colPtsMesh.visible = false;
            return;
        }
        const [cx, cy, cz, zone] = node;
        // 定位到碰撞节点踏面坐标
        collisionGroup.position.set(cx, cy, cz);

        if (zone === 2) {
            // BODY_HARD 机体禁行环 (亮橙)
            colPoleMat.color.setHex(0xff9100);
            colSphereMat.color.setHex(0xff9100);
            colSphereMat.emissive.setHex(0xff6d00);
            colHaloMat.color.setHex(0xffd180);
            colPtMat.color.setHex(0xff9100);
            colPtMat.emissive.setHex(0xff6d00);
        } else if (zone === 4) {
            // 顶盖碰撞 (紫红)
            colPoleMat.color.setHex(0xd500f9);
            colSphereMat.color.setHex(0xd500f9);
            colSphereMat.emissive.setHex(0xaa00ff);
            colHaloMat.color.setHex(0xea80fc);
            colPtMat.color.setHex(0xd500f9);
            colPtMat.emissive.setHex(0xaa00ff);
        } else {
            // zone === 3 或默认 真实物理障碍禁行 (鲜红)
            colPoleMat.color.setHex(0xff1744);
            colSphereMat.color.setHex(0xff1744);
            colSphereMat.emissive.setHex(0xd50000);
            colHaloMat.color.setHex(0xff8a80);
            colPtMat.color.setHex(0xff1744);
            colPtMat.emissive.setHex(0xd50000);
        }

        collisionGroup.visible = true;

        // 渲染触发碰撞的具体点云点
        if (hitPoints && Array.isArray(hitPoints) && hitPoints.length > 0) {
            const count = Math.min(hitPoints.length, maxColPts);
            for (let i = 0; i < count; i++) {
                const pt = hitPoints[i];
                colPtsDummy.position.set(pt[0], pt[1], pt[2]);
                colPtsDummy.scale.set(1, 1, 1);
                colPtsDummy.updateMatrix();
                colPtsMesh.setMatrixAt(i, colPtsDummy.matrix);
            }
            colPtsMesh.count = count;
            colPtsMesh.instanceMatrix.needsUpdate = true;
            colPtsMesh.visible = true;
        } else {
            colPtsMesh.visible = false;
        }
    }

    function updateCollisionAnimation() {
        const now = Date.now();
        if (collisionGroup.visible) {
            const bounce = 0.03 * Math.sin(now * 0.008);
            const curH = poleH + bounce;

            colSphereMesh.position.set(0, 0, curH);
            colHaloMesh.position.set(0, 0, curH);
            colHaloMesh.rotation.z = now * 0.002;

            colPoleMesh.scale.set(1, curH / poleH, 1);
            colPoleMesh.position.set(0, 0, curH * 0.5);
        }

        if (colPtsMesh.visible) {
            const glow = 0.8 + 0.4 * Math.sin(now * 0.01);
            colPtMat.emissiveIntensity = glow;
        }
    }

    function updatePose(pose) {
        if (!pose) return;
        currentPose = pose;
        dogMeshGroup.position.set(pose.x, pose.y, pose.z);
        dogMeshGroup.rotation.z = pose.yaw;

        const rosStatusEl = document.getElementById('ros-status');
        if (rosStatusEl) {
            rosStatusEl.innerText = `位姿: (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.z.toFixed(2)}) Yaw: ${(pose.yaw * 180 / Math.PI).toFixed(1)}°`;
        }
    }

    function focusRobot() {
        if (!controls) return;
        controls.target.set(currentPose.x, currentPose.y, currentPose.z);
    }

    /**
     * 跟随视角开关: 开启后相机平滑过渡到狗正后方近距视角,
     * 相机朝向始终与狗的航向一致 (追逐视角), 保留缩放自由度
     */
    function setFollowEnabled(enabled) {
        followEnabled = enabled;
        if (enabled) {
            followLerp = 0;
            smoothInit = false; // 从当前真实位姿重新起算平滑状态
            followStartCamPos = controls.object.position.clone();
            // 立即对准一次, 避免过渡期间目标点在远处
            controls.target.set(
                currentPose.x, currentPose.y,
                currentPose.z + modelParams.dog_height * 0.4);
        }
    }

    function easeInOut(t) {
        return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    }

    function normalizeAngle(a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    /**
     * 追逐视角期望相机位: 狗正后方 dist 处、上方 height 处, 随平滑航向旋转。
     * 输入用平滑后的中心与航向 (而非原始位姿), 消除位姿流量化与地形 z 抖动
     */
    function chaseCamDesired(out) {
        const dist = 1.4, height = 1.0;
        out.set(
            smoothCenter.x - Math.cos(smoothYaw) * dist,
            smoothCenter.y - Math.sin(smoothYaw) * dist,
            smoothCenter.z + height);
        return out;
    }

    /** 每帧调用: 跟随模式下驱动相机 (位置与朝向均与狗一致, 双重低通防抖) */
    function updateFollow() {
        if (!followEnabled || !controls) return;

        const rawCenter = new THREE.Vector3(
            currentPose.x, currentPose.y,
            currentPose.z + modelParams.dog_height * 0.4);

        if (!smoothInit) {
            smoothCenter.copy(rawCenter);
            smoothYaw = currentPose.yaw;
            smoothInit = true;
        }

        // 1. 位姿低通: 中心 (含 z) 与航向分别平滑, 航向按最短角差连续化
        smoothCenter.lerp(rawCenter, 0.10);
        smoothYaw += normalizeAngle(currentPose.yaw - smoothYaw) * 0.10;

        // 2. 相机趋近期望追逐位 (阻尼系数低于位姿平滑系数, 转向走弧线不突变)
        if (followLerp < 1) {
            // 开启瞬间的过渡: 相机滑入狗正后方的追逐位
            followLerp = Math.min(1, followLerp + 0.02);
            const k = easeInOut(followLerp);
            controls.object.position.lerpVectors(followStartCamPos, chaseCamDesired(_desiredTmp), k);
        } else {
            controls.object.position.lerp(chaseCamDesired(_desiredTmp), 0.12);
        }

        controls.target.copy(smoothCenter);
        lastRobotCenter.copy(smoothCenter);
    }

    // 绑定 UI
    document.getElementById('follow-robot')?.addEventListener('change', (e) => {
        setFollowEnabled(e.target.checked);
    });

    document.getElementById('show-robot')?.addEventListener('change', (e) => {
        dogMeshGroup.visible = e.target.checked;
    });

    document.getElementById('show-local-astar')?.addEventListener('change', (e) => {
        if (localAStarPathLine) localAStarPathLine.visible = e.target.checked;
        if (localPathLine) localPathLine.visible = e.target.checked;
    });

    document.getElementById('show-local-path')?.addEventListener('change', (e) => {
        if (localPathLine) localPathLine.visible = e.target.checked;
        if (localAStarPathLine) localAStarPathLine.visible = e.target.checked;
    });

    return {
        updatePose,
        updateCollisionNode,
        updateCollisionAnimation,
        updatePath,
        updateLocalPath,
        updateLocalAStarPath,
        focusRobot,
        setFollowEnabled,
        updateFollow,
        setDogVisible: (v) => { dogMeshGroup.visible = v; }
    };
}
