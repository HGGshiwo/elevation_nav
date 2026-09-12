import * as THREE from 'three';

/**
 * 机器狗模型渲染、位姿追踪与导航路径可视化模块
 * 完整保留原版机器狗刚体、朝向箭头、全局路径、局部路径与局部A*路径渲染
 */
export function initRobotTracker(scene, controls) {
    const dogMeshGroup = new THREE.Group();
    let currentPose = { x: 0, y: 0, z: 0, yaw: 0 };

    // 1. 构建机器狗刚体模型
    const bodyGeo = new THREE.BoxGeometry(0.55, 0.28, 0.18);
    const bodyMat = new THREE.MeshStandardMaterial({
        color: 0x2196f3,
        roughness: 0.4,
        metalness: 0.5
    });
    const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
    bodyMesh.position.set(0, 0, 0.12);
    dogMeshGroup.add(bodyMesh);

    // 头部标识 (橙色)
    const headGeo = new THREE.BoxGeometry(0.12, 0.20, 0.12);
    const headMat = new THREE.MeshStandardMaterial({ color: 0xff9800 });
    const headMesh = new THREE.Mesh(headGeo, headMat);
    headMesh.position.set(0.28, 0, 0.14);
    dogMeshGroup.add(headMesh);

    // 航向指示箭头 (沿 X 轴正向)
    const arrowDir = new THREE.Vector3(1, 0, 0);
    const arrowOrigin = new THREE.Vector3(0.32, 0, 0.14);
    const arrowHelper = new THREE.ArrowHelper(arrowDir, arrowOrigin, 0.45, 0x00e676, 0.12, 0.08);
    dogMeshGroup.add(arrowHelper);

    scene.add(dogMeshGroup);

    // 2. 导航路径线条对象
    let pathLine = null;
    let localPathLine = null;
    let localAStarPathLine = null;

    const pathMat = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 3 });
    const localPathMat = new THREE.LineBasicMaterial({ color: 0xff3d00, linewidth: 4 });
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

    // 绑定 UI
    document.getElementById('btn-focus-robot')?.addEventListener('click', focusRobot);

    document.getElementById('show-robot')?.addEventListener('change', (e) => {
        dogMeshGroup.visible = e.target.checked;
    });

    document.getElementById('show-local-astar')?.addEventListener('change', (e) => {
        if (localAStarPathLine) localAStarPathLine.visible = e.target.checked;
    });

    return {
        updatePose,
        updatePath,
        updateLocalPath,
        updateLocalAStarPath,
        focusRobot,
        setDogVisible: (v) => { dogMeshGroup.visible = v; }
    };
}
