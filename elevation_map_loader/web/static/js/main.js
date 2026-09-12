import * as THREE from 'three';
import { createMainScene } from './scene.js';
import { GridMapLayer } from './grid_map_layer.js';
import { RobotVisualizer } from './robot_visualizer.js';
import { PathVisualizer } from './path_visualizer.js';
import { PCDModal } from './pcd_modal.js';
import { WsClient } from './ws_client.js';
import { RoamController } from './controls_roam.js';

// ---- 1. 主场景初始化 ----
const container = document.getElementById('canvas-container');
const { scene, camera, renderer, controls, setUpdateCallback } = createMainScene(container);

// 键盘漫游控制器
const roamController = new RoamController(camera, controls);
setUpdateCallback(() => {
    roamController.update();
});

// ---- 2. 核心组件实例化 ----
const gridMapLayer = new GridMapLayer(scene);
const robotVisualizer = new RobotVisualizer(scene, controls);
const pathVisualizer = new PathVisualizer(scene);

// DOM 状态元素
const infoMapStatus = document.getElementById('info-map-status');
const infoMapRes = document.getElementById('info-map-res');
const infoMapSize = document.getElementById('info-map-size');
const infoRobotPose = document.getElementById('info-robot-pose');
const rosBadge = document.getElementById('ros-badge');

// ---- 3. 点云弹窗与高程图渲染回调 ----
function onGridMapReady(data) {
    gridMapLayer.loadGridMap(data);

    const meta = data.metadata;
    infoMapStatus.innerText = "已加载";
    infoMapStatus.style.color = "#7ee787";
    infoMapRes.innerText = `${meta.resolution}m`;
    infoMapSize.innerText = `${meta.cols}×${meta.rows} (${meta.length_x}m×${meta.length_y}m)`;

    // 将相机视角调整到高程图中心
    controls.target.set(meta.center_x, meta.center_y, (meta.min_elevation + meta.max_elevation) / 2);
    camera.position.set(meta.center_x, meta.center_y - Math.max(meta.length_x, meta.length_y) * 0.8, 15);
    controls.update();
}

const pcdModal = new PCDModal(onGridMapReady);

// 尝试加载后端当前已存在的高程图
fetch('/api/grid_map/current')
    .then(r => r.json())
    .then(res => {
        if (res.has_map && res.data) {
            onGridMapReady(res.data);
        }
    })
    .catch(() => {});

// ---- 4. WebSocket 实时数据绑定 ----
const wsClient = new WsClient(
    // 机器人位姿更新
    (pose) => {
        robotVisualizer.updatePose(pose);
        infoRobotPose.innerText = `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.z.toFixed(2)})`;
    },
    // 路径更新
    (globalPath, localPath) => {
        if (globalPath) pathVisualizer.updateGlobalPath(globalPath);
        if (localPath) pathVisualizer.updateLocalPath(localPath);
    },
    // 连接状态
    (status) => {
        if (status === 'connected') {
            rosBadge.innerText = "ROS在线";
            rosBadge.style.background = "#238636";
        } else if (status === 'connecting') {
            rosBadge.innerText = "连接中...";
            rosBadge.style.background = "#d29922";
        } else {
            rosBadge.innerText = "离线";
            rosBadge.style.background = "#da3633";
        }
    }
);

// ---- 5. UI 交互控制 ----
document.getElementById('show-terrain').addEventListener('change', (e) => {
    gridMapLayer.setTerrainVisible(e.target.checked);
});
document.getElementById('show-obstacle').addEventListener('change', (e) => {
    gridMapLayer.setObstacleVisible(e.target.checked);
});
document.getElementById('show-robot').addEventListener('change', (e) => {
    robotVisualizer.setVisible(e.target.checked);
});
document.getElementById('show-path').addEventListener('change', (e) => {
    pathVisualizer.setVisible(e.target.checked);
});

document.getElementById('btn-focus-robot').addEventListener('click', () => {
    robotVisualizer.focusRobot();
});

document.getElementById('btn-cancel-goal').addEventListener('click', () => {
    fetch('/api/nav/cancel_goal', { method: 'POST' });
});

// ---- 6. 射线拾取与目标点交互 ----
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

renderer.domElement.addEventListener('click', (e) => {
    const activeTool = document.querySelector('input[name="main-tool"]:checked')?.value;
    if (activeTool !== 'goal') return;

    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);

    if (gridMapLayer.terrainMesh) {
        const intersects = raycaster.intersectObject(gridMapLayer.terrainMesh);
        if (intersects.length > 0) {
            const hit = intersects[0].point;
            // 发布导航目标点
            fetch('/api/nav/set_goal', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    x: hit.x,
                    y: hit.y,
                    z: hit.z,
                    yaw: 0.0
                })
            });

            // 临时高亮点击标记
            createGoalMarker(hit);
        }
    }
});

let goalMarker = null;
function createGoalMarker(pos) {
    if (goalMarker) scene.remove(goalMarker);
    const geo = new THREE.ConeGeometry(0.2, 0.6, 16);
    const mat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    goalMarker = new THREE.Mesh(geo, mat);
    goalMarker.rotation.x = Math.PI; // 箭头朝下
    goalMarker.position.set(pos.x, pos.y, pos.z + 0.4);
    scene.add(goalMarker);
}
