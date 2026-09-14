import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/**
 * 3D 独立漫游与视角管理器 (RoamController)
 * 完整统一封装：
 * 1. 鼠标全向交互：OrbitControls (左/右键旋转、中键平移、滚轮缩放、阻尼平滑)
 * 2. 键盘自由飞行漫游：W/A/S/D 水平平移、Q/E/Space/C 升降漫游、Shift 2.5倍加速
 * 3. 视角聚焦控制：支持对准目标点与机器人平滑追踪
 */
export class RoamController {
    constructor(camera, domElement, options = {}) {
        this.camera = camera;
        this.domElement = domElement;

        // 1. 初始化并配置 OrbitControls
        this.controls = new OrbitControls(camera, domElement);
        this.controls.mouseButtons = {
            LEFT: null,                 // 左键由 MC 式第一人称视角旋转接管 (见 cameraLookEnabled), 不再环绕目标
            MIDDLE: THREE.MOUSE.PAN,
            RIGHT: THREE.MOUSE.ROTATE   // 右键保留环绕旋转, 便于绕开遮挡物观察
        };
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.minDistance = 0.5;
        this.controls.maxDistance = 1000.0;

        // 2. 键盘漫游状态
        this.keys = {};
        this.speed = options.speed || 0.10; // 基础移动步长
        this.enabled = true;
        this.keyRoamEnabled = true;

        // 3. MC 式第一人称视角旋转 (左键拖动原地转头, 相机位置不动)
        //    仅 view 工具下开启 (由编辑器按当前工具切换), 其余工具左键留给拾取/绘制
        this.cameraLookEnabled = true;
        this.lookSensitivity = 0.0022; // rad/px
        this._lookActive = false;
        this._lastX = 0;
        this._lastY = 0;
        this._lookYaw = 0;      // 航向角 (绕世界 Z 轴, Z-up 约定)
        this._lookPitch = 0;    // 俯仰角
        this._lookDist = 10.0;  // 拖动开始时的环绕距离, 用于把 target 同步回视线前方

        this._initListeners();
    }

    _initListeners() {
        window.addEventListener('keydown', (e) => {
            // 如果焦点在输入框/文本域中，不拦截键盘按键
            const tag = document.activeElement?.tagName?.toLowerCase();
            if (tag === 'input' || tag === 'textarea' || tag === 'select') return;

            this.keys[e.code] = true;
        });

        window.addEventListener('keyup', (e) => {
            this.keys[e.code] = false;
        });

        // 窗口失焦时清空按键状态，防止按键粘连
        window.addEventListener('blur', () => {
            this.keys = {};
        });

        // ---- MC 式第一人称视角旋转 ----
        this.domElement.addEventListener('mousedown', (e) => {
            if (e.button !== 0) return;
            if (!this.cameraLookEnabled || !this.enabled || !this.controls.enabled) return;
            this._lookActive = true;
            this._lastX = e.clientX;
            this._lastY = e.clientY;

            // 从当前视线方向反解 yaw/pitch, 起始无跳变
            const dir = new THREE.Vector3().subVectors(this.controls.target, this.camera.position).normalize();
            this._lookPitch = Math.asin(THREE.MathUtils.clamp(dir.z, -1, 1));
            this._lookYaw = Math.atan2(dir.y, dir.x);
            this._lookDist = this.camera.position.distanceTo(this.controls.target);
        });

        window.addEventListener('mousemove', (e) => {
            if (!this._lookActive) return;
            const dx = e.clientX - this._lastX;
            const dy = e.clientY - this._lastY;
            this._lastX = e.clientX;
            this._lastY = e.clientY;

            // 鼠标右移 = 视线右转, 鼠标上移 = 抬头 (MC 习惯, 非反转)
            this._lookYaw -= dx * this.lookSensitivity;
            this._lookPitch -= dy * this.lookSensitivity;
            const lim = Math.PI / 2 - 0.01;
            this._lookPitch = Math.max(-lim, Math.min(lim, this._lookPitch));

            // 相机位置不动, 仅按新方向重建视线 (Z-up: 航向绕世界 Z, 俯仰为仰角)
            const cp = Math.cos(this._lookPitch);
            const dir = new THREE.Vector3(
                cp * Math.cos(this._lookYaw),
                cp * Math.sin(this._lookYaw),
                Math.sin(this._lookPitch)
            );
            const newTarget = this.camera.position.clone().addScaledVector(dir, this._lookDist);
            this.camera.lookAt(newTarget);
            // target 跟随视线前方: WASD 前向与右键环绕/滚轮缩放无缝衔接, 无视角跳变
            this.controls.target.copy(newTarget);
        });

        window.addEventListener('mouseup', (e) => {
            if (e.button === 0) this._lookActive = false;
        });
    }

    setEnabled(enabled) {
        this.enabled = enabled;
        this.controls.enabled = enabled;
        if (!enabled) {
            this.keys = {};
        }
    }

    setKeyRoamEnabled(enabled) {
        this.keyRoamEnabled = enabled;
        if (!enabled) {
            this.keys = {};
        }
    }

    update() {
        if (!this.enabled) return;

        // 仅在漫游开启且当前处于视角漫游模式下响应键盘
        if (this.keyRoamEnabled && this.controls.enabled) {
            this._updateKeyboardRoam();
        }

        // 驱动 OrbitControls 阻尼与视角更新
        this.controls.update();
    }

    _updateKeyboardRoam() {
        // 计算相机视线的水平前向向量 (XY 平面投影)
        const forward = new THREE.Vector3();
        forward.subVectors(this.controls.target, this.camera.position);
        forward.z = 0; // 投影到水平面
        if (forward.lengthSq() > 1e-6) {
            forward.normalize();
        } else {
            forward.set(0, 1, 0);
        }

        // 侧向右向量
        const right = new THREE.Vector3();
        right.crossVectors(forward, new THREE.Vector3(0, 0, 1)).normalize();

        // 加速键 Shift (2.5倍速)
        const currentSpeed = (this.keys['ShiftLeft'] || this.keys['ShiftRight']) ? this.speed * 2.5 : this.speed;

        const moveDelta = new THREE.Vector3(0, 0, 0);

        // 前进 / 后退 (W / S 或 ArrowUp / ArrowDown)
        if (this.keys['KeyW'] || this.keys['ArrowUp']) {
            moveDelta.addScaledVector(forward, currentSpeed);
        }
        if (this.keys['KeyS'] || this.keys['ArrowDown']) {
            moveDelta.addScaledVector(forward, -currentSpeed);
        }

        // 左移 / 右移 (A / D 或 ArrowLeft / ArrowRight)
        if (this.keys['KeyA'] || this.keys['ArrowLeft']) {
            moveDelta.addScaledVector(right, -currentSpeed);
        }
        if (this.keys['KeyD'] || this.keys['ArrowRight']) {
            moveDelta.addScaledVector(right, currentSpeed);
        }

        // 升降 (Q / Space 上升, E / C 下降)
        if (this.keys['KeyQ'] || this.keys['Space']) {
            moveDelta.z += currentSpeed;
        }
        if (this.keys['KeyE'] || this.keys['KeyC']) {
            moveDelta.z -= currentSpeed;
        }

        if (moveDelta.lengthSq() > 1e-6) {
            // 同时平移相机和焦点，保持相对视角平稳连续，不产生跳跃
            this.camera.position.add(moveDelta);
            this.controls.target.add(moveDelta);
        }
    }

    /**
     * 聚焦到指定 3D 空间坐标
     */
    focusOn(targetPos, span = 10.0) {
        if (!targetPos) return;
        this.controls.target.set(targetPos.x, targetPos.y, targetPos.z);
        this.camera.position.set(targetPos.x, targetPos.y - span * 1.2, targetPos.z + span * 0.9);
        this.controls.update();
    }
}
