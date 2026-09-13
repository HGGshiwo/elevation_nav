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
            LEFT: THREE.MOUSE.ROTATE,
            MIDDLE: THREE.MOUSE.PAN,
            RIGHT: THREE.MOUSE.ROTATE
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
