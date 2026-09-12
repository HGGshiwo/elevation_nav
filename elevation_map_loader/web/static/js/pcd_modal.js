import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/**
 * 点云导入与 3D 预览模态弹窗管理器
 */
export class PCDModal {
    constructor(onGridMapGenerated) {
        this.onGridMapGenerated = onGridMapGenerated;

        // DOM 元素缓存
        this.modalEl = document.getElementById('pcd-modal');
        this.btnOpen = document.getElementById('btn-open-pcd-modal');
        this.btnClose = document.getElementById('btn-close-modal');
        this.btnCancel = document.getElementById('btn-modal-cancel');
        this.btnPreview = document.getElementById('btn-load-preview');
        this.btnConfirm = document.getElementById('btn-confirm-generate');

        this.inputPath = document.getElementById('input-pcd-path');
        this.inputRes = document.getElementById('input-res');
        this.inputStep = document.getElementById('input-step');
        this.inputStepRadius = document.getElementById('input-step-radius');
        this.inputSlope = document.getElementById('input-slope');
        this.inputFill = document.getElementById('input-fill');
        this.statusEl = document.getElementById('modal-status');

        this.badgeName = document.getElementById('badge-name');
        this.badgePoints = document.getElementById('badge-points');
        this.badgeSampled = document.getElementById('badge-sampled');
        this.badgeSpan = document.getElementById('badge-span');

        // 独立小视窗 Three.js 实例
        this.previewScene = null;
        this.previewCamera = null;
        this.previewRenderer = null;
        this.previewControls = null;
        this.pointCloudMesh = null;

        this._initEvents();
        this._initMiniScene();
    }

    _initEvents() {
        this.btnOpen.addEventListener('click', () => {
            this.modalEl.style.display = 'flex';
            this._onWindowResize();
            this.loadPreview();
        });

        const closeModal = () => { this.modalEl.style.display = 'none'; };
        this.btnClose.addEventListener('click', closeModal);
        this.btnCancel.addEventListener('click', closeModal);

        this.btnPreview.addEventListener('click', () => this.loadPreview());
        this.btnConfirm.addEventListener('click', () => this.generateGridMap());

        // 预设路径快捷点击
        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                this.inputPath.value = e.target.dataset.path;
                this.loadPreview();
            });
        });
    }

    _initMiniScene() {
        const container = document.getElementById('pcd-preview-canvas');
        if (!container) return;

        this.previewScene = new THREE.Scene();
        this.previewScene.background = new THREE.Color(0x0a0c10);

        this.previewCamera = new THREE.PerspectiveCamera(50, 1, 0.1, 500);
        this.previewCamera.position.set(0, -10, 8);
        this.previewCamera.up.set(0, 0, 1);

        this.previewRenderer = new THREE.WebGLRenderer({ antialias: true });
        container.appendChild(this.previewRenderer.domElement);

        this.previewControls = new OrbitControls(this.previewCamera, this.previewRenderer.domElement);
        this.previewControls.enableDamping = true;
        this.previewControls.dampingFactor = 0.1;

        // 辅助网格
        const grid = new THREE.GridHelper(30, 30, 0x30363d, 0x1f242c);
        grid.rotation.x = Math.PI / 2;
        this.previewScene.add(grid);

        const animate = () => {
            requestAnimationFrame(animate);
            if (this.modalEl.style.display === 'flex') {
                this.previewControls.update();
                this.previewRenderer.render(this.previewScene, this.previewCamera);
            }
        };
        animate();
    }

    _onWindowResize() {
        const container = document.getElementById('pcd-preview-canvas');
        if (!container || !this.previewRenderer) return;
        const w = container.clientWidth || 400;
        const h = container.clientHeight || 460;
        this.previewCamera.aspect = w / h;
        this.previewCamera.updateProjectionMatrix();
        this.previewRenderer.setSize(w, h);
    }

    async loadPreview() {
        const pcdPath = this.inputPath.value.trim();
        if (!pcdPath) {
            this.statusEl.innerText = "请输入有效的 PCD 路径";
            this.statusEl.style.color = "#f85149";
            return;
        }

        this.statusEl.innerText = "⏳ 正在读取点云元数据...";
        this.statusEl.style.color = "#58a6ff";

        try {
            const resp = await fetch('/api/pcd/preview', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ pcd_path: pcdPath, target_points: 15000 })
            });

            if (!resp.ok) {
                const err = await resp.json();
                throw new Error(err.detail || "读取点云异常");
            }

            const data = await resp.json();
            this._updatePreviewCloud(data);
            this.statusEl.innerText = "✅ 点云解析成功，可旋转检查";
            this.statusEl.style.color = "#7ee787";
        } catch (e) {
            this.statusEl.innerText = `❌ ${e.message}`;
            this.statusEl.style.color = "#f85149";
        }
    }

    _updatePreviewCloud(data) {
        // 更新 Badge
        this.badgeName.innerText = `${data.file_name} (${data.file_size_mb} MB)`;
        this.badgePoints.innerText = data.total_points.toLocaleString();
        this.badgeSampled.innerText = data.preview_points_count.toLocaleString();
        this.badgeSpan.innerText = `${data.span[0]}m × ${data.span[1]}m × ${data.span[2]}m`;

        // 移除旧点云
        if (this.pointCloudMesh) {
            this.previewScene.remove(this.pointCloudMesh);
            this.pointCloudMesh.geometry.dispose();
            this.pointCloudMesh.material.dispose();
            this.pointCloudMesh = null;
        }

        const flatPts = data.points;
        const count = flatPts.length / 3;
        const positions = new Float32Array(flatPts);
        const colors = new Float32Array(flatPts.length);

        const minZ = data.min_bound[2];
        const maxZ = data.max_bound[2];
        const zSpan = Math.max(0.1, maxZ - minZ);

        for (let i = 0; i < count; i++) {
            const z = flatPts[i * 3 + 2];
            const ratio = (z - minZ) / zSpan;
            const c = new THREE.Color();
            c.setHSL((220 - ratio * 180) / 360, 0.85, 0.5); // 蓝 -> 青 -> 绿 -> 橙
            colors[i * 3] = c.r;
            colors[i * 3 + 1] = c.g;
            colors[i * 3 + 2] = c.b;
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

        const mat = new THREE.PointsMaterial({
            size: 0.12,
            vertexColors: true,
            sizeAttenuation: true
        });

        this.pointCloudMesh = new THREE.Points(geo, mat);
        this.previewScene.add(this.pointCloudMesh);

        // 居中对齐相机目标
        const cx = (data.min_bound[0] + data.max_bound[0]) / 2;
        const cy = (data.min_bound[1] + data.max_bound[1]) / 2;
        const cz = (data.min_bound[2] + data.max_bound[2]) / 2;

        this.previewControls.target.set(cx, cy, cz);
        this.previewCamera.position.set(cx, cy - Math.max(data.span[0], data.span[1]) * 1.2, cz + data.span[2] * 1.5);
        this.previewControls.update();
    }

    async generateGridMap() {
        const pcdPath = this.inputPath.value.trim();
        const res = parseFloat(this.inputRes.value) || 0.10;
        const step = parseFloat(this.inputStep.value) || 0.25;
        const stepRadius = parseFloat(this.inputStepRadius?.value) || 0.25;
        const slope = parseFloat(this.inputSlope.value) || 30.0;
        const fill = parseFloat(this.inputFill.value) || 0.20;

        this.statusEl.innerText = "⏳ 正在构建 2.5D 高程图 (地表高程/台阶落差/障碍掩码)...";
        this.statusEl.style.color = "#58a6ff";
        this.btnConfirm.disabled = true;

        try {
            const resp = await fetch('/api/pcd/generate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    pcd_path: pcdPath,
                    resolution: res,
                    max_step_height: step,
                    step_radius: stepRadius,
                    max_slope_deg: slope,
                    fill_radius: fill
                })
            });

            if (!resp.ok) {
                const err = await resp.json();
                throw new Error(err.detail || "生成高程图失败");
            }

            const gridMapData = await resp.json();
            this.statusEl.innerText = "✅ 高程图生成完毕！";
            this.statusEl.style.color = "#7ee787";

            // 回调主视窗加载渲染 GridMap
            if (this.onGridMapGenerated) {
                this.onGridMapGenerated(gridMapData);
            }

            // 自动关闭弹窗
            setTimeout(() => {
                this.modalEl.style.display = 'none';
                this.btnConfirm.disabled = false;
            }, 600);

        } catch (e) {
            this.statusEl.innerText = `❌ ${e.message}`;
            this.statusEl.style.color = "#f85149";
            this.btnConfirm.disabled = false;
        }
    }
}
