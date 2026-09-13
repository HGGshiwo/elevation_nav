import * as THREE from 'three';

/**
 * 1:1 流形局部代价地图 3D 可视化器 (Local Costmap Visualizer)
 * 渲染实时 6.0m x 6.0m 贴地流形代价地毯与规划窗口外框
 */
export class LocalCostmapVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.visible = true;

        this.group = new THREE.Group();
        this.scene.add(this.group);

        // 离屏 Canvas 用于纹理绘制
        this.canvas = document.createElement('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.texture = null;

        // 贴地地毯网格 (Plane)
        this.planeMesh = null;

        // 局部窗口边界线框
        this.wireframeBox = null;
    }

    /**
     * 更新局部代价地图数据并重绘
     * @param {Object} costmapMsg { width, height, resolution, origin: {x, y, z}, data: base64_str }
     */
    update(costmapMsg) {
        if (!costmapMsg || !costmapMsg.data) return;

        const { width, height, resolution, origin, data: b64Data } = costmapMsg;
        const totalW = width * resolution;
        const totalH = height * resolution;

        // 解码 Base64 栅格数据
        const binaryStr = atob(b64Data);
        const len = binaryStr.length;
        const bytes = new Uint8Array(len);
        for (let i = 0; i < len; ++i) {
            bytes[i] = binaryStr.charCodeAt(i);
        }

        // 调整 Canvas 尺寸
        if (this.canvas.width !== width || this.canvas.height !== height) {
            this.canvas.width = width;
            this.canvas.height = height;
        }

        const imgData = this.ctx.createImageData(width, height);
        const rgba = imgData.data;

        // ROS OccupancyGrid 为行主序: index = r * width + c
        // Canvas (0, 0) 在左上角, 对应 ROS 坐标系的 (r = height - 1, c = 0)
        for (let r = 0; r < height; ++r) {
            const canvasY = height - 1 - r;
            for (let c = 0; c < width; ++c) {
                const rosIdx = r * width + c;
                const canvasIdx = (canvasY * width + c) * 4;
                const val = bytes[rosIdx];

                if (val === 0) {
                    // 自由地面 (微透亮青绿)
                    rgba[canvasIdx] = 56;
                    rgba[canvasIdx + 1] = 189;
                    rgba[canvasIdx + 2] = 248;
                    rgba[canvasIdx + 3] = 55;
                } else if (val > 0 && val < 100) {
                    // 通行代价/软膨胀区 (警示橙黄)
                    rgba[canvasIdx] = 251;
                    rgba[canvasIdx + 1] = 146;
                    rgba[canvasIdx + 2] = 60;
                    rgba[canvasIdx + 3] = Math.min(180, 80 + val);
                } else if (val === 100) {
                    // 致命障碍 / 悬崖 (鲜红半透)
                    rgba[canvasIdx] = 239;
                    rgba[canvasIdx + 1] = 68;
                    rgba[canvasIdx + 2] = 68;
                    rgba[canvasIdx + 3] = 190;
                } else {
                    // 未知 / 窗口外 (深灰透明)
                    rgba[canvasIdx] = 15;
                    rgba[canvasIdx + 1] = 23;
                    rgba[canvasIdx + 2] = 42;
                    rgba[canvasIdx + 3] = 20;
                }
            }
        }

        this.ctx.putImageData(imgData, 0, 0);

        // 创建或更新纹理
        if (!this.texture) {
            this.texture = new THREE.CanvasTexture(this.canvas);
            this.texture.minFilter = THREE.LinearFilter;
            this.texture.magFilter = THREE.LinearFilter;
        } else {
            this.texture.needsUpdate = true;
        }

        // 创建或更新地毯 PlaneMesh
        if (!this.planeMesh) {
            const planeGeo = new THREE.PlaneGeometry(totalW, totalH);
            const planeMat = new THREE.MeshBasicMaterial({
                map: this.texture,
                transparent: true,
                opacity: 0.85,
                depthWrite: false,
                side: THREE.DoubleSide
            });
            this.planeMesh = new THREE.Mesh(planeGeo, planeMat);
            this.group.add(this.planeMesh);
        } else {
            if (this.planeMesh.geometry.parameters.width !== totalW ||
                this.planeMesh.geometry.parameters.height !== totalH) {
                this.planeMesh.geometry.dispose();
                this.planeMesh.geometry = new THREE.PlaneGeometry(totalW, totalH);
            }
        }

        // 更新地毯位置 (中心对齐)
        const centerX = origin.x + totalW * 0.5;
        const centerY = origin.y + totalH * 0.5;
        const centerZ = origin.z + 0.02; // 贴地抬高 2cm 防 Z-Fighting
        this.planeMesh.position.set(centerX, centerY, centerZ);

        // 创建或更新外框边界线 (LineLoop)
        if (!this.wireframeBox) {
            const halfW = totalW * 0.5;
            const halfH = totalH * 0.5;
            const boxPts = [
                new THREE.Vector3(-halfW, -halfH, 0),
                new THREE.Vector3(halfW, -halfH, 0),
                new THREE.Vector3(halfW, halfH, 0),
                new THREE.Vector3(-halfW, halfH, 0)
            ];
            const boxGeo = new THREE.BufferGeometry().setFromPoints(boxPts);
            const boxMat = new THREE.LineBasicMaterial({
                color: 0x38bdf8,
                linewidth: 2,
                transparent: true,
                opacity: 0.7
            });
            this.wireframeBox = new THREE.LineLoop(boxGeo, boxMat);
            this.group.add(this.wireframeBox);
        } else {
            // 如果尺寸发生变化，重新设置线框尺寸
            const halfW = totalW * 0.5;
            const halfH = totalH * 0.5;
            const boxPts = [
                new THREE.Vector3(-halfW, -halfH, 0),
                new THREE.Vector3(halfW, -halfH, 0),
                new THREE.Vector3(halfW, halfH, 0),
                new THREE.Vector3(-halfW, halfH, 0)
            ];
            this.wireframeBox.geometry.setFromPoints(boxPts);
        }
        this.wireframeBox.position.set(centerX, centerY, centerZ + 0.005);

        this.group.visible = this.visible;
    }

    setVisible(visible) {
        this.visible = visible;
        this.group.visible = visible;
    }
}
