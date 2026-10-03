/**
 * 2D 分层踏面地图交互浏览与 3D 静态图精准联动模块 (FloorMapVisualizer)
 * - 异步调用服务端生成 2D 分层地图；
 * - 渲染多层切换标签、层高程统计与 2D 高分辨率画布；
 * - 支持 2D 像素与 3D 踏面节点 1-to-1 精确映射：点击 2D 像素点时，自动调焦并高亮 3D 场景中的对应静态图节点；
 * - 支持一键打包 ZIP 导出。
 */

import { makeDraggable } from './draggable.js';

export class FloorMapVisualizer {
    constructor(graphVisualizer) {
        this.graphVisualizer = graphVisualizer;
        this.layers = [];
        this.currentLayerId = 0;
        this.currentLookup = {};
        this.currentImage = null;
        this.isLoading = false;

        this.initDOM();
        this.bindEvents();
    }

    initDOM() {
        // 创建弹窗容器（若不存在）
        if (!document.getElementById('floor-map-modal')) {
            const modal = document.createElement('div');
            modal.id = 'floor-map-modal';
            modal.style.cssText = `
                position: absolute;
                top: 50%;
                left: 50%;
                transform: translate(-50%, -50%);
                width: 900px;
                max-width: 94vw;
                height: 640px;
                max-height: 90vh;
                background: rgba(26, 28, 34, 0.96);
                border: 1px solid #673ab7;
                border-radius: 10px;
                box-shadow: 0 10px 30px rgba(0, 0, 0, 0.8);
                display: none;
                flex-direction: column;
                z-index: 100;
                overflow: hidden;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
                color: #eee;
            `;

            modal.innerHTML = `
                <!-- 顶部标题栏 -->
                <div id="floor-modal-header" style="display: flex; justify-content: space-between; align-items: center; padding: 12px 18px; background: rgba(103, 58, 183, 0.35); border-bottom: 1px solid #555; cursor: grab;">
                    <div style="display: flex; align-items: center; gap: 10px;">
                        <span style="font-size: 15px; font-weight: bold; color: #b388ff;">2D 分层踏面地图与 3D 空间定位</span>
                        <span id="floor-layer-badge" style="font-size: 11px; background: #512da8; color: #fff; padding: 2px 8px; border-radius: 12px;">待生成</span>
                    </div>
                    <div style="display: flex; gap: 8px; align-items: center;">
                        <button id="btn-download-maps-zip" style="background: #ff9800; color: #fff; border: none; padding: 5px 12px; font-size: 12px; border-radius: 4px; cursor: pointer; font-weight: bold; display: flex; align-items: center; gap: 4px;">
                            打包下载全部 (ZIP)
                        </button>
                        <button id="btn-close-floor-modal" style="background: transparent; color: #aaa; border: none; font-size: 20px; cursor: pointer; padding: 0 6px;">&times;</button>
                    </div>
                </div>

                <!-- 层级切换与统计栏 -->
                <div style="display: flex; justify-content: space-between; align-items: center; padding: 8px 16px; background: #21242d; border-bottom: 1px solid #3a3f4d;">
                    <div id="floor-tabs-container" style="display: flex; gap: 6px; overflow-x: auto; max-width: 65%;">
                        <span style="font-size: 12px; color: #888; padding: 4px 0;">暂无分层数据，请点击左侧面板“生成 2D 分层地图”</span>
                    </div>
                    <div id="floor-layer-stats" style="font-size: 11px; color: #aaa; text-align: right;">
                        高程: - | 分辨率: 0.10m | 踏面节点: 0
                    </div>
                </div>

                <!-- 高程伪彩色带图例 (Turbo Colormap) -->
                <div id="floor-colorbar-wrap" style="display: flex; align-items: center; gap: 8px; padding: 4px 16px; background: #181b22; border-bottom: 1px solid #2d3240; font-size: 11px;">
                    <span style="color: #b0bec5; font-weight: bold;">高程色阶:</span>
                    <span id="colorbar-zmin" style="color: #448aff; font-weight: bold; min-width: 45px; text-align: right;">0.00m</span>
                    <div style="flex: 1; max-width: 260px; height: 10px; border-radius: 5px; background: linear-gradient(to right, #30123b, #4675ed, #1bcfd4, #61fc4c, #f3c63a, #f36315, #7a0402); box-shadow: inset 0 0 2px rgba(0,0,0,0.8);"></div>
                    <span id="colorbar-zmax" style="color: #ff5252; font-weight: bold; min-width: 45px;">0.00m</span>
                    <span style="color: #888; font-size: 10px; margin-left: 6px;">(低处深蓝 ➔ 中间黄绿 ➔ 高处亮红)</span>
                </div>

                <!-- 主视窗区：左侧 2D Canvas + 右侧 1-to-1 详情面板 -->
                <div style="display: flex; flex: 1; overflow: hidden; position: relative;">
                    <!-- 2D 画布区 -->
                    <div id="floor-canvas-wrap" style="flex: 1; position: relative; background: #15171c; display: flex; justify-content: center; align-items: center; overflow: hidden; cursor: crosshair;">
                        <canvas id="floor-2d-canvas" style="box-shadow: 0 0 10px rgba(0,0,0,0.5); border: 1px solid #333;"></canvas>
                        <div id="floor-canvas-hint" style="position: absolute; bottom: 8px; left: 10px; font-size: 11px; color: #888; background: rgba(0,0,0,0.6); padding: 2px 8px; border-radius: 4px; pointer-events: none;">
                            点击 2D 地图像素即可精确定位并联动 3D 视图 | 滚轮缩放 | 拖拽平移
                        </div>
                    </div>

                    <!-- 右侧/浮动 节点诊断与联动卡片 -->
                    <div style="width: 260px; background: #1e212b; border-left: 1px solid #333; padding: 14px; display: flex; flex-direction: column; justify-content: space-between;">
                        <div>
                            <div style="font-size: 13px; font-weight: bold; color: #00e5ff; margin-bottom: 10px; border-bottom: 1px solid #444; padding-bottom: 6px;">
                                1-to-1 空间节点映射
                            </div>
                            
                            <div class="row" style="margin-bottom: 6px; font-size: 12px; display: flex; justify-content: space-between;">
                                <span style="color: #aaa;">2D 栅格 (Col, Row):</span>
                                <span id="fnode-pixel" style="color: #fff; font-weight: bold;">-</span>
                            </div>
                            <div class="row" style="margin-bottom: 6px; font-size: 12px; display: flex; justify-content: space-between;">
                                <span style="color: #aaa;">世界坐标 X, Y:</span>
                                <span id="fnode-xy" style="color: #69f0ae; font-weight: bold;">-</span>
                            </div>
                            <div class="row" style="margin-bottom: 6px; font-size: 12px; display: flex; justify-content: space-between; align-items: center;">
                                <span style="color: #aaa;">踏面高程 Z:</span>
                                <div style="display: flex; align-items: center; gap: 5px;">
                                    <span id="fnode-color-dot" style="display: inline-block; width: 10px; height: 10px; border-radius: 50%; background: #fff; border: 1px solid #444;"></span>
                                    <span id="fnode-z" style="color: #ffd600; font-weight: bold;">-</span>
                                </div>
                            </div>
                            <div class="row" style="margin-bottom: 6px; font-size: 12px; display: flex; justify-content: space-between;">
                                <span style="color: #aaa;">3D 拓扑节点 ID:</span>
                                <span id="fnode-id" style="color: #e040fb; font-weight: bold;">-</span>
                            </div>
                            <div class="row" style="margin-bottom: 6px; font-size: 12px; display: flex; justify-content: space-between;">
                                <span style="color: #aaa;">通行度 (Trav):</span>
                                <span id="fnode-trav" style="color: #fff;">-</span>
                            </div>
                            <div class="row" style="margin-bottom: 10px; font-size: 12px; display: flex; justify-content: space-between;">
                                <span style="color: #aaa;">区域类型 (Zone):</span>
                                <span id="fnode-zone" style="color: #fff;">-</span>
                            </div>

                            <div id="fnode-status-badge" style="padding: 6px 10px; border-radius: 4px; font-size: 11px; text-align: center; background: #2a2e3d; color: #888; margin-bottom: 10px;">
                                点击 2D 踏面任意像素以开始
                            </div>
                        </div>

                        <div style="display: flex; flex-direction: column; gap: 8px;">
                            <button id="btn-focus-3d-node" style="width: 100%; background: #00bcd4; color: #fff; border: none; padding: 7px; border-radius: 4px; font-weight: bold; font-size: 12px; cursor: pointer;">
                                3D 视角聚焦此节点
                            </button>
                            <button id="btn-copy-node-info" style="width: 100%; background: #455a64; color: #fff; border: none; padding: 6px; border-radius: 4px; font-size: 11px; cursor: pointer;">
                                复制节点坐标与ID
                            </button>
                        </div>
                    </div>
                </div>
            `;
            document.body.appendChild(modal);

            const headerEl = document.getElementById('floor-modal-header');
            makeDraggable(modal, headerEl);
        }
    }

    bindEvents() {
        // 生成 2D 分层地图按钮
        const btnGen = document.getElementById('btn-generate-2d');
        if (btnGen) {
            btnGen.addEventListener('click', () => this.generateFloorMaps());
        }

        // 打开浏览弹窗按钮
        const btnOpen = document.getElementById('btn-open-2d-modal');
        if (btnOpen) {
            btnOpen.addEventListener('click', () => this.openModal());
        }

        // 关闭弹窗
        document.getElementById('btn-close-floor-modal')?.addEventListener('click', () => this.closeModal());

        // 打包下载 ZIP
        document.getElementById('btn-download-maps-zip')?.addEventListener('click', () => {
            window.open('/api/floor_maps/download_zip', '_blank');
        });

        // 3D 聚焦按钮
        document.getElementById('btn-focus-3d-node')?.addEventListener('click', () => {
            if (this.selectedNode && this.graphVisualizer) {
                this.graphVisualizer.setDebugMarkerA(this.selectedNode);
                this.graphVisualizer.focusOnNode(this.selectedNode);
            }
        });

        // 复制节点信息
        document.getElementById('btn-copy-node-info')?.addEventListener('click', () => {
            if (this.selectedNode) {
                const text = JSON.stringify(this.selectedNode, null, 2);
                navigator.clipboard?.writeText(text);
                alert("已复制节点详细数据到剪贴板！");
            }
        });

        // 画布交互 (点击吸附、平移与缩放)
        const canvas = document.getElementById('floor-2d-canvas');
        if (canvas) {
            canvas.addEventListener('click', (e) => this.onCanvasClick(e));
            canvas.addEventListener('mousemove', (e) => this.onCanvasMouseMove(e));
        }
    }

    async generateFloorMaps() {
        const btn = document.getElementById('btn-generate-2d');
        const statusEl = document.getElementById('status');
        if (btn) btn.textContent = "⏳ 正在展平建图中...";

        try {
            const resp = await fetch('/api/floor_maps/generate', { method: 'POST' });
            const data = await resp.json();
            if (!resp.ok) {
                throw new Error(data.detail || "生成失败");
            }

            if (statusEl) statusEl.textContent = `2D 分层建图完成: 共 ${data.total_layers} 个图层`;
            this.layers = data.layers || [];
            this.openModal();
            this.renderTabs();
            if (this.layers.length > 0) {
                this.selectLayer(0);
            }
        } catch (err) {
            alert(`生成 2D 分层地图失败: ${err.message}`);
            if (statusEl) statusEl.textContent = `生成失败: ${err.message}`;
        } finally {
            if (btn) btn.textContent = "📐 生成 2D 分层地图";
        }
    }

    async openModal() {
        const modal = document.getElementById('floor-map-modal');
        if (modal) modal.style.display = 'flex';

        if (this.layers.length === 0) {
            // 尝试读取现有图层列表
            try {
                const resp = await fetch('/api/floor_maps/layers');
                const data = await resp.json();
                if (data.has_layers && data.layers?.length > 0) {
                    this.layers = data.layers;
                    this.renderTabs();
                    this.selectLayer(0);
                }
            } catch (e) {
                console.warn("读取图层清单失败:", e);
            }
        }
    }

    closeModal() {
        const modal = document.getElementById('floor-map-modal');
        if (modal) modal.style.display = 'none';
    }

    renderTabs() {
        const container = document.getElementById('floor-tabs-container');
        if (!container) return;

        if (this.layers.length === 0) {
            container.innerHTML = '<span style="font-size: 12px; color: #888;">暂无分层数据</span>';
            return;
        }

        container.innerHTML = '';
        this.layers.forEach((layer, idx) => {
            const btn = document.createElement('button');
            btn.textContent = `层 ${idx + 1} (Z: ${layer.z_mean.toFixed(1)}m)`;
            btn.style.cssText = `
                padding: 4px 10px;
                font-size: 11px;
                border: 1px solid #555;
                background: ${idx === this.currentLayerId ? '#673ab7' : '#333'};
                color: #fff;
                border-radius: 4px;
                cursor: pointer;
                white-space: nowrap;
                transition: all 0.15s ease;
            `;
            btn.addEventListener('click', () => this.selectLayer(idx));
            container.appendChild(btn);
        });
    }

    async selectLayer(layerId) {
        this.currentLayerId = layerId;
        const layer = this.layers[layerId];
        if (!layer) return;

        this.renderTabs(); // 刷新高亮

        // 更新顶部状态信息
        const badge = document.getElementById('floor-layer-badge');
        if (badge) badge.textContent = `第 ${layerId + 1} 层 / 共 ${this.layers.length} 层`;

        const stats = document.getElementById('floor-layer-stats');
        if (stats) {
            stats.textContent = `高程区间: [${layer.z_min.toFixed(2)}m ~ ${layer.z_max.toFixed(2)}m] | 尺寸: ${layer.width}×${layer.height} | 踏面节点: ${layer.node_count} 个`;
        }

        // 更新色带刻度
        const zminEl = document.getElementById('colorbar-zmin');
        const zmaxEl = document.getElementById('colorbar-zmax');
        if (zminEl) zminEl.textContent = `${layer.z_min.toFixed(2)}m`;
        if (zmaxEl) zmaxEl.textContent = `${layer.z_max.toFixed(2)}m`;

        // 加载 1-to-1 节点查找表
        try {
            const resp = await fetch(`/api/floor_maps/layer/${layerId}/nodes`);
            if (resp.ok) {
                this.currentLookup = await resp.json();
            }
        } catch (e) {
            console.warn("加载节点查找表失败:", e);
        }

        // 加载并渲染彩色预览图像
        const img = new Image();
        img.src = `/static/generated_maps/${layer.vis_image_file}?t=${Date.now()}`;
        img.onload = () => {
            this.currentImage = img;
            this.drawCanvas(layer, img);
        };
    }

    drawCanvas(layer, img, highlightPixel = null) {
        const canvas = document.getElementById('floor-2d-canvas');
        if (!canvas || !img) return;

        const ctx = canvas.getContext('2d');
        const wrap = document.getElementById('floor-canvas-wrap');
        const maxW = (wrap ? wrap.clientWidth : 600) - 30;
        const maxH = (wrap ? wrap.clientHeight : 500) - 30;

        const scale = Math.min(maxW / img.width, maxH / img.height, 4.0);
        canvas.width = Math.round(img.width * scale);
        canvas.height = Math.round(img.height * scale);
        this.canvasScale = scale;

        ctx.imageSmoothingEnabled = false; // 保持栅格像素清晰
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

        // 绘制选中的像素十字高亮
        if (highlightPixel) {
            const { col, row } = highlightPixel;
            const px = col * scale;
            const py = row * scale;
            const pw = Math.max(1, scale);
            const ph = Math.max(1, scale);

            // 绘制方块高亮边框
            ctx.strokeStyle = '#00e5ff';
            ctx.lineWidth = 2;
            ctx.strokeRect(px - 2, py - 2, pw + 4, ph + 4);

            // 绘制准星十字线
            ctx.strokeStyle = '#ffd600';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.moveTo(px + pw / 2, 0);
            ctx.lineTo(px + pw / 2, canvas.height);
            ctx.moveTo(0, py + ph / 2);
            ctx.lineTo(canvas.width, py + ph / 2);
            ctx.stroke();
        }
    }

    onCanvasClick(e) {
        const canvas = document.getElementById('floor-2d-canvas');
        if (!canvas || !this.currentImage || !this.canvasScale) return;

        const rect = canvas.getBoundingClientRect();
        const clickX = e.clientX - rect.left;
        const clickY = e.clientY - rect.top;

        const col = Math.floor(clickX / this.canvasScale);
        const row = Math.floor(clickY / this.canvasScale);

        const key = `${col},${row}`;
        const node = this.currentLookup[key];

        const layer = this.layers[this.currentLayerId];
        this.drawCanvas(layer, this.currentImage, { col, row });

        if (node) {
            this.selectedNode = node;
            this.updateNodeCard(node, col, row);

            // 🌟 核心：1-to-1 精确联动 3D 静态拓扑图！
            if (this.graphVisualizer) {
                this.graphVisualizer.setDebugMarkerA(node);
                this.graphVisualizer.focusOnNode(node);
            }
        } else {
            this.selectedNode = null;
            this.clearNodeCard(col, row);
        }
    }

    onCanvasMouseMove(e) {
        const canvas = document.getElementById('floor-2d-canvas');
        if (!canvas || !this.canvasScale) return;

        const rect = canvas.getBoundingClientRect();
        const col = Math.floor((e.clientX - rect.left) / this.canvasScale);
        const row = Math.floor((e.clientY - rect.top) / this.canvasScale);

        const hint = document.getElementById('floor-canvas-hint');
        if (hint) {
            const key = `${col},${row}`;
            const node = this.currentLookup[key];
            if (node) {
                hint.textContent = `像素 (${col}, ${row}) -> 3D世界坐标: (${node.x.toFixed(2)}, ${node.y.toFixed(2)}, ${node.z.toFixed(2)}) | 节点 #${node.node_id}`;
                hint.style.color = '#00e5ff';
            } else {
                hint.textContent = `像素 (${col}, ${row}) [未知/空白区域]`;
                hint.style.color = '#888';
            }
        }
    }

    updateNodeCard(node, col, row) {
        document.getElementById('fnode-pixel').textContent = `(${col}, ${row})`;
        document.getElementById('fnode-xy').textContent = `${node.x.toFixed(2)}, ${node.y.toFixed(2)}`;
        document.getElementById('fnode-z').textContent = `${node.z.toFixed(2)} m`;
        document.getElementById('fnode-id').textContent = `#${node.node_id}`;
        document.getElementById('fnode-trav').textContent = `${node.traversability.toFixed(2)}`;

        const dot = document.getElementById('fnode-color-dot');
        if (dot) {
            dot.style.background = node.color || '#ffd600';
            dot.style.borderColor = '#fff';
        }
        
        const zoneLabels = ["0 (自由区)", "1 (软代价/近墙)", "2 (机体硬禁行)", "3 (障碍物)"];
        document.getElementById('fnode-zone').textContent = zoneLabels[node.cost_zone] || String(node.cost_zone);

        const badge = document.getElementById('fnode-status-badge');
        if (badge) {
            badge.style.background = '#388e3c';
            badge.style.color = '#fff';
            badge.textContent = '✅ 可通行踏面节点 (3D 视图已同步对齐)';
        }
    }

    clearNodeCard(col, row) {
        document.getElementById('fnode-pixel').textContent = `(${col}, ${row})`;
        document.getElementById('fnode-xy').textContent = `-`;
        document.getElementById('fnode-z').textContent = `-`;
        document.getElementById('fnode-id').textContent = `-`;
        document.getElementById('fnode-trav').textContent = `-`;
        document.getElementById('fnode-zone').textContent = `-`;

        const dot = document.getElementById('fnode-color-dot');
        if (dot) {
            dot.style.background = 'transparent';
            dot.style.borderColor = '#444';
        }

        const badge = document.getElementById('fnode-status-badge');
        if (badge) {
            badge.style.background = '#2a2e3d';
            badge.style.color = '#888';
            badge.textContent = '该像素无 3D 节点数据 (未知/悬空)';
        }
    }
}
