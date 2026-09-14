import * as THREE from 'three';

/**
 * 1:1 流形局部代价地图 3D 可视化器 (Local Costmap Visualizer)
 * 渲染实时 6.0m x 6.0m 贴地流形代价地毯与规划窗口外框
 * 支持调试模式: 按逐格成因码染色, 并支持点击查询格子诊断信息
 */

// 逐格成因码 (与 C++ CellReason 枚举一一对应)
export const CELL_REASON = {
    NO_NODE: 0,           // 从未被节点盖章 (默认致命) —— "看不见的障碍"主因
    FREE_NODE: 1,         // 可通行节点盖章
    SOFT_NODE: 2,         // 软代价节点 (贴墙减速带)
    BLOCK_HEADROOM: 3,    // 头顶净空不足
    BLOCK_LATERAL: 4,     // 侧向墙体硬阻挡
    BLOCK_OTHER: 5,       // 其它禁行
    SEAM: 6,              // 拓扑缝 (相邻踏面无边)
    CLOSING_FILLED: 7,    // 闭运算填充
    UNREACHED: 8          // 与机器人所在区域在地毯上不连通的自由透印
};

export const CELL_REASON_TEXT = {
    0: { label: '无节点盖章 (默认致命)', color: '#c084fc',
         detail: '附近无可通行踏面节点进入盖章半径 0.08m —— 点云空缺或图节点缺失, 地毯保留默认致命值 100。这就是"看起来没障碍却有障碍格"的主因。' },
    1: { label: '可通行节点盖章', color: '#69f0ae', detail: '由可通行踏面节点写入自由代价。' },
    2: { label: '软代价 (贴墙减速带)', color: '#ffab40', detail: '节点位于侧向障碍软膨胀带内, 可通行但代价升高。' },
    3: { label: '头顶净空不足', color: '#ff5252', detail: '胜出节点上方净空 < 机体高度 0.45m, 标记顶头禁行。' },
    4: { label: '侧向墙体阻挡', color: '#ff5252', detail: '节点足印硬半径 0.15m 内存在竖直墙体/台沿。' },
    5: { label: '其它禁行节点', color: '#e53935', detail: '节点被建图判定为禁行。' },
    6: { label: '拓扑缝 (禁止跨越)', color: '#ff00ff', detail: '相邻踏面格在流形图中无任何连通边 (错层/断坎/扫掠阻挡), 地毯在交界处画致命缝防止 2D 轨迹穿越。' },
    7: { label: '闭运算填充', color: '#ffeb3b', detail: '原为孤立致命补丁 (图节点缺失伪影), 已被形态学闭运算填充为可通行。' },
    8: { label: '透印隔离 (绕道图外到达)', color: '#5c6bc0', detail: '该格的自由来自图上联通、但在地毯上与机器人所在区域不连通的节点 —— 绕道图外到达的异层透印, 已按地毯连通性验证转为致命。' }
};

export class LocalCostmapVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.visible = true;
        this.debugMode = false;

        // 最近一次数据 (解码后)
        this.lastCostmap = null;   // {bytes, width, height, resolution, origin}
        this.lastDebug = null;     // 同结构, 值为成因码
        this.lastDebugNodes = null;// {width, height, ids: Int32Array} 胜出节点 id

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

        // 点击格子高亮框
        this.cellHighlight = null;
    }

    _decode(costmapMsg) {
        if (!costmapMsg || !costmapMsg.data) return null;
        const { width, height, resolution, origin, data: b64Data } = costmapMsg;
        const binaryStr = atob(b64Data);
        const bytes = new Uint8Array(binaryStr.length);
        for (let i = 0; i < binaryStr.length; ++i) {
            bytes[i] = binaryStr.charCodeAt(i);
        }
        return { bytes, width, height, resolution, origin };
    }

    /**
     * 更新局部代价地图数据并重绘
     * @param {Object} costmapMsg { width, height, resolution, origin: {x, y, z}, data: base64_str }
     */
    update(costmapMsg) {
        const decoded = this._decode(costmapMsg);
        if (!decoded) return;
        this.lastCostmap = decoded;
        if (!this.debugMode) this._render(decoded, false);
    }

    /** 更新逐格成因码调试图层 (仅调试模式下染色显示) */
    updateDebug(debugMsg) {
        const decoded = this._decode(debugMsg);
        if (!decoded) return;
        this.lastDebug = decoded;
        if (this.debugMode) this._render(decoded, true);
    }

    /**
     * 更新逐格胜出节点 id 调试图层
     * 新格式 [w, h, 表条数T, id×(w*h), (id, x_mm, y_mm, z_mm, trav_x100)×T]
     * 兼容旧格式 [w, h, id×(w*h)] (无坐标表)
     */
    updateDebugNodes(debugNodesMsg) {
        if (!debugNodesMsg || !debugNodesMsg.data) return;
        const binaryStr = atob(debugNodesMsg.data);
        const bytes = new Uint8Array(binaryStr.length);
        for (let i = 0; i < binaryStr.length; ++i) {
            bytes[i] = binaryStr.charCodeAt(i);
        }
        const d = new Int32Array(bytes.buffer, bytes.byteOffset, Math.floor(bytes.byteLength / 4));
        if (d.length < 2) return;
        const width = d[0], height = d[1];
        const idsCount = width * height;
        const oldFormat = d.length === 2 + idsCount;
        const tableCount = oldFormat ? 0 : d[2];
        const idsStart = oldFormat ? 2 : 3;
        const ids = d.slice(idsStart, idsStart + idsCount);
        const table = new Map();
        if (!oldFormat) {
            const tableStart = idsStart + idsCount;
            for (let i = 0; i < tableCount; ++i) {
                const base = tableStart + i * 5;
                table.set(d[base], {
                    x: d[base + 1] / 1000.0,
                    y: d[base + 2] / 1000.0,
                    z: d[base + 3] / 1000.0,
                    trav: d[base + 4] / 100.0
                });
            }
        }
        this.lastDebugNodes = { width, height, ids, table };
    }

    /** 按节点 id 查询胜出节点坐标表 (融合图/全局图模式均精确) */
    getWinnerNode(nodeId) {
        if (!this.lastDebugNodes || !this.lastDebugNodes.table) return null;
        return this.lastDebugNodes.table.get(nodeId) || null;
    }

    /** 切换调试染色模式 */
    setDebugMode(on) {
        if (this.debugMode === on) return;
        this.debugMode = on;
        if (on && this.lastDebug) {
            this._render(this.lastDebug, true);
        } else if (!on && this.lastCostmap) {
            this._render(this.lastCostmap, false);
        }
    }

    _render(grid, isDebug) {
        const { bytes, width, height, resolution, origin } = grid;
        const totalW = width * resolution;
        const totalH = height * resolution;

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
                let col;
                if (isDebug) {
                    col = this._debugColor(val);
                } else {
                    col = this._costColor(val);
                }
                rgba[canvasIdx] = col[0];
                rgba[canvasIdx + 1] = col[1];
                rgba[canvasIdx + 2] = col[2];
                rgba[canvasIdx + 3] = col[3];
            }
        }

        this.ctx.putImageData(imgData, 0, 0);

        // 创建或更新纹理
        // 固定 NearestFilter: LinearFilter 的格间插值会生成不属于任何真实格子的中间色
        // (如自由与致命之间的"半软"过渡), 掩盖格子级真相, 纯显示伪影 —— 不做平滑
        if (!this.texture) {
            this.texture = new THREE.CanvasTexture(this.canvas);
            this.texture.minFilter = THREE.NearestFilter;
            this.texture.magFilter = THREE.NearestFilter;
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

    _costColor(val) {
        if (val === 0) return [56, 189, 248, 55];            // 自由地面 (微透亮青绿)
        if (val > 0 && val < 100) return [251, 146, 60, Math.min(180, 80 + val)]; // 软膨胀橙黄
        if (val === 100) return [239, 68, 68, 190];          // 致命障碍 / 悬崖 (鲜红半透)
        return [15, 23, 42, 20];                              // 未知 / 窗口外 (深灰透明)
    }

    _debugColor(reason) {
        switch (reason) {
            case CELL_REASON.NO_NODE: return [168, 85, 247, 210];   // 紫: 无节点盖章 (默认致命)
            case CELL_REASON.FREE_NODE: return [56, 189, 248, 40];  // 微青: 自由
            case CELL_REASON.SOFT_NODE: return [251, 146, 60, 150]; // 橙: 软代价
            case CELL_REASON.BLOCK_HEADROOM: return [255, 87, 34, 210]; // 橙红: 净空不足
            case CELL_REASON.BLOCK_LATERAL: return [239, 68, 68, 220];  // 红: 侧向阻挡
            case CELL_REASON.BLOCK_OTHER: return [183, 28, 28, 220];    // 暗红: 其它禁行
            case CELL_REASON.SEAM: return [255, 0, 255, 230];       // 品红: 拓扑缝
            case CELL_REASON.CLOSING_FILLED: return [255, 235, 59, 110]; // 黄: 闭运算填充
            case CELL_REASON.UNREACHED: return [92, 107, 192, 220]; // 靛蓝: 透印隔离
            default: return [15, 23, 42, 20];
        }
    }

    /**
     * 世界坐标 -> 地毯格子诊断信息
     * @returns {Object|null} {r, c, cost, reason, resolution, origin}
     */
    getCellInfo(worldX, worldY) {
        const grid = this.debugMode ? (this.lastDebug || this.lastCostmap) : this.lastCostmap;
        if (!grid) return null;
        const { width, height, resolution, origin } = grid;
        const c = Math.floor((worldX - origin.x) / resolution);
        const r = Math.floor((worldY - origin.y) / resolution);
        if (r < 0 || r >= height || c < 0 || c >= width) return null;
        const idx = r * width + c;
        // 胜出节点 id: 仅当节点图层尺寸与当前网格一致时有效
        let nodeId = null;
        if (this.lastDebugNodes &&
            this.lastDebugNodes.width === width && this.lastDebugNodes.height === height) {
            nodeId = this.lastDebugNodes.ids[idx];
        }
        return {
            r, c,
            x: origin.x + (c + 0.5) * resolution,
            y: origin.y + (r + 0.5) * resolution,
            cost: this.lastCostmap ? this.lastCostmap.bytes[idx] : null,
            reason: this.lastDebug ? this.lastDebug.bytes[idx] : null,
            nodeId,
            resolution, origin
        };
    }

    /**
     * 高亮格子 (悬停/点击共用): 按地毯格实际尺寸 (5cm) 精确框选
     * 白色半透明填充 + 亮边框, 贴在地毯上方
     */
    highlightCell(worldX, worldY) {
        const grid = this.lastCostmap;
        if (!grid) return null;
        const info = this.getCellInfo(worldX, worldY);
        if (!info) {
            if (this.cellHighlight) this.cellHighlight.visible = false;
            return null;
        }

        if (!this.cellHighlight) {
            this.cellHighlight = new THREE.Group();
            const fillMat = new THREE.MeshBasicMaterial({
                color: 0xffffff, transparent: true, opacity: 0.30,
                depthWrite: false, side: THREE.DoubleSide
            });
            const fill = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), fillMat);
            fill.renderOrder = 2;
            const edge = new THREE.LineSegments(
                new THREE.EdgesGeometry(new THREE.PlaneGeometry(1, 1)),
                new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 2 })
            );
            edge.renderOrder = 3;
            this.cellHighlight.add(fill);
            this.cellHighlight.add(edge);
            this.scene.add(this.cellHighlight);
        }

        // 按地毯格实际尺寸与位置精确框选 (resolution = 0.05m)
        this.cellHighlight.scale.set(info.resolution, info.resolution, 1);
        this.cellHighlight.position.set(info.x, info.y, grid.origin.z + 0.035);
        this.cellHighlight.visible = true;
        return info;
    }

    clearHighlight() {
        if (this.cellHighlight) this.cellHighlight.visible = false;
    }

    setVisible(visible) {
        this.visible = visible;
        this.group.visible = visible;
    }
}
