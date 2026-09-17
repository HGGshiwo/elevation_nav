import * as THREE from 'three';

/**
 * 1:1 流形高程局部代价地图 3D 可视化器 (Local Costmap Visualizer)
 * 渲染实时 6.0m x 6.0m 贴地流形代价地毯与规划窗口外框
 * 支持调试模式: 按逐格成因码染色, 并支持点击查询格子物理高程、代价值与障碍来源诊断信息
 * 同时兼顾供给 TEB 的 3D 结构化几何障碍物显示
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
    3: { label: '头顶净空不足', color: '#ff5722', detail: '胜出节点上方净空 < 机体高度 0.45m, 标记顶头禁行。' },
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
        this.obstaclesVisible = false;

        // 最近一次数据 (解码后)
        this.lastCostmap = null;   // {bytes, width, height, resolution, origin}
        this.lastDebug = null;     // 同结构, 值为成因码
        this.lastDebugNodes = null;// {width, height, ids: Int32Array, table: Map} 胜出节点 id 与坐标表
        this.lastTebObstacles = null;
        this.gridHeights = null;   // Float32Array 实时 3D 逐格物理高程 (width * height)

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

        // TEB 结构化 3D 几何障碍物组
        this.obstaclesGroup = new THREE.Group();
        this.group.add(this.obstaclesGroup);
        this.obstaclesGroup.visible = false;
        this.obstacleMeshMap = new Map();
        this.selectedObstacleId = null;
        this.hoveredObstacleId = null;

        // ---- 障碍物材质 ----
        this.bodyMat = new THREE.MeshLambertMaterial({
            color: 0xef4444, transparent: true, opacity: 0.60, depthWrite: true, side: THREE.DoubleSide
        });
        this.edgeMat = new THREE.LineBasicMaterial({
            color: 0x3f0a0a, linewidth: 2, transparent: true, opacity: 0.95
        });
        this.groundStripMat = new THREE.MeshBasicMaterial({
            color: 0x991b1b, transparent: true, opacity: 0.75, depthWrite: false, side: THREE.DoubleSide
        });
        this.selectedBodyMat = new THREE.MeshLambertMaterial({
            color: 0xff3b30, emissive: 0x330000, transparent: true, opacity: 0.90, depthWrite: true, side: THREE.DoubleSide
        });
        this.selectedEdgeMat = new THREE.LineBasicMaterial({
            color: 0xffffff, linewidth: 3, transparent: true, opacity: 1.0
        });
        this.selectedGroundStripMat = new THREE.MeshBasicMaterial({
            color: 0xffffff, transparent: true, opacity: 0.85, depthWrite: false, side: THREE.DoubleSide
        });
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
     * 格式 [w, h, 表条数T, id×(w*h), (id, x_mm, y_mm, z_mm, trav_x100)×T]
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

        // 若当前已有代价地图，立即驱动顶点物理高程更新形成真实 3D 地形起伏
        const activeGrid = this.debugMode ? (this.lastDebug || this.lastCostmap) : this.lastCostmap;
        if (activeGrid && this.planeMesh) {
            this._updateGeometryHeights(activeGrid);
        }
    }

    /** 按节点 id 查询胜出节点坐标与通行度表 */
    getWinnerNode(nodeId) {
        if (!this.lastDebugNodes || !this.lastDebugNodes.table) return null;
        return this.lastDebugNodes.table.get(nodeId) || null;
    }

    /** 切换成因码调试染色模式 */
    setDebugMode(on) {
        if (this.debugMode === on) return;
        this.debugMode = on;
        if (on && this.lastDebug) {
            this._render(this.lastDebug, true);
        } else if (!on && this.lastCostmap) {
            this._render(this.lastCostmap, false);
        }
    }

    _costColor(val) {
        if (val === 0) return [56, 189, 248, 55];            // 自由地面 (微透亮青蓝)
        if (val > 0 && val < 100) return [251, 146, 60, Math.min(180, 80 + val)]; // 软膨胀橙黄
        if (val === 100) return [239, 68, 68, 195];          // 致命障碍 / 悬崖 (鲜红半透)
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
     * 核心 3D 地形曲面重建: 将胜出节点的物理真实高程 z 赋给每个网格顶点,
     * 并做平滑边界邻域插值和法向量重算, 使代价地图成为真正的 3D 贴地流形网格
     */
    _updateGeometryHeights(grid) {
        if (!this.planeMesh || !grid) return;
        const { width, height, resolution, origin } = grid;
        const totalW = width * resolution;
        const totalH = height * resolution;
        const totalCells = width * height;
        const baseZ = (origin && origin.z !== undefined) ? origin.z : 0.0;

        if (!this.gridHeights || this.gridHeights.length !== totalCells) {
            this.gridHeights = new Float32Array(totalCells);
        }
        const heights = this.gridHeights;

        if (this.lastDebugNodes &&
            this.lastDebugNodes.width === width &&
            this.lastDebugNodes.height === height) {
            const ids = this.lastDebugNodes.ids;
            const table = this.lastDebugNodes.table;
            const reasons = this.lastDebug ? this.lastDebug.bytes : null;
            for (let i = 0; i < totalCells; ++i) {
                const nid = ids[i];
                // 若为未联通障碍 (如透印隔离 UNREACHED、无节点 NO_NODE 或无胜出节点),
                // 高度严格与狗当前的 z (baseZ) 保持一致，避免虚假高峰或凹坑
                if (nid < 0 || !table || (reasons && (reasons[i] === CELL_REASON.UNREACHED || reasons[i] === CELL_REASON.NO_NODE))) {
                    heights[i] = baseZ;
                    continue;
                }
                const nd = table.get(nid);
                if (nd && Number.isFinite(nd.z)) {
                    heights[i] = nd.z;
                } else {
                    heights[i] = baseZ;
                }
            }
        } else {
            heights.fill(baseZ);
        }

        // 更新 PlaneGeometry 顶点的高程 (z 轴偏移)
        // Three.js PlaneGeometry:
        // iy = 0 对应顶端 (y = +totalH/2, 对应 ROS r = height - 1)
        // iy = height - 1 对应底端 (y = -totalH/2, 对应 ROS r = 0)
        // ix 对应列 c (x = -totalW/2 .. +totalW/2, 对应 ROS c = 0 .. width - 1)
        const posAttr = this.planeMesh.geometry.attributes.position;
        const posArr = posAttr.array;
        for (let iy = 0; iy < height; ++iy) {
            const r = height - 1 - iy;
            const rOffset = r * width;
            const iyOffset = iy * width;
            for (let ix = 0; ix < width; ++ix) {
                const rosIdx = rOffset + ix;
                const vIdx = (iyOffset + ix) * 3;
                const worldZ = heights[rosIdx];
                // 相对于 mesh.position.z 偏移，贴地微抬 1.5cm 避免与底层点云 z-fighting
                posArr[vIdx + 2] = worldZ - baseZ + 0.015;
            }
        }
        posAttr.needsUpdate = true;
        this.planeMesh.geometry.computeVertexNormals();
        this.planeMesh.geometry.computeBoundingSphere();
        this.planeMesh.geometry.computeBoundingBox();

        // 构造贴合 3D 起伏地形的边界线框 (沿外围四周真实高程采样)
        const halfW = totalW * 0.5;
        const halfH = totalH * 0.5;
        const segW = totalW / (width - 1);
        const segH = totalH / (height - 1);
        const boxPts = [];

        // 1. 底边 (r = 0, c: 0 -> width - 1)
        for (let c = 0; c < width; ++c) {
            boxPts.push(new THREE.Vector3(c * segW - halfW, -halfH, heights[0 * width + c] - baseZ + 0.02));
        }
        // 2. 右边 (c = width - 1, r: 1 -> height - 1)
        for (let r = 1; r < height; ++r) {
            boxPts.push(new THREE.Vector3(halfW, r * segH - halfH, heights[r * width + (width - 1)] - baseZ + 0.02));
        }
        // 3. 顶边 (r = height - 1, c: width - 2 -> 0)
        for (let c = width - 2; c >= 0; --c) {
            boxPts.push(new THREE.Vector3(c * segW - halfW, halfH, heights[(height - 1) * width + c] - baseZ + 0.02));
        }
        // 4. 左边 (c = 0, r: height - 2 -> 1)
        for (let r = height - 2; r >= 1; --r) {
            boxPts.push(new THREE.Vector3(-halfW, r * segH - halfH, heights[r * width + 0] - baseZ + 0.02));
        }

        if (!this.wireframeBox) {
            const boxGeo = new THREE.BufferGeometry().setFromPoints(boxPts);
            const boxMat = new THREE.LineBasicMaterial({
                color: 0x38bdf8,
                linewidth: 2,
                transparent: true,
                opacity: 0.8
            });
            this.wireframeBox = new THREE.LineLoop(boxGeo, boxMat);
            this.group.add(this.wireframeBox);
        } else {
            this.wireframeBox.geometry.dispose();
            this.wireframeBox.geometry = new THREE.BufferGeometry().setFromPoints(boxPts);
        }
        const centerX = origin.x + totalW * 0.5;
        const centerY = origin.y + totalH * 0.5;
        this.wireframeBox.position.set(centerX, centerY, baseZ);
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
                let col = isDebug ? this._debugColor(val) : this._costColor(val);
                rgba[canvasIdx] = col[0];
                rgba[canvasIdx + 1] = col[1];
                rgba[canvasIdx + 2] = col[2];
                rgba[canvasIdx + 3] = col[3];
            }
        }

        this.ctx.putImageData(imgData, 0, 0);

        if (!this.texture) {
            this.texture = new THREE.CanvasTexture(this.canvas);
            this.texture.minFilter = THREE.NearestFilter;
            this.texture.magFilter = THREE.NearestFilter;
        } else {
            this.texture.needsUpdate = true;
        }

        const widthSegments = width - 1;
        const heightSegments = height - 1;

        // 创建或更新地毯 3D 细分曲面 Mesh (节点分辨率 width x height)
        if (!this.planeMesh) {
            const planeGeo = new THREE.PlaneGeometry(totalW, totalH, widthSegments, heightSegments);
            const planeMat = new THREE.MeshBasicMaterial({
                map: this.texture,
                transparent: true,
                opacity: 0.85,
                depthWrite: false,
                side: THREE.DoubleSide
            });
            this.planeMesh = new THREE.Mesh(planeGeo, planeMat);
            this.planeMesh.renderOrder = 5;
            this.group.add(this.planeMesh);
        } else {
            if (this.planeMesh.geometry.parameters.width !== totalW ||
                this.planeMesh.geometry.parameters.height !== totalH ||
                this.planeMesh.geometry.parameters.widthSegments !== widthSegments ||
                this.planeMesh.geometry.parameters.heightSegments !== heightSegments) {
                this.planeMesh.geometry.dispose();
                this.planeMesh.geometry = new THREE.PlaneGeometry(totalW, totalH, widthSegments, heightSegments);
            }
        }

        // 更新地毯位置 (XY 中心对齐，Z 以 origin.z 为基底，具体 3D 起伏由各顶点 Z 缓冲表达)
        const centerX = origin.x + totalW * 0.5;
        const centerY = origin.y + totalH * 0.5;
        const baseZ = (origin.z !== undefined) ? origin.z : 0.0;
        this.planeMesh.position.set(centerX, centerY, baseZ);

        // 重算顶点 3D 高程与外框贴地起伏
        this._updateGeometryHeights(grid);

        this.planeMesh.visible = this.visible;
        if (this.wireframeBox) this.wireframeBox.visible = this.visible;
    }

    /**
     * 世界坐标 -> 地毯格子诊断信息 (包含物理高程 Z 与胜出节点)
     * @returns {Object|null} {r, c, x, y, z, cost, reason, nodeId, winnerNode, resolution, origin}
     */
    getCellInfo(worldX, worldY) {
        const grid = this.debugMode ? (this.lastDebug || this.lastCostmap) : this.lastCostmap;
        if (!grid) return null;
        const { width, height, resolution, origin } = grid;
        const c = Math.floor((worldX - origin.x) / resolution);
        const r = Math.floor((worldY - origin.y) / resolution);
        if (r < 0 || r >= height || c < 0 || c >= width) return null;
        const idx = r * width + c;

        let nodeId = null;
        let winnerNode = null;
        if (this.lastDebugNodes &&
            this.lastDebugNodes.width === width && this.lastDebugNodes.height === height) {
            nodeId = this.lastDebugNodes.ids[idx];
            if (nodeId !== null && nodeId >= 0) {
                winnerNode = this.getWinnerNode(nodeId);
            }
        }

        const z = (this.gridHeights && Number.isFinite(this.gridHeights[idx]))
            ? this.gridHeights[idx]
            : (winnerNode ? winnerNode.z : origin.z);

        return {
            r, c,
            x: origin.x + (c + 0.5) * resolution,
            y: origin.y + (r + 0.5) * resolution,
            z: z,
            cost: this.lastCostmap ? this.lastCostmap.bytes[idx] : null,
            reason: this.lastDebug ? this.lastDebug.bytes[idx] : null,
            nodeId,
            winnerNode,
            resolution, origin
        };
    }

    /**
     * 高精度射线拾取局部代价地图
     * 1. 优先与 3D 高程曲面三角形做精确碰撞
     * 2. 若在边缘或由于视角倾斜漏检，使用局部代价地图足印水平面投影做保底
     * @param {THREE.Raycaster} raycaster
     * @returns {THREE.Vector3|null}
     */
    raycastPoint(raycaster) {
        if (!this.visible || !this.planeMesh) return null;
        const grid = this.debugMode ? (this.lastDebug || this.lastCostmap) : (this.lastCostmap || this.lastDebug);
        if (!grid) return null;

        // 确保世界变换矩阵和包围球是最新的
        this.planeMesh.updateMatrixWorld(true);
        if (!this.planeMesh.geometry.boundingSphere) {
            this.planeMesh.geometry.computeBoundingSphere();
        }

        const hits = raycaster.intersectObject(this.planeMesh, false);
        if (hits.length > 0) {
            return hits[0].point;
        }

        // 备用保底：数学射线与代价地图水平面的交点
        const { width, height, resolution, origin } = grid;
        const totalW = width * resolution;
        const totalH = height * resolution;
        const baseZ = (origin && origin.z !== undefined) ? origin.z : 0.0;

        const ray = raycaster.ray;
        if (Math.abs(ray.direction.z) > 1e-4) {
            const t = (baseZ - ray.origin.z) / ray.direction.z;
            if (t > 0) {
                const pt = new THREE.Vector3().copy(ray.origin).addScaledVector(ray.direction, t);
                if (pt.x >= origin.x && pt.x <= origin.x + totalW &&
                    pt.y >= origin.y && pt.y <= origin.y + totalH) {
                    return pt;
                }
            }
        }
        return null;
    }

    /**
     * 高亮格子 (悬停/点击共用): 按地毯格实际尺寸 (5cm) 精确框选
     * 采用无遮挡 (depthTest: false) + 高渲染层级 (renderOrder: 999) 的 3D 方块框选，
     * 彻底解决在斜坡/台阶上因深度测试被曲面或点云埋入不见的问题
     */
    highlightCell(worldX, worldY) {
        const grid = this.debugMode ? (this.lastDebug || this.lastCostmap) : (this.lastCostmap || this.lastDebug);
        if (!grid) return null;
        const info = this.getCellInfo(worldX, worldY);
        if (!info) {
            if (this.cellHighlight) this.cellHighlight.visible = false;
            return null;
        }

        if (!this.cellHighlight) {
            this.cellHighlight = new THREE.Group();
            const boxGeo = new THREE.BoxGeometry(1, 1, 1);
            const fillMat = new THREE.MeshBasicMaterial({
                color: 0x00e5ff,
                transparent: true,
                opacity: 0.45,
                depthTest: false,
                depthWrite: false,
                side: THREE.DoubleSide
            });
            const fill = new THREE.Mesh(boxGeo, fillMat);
            fill.renderOrder = 998;

            const edgeGeo = new THREE.EdgesGeometry(boxGeo);
            const edgeMat = new THREE.LineBasicMaterial({
                color: 0xffffff,
                linewidth: 2,
                depthTest: false,
                depthWrite: false
            });
            const edge = new THREE.LineSegments(edgeGeo, edgeMat);
            edge.renderOrder = 999;

            this.cellHighlight.add(fill);
            this.cellHighlight.add(edge);
            this.scene.add(this.cellHighlight);
        }

        const boxH = 0.04; // 4cm 厚度 3D 悬浮高亮体
        this.cellHighlight.scale.set(info.resolution, info.resolution, boxH);
        const zCenter = (info.z !== undefined && info.z !== null ? info.z : grid.origin.z) + boxH * 0.5 + 0.015;
        this.cellHighlight.position.set(info.x, info.y, zCenter);
        this.cellHighlight.visible = true;
        return info;
    }

    clearHighlight() {
        if (this.cellHighlight) this.cellHighlight.visible = false;
    }

    setVisible(visible) {
        this.visible = visible;
        if (this.planeMesh) this.planeMesh.visible = visible;
        if (this.wireframeBox) this.wireframeBox.visible = visible;
        if (this.cellHighlight && !visible) this.cellHighlight.visible = false;
    }

    setObstaclesVisible(visible) {
        this.obstaclesVisible = visible;
        this.obstaclesGroup.visible = visible;
    }

    clearObstacles() {
        this.obstacleMeshMap.clear();
        while (this.obstaclesGroup.children.length > 0) {
            const child = this.obstaclesGroup.children[0];
            this.obstaclesGroup.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
    }

    updateTebObstacles(obstacles) {
        this.lastTebObstacles = obstacles;
        this.clearObstacles();
        if (!obstacles || obstacles.length === 0) return;

        const wallH = 0.40;
        const wallW = 0.14;

        for (const obs of obstacles) {
            if (!obs) continue;
            const obsId = obs.id !== undefined ? obs.id : `${obs.type}_${Math.random()}`;
            if (obs.type === 'line' && obs.start && obs.end) {
                const [x1, y1, z1] = obs.start;
                const [x2, y2, z2] = obs.end;
                const z = (z1 + z2) * 0.5 + 0.005;
                const dx = x2 - x1, dy = y2 - y1;
                const len = Math.hypot(dx, dy);
                if (len < 1e-4) continue;

                const boxGeo = new THREE.BoxGeometry(len, wallW, wallH);
                const bodyMesh = new THREE.Mesh(boxGeo, this.bodyMat);
                bodyMesh.position.set((x1 + x2) * 0.5, (y1 + y2) * 0.5, z + wallH * 0.5);
                bodyMesh.rotation.z = Math.atan2(dy, dx);
                this.obstaclesGroup.add(bodyMesh);

                const edgesGeo = new THREE.EdgesGeometry(boxGeo);
                const edgeMesh = new THREE.LineSegments(edgesGeo, this.edgeMat);
                edgeMesh.position.copy(bodyMesh.position);
                edgeMesh.rotation.copy(bodyMesh.rotation);
                this.obstaclesGroup.add(edgeMesh);
            } else if (obs.type === 'circle') {
                const { x, y, z: rawZ, radius: r } = obs;
                const z = (rawZ !== undefined ? rawZ : 0.0) + 0.005;
                const radius = Math.max(0.12, r || 0.16);

                const cylGeo = new THREE.CylinderGeometry(radius, radius, wallH, 24, 1, false);
                const bodyMesh = new THREE.Mesh(cylGeo, this.bodyMat);
                bodyMesh.rotation.x = Math.PI * 0.5;
                bodyMesh.position.set(x, y, z + wallH * 0.5);
                this.obstaclesGroup.add(bodyMesh);

                const edgesGeo = new THREE.EdgesGeometry(cylGeo, 25);
                const edgeMesh = new THREE.LineSegments(edgesGeo, this.edgeMat);
                edgeMesh.position.copy(bodyMesh.position);
                edgeMesh.rotation.copy(bodyMesh.rotation);
                this.obstaclesGroup.add(edgeMesh);
            }
        }
        this.obstaclesGroup.visible = this.obstaclesVisible;
    }
}
