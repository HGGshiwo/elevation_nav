/**
 * WebSocket 流式增量数据同步模块 (WebSocket Stream)
 * 完整保留原版：全双工通道、图层差量版本同步(unchanged 跳过)、状态主动分发与断线重连
 */
export function initWsStream(layers, robotTracker, getActiveRequestedLayers, graphVisualizer = null, corridorVisualizer = null, sfcDebugVisualizer = null, onSfcDebugUpdated = null, obstacleVisualizer = null, dynamicNodeVisualizer = null, reboundVisualizer = null) {
    let ws = null;
    let isConnecting = false;
    const clientLayerVersions = {};

    function connect() {
        if (isConnecting) return;
        isConnecting = true;

        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws/live`;

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            isConnecting = false;
            sendSubscription();
        };

        ws.onmessage = (event) => {
            try {
                const frame = JSON.parse(event.data);
                handleLiveFrame(frame);
            } catch (e) {
                console.error("[WsStream] 数据解析异常:", e);
            }
        };

        ws.onclose = () => {
            isConnecting = false;
            setTimeout(connect, 2000);
        };

        ws.onerror = () => {
            if (ws) ws.close();
        };
    }

    // 发送图层订阅配置与当前持有的版本戳
    function sendSubscription() {
        if (!ws || ws.readyState !== WebSocket.OPEN) return;
        const requested = getActiveRequestedLayers();
        ws.send(JSON.stringify({
            type: "subscribe",
            layers: requested,
            versions: clientLayerVersions
        }));
    }

    let currentGraphNodesVersion = -1;
    let currentGraphEdgesVersion = -1;
    let isFetchingStaticGraph = false;

    async function fetchStaticGraph() {
        if (isFetchingStaticGraph || !graphVisualizer) return;
        isFetchingStaticGraph = true;
        try {
            console.log("[WsStream] 正在通过 HTTP 一次性请求静态 3D 流形图 (/api/map/static_graph)...");
            const res = await fetch('/api/map/static_graph');
            if (!res.ok) throw new Error(`HTTP ${res.status}`);
            const data = await res.json();
            if (data.graph_nodes) {
                console.log(`[WsStream] 静态 3D 流形节点加载完成: ${data.graph_nodes.length} 个 (v=${data.graph_nodes_version})`);
                graphVisualizer.updateNodes(data.graph_nodes);
                currentGraphNodesVersion = data.graph_nodes_version;
            }
            if (data.graph_edges) {
                console.log(`[WsStream] 静态 3D 流形步态边加载完成: ${data.graph_edges.length} 条 (v=${data.graph_edges_version})`);
                graphVisualizer.updateEdges(data.graph_edges);
                currentGraphEdgesVersion = data.graph_edges_version;
            }
        } catch (e) {
            console.error("[WsStream] 请求静态 3D 流形图失败:", e);
        } finally {
            isFetchingStaticGraph = false;
        }
    }

    // 页面初次载入时一次性拉取静态图
    fetchStaticGraph();

    // 处理接收到的增量帧
    function handleLiveFrame(frame) {
        // 1. 机器人位姿与路径更新
        if (frame.robot_pose && robotTracker) {
            robotTracker.updatePose(frame.robot_pose);
        }
        if (robotTracker && robotTracker.updateCollisionNode) {
            robotTracker.updateCollisionNode(frame.collision_node);
        }
        if (frame.global_path && robotTracker) {
            robotTracker.updatePath(frame.global_path);
        }
        if (frame.local_path && robotTracker) {
            robotTracker.updateLocalPath(frame.local_path);
        }
        if (frame.local_astar_path && robotTracker) {
            robotTracker.updateLocalAStarPath(frame.local_astar_path);
        }

        // 2. 3D 流形拓扑图版本感知与按需更新 (仅版本号变更时通过 HTTP 重载 1 次)
        if (graphVisualizer) {
            const nv = frame.graph_nodes_version !== undefined ? frame.graph_nodes_version : -1;
            const ev = frame.graph_edges_version !== undefined ? frame.graph_edges_version : -1;
            if ((nv > 0 && nv !== currentGraphNodesVersion) || (ev > 0 && ev !== currentGraphEdgesVersion)) {
                fetchStaticGraph();
            }
        }

        // 3. 3D 拓扑流形管道更新 (后端 C++ 实时计算局部前瞻管道与真 3D 边界线框)
        if (corridorVisualizer) {
            if (frame.corridor_nodes || frame.corridor_lines) {
                corridorVisualizer.update(frame.corridor_nodes, frame.corridor_lines);
            }
        }

        // 3.1 局部可通行走廊单步扩散调试数据更新
        if (frame.sfc_corridors_debug) {
            if (sfcDebugVisualizer) {
                sfcDebugVisualizer.updateData(frame.sfc_corridors_debug);
            }
            if (onSfcDebugUpdated) {
                onSfcDebugUpdated(frame.sfc_corridors_debug);
            }
        }

        // 3.2 注入障碍物真值更新
        if (frame.injected_obstacles !== undefined && obstacleVisualizer) {
            obstacleVisualizer.update(frame.injected_obstacles);
        }

        // 3.3 融合引擎动态标记节点更新
        if (frame.dynamic_nodes !== undefined && dynamicNodeVisualizer) {
            dynamicNodeVisualizer.update(frame.dynamic_nodes);
        }

        // 3.4 rebound 定向排斥向量更新
        if (frame.rebound_arrows !== undefined && reboundVisualizer) {
            reboundVisualizer.update(frame.rebound_arrows);
        }

        // 4. 增量图层更新
        if (frame.layers) {
            Object.keys(frame.layers).forEach(layerName => {
                const lInfo = frame.layers[layerName];
                // 若服务端返回 unchanged，直接复用当前缓存
                if (lInfo.unchanged) return;

                const targetLayer = layers[layerName];
                if (targetLayer && lInfo) {
                    targetLayer.loadFromArray(lInfo.points, lInfo.scale, lInfo.intensity, lInfo.groups);
                    if (lInfo.version !== undefined) {
                        clientLayerVersions[layerName] = lInfo.version;
                    }
                }
            });
        }
    }

    connect();

    return {
        sendSubscription,
        reconnect: connect
    };
}
