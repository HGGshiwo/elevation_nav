/**
 * WebSocket 流式增量数据同步模块 (WebSocket Stream)
 * 完整保留原版：全双工通道、图层差量版本同步(unchanged 跳过)、状态主动分发与断线重连
 */
export function initWsStream(layers, robotTracker, getActiveRequestedLayers, graphVisualizer = null, corridorVisualizer = null) {
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

    // 处理接收到的增量帧
    function handleLiveFrame(frame) {
        // 1. 机器人位姿与路径更新
        if (frame.robot_pose && robotTracker) {
            robotTracker.updatePose(frame.robot_pose);
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

        // 2. 3D 流形拓扑图更新 (踏面节点与连通边)
        if (graphVisualizer) {
            if (frame.graph_nodes) {
                console.log(`[WsStream] 收到 3D 流形节点: ${frame.graph_nodes.length} 个`);
                graphVisualizer.updateNodes(frame.graph_nodes);
            }
            if (frame.graph_edges) {
                console.log(`[WsStream] 收到 3D 流形步态边: ${frame.graph_edges.length} 条`);
                graphVisualizer.updateEdges(frame.graph_edges);
            }
        }

        // 3. 3D 拓扑流形管道更新 (后端 C++ 实时计算局部前瞻管道与真 3D 边界)
        if (corridorVisualizer) {
            if (frame.corridor_nodes) {
                corridorVisualizer.update(frame.corridor_nodes, frame.corridor_lines, frame.corridor_walls);
            }
            if (frame.teb_obstacles !== undefined) {
                corridorVisualizer.updateTebObstacles(frame.teb_obstacles);
            }
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
