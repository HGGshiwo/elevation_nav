/**
 * WebSocket 全双工长连接客户端
 * 负责实时接收 ROS 底盘位姿与路径更新并分发
 */
export class WsClient {
    constructor(onPoseUpdate, onPathUpdate, onStatusChange) {
        this.onPoseUpdate = onPoseUpdate;
        this.onPathUpdate = onPathUpdate;
        this.onStatusChange = onStatusChange;

        this.ws = null;
        this.isConnecting = false;
        this.connect();
    }

    connect() {
        if (this.isConnecting) return;
        this.isConnecting = true;

        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws/live`;

        if (this.onStatusChange) this.onStatusChange('connecting');

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            this.isConnecting = false;
            if (this.onStatusChange) this.onStatusChange('connected');
        };

        this.ws.onmessage = (event) => {
            try {
                const frame = JSON.parse(event.data);
                if (frame.robot_pose && this.onPoseUpdate) {
                    this.onPoseUpdate(frame.robot_pose);
                }
                if (this.onPathUpdate) {
                    this.onPathUpdate(frame.global_path, frame.local_path);
                }
            } catch (e) {
                console.error("[WsClient] 解析数据帧异常:", e);
            }
        };

        this.ws.onclose = () => {
            this.isConnecting = false;
            if (this.onStatusChange) this.onStatusChange('disconnected');
            // 2秒后重连
            setTimeout(() => this.connect(), 2000);
        };

        this.ws.onerror = () => {
            this.ws.close();
        };
    }
}
