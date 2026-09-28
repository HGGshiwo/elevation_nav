/**
 * 地图存储与 ROS 同步管理模块 (Map Storage & Sync)
 * 完整保留原版：本地地图加载(load_map)、持久化保存(save_map)、实时同步至 ROS 与脏状态标记
 */
export function initMapStorage(layers, onLayersReloaded) {
    const statusEl = document.getElementById('status');
    const mapPathInput = document.getElementById('map-path');
    const loadingIndicator = document.getElementById('loading-indicator');
    const loadingMsg = document.getElementById('loading-msg');

    let isDirty = false;
    let isPreblockedDirty = false;

    function getMapPathComponents() {
        const full = mapPathInput ? mapPathInput.value.trim() : '/home/robot/maps/map';
        const lastSlash = full.lastIndexOf('/');
        if (lastSlash !== -1) {
            return {
                rootPath: full.substring(0, lastSlash) || '/',
                mapName: full.substring(lastSlash + 1) || 'map'
            };
        }
        return { rootPath: '/home/robot/maps', mapName: full || 'map' };
    }

    function showLoading(msg) {
        if (loadingIndicator) {
            if (loadingMsg) loadingMsg.innerText = msg;
            loadingIndicator.style.display = 'block';
        }
    }

    function hideLoading() {
        if (loadingIndicator) loadingIndicator.style.display = 'none';
    }

    // 0. 导入 PCD 文件并指令 C++ 节点生成高程图
    const pcdPathInput = document.getElementById('pcd-path');
    async function loadPcd() {
        const pcdPath = pcdPathInput ? pcdPathInput.value.trim() : '';
        if (!pcdPath) {
            alert("请输入 PCD 文件绝对路径");
            return;
        }

        showLoading(`正在调用 C++ 节点解析 PCD 并提取高程图: ${pcdPath}...`);
        if (statusEl) statusEl.innerText = "正在导入 PCD 点云...";

        try {
            const resp = await fetch('/api/load_pcd', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ pcd_path: pcdPath })
            });

            if (!resp.ok) {
                const err = await resp.json();
                throw new Error(err.detail || "导入 PCD 失败");
            }

            const resData = await resp.json();
            if (statusEl) statusEl.innerText = resData.message || "PCD 载入指令已发送";
        } catch (e) {
            alert(`导入 PCD 失败: ${e.message}`);
            if (statusEl) statusEl.innerText = `导入 PCD 失败: ${e.message}`;
        } finally {
            setTimeout(hideLoading, 1500);
        }
    }

    // 1. 加载地图
    async function loadMap() {
        const { rootPath, mapName } = getMapPathComponents();

        showLoading(`正在加载地图包: ${mapName}...`);
        if (statusEl) statusEl.innerText = "正在加载地图...";

        try {
            const resp = await fetch('/api/load_map', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ root_path: rootPath, map_name: mapName })
            });

            if (!resp.ok) {
                const err = await resp.json();
                throw new Error(err.detail || "加载地图失败");
            }

            const data = await resp.json();

            // 更新已加载的图层
            if (data.layers) {
                Object.keys(data.layers).forEach(name => {
                    const lData = data.layers[name];
                    if (layers[name] && lData) {
                        layers[name].loadFromArray(lData.points, lData.scale, lData.intensity, lData.groups);
                    }
                });
            }

            if (onLayersReloaded) onLayersReloaded();

            isDirty = false;
            isPreblockedDirty = false;
            if (statusEl) {
                statusEl.innerText = `地图加载成功: ${mapName}`;
                statusEl.style.color = "#8bc34a";
            }
        } catch (e) {
            if (statusEl) {
                statusEl.innerText = `加载错误: ${e.message}`;
                statusEl.style.color = "#f44336";
            }
        } finally {
            hideLoading();
        }
    }

    // 2. 永久保存地图
    async function saveMap() {
        const { rootPath, mapName } = getMapPathComponents();

        showLoading(`正在保存地图: ${mapName}...`);
        if (statusEl) statusEl.innerText = "正在保存地图...";

        try {
            const payload = {
                root_path: rootPath,
                map_name: mapName,
                layers: {
                    occupied: {
                        points: layers.occupied ? layers.occupied.getArray() : [],
                        scale: layers.occupied ? layers.occupied.scale : [0.2, 0.2, 0.2]
                    },
                    preblocked: {
                        points: layers.preblocked ? layers.preblocked.getArray() : [],
                        scale: layers.preblocked ? layers.preblocked.scale : [0.2, 0.2, 0.2]
                    }
                }
            };

            const resp = await fetch('/api/save_map', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });

            if (!resp.ok) throw new Error("保存地图返回失败");

            isDirty = false;
            isPreblockedDirty = false;
            if (statusEl) {
                statusEl.innerText = "地图保存成功！";
                statusEl.style.color = "#8bc34a";
            }
        } catch (e) {
            if (statusEl) {
                statusEl.innerText = `保存失败: ${e.message}`;
                statusEl.style.color = "#f44336";
            }
        } finally {
            hideLoading();
        }
    }

    // 3. 实时同步至 ROS 节点
    async function syncToRos() {
        if (statusEl) statusEl.innerText = "正在同步图层至 ROS...";
        try {
            const payload = {
                layers: {
                    occupied: layers.occupied ? layers.occupied.getArray() : [],
                    preblocked: layers.preblocked ? layers.preblocked.getArray() : []
                }
            };
            const resp = await fetch('/api/sync_map', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });
            if (resp.ok) {
                isDirty = false;
                isPreblockedDirty = false;
                if (statusEl) statusEl.innerText = "同步完成！";
            }
        } catch (e) {
            if (statusEl) statusEl.innerText = `同步失败: ${e.message}`;
        }
    }

    // 绑定按钮事件
    document.getElementById('btn-load-pcd')?.addEventListener('click', loadPcd);
    document.getElementById('btn-load')?.addEventListener('click', loadMap);
    document.getElementById('btn-save')?.addEventListener('click', saveMap);
    document.getElementById('btn-sync')?.addEventListener('click', syncToRos);

    return {
        loadMap,
        saveMap,
        syncToRos,
        markDirty: (layerName) => {
            isDirty = true;
            if (layerName === 'preblocked') isPreblockedDirty = true;
            if (statusEl) statusEl.innerText = "已编辑 (未保存)";
        },
        isDirty: () => isDirty
    };
}
