import * as THREE from 'three';

const MAX_INSTANCES = 500000;

// Create a white texture with a dark border. 
// When multiplied with vertex/instance colors, it creates natural borders on voxel faces.
function createBorderTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 64;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    
    // Fill background with white
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, 64, 64);
    
    // Draw border with dark gray (so there's a strong outline)
    ctx.strokeStyle = '#222222';
    ctx.lineWidth = 6;
    ctx.strokeRect(0, 0, 64, 64);
    
    const texture = new THREE.CanvasTexture(canvas);
    return texture;
}

const borderTexture = createBorderTexture();

/**
 * Maps risk cost values to HSL colors to create a green-to-red gradient heatmap.
 * @param {number} val The risk intensity value
 * @param {number} minVal Minimum cost value
 * @param {number} maxVal Maximum cost value
 * @returns {THREE.Color}
 */
function getRiskColor(val, minVal, maxVal) {
    const color = new THREE.Color();
    let ratio = 0.0;
    if (maxVal > minVal) {
        ratio = (val - minVal) / (maxVal - minVal);
    }
    ratio = Math.max(0, Math.min(1, ratio));
    
    // Green (low cost) -> Yellow (medium cost) -> Red (high cost)
    // HSL Hue: 120 (green) to 0 (red)
    const h = (120 - ratio * 120) / 360;
    const s = 0.95;
    const l = 0.45;
    color.setHSL(h, s, l);
    return color;
}

/**
 * Maps Z coordinates to HSL colors to create a smooth height gradient.
 * @param {string} layerName The name of the layer ('occupied', 'preblocked', etc.)
 * @param {number} z Current voxel Z height
 * @param {number} minZ Min Z of the layer
 * @param {number} maxZ Max Z of the layer
 * @returns {THREE.Color}
 */
function getVoxelColor(layerName, z, minZ, maxZ) {
    const color = new THREE.Color();
    let ratio = 0.5;
    if (maxZ > minZ) {
        ratio = (z - minZ) / (maxZ - minZ);
    }
    
    // Clamp ratio between 0 and 1
    ratio = Math.max(0, Math.min(1, ratio));
    
    if (layerName === 'occupied') {
        // Orange to Yellow gradient
        // Hue: from 15 (warm reddish orange at bottom) to 55 (bright yellow at top)
        // Lightness: from 0.35 (darker) to 0.7 (brighter)
        const h = (15 + ratio * 40) / 360;
        const s = 0.9;
        const l = 0.35 + ratio * 0.35;
        color.setHSL(h, s, l);
    } else if (layerName === 'preblocked') {
        // Red to Light Pinkish-Red gradient
        // Hue: from -15 (crimson/magenta-red) to 15 (orange-red)
        let h = (-15 + ratio * 30);
        if (h < 0) h += 360;
        h = h / 360;
        const s = 1.0;
        const l = 0.3 + ratio * 0.35;
        color.setHSL(h, s, l);
    } else if (layerName === 'local_octomap') {
        // Cyan / Sky Blue gradient
        const h = (190 + ratio * 30) / 360;
        const s = 0.85;
        const l = 0.4 + ratio * 0.3;
        color.setHSL(h, s, l);
    } else if (layerName === 'fused_octomap') {
        // Purple / Magenta gradient
        const h = (270 + ratio * 30) / 360;
        const s = 0.85;
        const l = 0.4 + ratio * 0.3;
        color.setHSL(h, s, l);
    } else if (layerName === 'emergency_stop_free') {
        color.setHex(0x00ff44);
    } else if (layerName === 'emergency_stop_occupied') {
        color.setHex(0xff0033);
    } else { // traversable (green gradient)
        const h = (100 + ratio * 40) / 360;
        const s = 0.8;
        const l = 0.3 + ratio * 0.3;
        color.setHSL(h, s, l);
    }
    return color;
}

/**
 * Manages the rendering and manipulation of voxels within a single layer using InstancedMesh.
 */
export class LayerManager {
    constructor(scene, layerName, scale) {
        this.scene = scene;
        this.layerName = layerName;
        this.scale = scale; // [x, y, z]
        this.voxelMap = new Map(); // key: "x,y,z", value: {x,y,z}
        this.voxelList = []; // flat array for O(1) query by instanceId
        
        // Use unit BoxGeometry (0.98 scale for clean cell border outline)
        const geometry = new THREE.BoxGeometry(0.98, 0.98, 0.98);
        
        // Expand bounding volume to prevent early frustum culling during raycast
        geometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 99999);
        geometry.boundingBox = new THREE.Box3(new THREE.Vector3(-99999, -99999, -99999), new THREE.Vector3(99999, 99999, 99999));
        
        // MeshStandardMaterial reacts to light; borderTexture provides outlines
        const material = new THREE.MeshStandardMaterial({ 
            map: borderTexture,
            roughness: 0.5,
            metalness: 0.1
        });
        
        this.mesh = new THREE.InstancedMesh(geometry, material, MAX_INSTANCES);
        this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        
        // Enable per-instance colors (3 floats per instance)
        this.mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(MAX_INSTANCES * 3), 3);
        this.mesh.count = 0;
        
        this.mesh.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 99999);
        this.mesh.boundingBox = new THREE.Box3(new THREE.Vector3(-99999, -99999, -99999), new THREE.Vector3(99999, 99999, 99999));
        
        this.scene.add(this.mesh);
        this.dummy = new THREE.Object3D();
    }

    hash(x, y, z) {
        return `${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}`;
    }

    addVoxel(x, y, z) {
        const key = this.hash(x, y, z);
        if (!this.voxelMap.has(key) && this.voxelMap.size < MAX_INSTANCES) {
            this.voxelMap.set(key, {x, y, z, scale: this.scale});
            this.updateInstancedMesh();
            return true;
        }
        return false;
    }

    removeVoxel(x, y, z) {
        const key = this.hash(x, y, z);
        if (this.voxelMap.has(key)) {
            this.voxelMap.delete(key);
            this.updateInstancedMesh();
            return true;
        }
        return false;
    }

    updateInstancedMesh() {
        let i = 0;
        this.voxelList = [];
        
        // Find min and max Z coordinates for gradient mapping, and intensity bounds
        let minZ = Infinity;
        let maxZ = -Infinity;
        let minVal = Infinity;
        let maxVal = -Infinity;
        this.voxelMap.forEach((pos) => {
            if (pos.z < minZ) minZ = pos.z;
            if (pos.z > maxZ) maxZ = pos.z;
            if (pos.intensity !== undefined) {
                if (pos.intensity < minVal) minVal = pos.intensity;
                if (pos.intensity > maxVal) maxVal = pos.intensity;
            }
        });

        this.voxelMap.forEach((pos) => {
            this.voxelList.push(pos);
            if (i < MAX_INSTANCES) {
                this.dummy.position.set(pos.x, pos.y, pos.z);
                const s = pos.scale || this.scale;
                this.dummy.scale.set(s[0], s[1], s[2]);
                this.dummy.updateMatrix();
                this.mesh.setMatrixAt(i, this.dummy.matrix);
                
                // Dynamic height or cost-based coloring
                let color;
                if (this.layerName === 'risk_cost') {
                    color = getRiskColor(pos.intensity, minVal, maxVal);
                } else {
                    color = getVoxelColor(this.layerName, pos.z, minZ, maxZ);
                }
                this.mesh.setColorAt(i, color);
                
                i++;
            }
        });
        
        this.mesh.count = Math.min(this.voxelMap.size, MAX_INSTANCES);
        this.mesh.instanceMatrix.needsUpdate = true;
        if (this.mesh.instanceColor) {
            this.mesh.instanceColor.needsUpdate = true;
        }
    }

    /**
     * 高性能极速渲染方法：
     * 专为动态高频图层（local_octomap, fused_octomap, emergency_stop 等）设计。
     * 直接在 Float32Array 缓冲区就地写入变换矩阵与颜色，完全绕过 toFixed(3) 字符串哈希和 Map 查找，耗时降低 95%+。
     */
    loadFast(groups, scale) {
        if (scale) this.scale = scale;
        if (!groups || groups.length === 0) {
            this.mesh.count = 0;
            this.mesh.instanceMatrix.needsUpdate = true;
            return;
        }

        const matrixArray = this.mesh.instanceMatrix.array;
        const colorArray = this.mesh.instanceColor ? this.mesh.instanceColor.array : null;

        let minZ = Infinity, maxZ = -Infinity;
        for (let g = 0; g < groups.length; g++) {
            const pts = groups[g].points || [];
            for (let i = 0; i < pts.length; i++) {
                const z = pts[i][2];
                if (z < minZ) minZ = z;
                if (z > maxZ) maxZ = z;
            }
        }
        if (minZ === Infinity) {
            this.mesh.count = 0;
            this.mesh.instanceMatrix.needsUpdate = true;
            return;
        }

        let idx = 0;
        const maxPts = MAX_INSTANCES;
        for (let g = 0; g < groups.length; g++) {
            const grp = groups[g];
            const s = grp.scale || this.scale || [0.1, 0.1, 0.1];
            const sx = s[0], sy = s[1], sz = s[2];
            const pts = grp.points || [];

            for (let i = 0; i < pts.length; i++) {
                if (idx >= maxPts) break;
                const p = pts[i];
                const px = p[0], py = p[1], pz = p[2];
                const offset = idx * 16;

                matrixArray[offset + 0] = sx;
                matrixArray[offset + 1] = 0;
                matrixArray[offset + 2] = 0;
                matrixArray[offset + 3] = 0;

                matrixArray[offset + 4] = 0;
                matrixArray[offset + 5] = sy;
                matrixArray[offset + 6] = 0;
                matrixArray[offset + 7] = 0;

                matrixArray[offset + 8] = 0;
                matrixArray[offset + 9] = 0;
                matrixArray[offset + 10] = sz;
                matrixArray[offset + 11] = 0;

                matrixArray[offset + 12] = px;
                matrixArray[offset + 13] = py;
                matrixArray[offset + 14] = pz;
                matrixArray[offset + 15] = 1;

                if (colorArray) {
                    const c = getVoxelColor(this.layerName, pz, minZ, maxZ);
                    const cOffset = idx * 3;
                    colorArray[cOffset + 0] = c.r;
                    colorArray[cOffset + 1] = c.g;
                    colorArray[cOffset + 2] = c.b;
                }

                idx++;
            }
            if (idx >= maxPts) break;
        }

        this.mesh.count = idx;
        this.mesh.instanceMatrix.needsUpdate = true;
        if (this.mesh.instanceColor) {
            this.mesh.instanceColor.needsUpdate = true;
        }
    }

    loadFromArray(points, scale, intensities, groups) {
        if (scale) {
            this.scale = scale;
        }

        // 对于只读动态图层，且有 groups 数据，直接走极速通道
        if (this.layerName !== 'occupied' && this.layerName !== 'preblocked' && groups && groups.length > 0) {
            this.loadFast(groups, scale);
            return;
        }

        this.voxelMap.clear();
        if (groups && groups.length > 0) {
            groups.forEach(group => {
                const grpScale = group.scale || this.scale || [0.1, 0.1, 0.1];
                const pts = group.points || [];
                pts.forEach(pt => {
                    this.voxelMap.set(this.hash(pt[0], pt[1], pt[2]), {
                        x: pt[0],
                        y: pt[1],
                        z: pt[2],
                        scale: grpScale
                    });
                });
            });
        } else if (points) {
            points.forEach((pt, idx) => {
                const val = (intensities && intensities[idx] !== undefined) ? intensities[idx] : 0;
                this.voxelMap.set(this.hash(pt[0], pt[1], pt[2]), {
                    x: pt[0], 
                    y: pt[1], 
                    z: pt[2], 
                    scale: this.scale,
                    intensity: val
                });
            });
        }
        this.updateInstancedMesh();
    }
    
    getArray() {
        return this.voxelList.map(p => [p.x, p.y, p.z]);
    }
    
    getVoxelByInstanceId(instanceId) {
        return this.voxelList[instanceId];
    }
}
