import * as THREE from 'three';

/**
 * 导航路径 3D 可视化器
 * 支持全局规划路径与局部规划轨迹实时渲染
 */
export class PathVisualizer {
    constructor(scene) {
        this.scene = scene;
        this.globalPathLine = null;
        this.localPathLine = null;
        this.visible = true;

        this.globalMat = new THREE.LineBasicMaterial({
            color: 0x38bdf8, // 亮青色
            linewidth: 3
        });

        this.localMat = new THREE.LineBasicMaterial({
            color: 0xfacc15, // 亮黄色
            linewidth: 4
        });
    }

    updateGlobalPath(points) {
        if (this.globalPathLine) {
            this.scene.remove(this.globalPathLine);
            this.globalPathLine.geometry.dispose();
            this.globalPathLine = null;
        }
        if (!points || points.length < 2) return;

        const positions = [];
        for (const p of points) {
            positions.push(p[0], p[1], p[2] + 0.05); // 略抬高防 Z-Fighting
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        this.globalPathLine = new THREE.Line(geo, this.globalMat);
        this.globalPathLine.visible = this.visible;
        this.scene.add(this.globalPathLine);
    }

    updateLocalPath(points) {
        if (this.localPathLine) {
            this.scene.remove(this.localPathLine);
            this.localPathLine.geometry.dispose();
            this.localPathLine = null;
        }
        if (!points || points.length < 2) return;

        const positions = [];
        for (const p of points) {
            positions.push(p[0], p[1], p[2] + 0.08);
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        this.localPathLine = new THREE.Line(geo, this.localMat);
        this.localPathLine.visible = this.visible;
        this.scene.add(this.localPathLine);
    }

    setVisible(visible) {
        this.visible = visible;
        if (this.globalPathLine) this.globalPathLine.visible = visible;
        if (this.localPathLine) this.localPathLine.visible = visible;
    }
}
