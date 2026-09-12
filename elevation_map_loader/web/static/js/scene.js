import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export function createMainScene(container) {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x121417);

    const camera = new THREE.PerspectiveCamera(
        55,
        window.innerWidth / window.innerHeight,
        0.1,
        1000
    );
    camera.position.set(0, -15, 12);
    camera.up.set(0, 0, 1); // Z-axis up (ROS standard)

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    // OrbitControls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.target.set(0, 0, 0);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(20, -30, 40);
    scene.add(dirLight);

    // Grid helper in XY plane
    const grid = new THREE.GridHelper(50, 50, 0x30363d, 0x21262d);
    grid.rotation.x = Math.PI / 2; // Rotate to lie on XY plane
    scene.add(grid);

    // Axes helper (X: Red, Y: Green, Z: Blue)
    const axes = new THREE.AxesHelper(2.0);
    scene.add(axes);

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    let updateCallback = null;
    function render() {
        requestAnimationFrame(render);
        if (updateCallback) updateCallback();
        controls.update();
        renderer.render(scene, camera);
    }
    render();

    return {
        scene,
        camera,
        renderer,
        controls,
        setUpdateCallback: (cb) => { updateCallback = cb; }
    };
}
