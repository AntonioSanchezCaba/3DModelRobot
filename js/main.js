/**
 * =====================================================
 * MAIN APPLICATION - 3D ROBOTIC ARM CONTROLLER
 * Three.js integration with kinematics and UI
 * =====================================================
 */

class RobotArmController {
    constructor() {
        // Three.js components
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        // Robot components
        this.robotGroup = null;
        this.base = null;
        this.link1 = null;
        this.link2 = null;
        this.link3 = null;
        this.endEffector = null;

        // Joint pivots for rotation
        this.joint1Pivot = null;
        this.joint2Pivot = null;
        this.joint3Pivot = null;

        // Visual helpers
        this.axesHelper = null;
        this.gridHelper = null;
        this.trajectoryLine = null;
        this.trajectoryPoints = [];
        this.targetMarker = null;

        // Kinematics
        this.kinematics = null;
        this.currentAngles = [0, 0, 0];

        // Animation
        this.clock = null;
        this.frameCount = 0;
        this.lastFPSUpdate = 0;

        // Settings
        this.showTrajectory = false;

        // Initialize
        this.init();
    }

    /**
     * Initialize the 3D scene
     */
    async init() {
        // Create kinematics instance
        this.kinematics = new RobotKinematics({
            L1: 50,
            L2: 100,
            L3: 80
        });

        // Setup Three.js
        this.setupScene();
        this.setupCamera();
        this.setupRenderer();
        this.setupLighting();
        this.setupControls();
        this.setupHelpers();

        // Create robot model
        this.createSimplifiedRobot();

        // Try to load STL model
        await this.loadSTLModel();

        // Create target marker
        this.createTargetMarker();

        // Setup trajectory line
        this.setupTrajectoryLine();

        // Start animation loop
        this.clock = new THREE.Clock();
        this.animate();

        // Initialize UI
        this.ui = new UIController(this);

        // Hide loading screen
        setTimeout(() => {
            document.getElementById('loading-screen').classList.add('hidden');
            document.getElementById('app').classList.add('visible');
        }, 1500);

        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());

        // Update status
        this.updateStatus('Modelo cargado correctamente');
    }

    /**
     * Setup Three.js scene
     */
    setupScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f0f1a);
        this.scene.fog = new THREE.Fog(0x0f0f1a, 500, 1500);
    }

    /**
     * Setup camera
     */
    setupCamera() {
        const container = document.getElementById('viewport-container');
        const aspect = container.clientWidth / container.clientHeight;

        this.camera = new THREE.PerspectiveCamera(45, aspect, 0.1, 2000);
        this.camera.position.set(300, 200, 300);
        this.camera.lookAt(0, 50, 0);
    }

    /**
     * Setup WebGL renderer
     */
    setupRenderer() {
        const canvas = document.getElementById('robot-canvas');
        const container = document.getElementById('viewport-container');

        this.renderer = new THREE.WebGLRenderer({
            canvas: canvas,
            antialias: true,
            alpha: true
        });

        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.2;
    }

    /**
     * Setup scene lighting
     */
    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404060, 0.5);
        this.scene.add(ambientLight);

        // Main directional light
        const mainLight = new THREE.DirectionalLight(0xffffff, 1);
        mainLight.position.set(100, 200, 100);
        mainLight.castShadow = true;
        mainLight.shadow.mapSize.width = 2048;
        mainLight.shadow.mapSize.height = 2048;
        mainLight.shadow.camera.near = 10;
        mainLight.shadow.camera.far = 500;
        mainLight.shadow.camera.left = -150;
        mainLight.shadow.camera.right = 150;
        mainLight.shadow.camera.top = 150;
        mainLight.shadow.camera.bottom = -150;
        this.scene.add(mainLight);

        // Fill light
        const fillLight = new THREE.DirectionalLight(0x6366f1, 0.3);
        fillLight.position.set(-100, 100, -100);
        this.scene.add(fillLight);

        // Rim light
        const rimLight = new THREE.DirectionalLight(0x10b981, 0.2);
        rimLight.position.set(0, -50, -100);
        this.scene.add(rimLight);

        // Point lights for highlights
        const pointLight1 = new THREE.PointLight(0x6366f1, 0.5, 300);
        pointLight1.position.set(100, 100, 100);
        this.scene.add(pointLight1);

        const pointLight2 = new THREE.PointLight(0x10b981, 0.3, 300);
        pointLight2.position.set(-100, 50, -50);
        this.scene.add(pointLight2);
    }

    /**
     * Setup orbit controls
     */
    setupControls() {
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.minDistance = 100;
        this.controls.maxDistance = 800;
        this.controls.maxPolarAngle = Math.PI / 2 + 0.3;
        this.controls.target.set(0, 50, 0);
        this.controls.update();
    }

    /**
     * Setup visual helpers (grid, axes)
     */
    setupHelpers() {
        // Grid
        this.gridHelper = new THREE.GridHelper(400, 40, 0x6366f1, 0x252542);
        this.gridHelper.material.opacity = 0.3;
        this.gridHelper.material.transparent = true;
        this.scene.add(this.gridHelper);

        // Ground plane
        const groundGeometry = new THREE.CircleGeometry(200, 64);
        const groundMaterial = new THREE.MeshStandardMaterial({
            color: 0x1a1a2e,
            metalness: 0.3,
            roughness: 0.8,
            transparent: true,
            opacity: 0.8
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -0.5;
        ground.receiveShadow = true;
        this.scene.add(ground);

        // Axes helper
        this.axesHelper = new THREE.AxesHelper(100);
        this.scene.add(this.axesHelper);

        // Create mini axes helper for corner display
        this.createMiniAxes();
    }

    /**
     * Create mini axes for corner display
     */
    createMiniAxes() {
        // This will be rendered in a separate div if needed
    }

    /**
     * Create simplified robot model (fallback)
     */
    createSimplifiedRobot() {
        this.robotGroup = new THREE.Group();

        // Materials
        const baseMaterial = new THREE.MeshStandardMaterial({
            color: 0x2a2a4a,
            metalness: 0.7,
            roughness: 0.3
        });

        const jointMaterial = new THREE.MeshStandardMaterial({
            color: 0x6366f1,
            metalness: 0.8,
            roughness: 0.2,
            emissive: 0x6366f1,
            emissiveIntensity: 0.1
        });

        const link1Material = new THREE.MeshStandardMaterial({
            color: 0x10b981,
            metalness: 0.6,
            roughness: 0.3
        });

        const link2Material = new THREE.MeshStandardMaterial({
            color: 0xf59e0b,
            metalness: 0.6,
            roughness: 0.3
        });

        const endEffectorMaterial = new THREE.MeshStandardMaterial({
            color: 0xef4444,
            metalness: 0.8,
            roughness: 0.2,
            emissive: 0xef4444,
            emissiveIntensity: 0.2
        });

        // Base platform
        const baseGeometry = new THREE.CylinderGeometry(40, 45, 15, 32);
        this.base = new THREE.Mesh(baseGeometry, baseMaterial);
        this.base.position.y = 7.5;
        this.base.castShadow = true;
        this.base.receiveShadow = true;
        this.robotGroup.add(this.base);

        // Joint 1 pivot (base rotation)
        this.joint1Pivot = new THREE.Group();
        this.joint1Pivot.position.y = 15;
        this.robotGroup.add(this.joint1Pivot);

        // Base motor housing
        const motorHousing1 = new THREE.Mesh(
            new THREE.CylinderGeometry(20, 20, this.kinematics.L1, 32),
            jointMaterial
        );
        motorHousing1.position.y = this.kinematics.L1 / 2;
        motorHousing1.castShadow = true;
        this.joint1Pivot.add(motorHousing1);

        // Shoulder joint sphere
        const shoulderJoint = new THREE.Mesh(
            new THREE.SphereGeometry(15, 32, 32),
            jointMaterial
        );
        shoulderJoint.position.y = this.kinematics.L1;
        shoulderJoint.castShadow = true;
        this.joint1Pivot.add(shoulderJoint);

        // Joint 2 pivot (shoulder)
        this.joint2Pivot = new THREE.Group();
        this.joint2Pivot.position.y = this.kinematics.L1;
        this.joint1Pivot.add(this.joint2Pivot);

        // Upper arm (Link 2)
        const link2Geometry = new THREE.BoxGeometry(12, this.kinematics.L2, 12);
        this.link2 = new THREE.Mesh(link2Geometry, link1Material);
        this.link2.position.y = this.kinematics.L2 / 2;
        this.link2.castShadow = true;
        this.joint2Pivot.add(this.link2);

        // Elbow joint sphere
        const elbowJoint = new THREE.Mesh(
            new THREE.SphereGeometry(12, 32, 32),
            jointMaterial
        );
        elbowJoint.position.y = this.kinematics.L2;
        elbowJoint.castShadow = true;
        this.joint2Pivot.add(elbowJoint);

        // Joint 3 pivot (elbow)
        this.joint3Pivot = new THREE.Group();
        this.joint3Pivot.position.y = this.kinematics.L2;
        this.joint2Pivot.add(this.joint3Pivot);

        // Forearm (Link 3)
        const link3Geometry = new THREE.BoxGeometry(10, this.kinematics.L3, 10);
        this.link3 = new THREE.Mesh(link3Geometry, link2Material);
        this.link3.position.y = this.kinematics.L3 / 2;
        this.link3.castShadow = true;
        this.joint3Pivot.add(this.link3);

        // End effector
        const endEffectorGeometry = new THREE.ConeGeometry(8, 20, 32);
        this.endEffector = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
        this.endEffector.position.y = this.kinematics.L3 + 10;
        this.endEffector.castShadow = true;
        this.joint3Pivot.add(this.endEffector);

        // Add NEMA 17 motor models
        this.addMotorModels();

        this.scene.add(this.robotGroup);
    }

    /**
     * Add NEMA 17 motor visual models
     */
    addMotorModels() {
        const motorGeometry = new THREE.BoxGeometry(42, 42, 48);
        const motorMaterial = new THREE.MeshStandardMaterial({
            color: 0x1a1a1a,
            metalness: 0.9,
            roughness: 0.3
        });

        // Motor 1 (Base) - inside base
        // Motor representation is integrated in the base

        // Add motor labels/indicators
        const createMotorLabel = (text, position) => {
            // In a full implementation, we'd add text sprites here
        };
    }

    /**
     * Load STL model
     */
    async loadSTLModel() {
        return new Promise((resolve) => {
            const loader = new THREE.STLLoader();

            loader.load(
                'assembly.stl',
                (geometry) => {
                    // Center and scale the geometry
                    geometry.computeBoundingBox();
                    const bbox = geometry.boundingBox;
                    const center = new THREE.Vector3();
                    bbox.getCenter(center);

                    geometry.translate(-center.x, -center.y, -center.z);

                    // Calculate scale to fit nicely
                    const size = new THREE.Vector3();
                    bbox.getSize(size);
                    const maxDim = Math.max(size.x, size.y, size.z);
                    const scale = 150 / maxDim;

                    // Create mesh
                    const material = new THREE.MeshStandardMaterial({
                        color: 0x6366f1,
                        metalness: 0.4,
                        roughness: 0.5,
                        transparent: true,
                        opacity: 0.3,
                        wireframe: false
                    });

                    const mesh = new THREE.Mesh(geometry, material);
                    mesh.scale.set(scale, scale, scale);
                    mesh.position.y = 100;
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;

                    // Add to scene as reference model (semi-transparent)
                    this.stlMesh = mesh;
                    this.scene.add(mesh);

                    this.updateStatus('Modelo STL cargado');
                    resolve(true);
                },
                (xhr) => {
                    const progress = (xhr.loaded / xhr.total * 100);
                    document.querySelector('.progress-fill').style.width = `${progress}%`;
                },
                (error) => {
                    console.warn('Could not load STL file, using simplified model');
                    this.updateStatus('Usando modelo simplificado');
                    resolve(false);
                }
            );
        });
    }

    /**
     * Create target marker for IK visualization
     */
    createTargetMarker() {
        const geometry = new THREE.SphereGeometry(5, 16, 16);
        const material = new THREE.MeshBasicMaterial({
            color: 0x10b981,
            transparent: true,
            opacity: 0.8
        });

        this.targetMarker = new THREE.Mesh(geometry, material);
        this.targetMarker.visible = false;

        // Add pulsing ring
        const ringGeometry = new THREE.RingGeometry(8, 12, 32);
        const ringMaterial = new THREE.MeshBasicMaterial({
            color: 0x10b981,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.5
        });
        const ring = new THREE.Mesh(ringGeometry, ringMaterial);
        ring.rotation.x = Math.PI / 2;
        this.targetMarker.add(ring);

        this.scene.add(this.targetMarker);
    }

    /**
     * Setup trajectory line
     */
    setupTrajectoryLine() {
        const material = new THREE.LineBasicMaterial({
            color: 0x6366f1,
            transparent: true,
            opacity: 0.6
        });

        const geometry = new THREE.BufferGeometry();
        this.trajectoryLine = new THREE.Line(geometry, material);
        this.trajectoryLine.visible = false;
        this.scene.add(this.trajectoryLine);
    }

    /**
     * Set joint angle
     */
    setJointAngle(jointIndex, angleDegrees) {
        this.currentAngles[jointIndex] = angleDegrees;
        const angleRad = angleDegrees * Math.PI / 180;

        switch (jointIndex) {
            case 0: // Base rotation (Y-axis)
                this.joint1Pivot.rotation.y = angleRad;
                break;
            case 1: // Shoulder (Z-axis in local space, appears as pitch)
                this.joint2Pivot.rotation.z = angleRad;
                break;
            case 2: // Elbow (Z-axis in local space)
                this.joint3Pivot.rotation.z = angleRad;
                break;
        }

        // Update trajectory
        if (this.showTrajectory) {
            this.addTrajectoryPoint();
        }
    }

    /**
     * Get current joint angles
     */
    getCurrentAngles() {
        return [...this.currentAngles];
    }

    /**
     * Add point to trajectory
     */
    addTrajectoryPoint() {
        const fk = this.kinematics.forwardKinematics(this.currentAngles);
        this.trajectoryPoints.push(new THREE.Vector3(
            fk.position.x,
            fk.position.z + 15, // Offset to match robot position
            fk.position.y
        ));

        // Limit points
        if (this.trajectoryPoints.length > 500) {
            this.trajectoryPoints.shift();
        }

        // Update line
        const positions = new Float32Array(this.trajectoryPoints.length * 3);
        this.trajectoryPoints.forEach((p, i) => {
            positions[i * 3] = p.x;
            positions[i * 3 + 1] = p.y;
            positions[i * 3 + 2] = p.z;
        });

        this.trajectoryLine.geometry.setAttribute(
            'position',
            new THREE.BufferAttribute(positions, 3)
        );
        this.trajectoryLine.geometry.attributes.position.needsUpdate = true;
    }

    /**
     * Show target marker
     */
    showTargetMarker(x, y, z) {
        this.targetMarker.position.set(x, z + 15, y);
        this.targetMarker.visible = true;
    }

    /**
     * Hide target marker
     */
    hideTargetMarker() {
        this.targetMarker.visible = false;
    }

    /**
     * Camera controls
     */
    zoomCamera(factor) {
        const direction = new THREE.Vector3();
        this.camera.getWorldDirection(direction);

        this.camera.position.addScaledVector(
            direction,
            (1 - factor) * this.camera.position.length() * 0.3
        );
    }

    setCameraView(viewName) {
        const duration = 500;
        const startPos = this.camera.position.clone();
        const startTarget = this.controls.target.clone();

        let endPos, endTarget;

        switch (viewName) {
            case 'front':
                endPos = new THREE.Vector3(0, 100, 350);
                endTarget = new THREE.Vector3(0, 80, 0);
                break;
            case 'top':
                endPos = new THREE.Vector3(0, 400, 0);
                endTarget = new THREE.Vector3(0, 0, 0);
                break;
            case 'isometric':
            default:
                endPos = new THREE.Vector3(300, 200, 300);
                endTarget = new THREE.Vector3(0, 50, 0);
                break;
        }

        const startTime = performance.now();

        const animateCamera = (time) => {
            const elapsed = time - startTime;
            const progress = Math.min(elapsed / duration, 1);
            const eased = 1 - Math.pow(1 - progress, 3);

            this.camera.position.lerpVectors(startPos, endPos, eased);
            this.controls.target.lerpVectors(startTarget, endTarget, eased);
            this.controls.update();

            if (progress < 1) {
                requestAnimationFrame(animateCamera);
            }
        };

        requestAnimationFrame(animateCamera);
    }

    /**
     * Toggle visual helpers
     */
    toggleAxes(show) {
        this.axesHelper.visible = show;
    }

    toggleGrid(show) {
        this.gridHelper.visible = show;
    }

    toggleTrajectory(show) {
        this.showTrajectory = show;
        this.trajectoryLine.visible = show;

        if (!show) {
            this.trajectoryPoints = [];
        }
    }

    /**
     * Update robot geometry based on new parameters
     */
    updateRobotGeometry() {
        // Update link lengths in the simplified model
        // This would rebuild the robot with new dimensions
        this.updateStatus('Geometría actualizada');
    }

    /**
     * Update status message
     */
    updateStatus(message) {
        document.getElementById('status-message').textContent = message;
    }

    /**
     * Handle window resize
     */
    onWindowResize() {
        const container = document.getElementById('viewport-container');
        const width = container.clientWidth;
        const height = container.clientHeight;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /**
     * Main animation loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        const delta = this.clock.getDelta();

        // Update controls
        this.controls.update();

        // Animate target marker
        if (this.targetMarker.visible) {
            this.targetMarker.children[0].rotation.z += delta * 2;
            const scale = 1 + Math.sin(Date.now() * 0.005) * 0.1;
            this.targetMarker.scale.set(scale, scale, scale);
        }

        // Animate end effector glow
        if (this.endEffector) {
            const intensity = 0.2 + Math.sin(Date.now() * 0.003) * 0.1;
            this.endEffector.material.emissiveIntensity = intensity;
        }

        // Render
        this.renderer.render(this.scene, this.camera);

        // FPS counter
        this.frameCount++;
        const now = performance.now();
        if (now - this.lastFPSUpdate >= 1000) {
            if (this.ui) {
                this.ui.updateFPS(this.frameCount);
            }
            this.frameCount = 0;
            this.lastFPSUpdate = now;
        }
    }
}

// Initialize application when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.robotController = new RobotArmController();
});
