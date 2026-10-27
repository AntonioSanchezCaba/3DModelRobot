/**
 * =====================================================
 * MAIN APPLICATION - 3D ROBOTIC ARM CONTROLLER
 * Three.js integration with kinematics and UI
 * =====================================================
 *
 * Coordinate System Mapping (both frames are right-handed, so this is a rotation,
 * not a mirror image: the 3D view turns the same way as the real arm):
 * - Kinematics: X=forward, Y=left, Z=up
 * - Three.js:   X=right, Y=up, Z=toward the viewer
 *
 * - Kinematics X -> Three.js X
 * - Kinematics Y -> Three.js -Z
 * - Kinematics Z -> Three.js Y (+ base platform height)
 */

// Height of the base platform: kinematic Z = 0 sits this far above the Three.js ground
const BASE_HEIGHT = 12;

// Longest end-effector trail drawn when "Mostrar trayectoria" is on
const MAX_TRAJECTORY_POINTS = 500;

// Largest joint change (degrees) a single drag move may cause; beyond it the arm
// would jump to another configuration, so it stops at the limit instead
const DRAG_MAX_JOINT_STEP = 20;

// How close (CSS px) the pointer must be to a handle's center to grab it
const HANDLE_PICK_RADIUS = 45;
const HANDLE_PICK_RADIUS_TOUCH = 60;

// Drag handle colors (sphere + ring) for each interaction state
const HANDLE_COLORS = {
    end: { idle: 0x00ff88, hover: 0x7dffc4 },
    elbow: { idle: 0x8888ff, hover: 0xb8b8ff },
    dragging: 0xffd400,  // Yellow: following the cursor
    limit: 0xff3b3b      // Red: held at the workspace / floor limit
};

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

        // Joint pivots for rotation
        this.joint1Pivot = null;  // Base rotation (around Y/vertical)
        this.joint2Pivot = null;  // Shoulder rotation (pitch)
        this.joint3Pivot = null;  // Elbow rotation (pitch)

        // Link meshes
        this.link1Mesh = null;    // Vertical tower
        this.link2Mesh = null;    // Upper arm
        this.link3Mesh = null;    // Forearm
        this.endEffectorMesh = null;

        // Visual helpers
        this.axesHelper = null;
        this.gridHelper = null;
        this.trajectoryLine = null;
        this.trajectoryCount = 0;
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

        // Interactive dragging
        this.isDragging = false;
        this.isHoveringHandle = false;
        this.dragHandle = null;        // End effector handle
        this.elbowDragHandle = null;   // Elbow joint handle
        this.activeHandle = null;      // Currently dragged handle ('end' or 'elbow')
        this.dragPlane = null;
        this.raycaster = null;
        this.mouse = null;
        this.intersection = null;

        // Initialize
        // init() is async: report failures (e.g. no WebGL) instead of loading forever
        this.init().catch(error => showStartupError(error));
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
        this.setupAxesIndicator();

        // Create robot model
        this.createRobotModel();

        // Create target marker
        this.createTargetMarker();

        // Setup trajectory line
        this.setupTrajectoryLine();

        // Setup interactive drag controls
        this.setupDragControls();

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

        // Verify kinematics on startup
        this.verifyInitialPosition();
    }

    /**
     * Verify initial position matches kinematics at startup
     */
    verifyInitialPosition() {
        const fk = this.kinematics.forwardKinematics([0, 0, 0]);
        // Home position should be X=180, Y=0, Z=50 (L2+L3, 0, L1)
        const ok = Math.abs(fk.position.x - 180) < 0.01 &&
                   Math.abs(fk.position.y) < 0.01 &&
                   Math.abs(fk.position.z - 50) < 0.01;
        if (!ok) {
            console.warn('Kinematics home position mismatch:', fk.position);
        }
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
        this.camera.position.set(250, 200, 250);
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
        this.controls.target.set(50, 50, 0);

        // Narrow screens: start further away so the whole arm is visible
        this.appliedDistanceFactor = this.viewDistanceFactor();
        this.camera.position.sub(this.controls.target)
            .multiplyScalar(this.appliedDistanceFactor)
            .add(this.controls.target);
        this.controls.update();

        // A handle highlighted on hover may no longer be under the pointer once the camera moves
        this.controls.addEventListener('change', () => {
            if (this.isHoveringHandle && !this.isDragging) this.clearHover();
        });
    }

    /**
     * How much to pull the camera back on narrow (portrait) viewports so the whole arm fits
     */
    viewDistanceFactor() {
        return Math.min(2.2, Math.max(1, 1.3 / this.camera.aspect));
    }

    /**
     * Setup visual helpers (grid, axes)
     */
    setupHelpers() {
        // Grid on XZ plane (Three.js default)
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

        // Axes helper (X=red, Y=green/up, Z=blue)
        this.axesHelper = new THREE.AxesHelper(100);
        // Blue axis along Three.js -Z = kinematic +Y (red X, green Z up, blue Y)
        this.axesHelper.scale.z = -1;
        this.scene.add(this.axesHelper);
    }

    /**
     * Create robot model that matches kinematics
     *
     * Robot structure:
     * - Base platform at Y=0
     * - Joint 1 pivot at Y=0 (rotates around Y axis = θ1)
     * - Vertical tower up to Y=L1 (shoulder height)
     * - Joint 2 pivot at Y=L1 (rotates around Z axis = θ2, pitch)
     * - Upper arm extends along X when θ2=0
     * - Joint 3 pivot at end of upper arm (rotates around Z = θ3)
     * - Forearm extends along X when θ3=0
     */
    createRobotModel() {
        const L1 = this.kinematics.L1;  // 50mm - base height
        const L2 = this.kinematics.L2;  // 100mm - upper arm
        const L3 = this.kinematics.L3;  // 80mm - forearm

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

        const link2Material = new THREE.MeshStandardMaterial({
            color: 0x10b981,
            metalness: 0.6,
            roughness: 0.3
        });

        const link3Material = new THREE.MeshStandardMaterial({
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

        // ============================================
        // BASE PLATFORM (fixed)
        // ============================================
        const baseGeometry = new THREE.CylinderGeometry(35, 40, 12, 32);
        this.base = new THREE.Mesh(baseGeometry, baseMaterial);
        this.base.position.y = 6;
        this.base.castShadow = true;
        this.base.receiveShadow = true;
        this.robotGroup.add(this.base);

        // ============================================
        // JOINT 1 - BASE ROTATION (θ1 around Y axis)
        // ============================================
        this.joint1Pivot = new THREE.Group();
        this.joint1Pivot.position.y = BASE_HEIGHT;  // On top of base
        this.robotGroup.add(this.joint1Pivot);

        // Vertical tower from base to shoulder (Link 1)
        const towerGeometry = new THREE.CylinderGeometry(15, 18, L1, 32);
        this.link1Mesh = new THREE.Mesh(towerGeometry, jointMaterial);
        this.link1Mesh.position.y = L1 / 2;
        this.link1Mesh.castShadow = true;
        this.joint1Pivot.add(this.link1Mesh);

        // Shoulder joint housing
        const shoulderHousing = new THREE.Mesh(
            new THREE.SphereGeometry(18, 32, 32),
            jointMaterial
        );
        shoulderHousing.position.y = L1;
        shoulderHousing.castShadow = true;
        this.joint1Pivot.add(shoulderHousing);

        // ============================================
        // JOINT 2 - SHOULDER (θ2 pitch rotation)
        // Position at Y = L1 (top of tower)
        // Rotates around local Z axis (perpendicular to arm plane)
        // When θ2=0: arm points along +X
        // When θ2=90: arm points along +Y (up)
        // ============================================
        this.joint2Pivot = new THREE.Group();
        this.joint2Pivot.position.y = L1;
        this.joint1Pivot.add(this.joint2Pivot);

        // Upper arm (Link 2) - extends along local +X when θ2=0
        const link2Geometry = new THREE.BoxGeometry(L2, 12, 12);
        this.link2Mesh = new THREE.Mesh(link2Geometry, link2Material);
        this.link2Mesh.position.x = L2 / 2;  // Center at half length along X
        this.link2Mesh.castShadow = true;
        this.joint2Pivot.add(this.link2Mesh);

        // Elbow joint sphere
        const elbowJoint = new THREE.Mesh(
            new THREE.SphereGeometry(10, 32, 32),
            jointMaterial
        );
        elbowJoint.position.x = L2;  // At end of upper arm
        elbowJoint.castShadow = true;
        this.joint2Pivot.add(elbowJoint);

        // ============================================
        // JOINT 3 - ELBOW (θ3 pitch rotation)
        // Position at X = L2 (end of upper arm)
        // Rotates around local Z axis
        // When θ3=0: forearm continues along same direction as upper arm
        // ============================================
        this.joint3Pivot = new THREE.Group();
        this.joint3Pivot.position.x = L2;
        this.joint2Pivot.add(this.joint3Pivot);

        // Forearm (Link 3) - extends along local +X when θ3=0
        // Forearm visual is shorter to leave room for the cone (tip = kinematic end effector).
        // The cone is 20mm long, or half the forearm when the forearm is very short.
        const coneLength = Math.min(20, L3 / 2);
        const forearmLength = L3 - coneLength;  // 60 for L3 = 80
        const link3Geometry = new THREE.BoxGeometry(forearmLength, 10, 10);
        this.link3Mesh = new THREE.Mesh(link3Geometry, link3Material);
        this.link3Mesh.position.x = forearmLength / 2;  // Center at 30
        this.link3Mesh.castShadow = true;
        this.joint3Pivot.add(this.link3Mesh);

        // End effector cone - TIP is at kinematic end effector position (L3 from elbow)
        // Cone is centered at origin, after rotation: base at local x=-coneLength/2, tip at +coneLength/2
        // To have base at forearmLength (60) and tip at L3 (80): position.x = 70
        const endEffectorGeometry = new THREE.ConeGeometry(8 * coneLength / 20, coneLength, 32);
        this.endEffectorMesh = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
        this.endEffectorMesh.position.x = forearmLength + coneLength / 2;  // 70: base at 60, tip at 80=L3
        this.endEffectorMesh.rotation.z = -Math.PI / 2;  // Point along X
        this.endEffectorMesh.castShadow = true;
        this.joint3Pivot.add(this.endEffectorMesh);

        // Add coordinate markers at joints
        this.addJointMarkers();

        this.scene.add(this.robotGroup);
    }

    /**
     * Add small coordinate markers at joints for debugging
     */
    addJointMarkers() {
        // Small axes at each joint
        const axesSize = 30;

        const shoulder = new THREE.AxesHelper(axesSize);
        shoulder.position.y = this.kinematics.L1;
        this.joint1Pivot.add(shoulder);
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

        // Fixed-size buffer; only the first trajectoryCount points are drawn
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_TRAJECTORY_POINTS * 3), 3));
        geometry.setDrawRange(0, 0);
        this.trajectoryCount = 0;

        this.trajectoryLine = new THREE.Line(geometry, material);
        this.trajectoryLine.visible = false;
        this.trajectoryLine.frustumCulled = false;  // Bounding sphere would be computed from the empty buffer
        this.scene.add(this.trajectoryLine);
    }

    /**
     * Setup interactive drag controls for end effector
     */
    setupDragControls() {
        // Initialize Three.js objects for raycasting
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.intersection = new THREE.Vector3();
        this.tmpVector = new THREE.Vector3();  // Scratch vector for per-event math

        // Create draggable handles
        this.createDragHandle();       // End effector
        this.createElbowDragHandle();  // Elbow joint

        // Create invisible drag plane
        this.createDragPlane();

        // Get canvas element
        const canvas = this.renderer.domElement;

        // IMPORTANT: Use capture phase (3rd param = true) so our handlers fire
        // BEFORE OrbitControls' handlers. This lets us intercept clicks on the
        // drag handle and stop OrbitControls from processing them.
        canvas.addEventListener('pointerdown', (e) => this.onPointerDown(e), true);
        canvas.addEventListener('pointermove', (e) => this.onPointerMove(e), true);
        canvas.addEventListener('pointerup', (e) => this.onPointerUp(e), true);
        canvas.addEventListener('pointercancel', (e) => this.onPointerUp(e), true);
        canvas.addEventListener('lostpointercapture', (e) => this.onPointerUp(e), true);

        // Pointer events also cover touch. Without this the browser would pan/zoom
        // the page (and cancel the pointer) while a finger drags a handle.
        canvas.style.touchAction = 'none';

        // Leaving the canvas ends hover feedback (cursor and highlight)
        canvas.addEventListener('pointerleave', () => {
            if (!this.isDragging) this.clearHover();
        });
    }

    /**
     * Build a drag handle: translucent sphere + ring, drawn on top of the model
     */
    createHandle(name, radius, ringRadius, ringTube, color, renderOrder) {
        const geometry = new THREE.SphereGeometry(radius, 32, 32);
        geometry.computeBoundingSphere();  // Ensure bounding sphere is computed for raycasting

        const material = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.7,
            depthTest: false  // Render on top of everything
        });

        const handle = new THREE.Mesh(geometry, material);
        handle.name = name;
        handle.renderOrder = renderOrder;

        // Outer ring for better visibility
        const ringGeometry = new THREE.TorusGeometry(ringRadius, ringTube, 16, 32);
        ringGeometry.computeBoundingSphere();
        const ringMaterial = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.9,
            depthTest: false
        });
        const ring = new THREE.Mesh(ringGeometry, ringMaterial);
        ring.renderOrder = renderOrder;
        handle.add(ring);

        this.scene.add(handle);
        return handle;
    }

    /**
     * Create draggable handle sphere at end effector
     */
    createDragHandle() {
        this.dragHandle = this.createHandle('dragHandle', 15, 20, 3, HANDLE_COLORS.end.idle, 999);
        this.updateDragHandlePosition();
    }

    /**
     * Create draggable handle at elbow joint
     */
    createElbowDragHandle() {
        this.elbowDragHandle = this.createHandle('elbowDragHandle', 12, 16, 2.5, HANDLE_COLORS.elbow.idle, 998);
        this.updateElbowDragHandlePosition();
    }

    /**
     * Color a handle (sphere and ring) for an interaction state
     * @param {string} which - 'end' or 'elbow'
     * @param {string} state - 'idle', 'hover', 'dragging' or 'limit'
     */
    setHandleState(which, state) {
        const handle = which === 'elbow' ? this.elbowDragHandle : this.dragHandle;
        if (!handle) return;

        const color = state === 'dragging' || state === 'limit'
            ? HANDLE_COLORS[state]
            : HANDLE_COLORS[which][state];
        const opacity = state === 'idle' ? 0.7 : 0.9;

        handle.material.color.setHex(color);
        handle.material.opacity = opacity;
        handle.children.forEach(child => {
            child.material.color.setHex(color);
        });
    }

    /**
     * Update elbow drag handle position based on forward kinematics
     */
    updateElbowDragHandlePosition() {
        if (!this.elbowDragHandle || !this.joint3Pivot) return;

        // The elbow handle sits on joint3Pivot (elbow)
        this.joint3Pivot.getWorldPosition(this.elbowDragHandle.position);
    }

    /**
     * Update drag handle position based on forward kinematics
     * This ensures the handle is always at the correct kinematic end effector position
     */
    updateDragHandlePosition() {
        if (!this.dragHandle || !this.joint3Pivot) return;

        // The kinematic end effector (and the cone tip) is L3 along the forearm
        // from the elbow. Refresh the world matrices first: the joints may have just rotated.
        this.joint3Pivot.updateWorldMatrix(true, false);
        this.dragHandle.position.set(this.kinematics.L3, 0, 0);
        this.joint3Pivot.localToWorld(this.dragHandle.position);

        // Also update elbow handle
        this.updateElbowDragHandlePosition();
    }

    /**
     * Create invisible plane for dragging
     */
    createDragPlane() {
        const geometry = new THREE.PlaneGeometry(1000, 1000);
        const material = new THREE.MeshBasicMaterial({
            visible: false,
            side: THREE.DoubleSide
        });

        this.dragPlane = new THREE.Mesh(geometry, material);
        this.scene.add(this.dragPlane);
    }

    /**
     * Update drag plane to face camera
     */
    updateDragPlane() {
        if (!this.dragPlane) return;

        // Position plane at the active handle
        if (this.activeHandle === 'elbow' && this.elbowDragHandle) {
            this.dragPlane.position.copy(this.elbowDragHandle.position);
        } else if (this.dragHandle) {
            this.dragPlane.position.copy(this.dragHandle.position);
        }

        // Make plane face the camera
        this.dragPlane.lookAt(this.camera.position);
        this.dragPlane.updateMatrixWorld();
    }

    /**
     * Get normalized mouse coordinates
     */
    getMouseCoords(event) {
        const canvas = this.renderer.domElement;
        const rect = canvas.getBoundingClientRect();

        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    }

    /**
     * Fallback click detection: project handle to screen and check pixel distance.
     * Used when raycaster misses due to depth or geometry issues.
     */
    isClickNearDragHandle(threshold = 30) {
        return this.isClickNearHandle(this.dragHandle, threshold);
    }

    /**
     * Check if click is near a specific handle (screen-space check)
     */
    isClickNearHandle(handle, threshold = 30) {
        return this.getHandleScreenDistance(handle) < threshold;
    }

    /**
     * Distance in CSS pixels between the pointer and a handle's center on screen
     */
    getHandleScreenDistance(handle) {
        if (!handle) return Infinity;

        const handleScreenPos = this.tmpVector.copy(handle.position).project(this.camera);

        const canvas = this.renderer.domElement;
        const handleX = (handleScreenPos.x + 1) / 2 * canvas.clientWidth;
        const handleY = (-handleScreenPos.y + 1) / 2 * canvas.clientHeight;
        const mouseX = (this.mouse.x + 1) / 2 * canvas.clientWidth;
        const mouseY = (-this.mouse.y + 1) / 2 * canvas.clientHeight;

        return Math.sqrt((handleX - mouseX) ** 2 + (handleY - mouseY) ** 2);
    }

    /**
     * Which handle is under the pointer ('end', 'elbow' or null)
     *
     * A handle counts if the ray hits it or the pointer is within `slack` pixels
     * of its center; when both qualify (they can overlap on screen) the one whose
     * center is closer to the pointer wins.
     */
    pickHandle(slack) {
        const candidates = [
            { which: 'end', handle: this.dragHandle, threshold: slack },
            { which: 'elbow', handle: this.elbowDragHandle, threshold: slack * 0.85 }
        ];

        let best = null;
        let bestDistance = Infinity;
        for (const { which, handle, threshold } of candidates) {
            if (!handle) continue;
            const hit = this.raycaster.intersectObject(handle, true).length > 0;
            const distance = this.getHandleScreenDistance(handle);
            if ((hit || distance < threshold) && distance < bestDistance) {
                best = which;
                bestDistance = distance;
            }
        }
        return best;
    }

    /**
     * Handle pointer down event (capture phase - fires BEFORE OrbitControls)
     */
    onPointerDown(event) {
        // Primary button / single touch only
        if (event.button !== 0 || this.isDragging) return;

        this.getMouseCoords(event);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        // Fingers are less precise than a mouse: accept touches a bit further away
        const clickedHandle = this.pickHandle(event.pointerType === 'touch' ? HANDLE_PICK_RADIUS_TOUCH : HANDLE_PICK_RADIUS);
        if (!clickedHandle) return;

        // CRITICAL: Stop the event from reaching OrbitControls
        event.stopImmediatePropagation();
        event.preventDefault();

        this.isDragging = true;
        this.activeHandle = clickedHandle;
        this.dragPointerId = event.pointerId;
        this.controls.enabled = false;

        // Keep receiving moves (and the release) even outside the canvas
        try {
            this.renderer.domElement.setPointerCapture(event.pointerId);
        } catch (err) {
            // Not fatal: the drag still works while the pointer stays on the canvas
        }

        // Grabbing the arm stops any preset/saved-position animation
        if (this.ui) this.ui.cancelAnimation();

        // Update drag plane based on which handle
        this.updateDragPlane();

        // Change handle color to indicate active dragging
        this.setHandleState(clickedHandle, 'dragging');
        this.setHandleState(clickedHandle === 'end' ? 'elbow' : 'end', 'idle');
        this.updateStatus(clickedHandle === 'end' ? 'Arrastrando efector final...' : 'Arrastrando codo...');

        // Visual feedback
        document.body.classList.add('dragging-active');
        document.body.style.cursor = 'grabbing';
        const indicator = document.getElementById('drag-indicator');
        if (indicator) indicator.classList.add('active');
    }

    /**
     * Handle pointer move event (capture phase)
     */
    onPointerMove(event) {
        if (this.isDragging) {
            if (event.pointerId !== this.dragPointerId) return;

            // CRITICAL: Stop OrbitControls from receiving move events while dragging
            event.stopImmediatePropagation();
            event.preventDefault();

            this.getMouseCoords(event);
            this.raycaster.setFromCamera(this.mouse, this.camera);

            // Keep drag plane updated to current handle position
            this.updateDragPlane();

            // Raycast against drag plane
            const intersects = this.raycaster.intersectObject(this.dragPlane);

            if (intersects.length > 0) {
                const point = intersects[0].point;

                if (this.activeHandle === 'end') {
                    // End effector dragging - full IK
                    this.handleEndEffectorDrag(point);
                } else if (this.activeHandle === 'elbow') {
                    // Elbow dragging - only affects θ1 and θ2
                    this.handleElbowDrag(point);
                }
            }
            return;
        }

        // Hover feedback is for mice and pens; touch has no hover
        if (event.pointerType === 'touch' || event.buttons !== 0) return;

        this.getMouseCoords(event);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        // Same pick test as pointerdown, so a highlighted handle is always grabbable
        const hovered = this.pickHandle(HANDLE_PICK_RADIUS);
        this.setHandleState('end', hovered === 'end' ? 'hover' : 'idle');
        this.setHandleState('elbow', hovered === 'elbow' ? 'hover' : 'idle');

        if (hovered) {
            document.body.style.cursor = 'grab';
            this.isHoveringHandle = true;
        } else if (this.isHoveringHandle) {
            document.body.style.cursor = 'default';
            this.isHoveringHandle = false;
        }
    }

    /**
     * Drop hover feedback: both handles idle, default cursor
     */
    clearHover() {
        this.setHandleState('end', 'idle');
        this.setHandleState('elbow', 'idle');
        if (this.isHoveringHandle) {
            document.body.style.cursor = 'default';
            this.isHoveringHandle = false;
        }
    }

    /**
     * Convert a Three.js world point to kinematics coordinates
     * (Three.js X → kin X, Three.js -Z → kin Y, Three.js Y - base height → kin Z)
     */
    threeToKinematics(point) {
        return {
            x: point.x,
            y: -point.z,
            z: point.y - BASE_HEIGHT
        };
    }

    /**
     * Handle end effector drag - full inverse kinematics
     */
    handleEndEffectorDrag(point) {
        const target = this.threeToKinematics(point);
        const tip = this.kinematics.forwardKinematics(this.currentAngles).position;

        // Follow the cursor as far as the arm can go (reach, joint limits, floor) without
        // jumping to another configuration (elbow flip, base spin) in a single move
        const options = { reference: this.currentAngles, maxStep: DRAG_MAX_JOINT_STEP };
        const clamped = this.kinematics.clampToWorkspace(target.x, target.y, target.z, tip, options);

        // Solution closest to the current pose, so the arm never flips while dragging
        const ikResult = this.kinematics.inverseKinematics(clamped.x, clamped.y, clamped.z, this.currentAngles, options);

        if (!ikResult.valid) {
            // Only possible if the current pose itself is no longer reachable
            this.setHandleState('end', 'limit');
            this.updateStatus(`⛔ ${ikResult.error || 'Posición no válida'}`);
            return;
        }

        this.applyAnglesFromIK(ikResult.angles);
        this.updateUIFromAngles(ikResult.angles);

        const position = `X=${clamped.x.toFixed(1)}, Y=${clamped.y.toFixed(1)}, Z=${clamped.z.toFixed(1)}`;
        if (clamped.wasClamped) {
            this.setHandleState('end', 'limit');
            this.updateStatus(`⚠ Límite del espacio de trabajo: ${position}`);
        } else {
            this.setHandleState('end', 'dragging');
            this.updateStatus(`Pos: ${position}`);
        }
    }

    /**
     * Handle elbow drag - only affects θ1 (base) and θ2 (shoulder)
     * θ3 (elbow) remains unchanged
     */
    handleElbowDrag(point) {
        const target = this.threeToKinematics(point);
        const kin = this.kinematics;
        const limits = kin.jointLimits.theta2;
        const theta3 = this.currentAngles[2];

        // Point the upper arm at the cursor: base toward it, shoulder at its elevation.
        // Straight above/below the base the direction is undefined, so keep θ1.
        const r = Math.sqrt(target.x * target.x + target.y * target.y);
        const theta1 = r > 1e-6 ? kin.radToDeg(Math.atan2(target.y, target.x)) : this.currentAngles[0];
        const desiredTheta2 = kin.radToDeg(Math.atan2(target.z - kin.L1, r));

        let theta2 = Math.min(limits.max, Math.max(limits.min, desiredTheta2));
        let limited = theta2 !== desiredTheta2;

        if (!kin.isConfigurationValid([theta1, theta2, theta3])) {
            // Too low: raising the shoulder lifts the whole arm, so find the lowest
            // shoulder angle that keeps it above the floor
            if (!kin.isConfigurationValid([theta1, limits.max, theta3])) {
                this.setHandleState('elbow', 'limit');
                this.updateStatus('⛔ Posición del codo no válida');
                return;
            }
            let low = theta2;
            let high = limits.max;
            for (let i = 0; i < 30; i++) {
                const mid = (low + high) / 2;
                if (kin.isConfigurationValid([theta1, mid, theta3])) high = mid;
                else low = mid;
            }
            theta2 = high;
            limited = true;
        }

        const angles = [theta1, theta2, theta3];
        this.setJointAngles(angles, { validate: false });
        this.updateUIFromAngles(angles);

        this.setHandleState('elbow', limited ? 'limit' : 'dragging');
        this.updateStatus(`${limited ? '⚠ Límite: ' : 'Codo: '}θ₁=${theta1.toFixed(1)}°, θ₂=${theta2.toFixed(1)}°`);
    }

    /**
     * Handle pointer up / cancel (capture phase)
     */
    onPointerUp(event) {
        if (!this.isDragging || event.pointerId !== this.dragPointerId) return;

        // Stop OrbitControls from seeing this event
        event.stopImmediatePropagation();
        event.preventDefault();

        try {
            if (this.renderer.domElement.hasPointerCapture(event.pointerId)) {
                this.renderer.domElement.releasePointerCapture(event.pointerId);
            }
        } catch (err) {
            // Capture may already be gone
        }

        this.isDragging = false;
        this.isHoveringHandle = false;
        this.activeHandle = null;
        this.dragPointerId = null;

        // Re-enable orbit controls
        this.controls.enabled = true;

        // Reset both handle colors
        this.setHandleState('end', 'idle');
        this.setHandleState('elbow', 'idle');

        // Remove visual feedback
        document.body.classList.remove('dragging-active');
        document.body.style.cursor = 'default';
        const indicator = document.getElementById('drag-indicator');
        if (indicator) indicator.classList.remove('active');

        // Show final position
        const fk = this.kinematics.forwardKinematics(this.currentAngles);
        this.updateStatus(`Posición: X=${fk.position.x.toFixed(1)}, Y=${fk.position.y.toFixed(1)}, Z=${fk.position.z.toFixed(1)} mm`);
    }

    /**
     * Apply angles from IK calculation (without updating UI)
     */
    applyAnglesFromIK(angles) {
        // IK results are validated by the caller
        this.setJointAngles(angles, { validate: false });
    }

    /**
     * Update UI sliders and inputs from angles
     */
    updateUIFromAngles(angles) {
        if (!this.ui) return;

        // Update each joint control in the UI
        for (let i = 0; i < 3; i++) {
            this.ui.setJointAngle(i, angles[i]);
        }

        // Update kinematics display
        this.ui.updateKinematicsDisplay();
        this.ui.updatePositionDisplay();
    }

    /**
     * Set joint angle with floor collision validation
     *
     * Joint mapping:
     * - Joint 0 (θ1): Base rotation around Y axis (Three.js Y = Kinematics Z)
     * - Joint 1 (θ2): Shoulder pitch around Z axis (local frame)
     * - Joint 2 (θ3): Elbow pitch around Z axis (local frame)
     *
     * @param {number} jointIndex - 0, 1, or 2
     * @param {number} angleDegrees - Angle in degrees
     * @param {boolean} skipValidation - Skip floor validation (for internal use)
     * @returns {boolean} True if angle was applied, false if blocked
     */
    setJointAngle(jointIndex, angleDegrees, skipValidation = false) {
        const angles = this.getCurrentAngles();
        angles[jointIndex] = angleDegrees;
        return this.setJointAngles(angles, { validate: !skipValidation });
    }

    /**
     * Set all three joint angles at once
     *
     * The whole configuration is validated together, so moving several joints
     * never gets rejected because of an intermediate one-joint-changed state.
     *
     * @param {Array} angles - [θ1, θ2, θ3] in degrees
     * @param {Object} options
     * @param {boolean} [options.validate=true] - Reject configurations outside joint limits or below the floor
     * @returns {boolean} True if the angles were applied, false if blocked
     */
    setJointAngles(angles, { validate = true } = {}) {
        if (validate && this.kinematics) {
            if (!this.kinematics.isWithinLimits(angles[0], angles[1], angles[2])) {
                this.updateStatus('⛔ Fuera de los límites articulares');
                return false;
            }
            const floorCheck = this.kinematics.isConfigurationAboveFloor(angles);
            if (!floorCheck.valid) {
                // Name the part that is below the floor, with its own height
                const part = floorCheck.elbowZ < 0 ? 'codo' : 'efector';
                const z = floorCheck.elbowZ < 0 ? floorCheck.elbowZ : floorCheck.endEffectorZ;
                this.updateStatus(`⛔ Configuración inválida: ${part} en Z=${z.toFixed(1)}mm`);
                return false;
            }
        }

        this.currentAngles = [angles[0], angles[1], angles[2]];
        if (this.kinematics) {
            // IK picks the solution closest to the robot's actual configuration
            this.kinematics.currentAngles = [...this.currentAngles];
        }

        // θ1: Base rotation around Y axis
        // Three.js Y+ rotation turns +X toward -Z, which is kinematic +Y: same sense as θ1
        this.joint1Pivot.rotation.y = angles[0] * Math.PI / 180;
        // θ2: Shoulder pitch around Z in local frame (positive = arm goes UP)
        this.joint2Pivot.rotation.z = angles[1] * Math.PI / 180;
        // θ3: Elbow pitch around Z in local frame (positive = forearm folds UP)
        this.joint3Pivot.rotation.z = angles[2] * Math.PI / 180;

        // Update drag handle position
        this.updateDragHandlePosition();

        // Update trajectory
        if (this.showTrajectory) {
            this.addTrajectoryPoint();
        }

        return true;
    }

    /**
     * Get current joint angles
     */
    getCurrentAngles() {
        return [...this.currentAngles];
    }

    /**
     * Add point to trajectory line
     */
    addTrajectoryPoint() {
        const fk = this.kinematics.forwardKinematics(this.currentAngles);

        // Convert kinematics coords to Three.js coords
        // Our mapping: kinX->threeX, kinY->-threeZ, kinZ->threeY
        const threeX = fk.position.x;
        const threeY = fk.position.z + BASE_HEIGHT;  // Add base height offset
        const threeZ = -fk.position.y;

        const geometry = this.trajectoryLine.geometry;
        const attribute = geometry.attributes.position;
        const positions = attribute.array;

        // Limit points: drop the oldest
        if (this.trajectoryCount === MAX_TRAJECTORY_POINTS) {
            positions.copyWithin(0, 3);
            this.trajectoryCount--;
        }

        positions[this.trajectoryCount * 3] = threeX;
        positions[this.trajectoryCount * 3 + 1] = threeY;
        positions[this.trajectoryCount * 3 + 2] = threeZ;
        this.trajectoryCount++;

        geometry.setDrawRange(0, this.trajectoryCount);
        attribute.needsUpdate = true;
    }

    /**
     * Show target marker at position (kinematics coordinates)
     */
    showTargetMarker(x, y, z) {
        // Convert kinematics coords to Three.js coords
        const threeX = x;
        const threeY = z + BASE_HEIGHT;  // Base height offset
        const threeZ = -y;

        this.targetMarker.position.set(threeX, threeY, threeZ);
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
                // View from +Z looking at robot
                endPos = new THREE.Vector3(100, 80, 300);
                endTarget = new THREE.Vector3(80, 60, 0);
                break;
            case 'top':
                // View from above
                endPos = new THREE.Vector3(100, 350, 0);
                endTarget = new THREE.Vector3(80, 0, 0);
                break;
            case 'isometric':
            default:
                endPos = new THREE.Vector3(250, 200, 250);
                endTarget = new THREE.Vector3(50, 50, 0);
                break;
        }

        // Same framing on narrow screens as at startup
        endPos.sub(endTarget).multiplyScalar(this.viewDistanceFactor()).add(endTarget);

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
            this.clearTrajectory();
        }
    }

    /**
     * Forget the drawn trajectory
     */
    clearTrajectory() {
        this.trajectoryCount = 0;
        this.trajectoryLine.geometry.setDrawRange(0, 0);
    }

    /**
     * Update robot geometry based on new parameters
     */
    updateRobotGeometry() {
        // Rebuild robot with new dimensions, freeing the old meshes' GPU memory
        if (this.robotGroup) {
            this.scene.remove(this.robotGroup);
            this.disposeObject(this.robotGroup);
        }
        this.createRobotModel();

        // The old path was drawn with the old link lengths
        this.clearTrajectory();

        // Keep the pose if it is still valid with the new lengths, otherwise go Home
        // (always valid: the whole arm is at shoulder height)
        const poseKept = this.kinematics.isConfigurationValid(this.currentAngles);
        this.setJointAngles(poseKept ? this.getCurrentAngles() : [0, 0, 0], { validate: false });
        if (this.ui) this.ui.syncControlsFromRobot();

        this.updateStatus(poseKept ? 'Geometría actualizada' : 'Geometría actualizada · brazo movido a Home');
        return poseKept;
    }

    /**
     * Free the GPU resources of an object tree removed from the scene
     */
    disposeObject(object) {
        object.traverse(child => {
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
                (Array.isArray(child.material) ? child.material : [child.material]).forEach(m => m.dispose());
            }
        });
    }

    /**
     * Update status message
     */
    updateStatus(message) {
        const statusEl = document.getElementById('status-message');
        if (statusEl) {
            statusEl.textContent = message;
        }
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

        // Keep the narrow-screen framing (e.g. a phone rotated to portrait), but keep
        // the user's viewing direction and zoom relative to it
        const factor = this.viewDistanceFactor();
        if (Math.abs(factor - this.appliedDistanceFactor) > 1e-6) {
            this.camera.position.sub(this.controls.target)
                .multiplyScalar(factor / this.appliedDistanceFactor)
                .add(this.controls.target);
            this.appliedDistanceFactor = factor;
            this.controls.update();
        }

        this.updateAxesIndicator(true);
    }

    /**
     * Orientation indicator (bottom-left): the kinematic X, Y, Z axes as the camera sees them
     */
    setupAxesIndicator() {
        const container = document.getElementById('axes-helper');
        if (!container) return;

        this.axesCanvas = document.createElement('canvas');
        this.axesCanvas.setAttribute('aria-hidden', 'true');
        container.append(this.axesCanvas);
        this.axesContext = this.axesCanvas.getContext('2d');
        this.axesDrawnQuaternion = new THREE.Quaternion(0, 0, 0, 0);  // Forces the first draw
        this.axesCameraInverse = new THREE.Quaternion();

        // Kinematic axes in Three.js coordinates, colored like the scene's axes helper
        this.axesIndicatorAxes = [
            { label: 'X', direction: new THREE.Vector3(1, 0, 0), view: new THREE.Vector3(), color: '#ef4444' },
            { label: 'Y', direction: new THREE.Vector3(0, 0, -1), view: new THREE.Vector3(), color: '#3b82f6' },
            { label: 'Z', direction: new THREE.Vector3(0, 1, 0), view: new THREE.Vector3(), color: '#22c55e' }
        ];
    }

    /**
     * Redraw the orientation indicator when the camera has turned (or when forced)
     */
    updateAxesIndicator(force = false) {
        if (!this.axesContext) return;

        const canvas = this.axesCanvas;
        const size = canvas.parentElement.clientWidth;
        const ratio = Math.min(window.devicePixelRatio || 1, 2);
        if (canvas.width !== Math.round(size * ratio)) {
            canvas.width = canvas.height = Math.round(size * ratio);
            canvas.style.width = canvas.style.height = `${size}px`;
            force = true;
        }
        if (!force && this.camera.quaternion.equals(this.axesDrawnQuaternion)) return;
        this.axesDrawnQuaternion.copy(this.camera.quaternion);

        const ctx = this.axesContext;
        ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
        ctx.clearRect(0, 0, size, size);

        const center = size / 2;
        const length = size * 0.3;
        this.axesCameraInverse.copy(this.camera.quaternion).invert();

        // View space: x right, y up, z toward the viewer. Draw the farthest axis first.
        const axes = this.axesIndicatorAxes;
        axes.forEach(axis => axis.view.copy(axis.direction).applyQuaternion(this.axesCameraInverse));
        [...axes].sort((a, b) => a.view.z - b.view.z).forEach(axis => {
            const x = center + axis.view.x * length;
            const y = center - axis.view.y * length;

            ctx.strokeStyle = axis.color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(center, center);
            ctx.lineTo(x, y);
            ctx.stroke();

            const labelX = center + axis.view.x * length * 1.32;
            const labelY = center - axis.view.y * length * 1.32;
            ctx.fillStyle = axis.color;
            ctx.beginPath();
            ctx.arc(labelX, labelY, Math.max(7, size * 0.08), 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = '#ffffff';
            ctx.font = `600 ${Math.max(9, Math.round(size * 0.1))}px Roboto, sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(axis.label, labelX, labelY + 0.5);
        });
    }

    /**
     * Main animation loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        const delta = this.clock.getDelta();

        // Update controls
        this.controls.update();
        this.updateAxesIndicator();

        // Animate target marker
        if (this.targetMarker && this.targetMarker.visible) {
            this.targetMarker.children[0].rotation.z += delta * 2;
            const scale = 1 + Math.sin(Date.now() * 0.005) * 0.1;
            this.targetMarker.scale.set(scale, scale, scale);
        }

        // Animate end effector glow
        if (this.endEffectorMesh) {
            const intensity = 0.2 + Math.sin(Date.now() * 0.003) * 0.1;
            this.endEffectorMesh.material.emissiveIntensity = intensity;
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

/**
 * Replace the loading message with an explanation when the app can't start
 */
function showStartupError(error) {
    console.error('RoboArm 3D could not start:', error);
    const loadingText = document.querySelector('.loading-text');
    if (!loadingText) return;

    const message = String(error && error.message ? error.message : error);
    loadingText.textContent = /webgl/i.test(message)
        ? 'Tu navegador o tarjeta gráfica no tiene WebGL disponible, necesario para la vista 3D.'
        : `Error al iniciar: ${message}`;
    loadingText.style.color = '#ef4444';
}

// Initialize application when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.robotController = new RobotArmController();
});
