/**
 * =====================================================
 * MAIN APPLICATION - 3D ROBOTIC ARM CONTROLLER
 * Three.js integration with kinematics and UI
 * =====================================================
 *
 * Coordinate System Mapping:
 * - Kinematics: X=forward, Y=left, Z=up
 * - Three.js:   X=right, Y=up, Z=forward
 *
 * For this application, we align them:
 * - Kinematics X -> Three.js X
 * - Kinematics Y -> Three.js Z
 * - Kinematics Z -> Three.js Y
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

        // Interactive dragging
        this.isDragging = false;
        this.isHoveringHandle = false;
        this.dragHandle = null;
        this.dragPlane = null;
        this.raycaster = null;
        this.mouse = null;
        this.intersection = null;

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
        this.createRobotModel();

        // Try to load STL model
        await this.loadSTLModel();

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
        this.controls.update();
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
        this.joint1Pivot.position.y = 12;  // On top of base
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
        // Forearm visual is shorter to leave room for the cone (tip = kinematic end effector)
        const forearmLength = Math.max(10, L3 - 20);  // Visual: leave 20mm for cone
        const link3Geometry = new THREE.BoxGeometry(forearmLength, 10, 10);
        this.link3Mesh = new THREE.Mesh(link3Geometry, link3Material);
        this.link3Mesh.position.x = forearmLength / 2;  // Center at 30
        this.link3Mesh.castShadow = true;
        this.joint3Pivot.add(this.link3Mesh);

        // End effector cone - TIP is at kinematic end effector position (L3 from elbow)
        // Cone height=20, centered at origin, after rotation: base at local x=-10, tip at +10
        // To have base at forearmLength (60) and tip at L3 (80): position.x = 70
        const endEffectorGeometry = new THREE.ConeGeometry(8, 20, 32);
        this.endEffectorMesh = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
        this.endEffectorMesh.position.x = forearmLength + 10;  // 70: base at 60, tip at 80=L3
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
     * Load STL model (reference overlay)
     */
    async loadSTLModel() {
        return new Promise((resolve) => {
            const loader = new THREE.STLLoader();

            loader.load(
                'assembly.stl',
                (geometry) => {
                    geometry.computeBoundingBox();
                    const bbox = geometry.boundingBox;
                    const center = new THREE.Vector3();
                    bbox.getCenter(center);

                    geometry.translate(-center.x, -center.y, -center.z);

                    const size = new THREE.Vector3();
                    bbox.getSize(size);
                    const maxDim = Math.max(size.x, size.y, size.z);
                    const scale = 150 / maxDim;

                    const material = new THREE.MeshStandardMaterial({
                        color: 0x6366f1,
                        metalness: 0.4,
                        roughness: 0.5,
                        transparent: true,
                        opacity: 0.2,
                        wireframe: false
                    });

                    const mesh = new THREE.Mesh(geometry, material);
                    mesh.scale.set(scale, scale, scale);
                    mesh.position.y = 80;
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;

                    this.stlMesh = mesh;
                    this.scene.add(mesh);

                    this.updateStatus('Modelo STL cargado');
                    resolve(true);
                },
                (xhr) => {
                    const progress = (xhr.loaded / xhr.total * 100);
                    const progressFill = document.querySelector('.progress-fill');
                    if (progressFill) {
                        progressFill.style.width = `${progress}%`;
                    }
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
     * Setup interactive drag controls for end effector
     */
    setupDragControls() {
        // Initialize Three.js objects for raycasting
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.intersection = new THREE.Vector3();

        // Create draggable handle at end effector
        this.createDragHandle();

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
        canvas.addEventListener('pointerleave', (e) => this.onPointerUp(e), true);

        // Touch support
        canvas.addEventListener('touchstart', (e) => this.onTouchStart(e), { passive: false, capture: true });
        canvas.addEventListener('touchmove', (e) => this.onTouchMove(e), { passive: false });
        canvas.addEventListener('touchend', (e) => this.onTouchEnd(e));
    }

    /**
     * Create draggable handle sphere at end effector
     */
    createDragHandle() {
        const geometry = new THREE.SphereGeometry(15, 32, 32);  // Slightly larger for easier clicking
        geometry.computeBoundingSphere();  // Ensure bounding sphere is computed for raycasting

        const material = new THREE.MeshBasicMaterial({
            color: 0x00ff88,
            transparent: true,
            opacity: 0.7,
            depthTest: false  // Render on top of everything
        });

        this.dragHandle = new THREE.Mesh(geometry, material);
        this.dragHandle.name = 'dragHandle';
        this.dragHandle.renderOrder = 999;  // Render last (on top)

        // Add outer ring for better visibility
        const ringGeometry = new THREE.TorusGeometry(20, 3, 16, 32);
        ringGeometry.computeBoundingSphere();
        const ringMaterial = new THREE.MeshBasicMaterial({
            color: 0x00ff88,
            transparent: true,
            opacity: 0.9,
            depthTest: false
        });
        const ring = new THREE.Mesh(ringGeometry, ringMaterial);
        ring.renderOrder = 999;
        this.dragHandle.add(ring);

        // Position at end effector
        this.updateDragHandlePosition();

        this.scene.add(this.dragHandle);
    }

    /**
     * Update drag handle position based on forward kinematics
     * This ensures the handle is always at the correct kinematic end effector position
     */
    updateDragHandlePosition() {
        if (!this.dragHandle || !this.endEffectorMesh) return;

        // Get the world position of the cone mesh center
        const worldPos = new THREE.Vector3();
        this.endEffectorMesh.getWorldPosition(worldPos);

        // The cone tip is 10 units along the mesh's local X axis (after z-rotation)
        // Transform this offset by the cone's world rotation to get actual tip position
        const tipOffset = new THREE.Vector3(10, 0, 0);
        const worldQuat = new THREE.Quaternion();
        this.endEffectorMesh.getWorldQuaternion(worldQuat);
        tipOffset.applyQuaternion(worldQuat);

        // Position drag handle exactly at the visual cone tip
        worldPos.add(tipOffset);
        this.dragHandle.position.copy(worldPos);
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
        if (!this.dragPlane || !this.dragHandle) return;

        // Position plane at drag handle
        this.dragPlane.position.copy(this.dragHandle.position);

        // Make plane face the camera
        this.dragPlane.lookAt(this.camera.position);
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
        const handleScreenPos = this.dragHandle.position.clone();
        handleScreenPos.project(this.camera);

        const canvas = this.renderer.domElement;
        const handleX = (handleScreenPos.x + 1) / 2 * canvas.clientWidth;
        const handleY = (-handleScreenPos.y + 1) / 2 * canvas.clientHeight;
        const mouseX = (this.mouse.x + 1) / 2 * canvas.clientWidth;
        const mouseY = (-this.mouse.y + 1) / 2 * canvas.clientHeight;

        const dist = Math.sqrt((handleX - mouseX) ** 2 + (handleY - mouseY) ** 2);
        return dist < threshold;
    }

    /**
     * Handle pointer down event (capture phase - fires BEFORE OrbitControls)
     */
    onPointerDown(event) {
        this.getMouseCoords(event);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        const intersects = this.raycaster.intersectObject(this.dragHandle, true);

        if (intersects.length > 0 || this.isClickNearDragHandle(40)) {
            // CRITICAL: Stop the event from reaching OrbitControls
            event.stopImmediatePropagation();
            event.preventDefault();

            this.isDragging = true;
            this.controls.enabled = false;

            // Update drag plane to face camera at handle position
            this.updateDragPlane();

            // Change handle color to indicate active dragging
            this.dragHandle.material.color.setHex(0xffff00);
            this.dragHandle.material.opacity = 0.9;

            // Visual feedback
            document.body.classList.add('dragging-active');
            document.body.style.cursor = 'grabbing';
            const indicator = document.getElementById('drag-indicator');
            if (indicator) indicator.classList.add('active');

            this.updateStatus('Arrastrando efector final...');
        }
    }

    /**
     * Handle pointer move event (capture phase)
     */
    onPointerMove(event) {
        this.getMouseCoords(event);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        if (this.isDragging) {
            // CRITICAL: Stop OrbitControls from receiving move events while dragging
            event.stopImmediatePropagation();
            event.preventDefault();

            // Keep drag plane updated to current handle position so mouse
            // projection is always relative to the arm's actual tip position
            this.updateDragPlane();

            // Raycast against drag plane
            const intersects = this.raycaster.intersectObject(this.dragPlane);

            if (intersects.length > 0) {
                const point = intersects[0].point;

                // Convert Three.js coords to kinematics coords
                let kinX = point.x;
                let kinY = point.z;
                let kinZ = point.y - 12;  // Remove base offset

                // Clamp position to workspace boundaries
                const clamped = this.kinematics.clampToWorkspace(kinX, kinY, kinZ);
                kinX = clamped.x;
                kinY = clamped.y;
                kinZ = clamped.z;

                // Calculate inverse kinematics with clamped position
                const ikResult = this.kinematics.inverseKinematics(kinX, kinY, kinZ);

                if (ikResult.valid) {
                    // Check if configuration keeps all arm parts above floor
                    const floorCheck = this.kinematics.isConfigurationAboveFloor(ikResult.angles);

                    if (floorCheck.valid) {
                        // Valid configuration - apply it
                        this.applyAnglesFromIK(ikResult.angles);
                        this.updateUIFromAngles(ikResult.angles);

                        if (clamped.wasClamped) {
                            this.dragHandle.material.color.setHex(0xff4444);  // Red: at boundary
                            this.updateStatus(`⚠ Límite: X=${kinX.toFixed(1)}, Y=${kinY.toFixed(1)}, Z=${kinZ.toFixed(1)}`);
                        } else {
                            this.dragHandle.material.color.setHex(0x00ff00);  // Green: normal drag
                            this.updateStatus(`Pos: X=${kinX.toFixed(1)}, Y=${kinY.toFixed(1)}, Z=${kinZ.toFixed(1)}`);
                        }
                    } else {
                        // Configuration would put arm below floor - don't apply
                        this.dragHandle.material.color.setHex(0xff0000);  // Bright red: blocked
                        this.updateStatus(`⛔ Codo bajo suelo (Z=${floorCheck.elbowZ.toFixed(1)}mm)`);
                    }
                } else {
                    // IK failed - don't move
                    this.updateStatus(`⛔ ${ikResult.error || 'Posición no válida'}`);
                    this.dragHandle.material.color.setHex(0xff0000);
                }
            }
        } else {
            // Hover detection using raycaster
            const intersects = this.raycaster.intersectObject(this.dragHandle, true);

            if (intersects.length > 0 || this.isClickNearDragHandle(50)) {
                this.dragHandle.material.color.setHex(0x00ffaa);
                this.dragHandle.material.opacity = 0.8;
                document.body.style.cursor = 'grab';
                this.isHoveringHandle = true;
            } else {
                this.dragHandle.material.color.setHex(0x00ff88);
                this.dragHandle.material.opacity = 0.7;
                if (this.isHoveringHandle) {
                    document.body.style.cursor = 'default';
                    this.isHoveringHandle = false;
                }
            }
        }
    }

    /**
     * Handle pointer up event (capture phase)
     */
    onPointerUp(event) {
        if (this.isDragging) {
            // Stop OrbitControls from seeing this event
            event.stopImmediatePropagation();
            event.preventDefault();

            this.isDragging = false;
            this.isHoveringHandle = false;

            // Re-enable orbit controls
            this.controls.enabled = true;

            // Reset handle color
            this.dragHandle.material.color.setHex(0x00ff88);
            this.dragHandle.material.opacity = 0.7;

            // Remove visual feedback
            document.body.classList.remove('dragging-active');
            document.body.style.cursor = 'default';
            const indicator = document.getElementById('drag-indicator');
            if (indicator) indicator.classList.remove('active');

            // Show final position
            const fk = this.kinematics.forwardKinematics(this.currentAngles);
            this.updateStatus(`Posición: X=${fk.position.x.toFixed(1)}, Y=${fk.position.y.toFixed(1)}, Z=${fk.position.z.toFixed(1)} mm`);
        }
    }

    /**
     * Handle touch start event
     */
    onTouchStart(event) {
        if (event.touches.length === 1) {
            const touch = event.touches[0];
            this.getMouseCoords(touch);
            this.raycaster.setFromCamera(this.mouse, this.camera);

            const intersects = this.raycaster.intersectObject(this.dragHandle, true);
            const isNearHandle = this.isClickNearDragHandle(50);

            if (intersects.length > 0 || isNearHandle) {
                event.stopImmediatePropagation();
                event.preventDefault();
                this.isDragging = true;
                this.controls.enabled = false;
                this.updateDragPlane();
                this.dragHandle.material.color.setHex(0xffff00);
            }
        }
    }

    /**
     * Handle touch move event
     */
    onTouchMove(event) {
        if (this.isDragging && event.touches.length === 1) {
            event.preventDefault();

            const touch = event.touches[0];
            this.getMouseCoords(touch);
            this.raycaster.setFromCamera(this.mouse, this.camera);

            // Keep drag plane at current handle position
            this.updateDragPlane();

            const intersects = this.raycaster.intersectObject(this.dragPlane);

            if (intersects.length > 0) {
                const point = intersects[0].point;

                let kinX = point.x;
                let kinY = point.z;
                let kinZ = point.y - 12;

                // Clamp position to workspace boundaries
                const clamped = this.kinematics.clampToWorkspace(kinX, kinY, kinZ);
                kinX = clamped.x;
                kinY = clamped.y;
                kinZ = clamped.z;

                const ikResult = this.kinematics.inverseKinematics(kinX, kinY, kinZ);

                if (ikResult.valid) {
                    // Check if configuration keeps all arm parts above floor
                    const floorCheck = this.kinematics.isConfigurationAboveFloor(ikResult.angles);

                    if (floorCheck.valid) {
                        this.applyAnglesFromIK(ikResult.angles);
                        this.updateUIFromAngles(ikResult.angles);

                        if (clamped.wasClamped) {
                            this.dragHandle.material.color.setHex(0xff4444);  // Red: at boundary
                        } else {
                            this.dragHandle.material.color.setHex(0x00ff00);  // Green: normal
                        }
                    } else {
                        // Configuration would put arm below floor
                        this.dragHandle.material.color.setHex(0xff0000);  // Bright red
                    }
                }
            }
        }
    }

    /**
     * Handle touch end event
     */
    onTouchEnd(event) {
        if (this.isDragging) {
            this.isDragging = false;
            this.isHoveringHandle = false;
            this.controls.enabled = true;
            this.dragHandle.material.color.setHex(0x00ff88);
            this.dragHandle.material.opacity = 0.7;
        }
    }

    /**
     * Apply angles from IK calculation (without updating UI)
     */
    applyAnglesFromIK(angles) {
        // Update internal state and 3D model
        for (let i = 0; i < 3; i++) {
            this.currentAngles[i] = angles[i];
            const angleRad = angles[i] * Math.PI / 180;

            switch (i) {
                case 0:
                    // Negate: Three.js Y+ rotation is toward -Z, but kinematics θ1+ is toward +kinY = +threeZ
                    this.joint1Pivot.rotation.y = -angleRad;
                    break;
                case 1:
                    this.joint2Pivot.rotation.z = angleRad;
                    break;
                case 2:
                    this.joint3Pivot.rotation.z = angleRad;
                    break;
            }
        }

        // Update drag handle position
        this.updateDragHandlePosition();

        // Update trajectory if enabled
        if (this.showTrajectory) {
            this.addTrajectoryPoint();
        }
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
        // Store previous angle in case we need to revert
        const previousAngle = this.currentAngles[jointIndex];

        // Temporarily set the new angle to test configuration
        this.currentAngles[jointIndex] = angleDegrees;

        // Check if this configuration keeps arm above floor (unless skipping validation)
        if (!skipValidation && this.kinematics) {
            const floorCheck = this.kinematics.isConfigurationAboveFloor(this.currentAngles);
            if (!floorCheck.valid) {
                // Revert to previous angle - configuration would put arm below floor
                this.currentAngles[jointIndex] = previousAngle;
                this.updateStatus(`⛔ Configuración inválida: codo en Z=${floorCheck.elbowZ.toFixed(1)}mm`);
                return false;
            }
        }

        // Apply the angle to the 3D model
        const angleRad = angleDegrees * Math.PI / 180;

        switch (jointIndex) {
            case 0:
                // θ1: Base rotation around Y axis
                // Negate: Three.js Y+ rotation is toward -Z, but kinematics θ1+ is toward +kinY = +threeZ
                this.joint1Pivot.rotation.y = -angleRad;
                break;
            case 1:
                // θ2: Shoulder pitch - rotation around Z axis in local frame
                // Positive θ2 = arm goes UP
                this.joint2Pivot.rotation.z = angleRad;
                break;
            case 2:
                // θ3: Elbow pitch - rotation around Z axis in local frame
                // Positive θ3 = forearm folds UP relative to upper arm
                this.joint3Pivot.rotation.z = angleRad;
                break;
        }

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
        // Kinematics: X=forward, Y=left, Z=up
        // Three.js: X=right, Y=up, Z=forward
        // Our mapping: kinX->threeX, kinY->threeZ, kinZ->threeY
        const threeX = fk.position.x;
        const threeY = fk.position.z + 12;  // Add base height offset
        const threeZ = fk.position.y;

        this.trajectoryPoints.push(new THREE.Vector3(threeX, threeY, threeZ));

        // Limit points
        if (this.trajectoryPoints.length > 500) {
            this.trajectoryPoints.shift();
        }

        // Update line geometry
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
     * Show target marker at position (kinematics coordinates)
     */
    showTargetMarker(x, y, z) {
        // Convert kinematics coords to Three.js coords
        const threeX = x;
        const threeY = z + 12;  // Base height offset
        const threeZ = y;

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
        // Rebuild robot with new dimensions
        if (this.robotGroup) {
            this.scene.remove(this.robotGroup);
        }
        this.createRobotModel();

        // Reapply current angles
        this.setJointAngle(0, this.currentAngles[0]);
        this.setJointAngle(1, this.currentAngles[1]);
        this.setJointAngle(2, this.currentAngles[2]);

        this.updateStatus('Geometría actualizada');
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

// Initialize application when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.robotController = new RobotArmController();
});
