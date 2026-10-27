/**
 * =====================================================
 * UI CONTROLLER MODULE
 * Handles all user interface interactions
 * =====================================================
 */

class UIController {
    constructor(robotController) {
        this.robot = robotController;
        this.currentSection = 'control';
        this.presets = {
            home: [0, 0, 0],
            pick: [45, 30, -45],
            place: [-45, 45, -30]
        };
        this.presetNames = {
            home: 'Home',
            pick: 'Pick',
            place: 'Place'
        };
        this.animationSpeed = 1.0;
        this.isAnimating = false;
        this.animationFrame = null;
        this.animationToken = 0;

        // User-named positions persisted in localStorage
        this.positionStore = new SavedPositionsStore({
            jointLimits: this.robot && this.robot.kinematics ? this.robot.kinematics.jointLimits : null
        });
        this.dialog = null;

        this.init();
    }

    /**
     * Initialize UI components
     */
    init() {
        this.bindNavigationEvents();
        this.bindJointControls();
        this.bindHeaderActions();
        this.bindViewportControls();
        this.bindKinematicsControls();
        this.bindSettingsControls();
        this.bindDialog();
        this.bindPresetButtons();
        this.bindSavedPositions();
        this.updateAllDisplays();
    }

    /**
     * Navigation tabs
     */
    bindNavigationEvents() {
        const navBtns = document.querySelectorAll('.nav-btn');
        const sections = document.querySelectorAll('.panel-section');

        navBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                const view = btn.dataset.view;

                // Update active button
                navBtns.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');

                // Show corresponding section
                sections.forEach(section => {
                    if (section.id === `section-${view}`) {
                        section.classList.remove('hidden');
                    } else if (section.id.startsWith('section-')) {
                        section.classList.add('hidden');
                    }
                });

                this.currentSection = view;
                this.showToast('info', 'Sección cambiada', `Vista: ${this.getSectionName(view)}`);
            });
        });
    }

    getSectionName(view) {
        const names = {
            control: 'Control de Articulaciones',
            kinematics: 'Cinemática',
            settings: 'Configuración'
        };
        return names[view] || view;
    }

    /**
     * Joint slider and input controls
     */
    bindJointControls() {
        const jointControls = document.querySelectorAll('.joint-control');

        jointControls.forEach((control, index) => {
            const slider = control.querySelector('.joint-slider');
            const input = control.querySelector('.angle-input');
            const quickBtns = control.querySelectorAll('.quick-btn');
            const sliderFill = control.querySelector('.slider-fill');

            // Slider change
            slider.addEventListener('input', (e) => {
                this.cancelAnimation();
                const value = parseFloat(e.target.value);
                input.value = Math.round(value);
                this.updateSliderFill(slider, sliderFill);
                this.updateJoint(index, value);
            });

            // Input change
            input.addEventListener('change', (e) => {
                let value = parseFloat(e.target.value);
                const min = parseFloat(slider.min);
                const max = parseFloat(slider.max);

                // Empty or non-numeric input: restore the robot's current angle
                if (!Number.isFinite(value)) {
                    this.setJointAngle(index, this.robot ? this.robot.getCurrentAngles()[index] : 0);
                    return;
                }

                // Clamp value
                value = Math.max(min, Math.min(max, value));
                e.target.value = value;
                slider.value = value;
                this.updateSliderFill(slider, sliderFill);
                this.cancelAnimation();
                this.updateJoint(index, value);
            });

            // Quick buttons
            quickBtns.forEach(btn => {
                btn.addEventListener('click', () => {
                    const angle = parseFloat(btn.dataset.angle);
                    this.animateToAngle(index, angle);
                });
            });

            // Highlight active joint
            control.addEventListener('mouseenter', () => {
                control.classList.add('active');
            });

            control.addEventListener('mouseleave', () => {
                control.classList.remove('active');
            });
        });
    }

    /**
     * Update slider fill visualization
     */
    updateSliderFill(slider, fill) {
        const min = parseFloat(slider.min);
        const max = parseFloat(slider.max);
        const value = parseFloat(slider.value);
        const percentage = ((value - min) / (max - min)) * 100;
        fill.style.width = `${percentage}%`;
    }

    /**
     * Update joint angle and trigger robot update
     */
    updateJoint(jointIndex, angle) {
        if (this.robot) {
            const success = this.robot.setJointAngle(jointIndex, angle);

            // If angle was rejected (arm would go below floor), revert slider
            if (success === false) {
                const control = document.querySelectorAll('.joint-control')[jointIndex];
                const slider = control.querySelector('.joint-slider');
                const input = control.querySelector('.angle-input');
                const sliderFill = control.querySelector('.slider-fill');

                // Get the actual current angle from the robot
                const currentAngle = this.robot.getCurrentAngles()[jointIndex];
                slider.value = currentAngle;
                input.value = Math.round(currentAngle);
                this.updateSliderFill(slider, sliderFill);
                return;
            }
        }
        this.updateKinematicsDisplay();
        this.updatePositionDisplay();
    }

    /**
     * Animate a single joint to a target angle (other joints keep their value)
     */
    animateToAngle(jointIndex, targetAngle) {
        if (!this.robot) return false;
        const target = this.robot.getCurrentAngles();
        target[jointIndex] = targetAngle;
        return this.animateToPosition(target);
    }

    /**
     * Header action buttons
     */
    bindHeaderActions() {
        // Reset button
        document.getElementById('btn-reset').addEventListener('click', () => {
            if (this.resetAllJoints()) {
                this.showToast('info', 'Posición reseteada', 'Todos los ángulos en 0°');
            }
        });

        // Home button
        document.getElementById('btn-home').addEventListener('click', () => {
            this.goToPreset('home');
        });

        // Fullscreen button
        document.getElementById('btn-fullscreen').addEventListener('click', () => {
            this.toggleFullscreen();
        });
    }

    /**
     * Reset all joints to 0
     */
    resetAllJoints() {
        return this.animateToPosition([0, 0, 0]);
    }

    /**
     * Toggle fullscreen mode
     */
    toggleFullscreen() {
        if (!document.fullscreenElement) {
            document.documentElement.requestFullscreen();
            document.getElementById('btn-fullscreen').innerHTML = '<i class="fas fa-compress"></i>';
        } else {
            document.exitFullscreen();
            document.getElementById('btn-fullscreen').innerHTML = '<i class="fas fa-expand"></i>';
        }
    }

    /**
     * Viewport camera controls
     */
    bindViewportControls() {
        document.getElementById('btn-zoom-in').addEventListener('click', () => {
            if (this.robot) this.robot.zoomCamera(0.8);
        });

        document.getElementById('btn-zoom-out').addEventListener('click', () => {
            if (this.robot) this.robot.zoomCamera(1.2);
        });

        document.getElementById('btn-view-front').addEventListener('click', () => {
            if (this.robot) this.robot.setCameraView('front');
            this.showToast('info', 'Vista', 'Vista frontal');
        });

        document.getElementById('btn-view-top').addEventListener('click', () => {
            if (this.robot) this.robot.setCameraView('top');
            this.showToast('info', 'Vista', 'Vista superior');
        });

        document.getElementById('btn-view-iso').addEventListener('click', () => {
            if (this.robot) this.robot.setCameraView('isometric');
            this.showToast('info', 'Vista', 'Vista isométrica');
        });
    }

    /**
     * Kinematics section controls
     */
    bindKinematicsControls() {
        // Calculate IK button
        document.getElementById('btn-calc-ik').addEventListener('click', () => {
            this.calculateInverseKinematics();
        });

        // Apply IK button
        document.getElementById('btn-apply-ik').addEventListener('click', () => {
            this.applyInverseKinematics();
        });

        // IK input fields - calculate on enter
        ['ik-target-x', 'ik-target-y', 'ik-target-z'].forEach(id => {
            document.getElementById(id).addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    this.calculateInverseKinematics();
                }
            });
        });
    }

    /**
     * Calculate inverse kinematics from UI inputs
     */
    calculateInverseKinematics() {
        const x = parseFloat(document.getElementById('ik-target-x').value);
        const y = parseFloat(document.getElementById('ik-target-y').value);
        const z = parseFloat(document.getElementById('ik-target-z').value);

        if (this.robot && this.robot.kinematics) {
            const result = this.robot.kinematics.inverseKinematics(x, y, z);

            const statusEl = document.getElementById('ik-status');
            const applyBtn = document.getElementById('btn-apply-ik');

            if (result.valid) {
                document.getElementById('ik-theta1').textContent = `${result.angles[0].toFixed(2)}°`;
                document.getElementById('ik-theta2').textContent = `${result.angles[1].toFixed(2)}°`;
                document.getElementById('ik-theta3').textContent = `${result.angles[2].toFixed(2)}°`;

                const configNames = {
                    elbow_up: 'codo arriba',
                    elbow_down: 'codo abajo',
                    elbow_up_back: 'codo arriba, por detrás',
                    elbow_down_back: 'codo abajo, por detrás'
                };
                statusEl.className = 'ik-status success';
                statusEl.innerHTML = '<i class="fas fa-check-circle"></i> ';
                statusEl.append(`Solución encontrada (${configNames[result.configuration] || result.configuration})`);

                applyBtn.disabled = false;
                applyBtn.dataset.angles = JSON.stringify(result.angles);

                // Show target marker in 3D
                if (this.robot) {
                    this.robot.showTargetMarker(x, y, z);
                }

                this.showToast('success', 'IK Calculado', 'Solución válida encontrada');
            } else {
                document.getElementById('ik-theta1').textContent = '--°';
                document.getElementById('ik-theta2').textContent = '--°';
                document.getElementById('ik-theta3').textContent = '--°';

                statusEl.className = 'ik-status error';
                statusEl.innerHTML = '<i class="fas fa-times-circle"></i> ';
                statusEl.append(result.error);

                applyBtn.disabled = true;
                delete applyBtn.dataset.angles;

                // Don't leave the marker pulsing at the previous, valid target
                if (this.robot) {
                    this.robot.hideTargetMarker();
                }

                this.showToast('error', 'Error IK', result.error);
            }
        }
    }

    /**
     * Apply calculated IK angles to robot
     */
    applyInverseKinematics() {
        const applyBtn = document.getElementById('btn-apply-ik');
        const anglesStr = applyBtn.dataset.angles;

        if (anglesStr) {
            const angles = JSON.parse(anglesStr);
            if (this.animateToPosition(angles)) {
                this.showToast('success', 'Posición aplicada', 'Moviendo a posición calculada');
            }
        }
    }

    /**
     * Animate robot to target angles along a floor-safe path
     *
     * A new call cancels the running animation and starts from wherever the arm is.
     *
     * @param {Array} targetAngles - [θ1, θ2, θ3] in degrees
     * @param {Object} options
     * @param {Function} [options.onComplete] - Called when the target is reached
     * @returns {boolean} False if the target is not reachable (nothing moves)
     */
    animateToPosition(targetAngles, { onComplete = null } = {}) {
        if (!this.robot || !this.robot.kinematics) return false;

        const target = targetAngles.map(Number);
        const start = this.robot.getCurrentAngles();
        const path = this.robot.kinematics.planJointPath(start, target);

        if (!path) {
            const kin = this.robot.kinematics;
            const reason = !target.every(Number.isFinite) ||
                !kin.isWithinLimits(target[0], target[1], target[2])
                ? 'Está fuera de los límites articulares'
                : 'El brazo quedaría por debajo del suelo';
            this.showToast('error', 'Posición no alcanzable', reason);
            return false;
        }

        this.cancelAnimation();

        // Cumulative joint-space length of each segment, to move at constant speed
        const lengths = [0];
        for (let i = 1; i < path.length; i++) {
            const segment = Math.max(...path[i].map((a, j) => Math.abs(a - path[i - 1][j])));
            lengths.push(lengths[i - 1] + segment);
        }
        const totalLength = lengths[lengths.length - 1];

        if (totalLength < 1e-6) {
            this.robot.setJointAngles(target, { validate: false });
            this.syncControlsFromRobot();
            this.robot.hideTargetMarker();
            if (onComplete) onComplete();
            return true;
        }

        // ~0.5 s for small moves, up to ~2.5 s for long multi-segment moves
        const duration = Math.min(2500, Math.max(500, 300 + totalLength * 6)) / this.animationSpeed;
        const startTime = performance.now();
        const token = ++this.animationToken;
        this.isAnimating = true;

        const animate = (currentTime) => {
            if (token !== this.animationToken) return;

            const progress = Math.min((currentTime - startTime) / duration, 1);

            // Smooth step function
            const eased = progress < 0.5
                ? 4 * progress * progress * progress
                : 1 - Math.pow(-2 * progress + 2, 3) / 2;

            const angles = progress < 1 ? this.samplePath(path, lengths, eased * totalLength) : target;
            this.robot.setJointAngles(angles, { validate: false });
            this.syncControlsFromRobot();

            if (progress < 1) {
                this.animationFrame = requestAnimationFrame(animate);
            } else {
                this.isAnimating = false;
                this.animationFrame = null;
                this.robot.hideTargetMarker();
                if (onComplete) onComplete();
            }
        };

        this.animationFrame = requestAnimationFrame(animate);
        return true;
    }

    /**
     * Joint angles at a given distance along a piecewise-linear path
     */
    samplePath(path, lengths, distance) {
        for (let i = 1; i < path.length; i++) {
            if (distance <= lengths[i] || i === path.length - 1) {
                const segment = lengths[i] - lengths[i - 1];
                const t = segment > 0 ? Math.min(1, Math.max(0, (distance - lengths[i - 1]) / segment)) : 1;
                return path[i - 1].map((a, j) => a + (path[i][j] - a) * t);
            }
        }
        return [...path[path.length - 1]];
    }

    /**
     * Stop the running animation, leaving the arm where it is
     */
    cancelAnimation() {
        this.animationToken++;
        if (this.animationFrame !== null) {
            cancelAnimationFrame(this.animationFrame);
            this.animationFrame = null;
        }
        this.isAnimating = false;
    }

    /**
     * Refresh sliders, inputs and readouts from the robot's actual angles
     */
    syncControlsFromRobot() {
        if (!this.robot) return;
        const angles = this.robot.getCurrentAngles();
        for (let i = 0; i < 3; i++) {
            this.setJointAngle(i, angles[i]);
        }
        this.updateKinematicsDisplay();
        this.updatePositionDisplay();
    }

    /**
     * Settings controls
     */
    bindSettingsControls() {
        // Robot parameters
        document.getElementById('btn-apply-params').addEventListener('click', () => {
            if (!this.robot || !this.robot.kinematics) return;

            // Read and check each length against its input's min/max
            const lengths = [];
            for (const [id, label] of [['param-l1', 'L1'], ['param-l2', 'L2'], ['param-l3', 'L3']]) {
                const input = document.getElementById(id);
                const value = parseFloat(input.value);
                const min = parseFloat(input.min) || 1;
                const max = parseFloat(input.max) || Infinity;

                if (!Number.isFinite(value) || value < min || value > max) {
                    const range = Number.isFinite(max) ? `entre ${min} y ${max} mm` : `de al menos ${min} mm`;
                    this.showToast('error', 'Parámetro inválido', `${label} debe ser un número ${range}`);
                    input.focus();
                    return;
                }
                lengths.push(value);
            }
            const [L1, L2, L3] = lengths;

            this.cancelAnimation();
            this.robot.kinematics.updateLinkLengths(L1, L2, L3);
            const poseKept = this.robot.updateRobotGeometry();

            // Update DH table
            document.getElementById('dh-d1').textContent = L1;
            document.getElementById('dh-a2').textContent = L2;
            document.getElementById('dh-a3').textContent = L3;

            // Saved positions show coordinates computed with the link lengths
            this.renderSavedPositions();

            if (poseKept) {
                this.showToast('success', 'Parámetros actualizados', 'Geometría del robot actualizada');
            } else {
                this.showToast('warning', 'Parámetros actualizados',
                    'La pose anterior quedaba por debajo del suelo con las nuevas longitudes: brazo en Home');
            }
        });

        // Motor resolution
        document.getElementById('motor-steps').addEventListener('input', () => this.updateMotorStepsDisplay());
        document.getElementById('motor-microstepping').addEventListener('change', () => this.updateMotorStepsDisplay());

        // Animation speed
        document.getElementById('anim-speed').addEventListener('input', (e) => {
            this.animationSpeed = parseFloat(e.target.value);
            document.getElementById('anim-speed-value').textContent = `${this.animationSpeed.toFixed(1)}x`;
        });

        // Visual settings
        document.getElementById('show-axes').addEventListener('change', (e) => {
            if (this.robot) this.robot.toggleAxes(e.target.checked);
        });

        document.getElementById('show-grid').addEventListener('change', (e) => {
            if (this.robot) this.robot.toggleGrid(e.target.checked);
        });

        document.getElementById('show-trajectory').addEventListener('change', (e) => {
            if (this.robot) this.robot.toggleTrajectory(e.target.checked);
        });

        document.getElementById('dark-mode').addEventListener('change', (e) => {
            // Dark mode is default, could toggle light mode here
            document.body.classList.toggle('light-mode', !e.target.checked);
        });
    }

    /**
     * Preset position buttons
     */
    bindPresetButtons() {
        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const preset = btn.dataset.preset;
                if (preset === 'custom') {
                    this.openSavePositionDialog();
                } else {
                    this.goToPreset(preset);
                }
            });
        });
    }

    /**
     * Go to preset position
     */
    goToPreset(presetName) {
        const angles = this.presets[presetName];
        if (angles && this.animateToPosition(angles)) {
            this.showToast('info', 'Posición', `Moviendo a: ${this.presetNames[presetName] || presetName}`);
        }
    }

    /**
     * Saved (named) positions panel
     */
    bindSavedPositions() {
        const list = document.getElementById('saved-positions-list');

        // One delegated handler for go / rename / delete on every list item
        list.addEventListener('click', (e) => {
            const item = e.target.closest('.saved-position');
            if (!item) return;
            const id = item.dataset.id;
            const actionBtn = e.target.closest('[data-action]');
            const action = actionBtn ? actionBtn.dataset.action : null;

            if (action === 'rename') {
                this.openRenamePositionDialog(id);
            } else if (action === 'delete') {
                this.confirmDeletePosition(id);
            } else if (e.target.closest('.saved-position-go')) {
                this.goToSavedPosition(id);
            }
        });

        document.getElementById('btn-export-positions').addEventListener('click', () => {
            this.exportSavedPositions();
        });

        const fileInput = document.getElementById('import-positions-file');
        document.getElementById('btn-import-positions').addEventListener('click', () => {
            fileInput.value = '';
            fileInput.click();
        });
        fileInput.addEventListener('change', () => {
            if (fileInput.files && fileInput.files[0]) {
                this.importSavedPositions(fileInput.files[0]);
            }
        });

        this.positionStore.onChange(() => this.renderSavedPositions());

        // Keep several open tabs in sync
        window.addEventListener('storage', (e) => {
            if (e.key === this.positionStore.storageKey) {
                this.positionStore.reload();
            }
        });

        this.renderSavedPositions();
    }

    /**
     * Rebuild the saved positions list
     */
    renderSavedPositions() {
        const list = document.getElementById('saved-positions-list');
        const empty = document.getElementById('saved-positions-empty');
        const count = document.getElementById('saved-positions-count');
        const exportBtn = document.getElementById('btn-export-positions');
        const positions = this.positionStore.list();

        list.replaceChildren();
        count.textContent = positions.length;
        empty.hidden = positions.length > 0;
        exportBtn.disabled = positions.length === 0;

        positions.forEach(position => {
            const item = document.createElement('li');
            item.className = 'saved-position';
            item.dataset.id = position.id;

            // Built with textContent: names are user input
            const goBtn = document.createElement('button');
            goBtn.type = 'button';
            goBtn.className = 'saved-position-go';
            goBtn.title = `Mover a "${position.name}"`;

            const name = document.createElement('span');
            name.className = 'saved-position-name';
            name.textContent = position.name;

            const angles = document.createElement('span');
            angles.className = 'saved-position-meta';
            angles.textContent = this.formatAngles(position.angles);

            goBtn.append(name, angles);

            const coords = this.formatCoordinates(position.angles);
            if (coords) {
                const xyz = document.createElement('span');
                xyz.className = 'saved-position-meta';
                xyz.textContent = coords;
                goBtn.append(xyz);
            }

            const actions = document.createElement('div');
            actions.className = 'saved-position-actions';
            actions.append(
                this.createIconButton('rename', 'fa-pen', `Renombrar "${position.name}"`),
                this.createIconButton('delete', 'fa-trash', `Eliminar "${position.name}"`)
            );

            item.append(goBtn, actions);
            list.append(item);
        });

        this.highlightActivePositions();
    }

    createIconButton(action, icon, label) {
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = `icon-btn${action === 'delete' ? ' danger' : ''}`;
        btn.dataset.action = action;
        btn.title = label;
        btn.setAttribute('aria-label', label);
        const i = document.createElement('i');
        i.className = `fas ${icon}`;
        btn.append(i);
        return btn;
    }

    formatAngles(angles) {
        return `θ ${angles.map(a => `${(Math.round(a * 10) / 10).toFixed(1)}°`).join(' · ')}`;
    }

    formatCoordinates(angles) {
        if (!this.robot || !this.robot.kinematics) return '';
        const p = this.robot.kinematics.forwardKinematics(angles).position;
        return `X ${p.x.toFixed(1)} · Y ${p.y.toFixed(1)} · Z ${p.z.toFixed(1)} mm`;
    }

    /**
     * Mark presets and saved positions that match the arm's current pose
     */
    highlightActivePositions() {
        if (!this.robot) return;
        const current = this.robot.getCurrentAngles();
        const matches = (angles) => angles.every((a, i) => Math.abs(a - current[i]) < 0.05);

        document.querySelectorAll('.preset-btn[data-preset]').forEach(btn => {
            const angles = this.presets[btn.dataset.preset];
            btn.classList.toggle('active', !!angles && matches(angles));
        });

        const positions = this.positionStore.list();
        document.querySelectorAll('#saved-positions-list .saved-position').forEach(item => {
            const position = positions.find(p => p.id === item.dataset.id);
            item.classList.toggle('active', !!position && matches(position.angles));
        });
    }

    /**
     * Ask for a name and save the current pose
     */
    async openSavePositionDialog() {
        if (!this.robot) return;
        this.cancelAnimation();
        const angles = this.robot.getCurrentAngles();

        const result = await this.showDialog({
            title: 'Guardar posición',
            message: 'Guarda la posición actual del brazo con un nombre.',
            input: {
                label: 'Nombre',
                value: this.suggestPositionName(),
                placeholder: 'Ej. Recoger pieza A'
            },
            preview: [this.formatAngles(angles), this.formatCoordinates(angles)],
            confirmLabel: 'Guardar',
            validate: (value) => {
                const check = this.positionStore.validateName(value);
                if (!check.ok) return { error: check.error };
                if (check.existing) {
                    return {
                        warning: `Ya existe "${check.existing.name}": se reemplazará con la posición actual.`,
                        confirmLabel: 'Reemplazar'
                    };
                }
                return {};
            }
        });
        if (!result.confirmed) return;

        try {
            const { position, overwritten, persisted } = this.positionStore.save(result.value, angles);
            this.showToast('success', overwritten ? 'Posición actualizada' : 'Posición guardada',
                `"${position.name}" · ${this.formatAngles(position.angles)}`);
            if (!persisted) this.warnNotPersisted();
        } catch (err) {
            this.showToast('error', 'No se pudo guardar', err.message);
        }
    }

    /**
     * First free "Posición N" name
     */
    suggestPositionName() {
        let n = this.positionStore.list().length + 1;
        while (this.positionStore.findByName(`Posición ${n}`)) n++;
        return `Posición ${n}`;
    }

    async openRenamePositionDialog(id) {
        const position = this.positionStore.get(id);
        if (!position) return;

        const result = await this.showDialog({
            title: 'Renombrar posición',
            input: { label: 'Nuevo nombre', value: position.name, placeholder: position.name },
            preview: [this.formatAngles(position.angles), this.formatCoordinates(position.angles)],
            confirmLabel: 'Renombrar',
            validate: (value) => {
                const check = this.positionStore.validateName(value, id);
                if (!check.ok) return { error: check.error };
                if (check.existing) return { error: `Ya existe una posición llamada "${check.existing.name}"` };
                return {};
            }
        });
        if (!result.confirmed) return;

        try {
            const { position: renamed, persisted } = this.positionStore.rename(id, result.value);
            this.showToast('success', 'Posición renombrada', `"${position.name}" → "${renamed.name}"`);
            if (!persisted) this.warnNotPersisted();
        } catch (err) {
            this.showToast('error', 'No se pudo renombrar', err.message);
        }
    }

    async confirmDeletePosition(id) {
        const position = this.positionStore.get(id);
        if (!position) return;

        const result = await this.showDialog({
            title: 'Eliminar posición',
            message: `¿Eliminar "${position.name}"? Esta acción no se puede deshacer.`,
            preview: [this.formatAngles(position.angles), this.formatCoordinates(position.angles)],
            confirmLabel: 'Eliminar',
            danger: true
        });
        if (!result.confirmed) return;

        if (this.positionStore.remove(id)) {
            this.showToast('info', 'Posición eliminada', `"${position.name}"`);
        }
    }

    goToSavedPosition(id) {
        const position = this.positionStore.get(id);
        if (!position) return;
        if (this.animateToPosition(position.angles)) {
            this.showToast('info', 'Posición', `Moviendo a: ${position.name}`);
        }
    }

    /**
     * Download all saved positions as a JSON file
     */
    exportSavedPositions() {
        const positions = this.positionStore.list();
        if (positions.length === 0) {
            this.showToast('warning', 'Nada que exportar', 'Aún no hay posiciones guardadas');
            return;
        }

        const json = this.positionStore.exportJSON({
            motor: this.getMotorSettings(),
            stepsFor: angles => this.getMotorSteps(angles)
        });
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        link.href = url;
        link.download = `roboarm-posiciones-${new Date().toISOString().slice(0, 10)}.json`;
        document.body.append(link);
        link.click();
        link.remove();
        setTimeout(() => URL.revokeObjectURL(url), 1000);

        this.showToast('success', 'Posiciones exportadas', `${positions.length} posición(es) en ${link.download}`);
    }

    /**
     * Merge positions from a JSON file exported earlier
     */
    async importSavedPositions(file) {
        try {
            const text = await file.text();
            const { added, updated, skipped, persisted } = this.positionStore.importJSON(text);

            if (added === 0 && updated === 0) {
                this.showToast('warning', 'Nada importado',
                    skipped ? `${skipped} entrada(s) no válidas` : 'El archivo no contiene posiciones');
                return;
            }

            const parts = [];
            if (added) parts.push(`${added} nueva(s)`);
            if (updated) parts.push(`${updated} actualizada(s)`);
            if (skipped) parts.push(`${skipped} ignorada(s)`);
            this.showToast('success', 'Posiciones importadas', parts.join(' · '));
            if (!persisted) this.warnNotPersisted();
        } catch (err) {
            this.showToast('error', 'No se pudo importar', err.message);
        }
    }

    warnNotPersisted() {
        this.showToast('warning', 'Solo en esta sesión',
            'El navegador no permite guardar datos: las posiciones se perderán al recargar', 6000);
    }

    /**
     * Modal dialog (save / rename / confirm)
     */
    bindDialog() {
        const backdrop = document.getElementById('dialog-backdrop');
        const form = document.getElementById('dialog-form');
        const input = document.getElementById('dialog-input');
        const cancelBtn = document.getElementById('dialog-cancel');

        form.addEventListener('submit', (e) => {
            e.preventDefault();
            this.closeDialog(true);
        });
        cancelBtn.addEventListener('click', () => this.closeDialog(false));
        backdrop.addEventListener('mousedown', (e) => {
            if (e.target === backdrop) this.closeDialog(false);
        });
        // Document level: works even if focus left the dialog (e.g. a click on its padding)
        document.addEventListener('keydown', (e) => {
            if (!this.dialog) return;
            if (e.key === 'Escape') {
                e.preventDefault();
                this.closeDialog(false);
            } else if (e.key === 'Tab') {
                this.trapDialogFocus(e);
            }
        });
        input.addEventListener('input', () => this.validateDialog());
    }

    /**
     * Open the modal dialog
     * @param {Object} options - {title, message, input:{label,value,placeholder}, preview:[lines], confirmLabel, danger, validate(value) -> {error, warning, confirmLabel}}
     * @returns {Promise<{confirmed: boolean, value: string}>}
     */
    showDialog(options) {
        // Only one dialog at a time
        if (this.dialog) this.closeDialog(false);

        const backdrop = document.getElementById('dialog-backdrop');
        const inputGroup = document.getElementById('dialog-input-group');
        const input = document.getElementById('dialog-input');
        const message = document.getElementById('dialog-message');
        const preview = document.getElementById('dialog-preview');
        const confirmBtn = document.getElementById('dialog-confirm');

        document.getElementById('dialog-title').textContent = options.title;
        message.textContent = options.message || '';
        message.hidden = !options.message;

        inputGroup.hidden = !options.input;
        if (options.input) {
            document.getElementById('dialog-input-label').textContent = options.input.label || 'Nombre';
            input.value = options.input.value || '';
            input.placeholder = options.input.placeholder || '';
            input.maxLength = SavedPositionsStore.MAX_NAME_LENGTH;
        }

        const lines = (options.preview || []).filter(Boolean);
        preview.replaceChildren(...lines.map(text => {
            const line = document.createElement('div');
            line.textContent = text;
            return line;
        }));
        preview.hidden = lines.length === 0;

        confirmBtn.classList.toggle('danger', !!options.danger);

        return new Promise(resolve => {
            this.dialog = {
                options,
                resolve,
                returnFocus: document.activeElement
            };
            this.validateDialog();
            backdrop.hidden = false;

            requestAnimationFrame(() => {
                if (options.input) {
                    input.focus();
                    input.select();
                } else {
                    confirmBtn.focus();
                }
            });
        });
    }

    /**
     * Run the dialog's validator and update hint/confirm button
     * @returns {boolean} True if the dialog can be confirmed
     */
    validateDialog() {
        if (!this.dialog) return false;
        const { options } = this.dialog;
        const hint = document.getElementById('dialog-hint');
        const confirmBtn = document.getElementById('dialog-confirm');
        const value = document.getElementById('dialog-input').value;

        const state = options.validate ? (options.validate(value) || {}) : {};
        hint.textContent = state.error || state.warning || '';
        hint.className = `dialog-hint${state.error ? ' error' : state.warning ? ' warning' : ''}`;
        confirmBtn.textContent = state.confirmLabel || options.confirmLabel || 'Aceptar';
        confirmBtn.disabled = !!state.error;
        return !state.error;
    }

    closeDialog(confirmed) {
        if (!this.dialog) return;
        if (confirmed && !this.validateDialog()) return;

        const { resolve, returnFocus } = this.dialog;
        const value = document.getElementById('dialog-input').value;
        this.dialog = null;
        document.getElementById('dialog-backdrop').hidden = true;

        if (returnFocus && typeof returnFocus.focus === 'function' && document.contains(returnFocus)) {
            returnFocus.focus();
        }
        resolve({ confirmed, value });
    }

    /**
     * Keep Tab focus inside the open dialog
     */
    trapDialogFocus(e) {
        const focusable = [...document.querySelectorAll('#dialog-backdrop .dialog input, #dialog-backdrop .dialog button')]
            .filter(el => !el.disabled && el.offsetParent !== null);
        if (focusable.length === 0) return;

        const first = focusable[0];
        const last = focusable[focusable.length - 1];
        if (!focusable.includes(document.activeElement)) {
            // Focus is outside the dialog: bring it back in
            e.preventDefault();
            (e.shiftKey ? last : first).focus();
        } else if (e.shiftKey && document.activeElement === first) {
            e.preventDefault();
            last.focus();
        } else if (!e.shiftKey && document.activeElement === last) {
            e.preventDefault();
            first.focus();
        }
    }

    /**
     * Update all displays with current robot state
     */
    updateAllDisplays() {
        this.updateKinematicsDisplay();
        this.updatePositionDisplay();
    }

    /**
     * Update forward kinematics display
     */
    updateKinematicsDisplay() {
        if (this.robot && this.robot.kinematics) {
            const angles = this.robot.getCurrentAngles();
            const fk = this.robot.kinematics.forwardKinematics(angles);

            document.getElementById('fk-x').textContent = `${fk.position.x.toFixed(2)} mm`;
            document.getElementById('fk-y').textContent = `${fk.position.y.toFixed(2)} mm`;
            document.getElementById('fk-z').textContent = `${fk.position.z.toFixed(2)} mm`;

            // Update DH theta values
            document.getElementById('dh-theta1').textContent = `${angles[0].toFixed(1)}°`;
            document.getElementById('dh-theta2').textContent = `${angles[1].toFixed(1)}°`;
            document.getElementById('dh-theta3').textContent = `${angles[2].toFixed(1)}°`;

            this.updateMotorStepsDisplay();
        }
    }

    /**
     * Stepper resolution from the settings (invalid input falls back to NEMA 17 defaults)
     * @returns {Object} {stepsPerRev, microstepping}
     */
    getMotorSettings() {
        const stepsInput = document.getElementById('motor-steps');
        let stepsPerRev = parseInt(stepsInput.value, 10);
        if (!Number.isFinite(stepsPerRev) || stepsPerRev < parseInt(stepsInput.min, 10) ||
            stepsPerRev > parseInt(stepsInput.max, 10)) {
            stepsPerRev = 200;
        }
        const microstepping = parseInt(document.getElementById('motor-microstepping').value, 10) || 16;
        return { stepsPerRev, microstepping };
    }

    /**
     * Motor positions in steps from Home for the given (or current) angles
     */
    getMotorSteps(angles) {
        const { stepsPerRev, microstepping } = this.getMotorSettings();
        return angles.map(angle => this.robot.kinematics.calculateMotorSteps(angle, stepsPerRev, microstepping));
    }

    updateMotorStepsDisplay() {
        if (!this.robot || !this.robot.kinematics) return;
        const { stepsPerRev, microstepping } = this.getMotorSettings();
        const steps = this.getMotorSteps(this.robot.getCurrentAngles());

        steps.forEach((value, i) => {
            document.getElementById(`steps-joint${i + 1}`).textContent = `${value} pasos`;
        });
        const perDegree = stepsPerRev * microstepping / 360;
        document.getElementById('steps-resolution').textContent =
            `${(stepsPerRev * microstepping).toLocaleString('es')} pasos/rev · ${perDegree.toFixed(2)} pasos/°`;
    }

    /**
     * Update position display overlay
     */
    updatePositionDisplay() {
        if (this.robot && this.robot.kinematics) {
            const angles = this.robot.getCurrentAngles();
            const fk = this.robot.kinematics.forwardKinematics(angles);

            document.getElementById('pos-x').textContent = fk.position.x.toFixed(2);
            document.getElementById('pos-y').textContent = fk.position.y.toFixed(2);
            document.getElementById('pos-z').textContent = fk.position.z.toFixed(2);

            this.highlightActivePositions();
        }
    }

    /**
     * Set joint angle from external source
     */
    setJointAngle(jointIndex, angle) {
        const control = document.querySelectorAll('.joint-control')[jointIndex];
        if (control) {
            const slider = control.querySelector('.joint-slider');
            const input = control.querySelector('.angle-input');
            const sliderFill = control.querySelector('.slider-fill');

            slider.value = angle;
            input.value = Math.round(angle);
            this.updateSliderFill(slider, sliderFill);
        }
    }

    /**
     * Show toast notification
     */
    showToast(type, title, message, duration = 3000) {
        const container = document.getElementById('toast-container');

        const icons = {
            success: 'fa-check-circle',
            error: 'fa-times-circle',
            warning: 'fa-exclamation-triangle',
            info: 'fa-info-circle'
        };

        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        toast.innerHTML = `
            <i class="toast-icon fas ${icons[type]}"></i>
            <div class="toast-content">
                <div class="toast-title"></div>
                <div class="toast-message"></div>
            </div>
            <button class="toast-close" aria-label="Cerrar"><i class="fas fa-times"></i></button>
        `;
        // textContent: messages can contain user-entered position names
        toast.querySelector('.toast-title').textContent = title;
        toast.querySelector('.toast-message').textContent = message;

        container.appendChild(toast);

        // Close button
        toast.querySelector('.toast-close').addEventListener('click', () => {
            toast.remove();
        });

        // Auto remove
        setTimeout(() => {
            toast.style.animation = 'slideIn 0.25s ease reverse';
            setTimeout(() => toast.remove(), 250);
        }, duration);
    }

    /**
     * Update status message
     */
    setStatusMessage(message) {
        document.getElementById('status-message').textContent = message;
    }

    /**
     * Update FPS counter
     */
    updateFPS(fps) {
        document.getElementById('fps-counter').textContent = `${Math.round(fps)} FPS`;
    }
}

// Export
window.UIController = UIController;
