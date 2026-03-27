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
            place: [-45, 45, -30],
            custom: null
        };
        this.animationSpeed = 1.0;
        this.isAnimating = false;

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
        this.bindPresetButtons();
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

                // Clamp value
                value = Math.max(min, Math.min(max, value));
                e.target.value = value;
                slider.value = value;
                this.updateSliderFill(slider, sliderFill);
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
     * Animate joint to target angle
     */
    animateToAngle(jointIndex, targetAngle) {
        const control = document.querySelectorAll('.joint-control')[jointIndex];
        const slider = control.querySelector('.joint-slider');
        const input = control.querySelector('.angle-input');
        const sliderFill = control.querySelector('.slider-fill');

        const startAngle = parseFloat(slider.value);
        const duration = 500 / this.animationSpeed;
        const startTime = performance.now();

        const animate = (currentTime) => {
            const elapsed = currentTime - startTime;
            const progress = Math.min(elapsed / duration, 1);

            // Ease-out cubic
            const eased = 1 - Math.pow(1 - progress, 3);
            const currentAngle = startAngle + (targetAngle - startAngle) * eased;

            slider.value = currentAngle;
            input.value = Math.round(currentAngle);
            this.updateSliderFill(slider, sliderFill);
            this.updateJoint(jointIndex, currentAngle);

            if (progress < 1) {
                requestAnimationFrame(animate);
            }
        };

        requestAnimationFrame(animate);
    }

    /**
     * Header action buttons
     */
    bindHeaderActions() {
        // Reset button
        document.getElementById('btn-reset').addEventListener('click', () => {
            this.resetAllJoints();
            this.showToast('info', 'Posición reseteada', 'Todos los ángulos en 0°');
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
        [0, 1, 2].forEach(index => {
            this.animateToAngle(index, 0);
        });
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

                statusEl.className = 'ik-status success';
                statusEl.innerHTML = `<i class="fas fa-check-circle"></i> Solución encontrada (${result.configuration})`;

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
                statusEl.innerHTML = `<i class="fas fa-times-circle"></i> ${result.error}`;

                applyBtn.disabled = true;

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
            this.animateToPosition(angles);
            this.showToast('success', 'Posición aplicada', 'Moviendo a posición calculada');
        }
    }

    /**
     * Animate robot to target angles
     */
    animateToPosition(targetAngles) {
        if (this.isAnimating) return;
        this.isAnimating = true;

        const controls = document.querySelectorAll('.joint-control');
        const startAngles = [];

        controls.forEach((control, i) => {
            startAngles.push(parseFloat(control.querySelector('.joint-slider').value));
        });

        const duration = 1000 / this.animationSpeed;
        const startTime = performance.now();

        const animate = (currentTime) => {
            const elapsed = currentTime - startTime;
            const progress = Math.min(elapsed / duration, 1);

            // Smooth step function
            const eased = progress < 0.5
                ? 4 * progress * progress * progress
                : 1 - Math.pow(-2 * progress + 2, 3) / 2;

            controls.forEach((control, i) => {
                const slider = control.querySelector('.joint-slider');
                const input = control.querySelector('.angle-input');
                const sliderFill = control.querySelector('.slider-fill');

                const currentAngle = startAngles[i] + (targetAngles[i] - startAngles[i]) * eased;

                slider.value = currentAngle;
                input.value = Math.round(currentAngle);
                this.updateSliderFill(slider, sliderFill);
                this.updateJoint(i, currentAngle);
            });

            if (progress < 1) {
                requestAnimationFrame(animate);
            } else {
                this.isAnimating = false;
                if (this.robot) {
                    this.robot.hideTargetMarker();
                }
            }
        };

        requestAnimationFrame(animate);
    }

    /**
     * Settings controls
     */
    bindSettingsControls() {
        // Robot parameters
        document.getElementById('btn-apply-params').addEventListener('click', () => {
            const L1 = parseFloat(document.getElementById('param-l1').value);
            const L2 = parseFloat(document.getElementById('param-l2').value);
            const L3 = parseFloat(document.getElementById('param-l3').value);

            if (this.robot && this.robot.kinematics) {
                this.robot.kinematics.updateLinkLengths(L1, L2, L3);
                this.robot.updateRobotGeometry();

                // Update DH table
                document.getElementById('dh-d1').textContent = L1;
                document.getElementById('dh-a2').textContent = L2;
                document.getElementById('dh-a3').textContent = L3;

                this.showToast('success', 'Parámetros actualizados', 'Geometría del robot actualizada');
            }
        });

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
                    this.saveCustomPreset();
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
        if (angles) {
            this.animateToPosition(angles);
            this.showToast('info', 'Posición', `Moviendo a: ${presetName}`);
        }
    }

    /**
     * Save current position as custom preset
     */
    saveCustomPreset() {
        const controls = document.querySelectorAll('.joint-control');
        const angles = [];

        controls.forEach(control => {
            angles.push(parseFloat(control.querySelector('.joint-slider').value));
        });

        this.presets.custom = angles;
        this.showToast('success', 'Posición guardada', `Ángulos: [${angles.map(a => a.toFixed(0)).join('°, ')}°]`);
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
        }
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
                <div class="toast-title">${title}</div>
                <div class="toast-message">${message}</div>
            </div>
            <button class="toast-close"><i class="fas fa-times"></i></button>
        `;

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
