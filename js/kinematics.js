/**
 * =====================================================
 * ROBOTIC ARM KINEMATICS MODULE
 * 3 DOF Arm Kinematics (Base, Shoulder, Elbow)
 * =====================================================
 *
 * This module implements forward and inverse kinematics
 * for a 3-DOF robotic arm using geometric approach.
 *
 * Joint Configuration:
 * - Joint 1 (Base): Rotation around Z-axis (vertical)
 * - Joint 2 (Shoulder): Rotation around Y-axis (pitch)
 * - Joint 3 (Elbow): Rotation around Y-axis (pitch)
 *
 * Coordinate System:
 * - X: Forward (when θ1=0)
 * - Y: Left
 * - Z: Up (vertical)
 *
 * Home Position (all angles = 0):
 * - Robot arm extended horizontally along +X axis
 * - End effector at (L2+L3, 0, L1)
 */

class RobotKinematics {
    constructor(config = {}) {
        // Robot arm link lengths (in mm)
        this.L1 = config.L1 || 50;   // Base height
        this.L2 = config.L2 || 100;  // Upper arm length
        this.L3 = config.L3 || 80;   // Forearm length

        // Joint limits (in degrees)
        this.jointLimits = {
            theta1: { min: -180, max: 180 },  // Base rotation
            theta2: { min: -90, max: 90 },    // Shoulder
            theta3: { min: -135, max: 135 }   // Elbow
        };

        // Current joint angles (in degrees)
        this.currentAngles = [0, 0, 0];
    }

    /**
     * Convert degrees to radians
     */
    degToRad(degrees) {
        return degrees * (Math.PI / 180);
    }

    /**
     * Convert radians to degrees
     */
    radToDeg(radians) {
        return radians * (180 / Math.PI);
    }

    /**
     * Update robot link lengths
     */
    updateLinkLengths(L1, L2, L3) {
        this.L1 = L1;
        this.L2 = L2;
        this.L3 = L3;
    }

    /**
     * Forward Kinematics - Calculate end effector position from joint angles
     * Uses geometric approach for clarity and correctness
     *
     * @param {Array} angles - Joint angles in degrees [theta1, theta2, theta3]
     * @returns {Object} End effector position {x, y, z} and joint positions
     *
     * Geometry:
     * - θ1: Base rotation around Z axis
     * - θ2: Shoulder angle from horizontal (positive = up)
     * - θ3: Elbow angle relative to upper arm (positive = up/fold)
     */
    forwardKinematics(angles) {
        // Convert to radians
        const theta1 = this.degToRad(angles[0]);
        const theta2 = this.degToRad(angles[1]);
        const theta3 = this.degToRad(angles[2]);

        // Store current angles
        this.currentAngles = [...angles];

        // Calculate positions using geometric approach
        // Shoulder position (fixed relative to base rotation)
        const shoulderX = 0;
        const shoulderY = 0;
        const shoulderZ = this.L1;

        // Elbow position
        // The upper arm (L2) rotates by theta2 from horizontal in the vertical plane
        // that is rotated by theta1 around Z
        const elbowLocalX = this.L2 * Math.cos(theta2);  // Horizontal projection
        const elbowLocalZ = this.L2 * Math.sin(theta2);  // Vertical projection

        // Rotate by theta1 around Z axis
        const elbowX = elbowLocalX * Math.cos(theta1);
        const elbowY = elbowLocalX * Math.sin(theta1);
        const elbowZ = shoulderZ + elbowLocalZ;

        // End effector position
        // The forearm (L3) rotates by (theta2 + theta3) from horizontal
        const totalArmAngle = theta2 + theta3;
        const endLocalX = this.L3 * Math.cos(totalArmAngle);  // Horizontal projection
        const endLocalZ = this.L3 * Math.sin(totalArmAngle);  // Vertical projection

        // Add to elbow position (rotated by theta1)
        const endX = elbowX + endLocalX * Math.cos(theta1);
        const endY = elbowY + endLocalX * Math.sin(theta1);
        const endZ = elbowZ + endLocalZ;

        // Create positions object
        const positions = {
            base: { x: 0, y: 0, z: 0 },
            shoulder: { x: shoulderX, y: shoulderY, z: shoulderZ },
            elbow: { x: elbowX, y: elbowY, z: elbowZ },
            endEffector: { x: endX, y: endY, z: endZ }
        };

        return {
            position: positions.endEffector,
            positions: positions,
            angles: angles
        };
    }

    /**
     * Inverse Kinematics - Calculate joint angles from target position
     * Uses geometric approach for 3-DOF arm
     *
     * @param {number} x - Target X position (mm)
     * @param {number} y - Target Y position (mm)
     * @param {number} z - Target Z position (mm)
     * @returns {Object} Result with angles and validity status
     */
    inverseKinematics(x, y, z) {
        const result = {
            valid: false,
            angles: [0, 0, 0],
            error: null,
            solutions: []
        };

        try {
            // Handle special case: target at origin XY plane
            if (Math.abs(x) < 0.001 && Math.abs(y) < 0.001) {
                // Target is on Z axis, theta1 is undefined - use current value
                result.error = "Objetivo en el eje Z - θ₁ indefinido";
                return result;
            }

            // Joint 1: Base rotation
            // theta1 = atan2(y, x)
            const theta1 = Math.atan2(y, x);

            // Calculate the position in the arm plane (after theta1 rotation)
            // r is the horizontal distance from the shoulder
            const r = Math.sqrt(x * x + y * y);

            // Height relative to shoulder
            const zRel = z - this.L1;

            // Distance from shoulder to target in the arm plane
            const D = Math.sqrt(r * r + zRel * zRel);

            // Check reachability
            const maxReach = this.L2 + this.L3;
            const minReach = Math.abs(this.L2 - this.L3);

            if (D > maxReach) {
                result.error = `Fuera de alcance. Distancia: ${D.toFixed(1)}mm > Max: ${maxReach.toFixed(1)}mm`;
                return result;
            }

            if (D < minReach) {
                result.error = `Muy cercano. Distancia: ${D.toFixed(1)}mm < Min: ${minReach.toFixed(1)}mm`;
                return result;
            }

            // Law of cosines to find elbow angle (theta3)
            // D² = L2² + L3² + 2·L2·L3·cos(θ3)  (note: +2 because θ3=0 means straight)
            // Rearranging: cos(θ3) = (D² - L2² - L3²) / (2·L2·L3)
            const cosTheta3 = (D * D - this.L2 * this.L2 - this.L3 * this.L3) /
                             (2 * this.L2 * this.L3);

            // Clamp to valid range due to floating point errors
            const cosTheta3Clamped = Math.max(-1, Math.min(1, cosTheta3));

            // Two solutions: elbow up (negative θ3) and elbow down (positive θ3)
            const theta3_elbowDown = Math.acos(cosTheta3Clamped);
            const theta3_elbowUp = -theta3_elbowDown;

            // Calculate theta2 for each solution
            // Using the angle to the target and the internal angle of the arm triangle
            const phi = Math.atan2(zRel, r);  // Angle from horizontal to target

            // For elbow down configuration
            const psi1 = Math.atan2(
                this.L3 * Math.sin(theta3_elbowDown),
                this.L2 + this.L3 * Math.cos(theta3_elbowDown)
            );
            const theta2_elbowDown = phi - psi1;

            // For elbow up configuration
            const psi2 = Math.atan2(
                this.L3 * Math.sin(theta3_elbowUp),
                this.L2 + this.L3 * Math.cos(theta3_elbowUp)
            );
            const theta2_elbowUp = phi - psi2;

            // Convert to degrees
            const solutions = [
                {
                    theta1: this.radToDeg(theta1),
                    theta2: this.radToDeg(theta2_elbowDown),
                    theta3: this.radToDeg(theta3_elbowDown),
                    config: 'elbow_down'
                },
                {
                    theta1: this.radToDeg(theta1),
                    theta2: this.radToDeg(theta2_elbowUp),
                    theta3: this.radToDeg(theta3_elbowUp),
                    config: 'elbow_up'
                }
            ];

            // Filter solutions by joint limits
            const validSolutions = solutions.filter(sol => {
                return this.isWithinLimits(sol.theta1, sol.theta2, sol.theta3);
            });

            if (validSolutions.length === 0) {
                result.error = "Sin soluciones dentro de límites articulares";
                return result;
            }

            // Select best solution (closest to current configuration)
            let bestSolution = validSolutions[0];

            if (validSolutions.length > 1) {
                let minDist = Infinity;
                for (const sol of validSolutions) {
                    const dist = Math.sqrt(
                        Math.pow(sol.theta1 - this.currentAngles[0], 2) +
                        Math.pow(sol.theta2 - this.currentAngles[1], 2) +
                        Math.pow(sol.theta3 - this.currentAngles[2], 2)
                    );
                    if (dist < minDist) {
                        minDist = dist;
                        bestSolution = sol;
                    }
                }
            }

            result.valid = true;
            result.angles = [bestSolution.theta1, bestSolution.theta2, bestSolution.theta3];
            result.solutions = validSolutions;
            result.configuration = bestSolution.config;

        } catch (err) {
            result.error = `Error en cálculo: ${err.message}`;
        }

        return result;
    }

    /**
     * Check if angles are within joint limits
     */
    isWithinLimits(theta1, theta2, theta3) {
        return (
            theta1 >= this.jointLimits.theta1.min &&
            theta1 <= this.jointLimits.theta1.max &&
            theta2 >= this.jointLimits.theta2.min &&
            theta2 <= this.jointLimits.theta2.max &&
            theta3 >= this.jointLimits.theta3.min &&
            theta3 <= this.jointLimits.theta3.max
        );
    }

    /**
     * Verify forward/inverse kinematics consistency
     * @param {Array} angles - Test angles [theta1, theta2, theta3]
     * @returns {Object} Verification result
     */
    verifyKinematics(angles) {
        const fk = this.forwardKinematics(angles);
        const ik = this.inverseKinematics(fk.position.x, fk.position.y, fk.position.z);

        const error = ik.valid ? {
            theta1: Math.abs(angles[0] - ik.angles[0]),
            theta2: Math.abs(angles[1] - ik.angles[1]),
            theta3: Math.abs(angles[2] - ik.angles[2])
        } : null;

        return {
            inputAngles: angles,
            fkPosition: fk.position,
            ikAngles: ik.angles,
            ikValid: ik.valid,
            error: error
        };
    }

    /**
     * Calculate motor steps for given angle change
     * @param {number} angleDelta - Angle change in degrees
     * @param {number} stepsPerRev - Steps per revolution (default 200 for NEMA 17)
     * @param {number} microstepping - Microstepping factor (default 16)
     * @param {number} gearRatio - Gear reduction ratio (default 1)
     * @returns {number} Number of motor steps
     */
    calculateMotorSteps(angleDelta, stepsPerRev = 200, microstepping = 16, gearRatio = 1) {
        const totalSteps = stepsPerRev * microstepping * gearRatio;
        const stepsPerDegree = totalSteps / 360;
        return Math.round(angleDelta * stepsPerDegree);
    }

    /**
     * Generate trajectory between two positions
     * @param {Array} startAngles - Start joint angles [theta1, theta2, theta3]
     * @param {Array} endAngles - End joint angles [theta1, theta2, theta3]
     * @param {number} steps - Number of interpolation steps
     * @returns {Array} Array of intermediate angle configurations
     */
    generateTrajectory(startAngles, endAngles, steps = 50) {
        const trajectory = [];

        for (let i = 0; i <= steps; i++) {
            const t = i / steps;
            // Smooth interpolation (ease-in-out)
            const smoothT = t < 0.5
                ? 2 * t * t
                : 1 - Math.pow(-2 * t + 2, 2) / 2;

            const angles = startAngles.map((start, idx) => {
                return start + (endAngles[idx] - start) * smoothT;
            });

            trajectory.push({
                t: t,
                angles: angles,
                position: this.forwardKinematics(angles).position
            });
        }

        return trajectory;
    }

    /**
     * Get robot parameters for display
     */
    getParameters() {
        return {
            linkLengths: { L1: this.L1, L2: this.L2, L3: this.L3 },
            jointLimits: this.jointLimits,
            maxReach: this.L2 + this.L3,
            minReach: Math.abs(this.L2 - this.L3)
        };
    }

    /**
     * Get DH parameters for display (informational)
     */
    getDHParameters() {
        return [
            { link: 1, a: 0, alpha: 90, d: this.L1, theta: 'θ₁' },
            { link: 2, a: this.L2, alpha: 0, d: 0, theta: 'θ₂' },
            { link: 3, a: this.L3, alpha: 0, d: 0, theta: 'θ₃' }
        ];
    }
}

// Export for use in other modules
window.RobotKinematics = RobotKinematics;
