/**
 * =====================================================
 * ROBOTIC ARM KINEMATICS MODULE
 * 3 DOF Arm Kinematics (Base, Shoulder, Elbow)
 * =====================================================
 *
 * This module implements forward and inverse kinematics
 * for a 3-DOF robotic arm using Denavit-Hartenberg parameters.
 *
 * Joint Configuration:
 * - Joint 1 (Base): Rotation around Z-axis
 * - Joint 2 (Shoulder): Rotation around Y-axis
 * - Joint 3 (Elbow): Rotation around Y-axis
 */

class RobotKinematics {
    constructor(config = {}) {
        // Robot arm link lengths (in mm)
        this.L1 = config.L1 || 50;   // Base height (d1)
        this.L2 = config.L2 || 100;  // Upper arm length (a2)
        this.L3 = config.L3 || 80;   // Forearm length (a3)

        // Joint limits (in degrees)
        this.jointLimits = {
            theta1: { min: -180, max: 180 },  // Base rotation
            theta2: { min: -90, max: 90 },    // Shoulder
            theta3: { min: -135, max: 135 }   // Elbow
        };

        // Current joint angles (in radians)
        this.currentAngles = [0, 0, 0];

        // DH Parameters [a, alpha, d, theta]
        // theta is variable, others are constants
        this.dhParams = [
            { a: 0,       alpha: Math.PI/2, d: this.L1, theta: 0 },  // Joint 1
            { a: this.L2, alpha: 0,         d: 0,       theta: 0 },  // Joint 2
            { a: this.L3, alpha: 0,         d: 0,       theta: 0 }   // Joint 3
        ];
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
        this.dhParams[0].d = L1;
        this.dhParams[1].a = L2;
        this.dhParams[2].a = L3;
    }

    /**
     * Create a 4x4 Denavit-Hartenberg transformation matrix
     * @param {number} a - Link length
     * @param {number} alpha - Link twist
     * @param {number} d - Link offset
     * @param {number} theta - Joint angle
     * @returns {Array} 4x4 transformation matrix
     */
    dhMatrix(a, alpha, d, theta) {
        const ct = Math.cos(theta);
        const st = Math.sin(theta);
        const ca = Math.cos(alpha);
        const sa = Math.sin(alpha);

        return [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ]
        ];
    }

    /**
     * Multiply two 4x4 matrices
     */
    multiplyMatrices(A, B) {
        const result = [];
        for (let i = 0; i < 4; i++) {
            result[i] = [];
            for (let j = 0; j < 4; j++) {
                result[i][j] = 0;
                for (let k = 0; k < 4; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    /**
     * Forward Kinematics - Calculate end effector position from joint angles
     * @param {Array} angles - Joint angles in degrees [theta1, theta2, theta3]
     * @returns {Object} End effector position {x, y, z} and transformation matrices
     */
    forwardKinematics(angles) {
        // Convert to radians
        const theta1 = this.degToRad(angles[0]);
        const theta2 = this.degToRad(angles[1]);
        const theta3 = this.degToRad(angles[2]);

        // Store current angles
        this.currentAngles = [theta1, theta2, theta3];

        // Calculate individual transformation matrices
        const T01 = this.dhMatrix(
            this.dhParams[0].a,
            this.dhParams[0].alpha,
            this.dhParams[0].d,
            theta1
        );

        const T12 = this.dhMatrix(
            this.dhParams[1].a,
            this.dhParams[1].alpha,
            this.dhParams[1].d,
            theta2
        );

        const T23 = this.dhMatrix(
            this.dhParams[2].a,
            this.dhParams[2].alpha,
            this.dhParams[2].d,
            theta3
        );

        // Calculate cumulative transformations
        const T02 = this.multiplyMatrices(T01, T12);
        const T03 = this.multiplyMatrices(T02, T23);

        // Extract joint positions for visualization
        const positions = {
            base: { x: 0, y: 0, z: 0 },
            shoulder: { x: T01[0][3], y: T01[1][3], z: T01[2][3] },
            elbow: { x: T02[0][3], y: T02[1][3], z: T02[2][3] },
            endEffector: { x: T03[0][3], y: T03[1][3], z: T03[2][3] }
        };

        // Extract rotation matrix from T03
        const rotationMatrix = [
            [T03[0][0], T03[0][1], T03[0][2]],
            [T03[1][0], T03[1][1], T03[1][2]],
            [T03[2][0], T03[2][1], T03[2][2]]
        ];

        return {
            position: positions.endEffector,
            positions: positions,
            transformations: { T01, T02, T03 },
            rotationMatrix: rotationMatrix
        };
    }

    /**
     * Inverse Kinematics - Calculate joint angles from target position
     * Uses geometric approach for 3-DOF arm
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
            // Joint 1: Base rotation (theta1)
            // theta1 = atan2(y, x)
            const theta1 = Math.atan2(y, x);

            // Calculate the position in the plane of joints 2 and 3
            const r = Math.sqrt(x * x + y * y);  // Horizontal distance
            const zOffset = z - this.L1;         // Height above shoulder

            // Distance from shoulder to target in the arm plane
            const D = Math.sqrt(r * r + zOffset * zOffset);

            // Check if target is reachable
            const maxReach = this.L2 + this.L3;
            const minReach = Math.abs(this.L2 - this.L3);

            if (D > maxReach) {
                result.error = `Objetivo fuera de alcance. Distancia: ${D.toFixed(1)}mm, Alcance máximo: ${maxReach.toFixed(1)}mm`;
                // Provide closest reachable position
                const scale = maxReach / D * 0.95;
                return this.inverseKinematics(x * scale, y * scale, z);
            }

            if (D < minReach && D > 0.001) {
                result.error = `Objetivo muy cercano. Distancia: ${D.toFixed(1)}mm, Alcance mínimo: ${minReach.toFixed(1)}mm`;
                return result;
            }

            // Law of cosines to find theta3 (elbow angle)
            // D² = L2² + L3² - 2*L2*L3*cos(π - theta3)
            // cos(theta3) = (L2² + L3² - D²) / (2*L2*L3)
            const cosTheta3 = (this.L2 * this.L2 + this.L3 * this.L3 - D * D) /
                             (2 * this.L2 * this.L3);

            // Check for valid cosine value
            if (Math.abs(cosTheta3) > 1) {
                result.error = "Posición no alcanzable por límites articulares";
                return result;
            }

            // Two solutions: elbow up and elbow down
            const theta3_1 = Math.acos(cosTheta3);   // Elbow down
            const theta3_2 = -Math.acos(cosTheta3);  // Elbow up

            // Calculate theta2 for each solution
            // Using geometric relationships
            const alpha = Math.atan2(zOffset, r);  // Angle to target from horizontal

            // For theta3_1 (elbow down)
            const beta1 = Math.atan2(
                this.L3 * Math.sin(theta3_1),
                this.L2 + this.L3 * Math.cos(theta3_1)
            );
            const theta2_1 = alpha + beta1;

            // For theta3_2 (elbow up)
            const beta2 = Math.atan2(
                this.L3 * Math.sin(theta3_2),
                this.L2 + this.L3 * Math.cos(theta3_2)
            );
            const theta2_2 = alpha + beta2;

            // Convert to degrees
            const solutions = [
                {
                    theta1: this.radToDeg(theta1),
                    theta2: this.radToDeg(theta2_1),
                    theta3: this.radToDeg(theta3_1),
                    config: 'elbow_down'
                },
                {
                    theta1: this.radToDeg(theta1),
                    theta2: this.radToDeg(theta2_2),
                    theta3: this.radToDeg(theta3_2),
                    config: 'elbow_up'
                }
            ];

            // Filter solutions by joint limits
            const validSolutions = solutions.filter(sol => {
                return this.isWithinLimits(sol.theta1, sol.theta2, sol.theta3);
            });

            if (validSolutions.length === 0) {
                result.error = "No hay soluciones dentro de los límites articulares";
                return result;
            }

            // Select the best solution (prefer elbow down, or closest to current position)
            let bestSolution = validSolutions[0];

            // Prefer solution closest to current configuration
            if (validSolutions.length > 1) {
                const currentDeg = this.currentAngles.map(a => this.radToDeg(a));
                let minDist = Infinity;

                for (const sol of validSolutions) {
                    const dist = Math.sqrt(
                        Math.pow(sol.theta1 - currentDeg[0], 2) +
                        Math.pow(sol.theta2 - currentDeg[1], 2) +
                        Math.pow(sol.theta3 - currentDeg[2], 2)
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
            result.error = `Error en el cálculo: ${err.message}`;
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
     * Calculate workspace boundary points
     * @returns {Array} Array of {x, y, z} points on workspace boundary
     */
    calculateWorkspace() {
        const points = [];
        const angleStep = 10; // degrees

        for (let t1 = this.jointLimits.theta1.min; t1 <= this.jointLimits.theta1.max; t1 += angleStep) {
            for (let t2 = this.jointLimits.theta2.min; t2 <= this.jointLimits.theta2.max; t2 += angleStep) {
                for (let t3 = this.jointLimits.theta3.min; t3 <= this.jointLimits.theta3.max; t3 += angleStep) {
                    const fk = this.forwardKinematics([t1, t2, t3]);
                    points.push(fk.position);
                }
            }
        }

        return points;
    }

    /**
     * Calculate Jacobian matrix for velocity kinematics
     * @param {Array} angles - Current joint angles in degrees
     * @returns {Array} 6x3 Jacobian matrix
     */
    calculateJacobian(angles) {
        const theta1 = this.degToRad(angles[0]);
        const theta2 = this.degToRad(angles[1]);
        const theta3 = this.degToRad(angles[2]);

        const c1 = Math.cos(theta1);
        const s1 = Math.sin(theta1);
        const c2 = Math.cos(theta2);
        const s2 = Math.sin(theta2);
        const c23 = Math.cos(theta2 + theta3);
        const s23 = Math.sin(theta2 + theta3);

        // Position of end effector
        const px = c1 * (this.L2 * c2 + this.L3 * c23);
        const py = s1 * (this.L2 * c2 + this.L3 * c23);

        // Linear velocity Jacobian (3x3)
        const Jv = [
            [-py, -c1 * (this.L2 * s2 + this.L3 * s23), -c1 * this.L3 * s23],
            [px,  -s1 * (this.L2 * s2 + this.L3 * s23), -s1 * this.L3 * s23],
            [0,    this.L2 * c2 + this.L3 * c23,         this.L3 * c23]
        ];

        // Angular velocity Jacobian (3x3)
        const Jw = [
            [0, s1, s1],
            [0, -c1, -c1],
            [1, 0, 0]
        ];

        return { Jv, Jw, full: [...Jv, ...Jw] };
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
            // Use smooth interpolation (ease-in-out)
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
            dhParams: this.dhParams,
            maxReach: this.L2 + this.L3,
            minReach: Math.abs(this.L2 - this.L3)
        };
    }
}

// Export for use in other modules
window.RobotKinematics = RobotKinematics;
