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
        const lengths = [L1, L2, L3];
        if (!lengths.every(L => Number.isFinite(L) && L > 0)) {
            throw new Error('Las longitudes deben ser números mayores que 0');
        }
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
     * Up to four solutions exist: the arm can lean toward the target (front) or
     * reach back over the base (back, θ1 + 180°), each with elbow up or down.
     * Solutions outside the joint limits or below the floor are discarded, and the
     * one closest to the reference configuration is returned.
     *
     * @param {number} x - Target X position (mm)
     * @param {number} y - Target Y position (mm)
     * @param {number} z - Target Z position (mm)
     * @param {Array} [referenceAngles] - Configuration to stay close to (defaults to currentAngles)
     * @returns {Object} Result with angles and validity status
     */
    inverseKinematics(x, y, z, referenceAngles = this.currentAngles) {
        const result = {
            valid: false,
            angles: [0, 0, 0],
            error: null,
            solutions: []
        };

        if (![x, y, z].every(Number.isFinite)) {
            result.error = 'Coordenadas inválidas';
            return result;
        }

        const reference = Array.isArray(referenceAngles) && referenceAngles.every(Number.isFinite)
            ? referenceAngles
            : [0, 0, 0];

        // r is the horizontal distance from the base axis
        const r = Math.sqrt(x * x + y * y);

        // Height relative to shoulder
        const zRel = z - this.L1;

        // Distance from shoulder to target in the arm plane
        const D = Math.sqrt(r * r + zRel * zRel);

        // Check reachability
        const maxReach = this.L2 + this.L3;
        const minReach = this.getMinReach();

        if (D > maxReach + 1e-9) {
            result.error = `Fuera de alcance. Distancia: ${D.toFixed(1)}mm > Max: ${maxReach.toFixed(1)}mm`;
            return result;
        }

        if (D < minReach - 1e-9) {
            result.error = `Muy cercano. Distancia: ${D.toFixed(1)}mm < Min: ${minReach.toFixed(1)}mm`;
            return result;
        }

        // Law of cosines to find elbow angle (theta3)
        // D² = L2² + L3² + 2·L2·L3·cos(θ3)  (note: +2 because θ3=0 means straight)
        // Rearranging: cos(θ3) = (D² - L2² - L3²) / (2·L2·L3)
        const cosTheta3 = (D * D - this.L2 * this.L2 - this.L3 * this.L3) /
                         (2 * this.L2 * this.L3);

        // Clamp to valid range due to floating point errors
        const theta3Abs = Math.acos(Math.max(-1, Math.min(1, cosTheta3)));

        // Joint 1: Base rotation. On the Z axis θ1 is free: keep the reference value.
        const theta1Front = r < 1e-9 ? this.degToRad(reference[0]) : Math.atan2(y, x);

        const candidates = [];
        for (const side of ['front', 'back']) {
            // Reaching back flips the base by 180° and mirrors the target in the arm plane
            const theta1 = side === 'front' ? theta1Front : theta1Front + Math.PI;
            const planarX = side === 'front' ? r : -r;
            const phi = Math.atan2(zRel, planarX);  // Angle from horizontal to target

            // Elbow down: positive θ3, elbow up: negative θ3
            for (const [elbow, theta3] of [['elbow_down', theta3Abs], ['elbow_up', -theta3Abs]]) {
                const psi = Math.atan2(
                    this.L3 * Math.sin(theta3),
                    this.L2 + this.L3 * Math.cos(theta3)
                );
                candidates.push({
                    theta1: this.normalizeAngle(this.radToDeg(theta1)),
                    theta2: this.normalizeAngle(this.radToDeg(phi - psi)),
                    theta3: this.radToDeg(theta3),
                    config: side === 'front' ? elbow : `${elbow}_back`
                });
            }
        }

        // Fully extended / fully folded: elbow up and down coincide
        const solutions = candidates.filter((sol, i) => !candidates.slice(0, i).some(other =>
            Math.abs(other.theta1 - sol.theta1) < 1e-9 &&
            Math.abs(other.theta2 - sol.theta2) < 1e-9 &&
            Math.abs(other.theta3 - sol.theta3) < 1e-9));

        // Filter solutions by joint limits
        const withinLimits = solutions.filter(sol => this.isWithinLimits(sol.theta1, sol.theta2, sol.theta3));
        if (withinLimits.length === 0) {
            result.error = "Sin soluciones dentro de límites articulares";
            return result;
        }

        // ...and by the floor: no part of the arm may go below Z = 0
        const validSolutions = withinLimits.filter(sol =>
            this.isConfigurationAboveFloor([sol.theta1, sol.theta2, sol.theta3]).valid);
        if (validSolutions.length === 0) {
            result.error = "Todas las soluciones dejan el brazo bajo el suelo";
            return result;
        }

        // Select best solution (closest to reference configuration). The base can't
        // wrap past ±180°, so plain differences are the real joint travel.
        let bestSolution = validSolutions[0];
        let minDist = Infinity;
        for (const sol of validSolutions) {
            const dist = Math.hypot(
                sol.theta1 - reference[0],
                sol.theta2 - reference[1],
                sol.theta3 - reference[2]
            );
            if (dist < minDist) {
                minDist = dist;
                bestSolution = sol;
            }
        }

        const limits = this.jointLimits;
        const clampToLimit = (value, limit) => Math.min(limit.max, Math.max(limit.min, value));

        result.valid = true;
        result.angles = [
            clampToLimit(bestSolution.theta1, limits.theta1),
            clampToLimit(bestSolution.theta2, limits.theta2),
            clampToLimit(bestSolution.theta3, limits.theta3)
        ];
        result.solutions = validSolutions;
        result.configuration = bestSolution.config;

        return result;
    }

    /**
     * Wrap an angle in degrees to (-180, 180]
     */
    normalizeAngle(degrees) {
        let a = degrees % 360;
        if (a > 180) a -= 360;
        if (a <= -180) a += 360;
        return a;
    }

    /**
     * Shortest shoulder-to-wrist distance the elbow limit allows
     * (the elbow can't fold all the way back, so this is more than |L2 - L3|)
     */
    getMinReach() {
        const maxFold = this.degToRad(Math.min(180, Math.max(
            Math.abs(this.jointLimits.theta3.min),
            Math.abs(this.jointLimits.theta3.max)
        )));
        return Math.sqrt(Math.max(0,
            this.L2 * this.L2 + this.L3 * this.L3 + 2 * this.L2 * this.L3 * Math.cos(maxFold)));
    }

    /**
     * Check if angles are within joint limits
     */
    isWithinLimits(theta1, theta2, theta3) {
        // Tiny tolerance: IK results land on a limit with floating-point noise
        const eps = 1e-9;
        return (
            theta1 >= this.jointLimits.theta1.min - eps &&
            theta1 <= this.jointLimits.theta1.max + eps &&
            theta2 >= this.jointLimits.theta2.min - eps &&
            theta2 <= this.jointLimits.theta2.max + eps &&
            theta3 >= this.jointLimits.theta3.min - eps &&
            theta3 <= this.jointLimits.theta3.max + eps
        );
    }

    /**
     * Check if a configuration keeps all arm parts above the floor (Z >= 0)
     * @param {Array} angles - Joint angles [theta1, theta2, theta3] in degrees
     * @returns {Object} {valid: boolean, elbowZ: number, endEffectorZ: number}
     */
    isConfigurationAboveFloor(angles) {
        // Heights only depend on θ2 and θ3. Computed directly (not via
        // forwardKinematics) so checking a candidate has no side effects.
        const theta2 = this.degToRad(angles[1]);
        const theta3 = this.degToRad(angles[2]);
        const elbowZ = this.L1 + this.L2 * Math.sin(theta2);
        const endZ = elbowZ + this.L3 * Math.sin(theta2 + theta3);

        // Both elbow and end effector must be at or above floor level.
        // The links are straight segments, so the whole arm is then above it.
        // (-1e-9: a joint exactly on the floor must not fail from rounding)
        const minFloorZ = -1e-9;

        return {
            valid: elbowZ >= minFloorZ && endZ >= minFloorZ,
            elbowZ: elbowZ,
            endEffectorZ: endZ
        };
    }

    /**
     * Check joint limits and the floor constraint together
     * @param {Array} angles - Joint angles [theta1, theta2, theta3] in degrees
     */
    isConfigurationValid(angles) {
        if (!Array.isArray(angles) || angles.length !== 3 || !angles.every(Number.isFinite)) {
            return false;
        }
        return this.isWithinLimits(angles[0], angles[1], angles[2]) &&
               this.isConfigurationAboveFloor(angles).valid;
    }

    /**
     * Shoulder angle above which the elbow is high enough that the forearm
     * cannot reach the floor, whatever the elbow angle is
     * (L1 + L2·sin(θ2) ≥ L3)
     */
    getSafeShoulderAngle() {
        const limits = this.jointLimits.theta2;
        const s = (this.L3 - this.L1) / this.L2;
        if (s <= -1) return limits.min;
        if (s >= 1) return limits.max;
        // 1° margin keeps the forearm clear of the floor plane
        return Math.min(limits.max, Math.max(limits.min, this.radToDeg(Math.asin(s)) + 1));
    }

    /**
     * Check that every configuration along a joint-space path stays above the floor
     * @param {Array} waypoints - Joint configurations, linearly interpolated between
     * @param {number} stepDeg - Sampling resolution in degrees
     */
    isPathAboveFloor(waypoints, stepDeg = 1) {
        for (let w = 0; w < waypoints.length - 1; w++) {
            const from = waypoints[w];
            const to = waypoints[w + 1];
            const maxDelta = Math.max(...from.map((a, i) => Math.abs(to[i] - a)));
            const samples = Math.max(1, Math.ceil(maxDelta / stepDeg));

            for (let s = 0; s <= samples; s++) {
                const t = s / samples;
                const angles = from.map((a, i) => a + (to[i] - a) * t);
                if (!this.isConfigurationAboveFloor(angles).valid) return false;
            }
        }
        return true;
    }

    /**
     * Plan a joint-space path from start to target that never goes below the floor
     *
     * Moving all joints at once can sweep the arm through the floor even when both
     * ends are valid. In that case the shoulder is raised first (which only raises
     * the arm), base and elbow move while the arm is high, then the shoulder lowers
     * onto the target.
     *
     * @param {Array} start - Current joint angles
     * @param {Array} target - Target joint angles
     * @returns {Array|null} Waypoints [start, ..., target], or null if the target is invalid/unreachable
     */
    planJointPath(start, target) {
        if (!this.isConfigurationValid(target)) return null;

        const direct = [[...start], [...target]];

        // Escaping an invalid start (e.g. after changing link lengths): go straight there
        if (!this.isConfigurationAboveFloor(start).valid) return direct;

        if (this.isPathAboveFloor(direct)) return direct;

        const candidates = [this.getSafeShoulderAngle(), this.jointLimits.theta2.max];
        for (const safe of candidates) {
            const lift = Math.max(start[1], target[1], safe);
            const path = [
                [...start],
                [start[0], lift, start[2]],
                [target[0], lift, target[2]],
                [...target]
            ].filter((waypoint, i, all) =>
                i === 0 || waypoint.some((a, j) => Math.abs(a - all[i - 1][j]) > 1e-9)
            );
            if (this.isPathAboveFloor(path)) return path;
        }
        return null;
    }

    /**
     * Clamp a target position to the reachable workspace
     *
     * Returns the target itself when the arm can reach it (inside the joint limits
     * and above the floor). Otherwise the target is projected onto the reachable
     * shell around the shoulder; if that point is still blocked (floor, joint limits),
     * the furthest reachable point on the way from `fromPoint` toward it is returned,
     * so a dragged arm follows the cursor as far as it physically can.
     *
     * @param {number} x - Target X coordinate (kinematics frame)
     * @param {number} y - Target Y coordinate (kinematics frame)
     * @param {number} z - Target Z coordinate (kinematics frame)
     * @param {Object} [fromPoint] - A reachable {x, y, z} to fall back toward (e.g. the current tip)
     * @returns {Object} Clamped position {x, y, z, wasClamped}
     */
    clampToWorkspace(x, y, z, fromPoint = null) {
        const anchor = fromPoint && this.isPointReachable(fromPoint.x, fromPoint.y, fromPoint.z)
            ? { x: fromPoint.x, y: fromPoint.y, z: fromPoint.z }
            : this.getAnchorPoint(x, y);

        if (![x, y, z].every(Number.isFinite)) {
            return { ...anchor, wasClamped: true };
        }

        if (this.isPointReachable(x, y, z)) {
            return { x, y, z, wasClamped: false };
        }

        let candidate = this.projectToReachShell(x, y, z, anchor);
        if (!this.isPointReachable(candidate.x, candidate.y, candidate.z)) {
            candidate = this.furthestReachablePoint(anchor, candidate);
        }
        return { ...candidate, wasClamped: true };
    }

    /**
     * True if some valid configuration (joint limits, floor) puts the tip at (x, y, z)
     */
    isPointReachable(x, y, z) {
        return this.inverseKinematics(x, y, z).valid;
    }

    /**
     * Project a point onto the shell minReach ≤ |p - shoulder| ≤ maxReach, at or above
     * the floor, keeping its direction around the base
     */
    projectToReachShell(x, y, z, anchor) {
        const maxReach = this.L2 + this.L3;
        const minReach = this.getMinReach();
        const margin = 1e-6;

        const r = Math.sqrt(x * x + y * y);
        // Direction around the base; on the Z axis use the anchor's
        const azimuth = r > 1e-9 ? Math.atan2(y, x) : Math.atan2(anchor.y, anchor.x);

        let planarR = r;
        let zRel = Math.max(z, 0) - this.L1;
        const D = Math.sqrt(planarR * planarR + zRel * zRel);

        if (D > maxReach) {
            // Pull toward the shoulder; the shoulder is above the floor, so z stays ≥ 0
            const scale = (maxReach - margin) / D;
            planarR *= scale;
            zRel *= scale;
        } else if (D < minReach) {
            if (D < 1e-9) {
                planarR = minReach + margin;
                zRel = 0;
            } else {
                const scale = (minReach + margin) / D;
                planarR *= scale;
                zRel *= scale;
            }
            if (zRel + this.L1 < 0) {
                // Pushed out through the floor: use the point of the inner shell at floor level
                zRel = -this.L1;
                planarR = Math.sqrt(Math.max(0, (minReach + margin) ** 2 - zRel * zRel));
            }
        }

        return {
            x: planarR * Math.cos(azimuth),
            y: planarR * Math.sin(azimuth),
            z: zRel + this.L1
        };
    }

    /**
     * A comfortably reachable point in the direction of (x, y): mid-reach at shoulder height
     */
    getAnchorPoint(x, y) {
        const azimuth = Math.hypot(x, y) > 1e-9 ? Math.atan2(y, x) : 0;
        const r = (this.getMinReach() + this.L2 + this.L3) / 2;
        return { x: r * Math.cos(azimuth), y: r * Math.sin(azimuth), z: this.L1 };
    }

    /**
     * Furthest reachable point on the segment from `from` (reachable) toward `to`
     */
    furthestReachablePoint(from, to) {
        const at = t => ({
            x: from.x + (to.x - from.x) * t,
            y: from.y + (to.y - from.y) * t,
            z: from.z + (to.z - from.z) * t
        });
        const reachable = t => {
            const p = at(t);
            return this.isPointReachable(p.x, p.y, p.z);
        };

        // Coarse scan from the target back toward the start...
        const steps = 32;
        let lo = 0;
        let hi = 1;
        for (let i = steps - 1; i >= 0; i--) {
            if (reachable(i / steps)) {
                lo = i / steps;
                hi = (i + 1) / steps;
                break;
            }
        }
        // ...then refine the boundary
        for (let i = 0; i < 20; i++) {
            const mid = (lo + hi) / 2;
            if (reachable(mid)) lo = mid;
            else hi = mid;
        }
        return at(lo);
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
            minReach: this.getMinReach()
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
