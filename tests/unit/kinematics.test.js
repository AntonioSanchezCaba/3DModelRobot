const test = require('node:test');
const assert = require('node:assert/strict');
const { loadBrowserScript } = require('../helpers/load-browser-script');

const RobotKinematics = loadBrowserScript('js/kinematics.js', 'RobotKinematics');

const close = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol;

/** Deterministic pseudo-random numbers so failures are reproducible */
function seededRandom(seed) {
    let s = seed >>> 0;
    return () => {
        s = (s * 1664525 + 1013904223) >>> 0;
        return s / 4294967296;
    };
}

function randomValidConfig(k, rand) {
    for (;;) {
        const angles = [rand() * 360 - 180, rand() * 180 - 90, rand() * 270 - 135];
        if (k.isConfigurationValid(angles)) return angles;
    }
}

test('home position is (L2+L3, 0, L1)', () => {
    const k = new RobotKinematics();
    const { position, positions } = k.forwardKinematics([0, 0, 0]);
    assert.ok(close(position.x, 180) && close(position.y, 0) && close(position.z, 50));
    assert.ok(close(positions.elbow.x, 100) && close(positions.elbow.z, 50));
});

test('forward kinematics matches hand-computed poses', () => {
    const k = new RobotKinematics();

    // Arm straight up
    let p = k.forwardKinematics([0, 90, 0]).position;
    assert.ok(close(p.x, 0) && close(p.z, 230));

    // Base rotated 90°: arm points along +Y
    p = k.forwardKinematics([90, 0, 0]).position;
    assert.ok(close(p.x, 0) && close(p.y, 180) && close(p.z, 50));

    // Elbow folded up 90°
    p = k.forwardKinematics([0, 0, 90]).position;
    assert.ok(close(p.x, 100) && close(p.z, 130));
});

test('IK(FK(q)) reaches the same point, and one solution equals q', () => {
    const k = new RobotKinematics();
    let checked = 0;

    for (let t1 = -170; t1 <= 170; t1 += 20) {
        for (let t2 = -30; t2 <= 90; t2 += 10) {
            for (let t3 = -130; t3 <= 130; t3 += 10) {
                const q = [t1, t2, t3];
                if (!k.isConfigurationValid(q)) continue;

                const target = k.forwardKinematics(q).position;
                if (Math.hypot(target.x, target.y) < 1e-3) continue;  // θ1 undefined on the Z axis

                const ik = k.inverseKinematics(target.x, target.y, target.z);
                assert.ok(ik.valid, `IK failed for ${q}: ${ik.error}`);

                const reached = k.forwardKinematics(ik.angles).position;
                assert.ok(
                    close(reached.x, target.x, 1e-6) && close(reached.y, target.y, 1e-6) && close(reached.z, target.z, 1e-6),
                    `IK of ${q} reaches ${JSON.stringify(reached)} instead of ${JSON.stringify(target)}`
                );

                // 1e-4°: acos loses precision next to the fully-extended singularity (θ3 ≈ 0)
                const matches = ik.solutions.some(s =>
                    close(s.theta1, t1, 1e-4) && close(s.theta2, t2, 1e-4) && close(s.theta3, t3, 1e-4));
                assert.ok(matches, `no IK solution equals ${q}`);
                checked++;
            }
        }
    }
    assert.ok(checked > 1000, `only ${checked} poses checked`);
});

test('IK reports unreachable targets', () => {
    const k = new RobotKinematics();
    assert.equal(k.inverseKinematics(500, 0, 50).valid, false);  // beyond L2+L3
    assert.equal(k.inverseKinematics(5, 0, 50).valid, false);    // inside |L2-L3|
    assert.equal(k.inverseKinematics(0, 0, 100).valid, false);   // on the Z axis
});

test('floor check: elbow and end effector must stay at Z >= 0', () => {
    const k = new RobotKinematics();

    // Elbow exactly on the floor at θ2 = -30° (50 + 100·sin(-30°) = 0)
    assert.equal(k.isConfigurationAboveFloor([0, -30, 30]).valid, true);
    assert.equal(k.isConfigurationAboveFloor([0, -31, 31]).valid, false);

    // Forearm pointing down from a level upper arm: 50 - 80 < 0
    assert.equal(k.isConfigurationAboveFloor([0, 0, -90]).valid, false);
    assert.equal(k.isConfigurationAboveFloor([0, 0, -38]).valid, true);

    const check = k.isConfigurationAboveFloor([0, 0, -90]);
    assert.ok(close(check.elbowZ, 50) && close(check.endEffectorZ, -30));
});

test('floor check has no side effects on the IK reference pose', () => {
    const k = new RobotKinematics();
    k.currentAngles = [1, 2, 3];
    k.isConfigurationAboveFloor([0, -80, 0]);
    k.isConfigurationValid([10, 10, 10]);
    k.planJointPath([0, 0, 0], [45, 30, -45]);
    assert.deepEqual(k.currentAngles, [1, 2, 3]);
});

test('isConfigurationValid checks limits, floor and input sanity', () => {
    const k = new RobotKinematics();
    assert.equal(k.isConfigurationValid([0, 0, 0]), true);
    assert.equal(k.isConfigurationValid([0, 95, 0]), false);
    assert.equal(k.isConfigurationValid([0, 0, 140]), false);
    assert.equal(k.isConfigurationValid([0, -60, 0]), false);
    assert.equal(k.isConfigurationValid([0, NaN, 0]), false);
    assert.equal(k.isConfigurationValid([0, 0]), false);
});

test('planned paths never cross the floor and end exactly at the target', () => {
    const k = new RobotKinematics();
    const rand = seededRandom(42);
    let lifted = 0;

    for (let i = 0; i < 3000; i++) {
        const start = randomValidConfig(k, rand);
        const target = randomValidConfig(k, rand);
        const path = k.planJointPath(start, target);

        assert.ok(path, `no path from ${start} to ${target}`);
        assert.deepEqual(path[0], start);
        assert.deepEqual(path[path.length - 1], target);
        assert.ok(k.isPathAboveFloor(path, 0.25), `path from ${start} to ${target} crosses the floor`);
        if (path.length > 2) lifted++;
    }
    assert.ok(lifted > 0, 'the lift strategy was exercised');
});

test('a move that would sweep through the floor is routed over it', () => {
    const k = new RobotKinematics();
    const start = [0, -25, 40];
    const target = [0, 30, -135];

    assert.equal(k.isPathAboveFloor([start, target]), false, 'straight line goes through the floor');
    const path = k.planJointPath(start, target);
    assert.ok(path.length > 2);
    assert.ok(k.isPathAboveFloor(path, 0.1));
});

test('planJointPath refuses invalid targets', () => {
    const k = new RobotKinematics();
    assert.equal(k.planJointPath([0, 0, 0], [0, -80, 0]), null);
    assert.equal(k.planJointPath([0, 0, 0], [0, 0, 200]), null);
    assert.equal(k.planJointPath([0, 0, 0], [0, NaN, 0]), null);
});

test('safe shoulder angle keeps any elbow angle above the floor', () => {
    const k = new RobotKinematics();
    const safe = k.getSafeShoulderAngle();
    for (let t3 = -135; t3 <= 135; t3 += 1) {
        assert.ok(k.isConfigurationAboveFloor([0, safe, t3]).valid, `θ3=${t3} at θ2=${safe}`);
    }
});

test('clampToWorkspace always returns a point the arm can reach above the floor', () => {
    const k = new RobotKinematics();
    const rand = seededRandom(7);
    const failures = [];

    for (let i = 0; i < 5000; i++) {
        const x = rand() * 600 - 300;
        const y = rand() * 600 - 300;
        const z = rand() * 600 - 300;
        if (Math.hypot(x, y) < 1) continue;

        const c = k.clampToWorkspace(x, y, z);
        const ik = k.inverseKinematics(c.x, c.y, c.z);
        const ok = ik.valid && ik.solutions.some(s =>
            k.isConfigurationAboveFloor([s.theta1, s.theta2, s.theta3]).valid);
        if (!ok && failures.length < 5) failures.push({ input: [x, y, z], clamped: c, error: ik.error });
    }
    assert.deepEqual(failures, []);
});
