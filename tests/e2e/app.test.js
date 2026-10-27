/**
 * End-to-end tests: the real app in headless Chromium.
 *
 *   npm install && npx playwright install chromium
 *   npm run test:e2e
 *
 * A tiny static server serves the repository, so no other server is needed.
 * Three.js is loaded from its CDN, so the tests need internet access.
 */
const test = require('node:test');
const assert = require('node:assert/strict');
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { chromium } = require('playwright');

const ROOT = path.resolve(__dirname, '..', '..');
const MIME = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json',
    '.png': 'image/png',
    '.svg': 'image/svg+xml'
};

let server;
let baseUrl;
let browser;

function startServer() {
    return new Promise(resolve => {
        const srv = http.createServer((req, res) => {
            const urlPath = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
            const file = path.join(ROOT, urlPath === '/' ? 'index.html' : urlPath);
            if (!file.startsWith(ROOT) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
                res.writeHead(404);
                res.end('Not found');
                return;
            }
            res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream' });
            fs.createReadStream(file).pipe(res);
        });
        srv.listen(0, '127.0.0.1', () => resolve(srv));
    });
}

test.before(async () => {
    server = await startServer();
    baseUrl = `http://127.0.0.1:${server.address().port}/`;
    browser = await chromium.launch({
        args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist']
    });
});

test.after(async () => {
    if (browser) await browser.close();
    if (server) server.close();
});

// CDN files (three.js, fonts, icons) are fetched once and replayed to every browser context
const cdnCache = new Map();

async function serveExternalFromCache(route) {
    const url = route.request().url();
    if (url.startsWith(baseUrl)) return route.continue();

    let cached = cdnCache.get(url);
    if (!cached) {
        const response = await route.fetch();
        cached = { status: response.status(), headers: response.headers(), body: await response.body() };
        cdnCache.set(url, cached);
    }
    return route.fulfill(cached);
}

async function newContext(t, viewport = { width: 1600, height: 1000 }) {
    const ctx = await browser.newContext({ viewport, acceptDownloads: true });
    await ctx.route('**/*', serveExternalFromCache);
    // Close even when the test fails, so no page keeps rendering in the background
    t.after(() => ctx.close());
    return ctx;
}

/**
 * Open the app in a browser context (a fresh one has empty localStorage) and wait until it is ready
 */
async function openApp(t, { viewport, context = null } = {}) {
    const ctx = context || await newContext(t, viewport);
    const page = await ctx.newPage();
    const errors = [];
    page.on('console', msg => {
        if (msg.type() === 'error') errors.push(msg.text());
    });
    page.on('pageerror', err => errors.push(`pageerror: ${err.message}`));
    page.on('dialog', dialog => {
        errors.push(`unexpected native dialog: ${dialog.message()}`);
        dialog.dismiss();
    });

    await page.goto(baseUrl);
    await page.waitForFunction(() =>
        window.robotController && window.robotController.ui &&
        document.getElementById('app').classList.contains('visible'), null, { timeout: 30000 });
    return { page, context: ctx, errors };
}

/** Screen position (page pixels) of a drag handle */
function handleOnScreen(page, which) {
    return page.evaluate(which => {
        const rc = window.robotController;
        const handle = which === 'elbow' ? rc.elbowDragHandle : rc.dragHandle;
        const v = handle.position.clone().project(rc.camera);
        const rect = rc.renderer.domElement.getBoundingClientRect();
        return { x: rect.left + (v.x + 1) / 2 * rect.width, y: rect.top + (-v.y + 1) / 2 * rect.height };
    }, which);
}

function robotState(page) {
    return page.evaluate(() => {
        const rc = window.robotController;
        const angles = rc.getCurrentAngles();
        const fk = rc.kinematics.forwardKinematics(angles).position;
        return {
            angles,
            valid: rc.kinematics.isConfigurationValid(angles),
            dragging: rc.isDragging,
            sliders: [...document.querySelectorAll('.joint-slider')].map(s => parseFloat(s.value)),
            handle: rc.dragHandle.position.toArray(),
            fkThree: [fk.x, fk.z + 12, -fk.y],
            handleColor: rc.dragHandle.material.color.getHex(),
            ringColor: rc.dragHandle.children[0].material.color.getHex(),
            status: document.getElementById('status-message').textContent
        };
    });
}

function waitForAnimation(page) {
    return page.waitForFunction(() => !window.robotController.ui.isAnimating, null, { timeout: 10000 });
}

/** Record the lowest arm point reached while the page runs (to catch moves through the floor) */
function trackLowestPoint(page) {
    return page.evaluate(() => {
        const rc = window.robotController;
        window.__lowestZ = Infinity;
        const original = rc.setJointAngles.bind(rc);
        rc.setJointAngles = (angles, options) => {
            const ok = original(angles, options);
            const check = rc.kinematics.isConfigurationAboveFloor(rc.getCurrentAngles());
            window.__lowestZ = Math.min(window.__lowestZ, check.elbowZ, check.endEffectorZ);
            return ok;
        };
    });
}

const near = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol;
const sameAngles = (a, b, tol = 1e-6) => a.every((v, i) => near(v, b[i], tol));

test('loads without errors and starts at Home', async (t) => {
    const { page, context, errors } = await openApp(t);
    const state = await robotState(page);

    assert.deepEqual(state.angles, [0, 0, 0]);
    assert.ok(state.handle.every((v, i) => near(v, state.fkThree[i], 1e-6)), 'handle sits on the kinematic tip');
    assert.equal(await page.textContent('#fk-x'), '180.00 mm');
    assert.deepEqual(errors, []);
    await context.close();
});

test('sliders move the arm and refuse poses below the floor', async (t) => {
    const { page, context, errors } = await openApp(t);

    await page.locator('#joint2-slider').fill('40');
    let state = await robotState(page);
    assert.ok(near(state.angles[1], 40));

    // θ2 = -60° puts the elbow under the floor: rejected, slider snaps back
    await page.locator('#joint2-slider').fill('-60');
    state = await robotState(page);
    assert.ok(near(state.angles[1], 40));
    assert.ok(near(state.sliders[1], 40));
    assert.match(state.status, /inválida/);

    // Empty number input restores the current angle instead of sending NaN
    await page.fill('#joint1-input', '');
    await page.locator('#joint1-input').dispatchEvent('change');
    state = await robotState(page);
    assert.ok(state.angles.every(Number.isFinite));
    assert.equal(await page.inputValue('#joint1-input'), '0');

    assert.deepEqual(errors, []);
    await context.close();
});

test('dragging the end effector keeps it exactly under the cursor', async (t) => {
    const { page, context, errors } = await openApp(t);
    const start = await handleOnScreen(page, 'end');

    await page.mouse.move(start.x, start.y);
    await page.mouse.down();
    let state = await robotState(page);
    assert.ok(state.dragging);
    assert.equal(state.handleColor, 0xffd400, 'yellow while dragging');
    assert.equal(state.ringColor, 0xffd400, 'ring changes color too');

    for (const [dx, dy] of [[-30, -20], [-80, -60], [-120, 10], [-60, 40]]) {
        const target = { x: start.x + dx, y: start.y + dy };
        await page.mouse.move(target.x, target.y, { steps: 8 });
        const now = await handleOnScreen(page, 'end');
        assert.ok(Math.hypot(now.x - target.x, now.y - target.y) < 1.5, `handle follows the cursor at ${dx},${dy}`);
        state = await robotState(page);
        assert.ok(state.valid);
        assert.ok(state.sliders.every((v, i) => Math.abs(v - state.angles[i]) <= 0.05), 'sliders follow');
    }

    await page.mouse.up();
    state = await robotState(page);
    assert.equal(state.dragging, false);
    assert.equal(state.handleColor, 0x00ff88);
    assert.deepEqual(errors, []);
    await context.close();
});

test('dragging far outside the workspace keeps the arm valid and shows the limit color', async (t) => {
    const { page, context, errors } = await openApp(t);
    const start = await handleOnScreen(page, 'end');

    await page.mouse.move(start.x, start.y);
    await page.mouse.down();
    let limited = 0;
    for (const [dx, dy] of [[600, 400], [-700, 300], [0, 500], [-200, -600]]) {
        await page.mouse.move(start.x + dx, start.y + dy, { steps: 20 });
        const state = await robotState(page);
        assert.ok(state.valid, `valid pose after dragging to ${dx},${dy}`);

        // Red exactly when the arm is held at a limit, yellow while it can follow
        const atLimit = /Límite/.test(state.status);
        assert.equal(state.handleColor, atLimit ? 0xff3b3b : 0xffd400, `color matches status "${state.status}"`);
        if (atLimit) limited++;
    }
    assert.ok(limited >= 2, `far drags hit the limit (${limited}/4)`);
    await page.mouse.up();
    assert.deepEqual(errors, []);
    await context.close();
});

test('dragging the elbow moves base and shoulder only, never below the floor', async (t) => {
    const { page, context, errors } = await openApp(t);
    await page.evaluate(() => {
        window.robotController.setJointAngles([0, 20, -60]);
        window.robotController.ui.syncControlsFromRobot();
    });
    const start = await handleOnScreen(page, 'elbow');

    await page.mouse.move(start.x, start.y);
    await page.mouse.down();
    await page.mouse.move(start.x + 60, start.y - 50, { steps: 10 });
    let state = await robotState(page);
    assert.ok(near(state.angles[2], -60), 'elbow angle unchanged');
    assert.ok(!near(state.angles[0], 0) || !near(state.angles[1], 20), 'base/shoulder moved');

    await page.mouse.move(start.x + 60, start.y + 600, { steps: 20 });
    state = await robotState(page);
    assert.ok(state.valid, 'stopped at the floor');
    await page.mouse.up();

    assert.deepEqual(errors, []);
    await context.close();
});

test('empty space still orbits the camera, and a drag released off the canvas ends cleanly', async (t) => {
    const { page, context, errors } = await openApp(t, { viewport: { width: 1600, height: 900 } });

    const before = await page.evaluate(() => window.robotController.camera.position.toArray());
    await page.mouse.move(200, 700);
    await page.mouse.down();
    await page.mouse.move(350, 650, { steps: 10 });
    await page.mouse.up();
    const after = await page.evaluate(() => window.robotController.camera.position.toArray());
    assert.ok(before.some((v, i) => Math.abs(v - after[i]) > 1), 'camera orbited');
    assert.equal((await robotState(page)).dragging, false);

    const start = await handleOnScreen(page, 'end');
    await page.mouse.move(start.x, start.y);
    await page.mouse.down();
    await page.mouse.move(1500, 300, { steps: 10 });  // over the side panel
    assert.equal((await robotState(page)).dragging, true);
    await page.mouse.up();
    assert.equal((await robotState(page)).dragging, false);
    assert.equal(await page.evaluate(() => window.robotController.controls.enabled), true);

    assert.deepEqual(errors, []);
    await context.close();
});

test('presets reach their exact target from awkward poses without crossing the floor', async (t) => {
    const { page, context, errors } = await openApp(t);
    await trackLowestPoint(page);

    const starts = [[0, -25, 40], [10, 80, -130], [-120, 5, 120], [170, -20, 30]];
    const presets = { home: [0, 0, 0], pick: [45, 30, -45], place: [-45, 45, -30] };

    for (const start of starts) {
        for (const [name, target] of Object.entries(presets)) {
            await page.evaluate(start => window.robotController.setJointAngles(start), start);
            await page.click(`.preset-btn[data-preset="${name}"]`);
            await waitForAnimation(page);
            const state = await robotState(page);
            assert.ok(sameAngles(state.angles, target), `${name} from ${start}: got ${state.angles}`);
            assert.ok(sameAngles(state.sliders, target, 0.05));
        }
    }
    assert.ok(await page.evaluate(() => window.__lowestZ) >= -1e-9, 'never below the floor');
    assert.deepEqual(errors, []);
    await context.close();
});

test('saved positions: name, validate, overwrite, persist, go to, rename, delete', async (t) => {
    const context = await newContext(t);
    let { page, errors } = await openApp(t, { context });

    assert.equal(await page.isVisible('#saved-positions-empty'), true);
    assert.equal(await page.isDisabled('#btn-export-positions'), true);

    // Save the current pose with a name
    await page.evaluate(() => {
        window.robotController.setJointAngles([30, 20, -40]);
        window.robotController.ui.syncControlsFromRobot();
    });
    await page.click('#btn-save-position');
    await page.waitForSelector('#dialog-backdrop:not([hidden])');
    assert.equal(await page.inputValue('#dialog-input'), 'Posición 1', 'suggested name');
    assert.match(await page.textContent('#dialog-preview'), /30\.0°/);

    await page.fill('#dialog-input', '   ');
    assert.equal(await page.isDisabled('#dialog-confirm'), true, 'blank name blocked');
    await page.fill('#dialog-input', 'x'.repeat(60));
    assert.equal((await page.inputValue('#dialog-input')).length, 40, 'name length capped');

    await page.fill('#dialog-input', '  Recoger   pieza A ');
    await page.keyboard.press('Enter');
    await page.waitForSelector('#dialog-backdrop', { state: 'hidden' });
    assert.deepEqual(await page.$$eval('.saved-position-name', els => els.map(e => e.textContent)), ['Recoger pieza A']);
    assert.equal(await page.locator('.saved-position.active').count(), 1, 'current pose highlighted');

    // A name that looks like HTML is shown as text, never executed
    await page.evaluate(() => window.robotController.setJointAngles([-60, 45, -90]));
    await page.click('#btn-save-position');
    await page.fill('#dialog-input', '<img src=x onerror="window.__xss=1">');
    await page.click('#dialog-confirm');
    await page.waitForSelector('#dialog-backdrop', { state: 'hidden' });
    assert.equal(await page.evaluate(() => window.__xss), undefined);
    assert.equal(await page.locator('.saved-position').count(), 2);

    // Same name in another case: confirm button turns into "Reemplazar"
    await page.evaluate(() => window.robotController.setJointAngles([10, 10, 10]));
    await page.click('#btn-save-position');
    await page.fill('#dialog-input', 'recoger pieza a');
    assert.equal((await page.textContent('#dialog-confirm')).trim(), 'Reemplazar');
    await page.click('#dialog-confirm');
    await page.waitForSelector('#dialog-backdrop', { state: 'hidden' });
    assert.equal(await page.locator('.saved-position').count(), 2);

    // Escape cancels without saving
    await page.click('#btn-save-position');
    await page.keyboard.press('Escape');
    await page.waitForSelector('#dialog-backdrop', { state: 'hidden' });
    assert.equal(await page.locator('.saved-position').count(), 2);

    // Survives a reload
    await page.close();
    ({ page, errors } = await openApp(t, { context }));
    assert.equal(await page.locator('.saved-position').count(), 2);
    const stored = await page.evaluate(() => window.robotController.ui.positionStore.findByName('Recoger pieza A'));
    assert.deepEqual(stored.angles, [10, 10, 10]);

    // Go to a saved position
    await page.locator('.saved-position', { hasText: 'onerror' }).locator('.saved-position-go').click();
    await waitForAnimation(page);
    assert.ok(sameAngles((await robotState(page)).angles, [-60, 45, -90]));
    assert.equal(await page.locator('.saved-position.active').count(), 1);

    // Rename: another position's name is refused, a new one works
    await page.locator('.saved-position', { hasText: 'onerror' }).locator('[data-action=rename]').click();
    await page.fill('#dialog-input', 'RECOGER PIEZA A');
    assert.equal(await page.isDisabled('#dialog-confirm'), true);
    await page.fill('#dialog-input', 'Soltar pieza');
    await page.click('#dialog-confirm');
    await page.waitForSelector('#dialog-backdrop', { state: 'hidden' });
    assert.deepEqual(
        await page.$$eval('.saved-position-name', els => els.map(e => e.textContent)),
        ['Recoger pieza A', 'Soltar pieza']
    );

    // Delete asks first; cancel keeps it, confirm removes it
    await page.locator('.saved-position', { hasText: 'Soltar pieza' }).locator('[data-action=delete]').click();
    await page.click('#dialog-cancel');
    assert.equal(await page.locator('.saved-position').count(), 2);
    await page.locator('.saved-position', { hasText: 'Soltar pieza' }).locator('[data-action=delete]').click();
    await page.click('#dialog-confirm');
    await page.waitForSelector('#dialog-backdrop', { state: 'hidden' });
    assert.deepEqual(await page.$$eval('.saved-position-name', els => els.map(e => e.textContent)), ['Recoger pieza A']);

    assert.deepEqual(errors, []);
    await context.close();
});

test('saved positions: a move that would sweep through the floor is routed over it', async (t) => {
    const { page, context, errors } = await openApp(t);
    await page.evaluate(() => {
        window.robotController.ui.positionStore.save('Plegado', [0, 30, -135]);
        window.robotController.setJointAngles([0, -25, 40]);
    });
    await trackLowestPoint(page);

    await page.locator('.saved-position', { hasText: 'Plegado' }).locator('.saved-position-go').click();
    await waitForAnimation(page);
    assert.ok(sameAngles((await robotState(page)).angles, [0, 30, -135]));
    assert.ok(await page.evaluate(() => window.__lowestZ) >= -1e-9);

    // An unreachable target is refused and nothing moves
    const moved = await page.evaluate(() => window.robotController.ui.animateToPosition([0, -80, 0]));
    assert.equal(moved, false);
    assert.ok(sameAngles((await robotState(page)).angles, [0, 30, -135]));

    assert.deepEqual(errors, []);
    await context.close();
});

test('saved positions: export and import a JSON file', async (t) => {
    const { page, context, errors } = await openApp(t);
    await page.evaluate(() => {
        const store = window.robotController.ui.positionStore;
        store.save('A', [45, -10, 90]);
        store.save('B', [-90, 60, -120]);
    });

    const [download] = await Promise.all([page.waitForEvent('download'), page.click('#btn-export-positions')]);
    const file = await download.path();
    const data = JSON.parse(fs.readFileSync(file, 'utf8'));
    assert.deepEqual(data.positions.map(p => p.name), ['A', 'B']);
    assert.deepEqual(data.motor, { stepsPerRev: 200, microstepping: 16 });
    assert.deepEqual(data.positions[0].steps, [400, -89, 800]);

    await page.evaluate(() => {
        const store = window.robotController.ui.positionStore;
        store.list().forEach(p => store.remove(p.id));
    });
    assert.equal(await page.locator('.saved-position').count(), 0);

    await page.setInputFiles('#import-positions-file', file);
    await page.waitForFunction(() => document.querySelectorAll('.saved-position').length === 2);
    assert.deepEqual(await page.$$eval('.saved-position-name', els => els.map(e => e.textContent)), ['A', 'B']);

    assert.deepEqual(errors, []);
    await context.close();
});

test('inverse kinematics panel picks a solution above the floor and applies it', async (t) => {
    const { page, context, errors } = await openApp(t);
    await page.click('.nav-btn[data-view="kinematics"]');

    // Default target (100, 100, 50): one branch has the elbow under the floor
    await page.click('#btn-calc-ik');
    assert.match(await page.textContent('#ik-status'), /Solución encontrada/);
    await page.click('#btn-apply-ik');
    await waitForAnimation(page);
    const fk = await page.evaluate(() => {
        const rc = window.robotController;
        return rc.kinematics.forwardKinematics(rc.getCurrentAngles()).position;
    });
    assert.ok(near(fk.x, 100, 1e-6) && near(fk.y, 100, 1e-6) && near(fk.z, 50, 1e-6));
    assert.ok((await robotState(page)).valid);

    // Unreachable target: error, Apply disabled, marker hidden
    await page.fill('#ik-target-x', '500');
    await page.click('#btn-calc-ik');
    assert.match(await page.textContent('#ik-status'), /Fuera de alcance/);
    assert.equal(await page.isDisabled('#btn-apply-ik'), true);
    assert.equal(await page.evaluate(() => window.robotController.targetMarker.visible), false);

    assert.deepEqual(errors, []);
    await context.close();
});

test('link lengths: invalid values are refused, valid ones rebuild the arm', async (t) => {
    const { page, context, errors } = await openApp(t);
    await page.click('.nav-btn[data-view="settings"]');

    await page.fill('#param-l2', '');
    await page.click('#btn-apply-params');
    assert.equal(await page.evaluate(() => window.robotController.kinematics.L2), 100, 'empty value refused');
    await page.fill('#param-l2', '-50');
    await page.click('#btn-apply-params');
    assert.equal(await page.evaluate(() => window.robotController.kinematics.L2), 100, 'negative value refused');

    // A pose that becomes invalid with the new lengths sends the arm Home
    await page.evaluate(() => window.robotController.setJointAngles([0, -20, 0]));
    await page.fill('#param-l2', '200');
    await page.fill('#param-l3', '150');
    await page.click('#btn-apply-params');
    const state = await robotState(page);
    assert.deepEqual(state.angles, [0, 0, 0]);
    assert.ok(state.handle.every((v, i) => near(v, state.fkThree[i], 1e-6)), 'handle on the new tip');
    assert.equal(await page.textContent('#fk-x'), '350.00 mm', 'readouts refreshed');
    assert.equal(await page.textContent('#dh-a2'), '200');

    assert.deepEqual(errors, []);
    await context.close();
});

test('motor steps follow the joint angles and resolution', async (t) => {
    const { page, context, errors } = await openApp(t);
    await page.evaluate(() => {
        window.robotController.setJointAngles([90, -10, 45]);
        window.robotController.ui.syncControlsFromRobot();
    });
    await page.click('.nav-btn[data-view="settings"]');
    assert.equal(await page.textContent('#steps-joint1'), '800 pasos');
    assert.equal(await page.textContent('#steps-joint2'), '-89 pasos');
    assert.equal(await page.textContent('#steps-joint3'), '400 pasos');

    await page.selectOption('#motor-microstepping', '32');
    assert.equal(await page.textContent('#steps-joint1'), '1600 pasos');

    assert.deepEqual(errors, []);
    await context.close();
});

test('typing an angle during an animation wins, and quick buttons keep the animation target', async (t) => {
    const { page, errors } = await openApp(t);

    // Slow animation so it is still running while the user types
    await page.evaluate(() => {
        window.robotController.ui.animationSpeed = 0.1;
        window.robotController.ui.animateToPosition([170, 0, 0]);
    });
    await page.click('#joint1-input', { clickCount: 3 });
    await page.keyboard.type('60');
    await page.keyboard.press('Enter');
    let state = await robotState(page);
    assert.ok(near(state.angles[0], 60), `θ1 = ${state.angles[0]}`);
    assert.equal(await page.inputValue('#joint1-input'), '60');

    // A quick button pressed mid-animation changes one joint of the target, not of the mid-way pose
    await page.evaluate(() => {
        window.robotController.ui.animationSpeed = 1;
        window.robotController.setJointAngles([0, 0, 0]);
        window.robotController.ui.goToPreset('pick');
    });
    await page.locator('.joint-control[data-joint="0"] .quick-btn[data-angle="-90"]').click();
    await waitForAnimation(page);
    state = await robotState(page);
    assert.ok(sameAngles(state.angles, [-90, 30, -45]), `got ${state.angles}`);

    assert.deepEqual(errors, []);
});

test('changing link lengths clears a stale IK result', async (t) => {
    const { page, errors } = await openApp(t);
    await page.click('.nav-btn[data-view="kinematics"]');
    await page.click('#btn-calc-ik');
    assert.equal(await page.isDisabled('#btn-apply-ik'), false);

    await page.click('.nav-btn[data-view="settings"]');
    await page.fill('#param-l3', '60');
    await page.click('#btn-apply-params');

    assert.equal(await page.isDisabled('#btn-apply-ik'), true);
    assert.equal(await page.textContent('#ik-theta1'), '--°');
    assert.equal(await page.evaluate(() => window.robotController.targetMarker.visible), false);
    assert.deepEqual(errors, []);
});

test('switching tabs shows the new section from its top', async (t) => {
    const { page, errors } = await openApp(t, { viewport: { width: 1280, height: 720 } });
    await page.evaluate(() => { document.querySelector('.control-panel').scrollTop = 10000; });
    await page.click('.nav-btn[data-view="kinematics"]');
    const top = await page.evaluate(() => {
        const panel = document.querySelector('.control-panel').getBoundingClientRect();
        return document.getElementById('section-kinematics').getBoundingClientRect().top - panel.top;
    });
    assert.ok(top >= 0 && top < 50, `section starts ${top}px from the panel top`);
    assert.deepEqual(errors, []);
});

test('importing skips unreachable poses; the 3D view is not mirrored', async (t) => {
    const { page, errors } = await openApp(t);
    const file = path.join(require('node:os').tmpdir(), `roboarm-import-${process.pid}.json`);
    fs.writeFileSync(file, JSON.stringify({ positions: [
        { name: 'Bien', angles: [10, 20, -30] },
        { name: 'Bajo el suelo', angles: [0, -90, 0] }
    ] }));
    await page.setInputFiles('#import-positions-file', file);
    await page.waitForFunction(() => document.querySelectorAll('.saved-position').length === 1);
    assert.match(await page.textContent('#toast-container'), /1 fuera de alcance/);
    fs.unlinkSync(file);

    // θ1 = +90° turns the arm toward kinematic +Y, i.e. Three.js -Z (a rotation, not a mirror)
    await page.evaluate(() => window.robotController.setJointAngles([90, 0, 0]));
    const handle = (await robotState(page)).handle;
    assert.ok(near(handle[0], 0, 1e-6) && near(handle[2], -180, 1e-6), `handle at ${handle}`);

    assert.deepEqual(errors, []);
});
