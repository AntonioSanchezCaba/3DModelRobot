const test = require('node:test');
const assert = require('node:assert/strict');
const { loadBrowserScript } = require('../helpers/load-browser-script');

const SavedPositionsStore = loadBrowserScript('js/positions.js', 'SavedPositionsStore');

const JOINT_LIMITS = {
    theta1: { min: -180, max: 180 },
    theta2: { min: -90, max: 90 },
    theta3: { min: -135, max: 135 }
};

/** In-memory stand-in for window.localStorage */
class MemoryStorage {
    constructor(initial = {}) {
        this.data = { ...initial };
    }
    getItem(key) {
        return Object.prototype.hasOwnProperty.call(this.data, key) ? this.data[key] : null;
    }
    setItem(key, value) {
        this.data[key] = String(value);
    }
    removeItem(key) {
        delete this.data[key];
    }
}

function createStore(storage = new MemoryStorage()) {
    return new SavedPositionsStore({ storage, jointLimits: JOINT_LIMITS });
}

test('saves a named position and persists it', () => {
    const storage = new MemoryStorage();
    const store = createStore(storage);

    const { position, overwritten, persisted } = store.save('Recoger pieza', [10, 20, -30]);
    assert.equal(position.name, 'Recoger pieza');
    assert.deepEqual(position.angles, [10, 20, -30]);
    assert.equal(overwritten, false);
    assert.equal(persisted, true);

    // A fresh store reading the same storage sees it
    const reloaded = createStore(storage);
    assert.deepEqual(reloaded.list().map(p => p.name), ['Recoger pieza']);
    assert.deepEqual(reloaded.list()[0].angles, [10, 20, -30]);
});

test('normalizes whitespace in names', () => {
    const store = createStore();
    const { position } = store.save('   Pick    A  ', [0, 0, 0]);
    assert.equal(position.name, 'Pick A');
});

test('saving an existing name (any case) overwrites it', () => {
    const store = createStore();
    const first = store.save('Pick A', [1, 2, 3]).position;
    const second = store.save('pick a', [4, 5, 6]);

    assert.equal(second.overwritten, true);
    assert.equal(second.position.id, first.id);
    assert.equal(second.position.name, 'Pick A', 'the existing name is kept');
    assert.equal(store.list().length, 1);
    assert.deepEqual(store.list()[0].angles, [4, 5, 6]);
});

test('validateName rejects empty and too-long names and reports duplicates', () => {
    const store = createStore();
    store.save('Existing', [0, 0, 0]);

    assert.equal(store.validateName('   ').ok, false);
    assert.equal(store.validateName('x'.repeat(SavedPositionsStore.MAX_NAME_LENGTH + 1)).ok, false);
    assert.equal(store.validateName('x'.repeat(SavedPositionsStore.MAX_NAME_LENGTH)).ok, true);

    const dup = store.validateName('EXISTING');
    assert.equal(dup.ok, true);
    assert.equal(dup.existing.name, 'Existing');

    // The position itself may keep its name when renaming
    const id = store.findByName('Existing').id;
    assert.equal(store.validateName('existing', id).existing, null);
});

test('save rejects invalid names and angles', () => {
    const store = createStore();
    assert.throws(() => store.save('', [0, 0, 0]));
    assert.throws(() => store.save('A', [0, 0]));
    assert.throws(() => store.save('A', [0, NaN, 0]));
    assert.throws(() => store.save('A', [0, 95, 0]), 'outside θ2 limit');
    assert.equal(store.list().length, 0);
});

test('rename works and refuses names owned by another position', () => {
    const store = createStore();
    const a = store.save('A', [0, 0, 0]).position;
    store.save('B', [1, 1, 1]);

    assert.throws(() => store.rename(a.id, 'b'), /Ya existe/);
    assert.equal(store.rename(a.id, 'Alpha').position.name, 'Alpha');
    // Changing only the case of its own name is allowed
    assert.equal(store.rename(a.id, 'ALPHA').position.name, 'ALPHA');
    assert.throws(() => store.rename('missing-id', 'Z'));
});

test('remove deletes and persists', () => {
    const storage = new MemoryStorage();
    const store = createStore(storage);
    const a = store.save('A', [0, 0, 0]).position;
    store.save('B', [1, 1, 1]);

    assert.equal(store.remove(a.id).name, 'A');
    assert.equal(store.remove(a.id), null);
    assert.deepEqual(createStore(storage).list().map(p => p.name), ['B']);
});

test('list and get return copies that cannot mutate the store', () => {
    const store = createStore();
    const id = store.save('A', [1, 2, 3]).position.id;

    store.list()[0].angles[0] = 99;
    store.get(id).angles[1] = 99;
    assert.deepEqual(store.get(id).angles, [1, 2, 3]);
});

test('change listeners fire on every mutation', () => {
    const store = createStore();
    let calls = 0;
    store.onChange(() => calls++);

    const id = store.save('A', [0, 0, 0]).position.id;
    store.rename(id, 'B');
    store.remove(id);
    store.importJSON(JSON.stringify({ positions: [{ name: 'C', angles: [0, 0, 0] }] }));
    assert.equal(calls, 4);
});

test('export and import round-trip', () => {
    const source = createStore();
    source.save('A', [10, 20, 30]);
    source.save('B', [-45, 45, -30]);

    const target = createStore();
    target.save('b', [0, 0, 0]);
    const result = target.importJSON(source.exportJSON());

    assert.equal(result.added, 1);
    assert.equal(result.updated, 1);
    assert.equal(result.skipped, 0);
    assert.deepEqual(target.findByName('A').angles, [10, 20, 30]);
    assert.deepEqual(target.findByName('B').angles, [-45, 45, -30]);
    assert.equal(target.list().length, 2);
});

test('import skips invalid entries and rejects non-position files', () => {
    const store = createStore();
    const result = store.importJSON(JSON.stringify({
        positions: [
            { name: 'ok', angles: [0, 0, 0] },
            { name: '', angles: [0, 0, 0] },
            { name: 'nan', angles: [0, 'x', 0] },
            { name: 'limits', angles: [0, 0, 500] },
            { name: 'short', angles: [0, 0] },
            null,
            'text'
        ]
    }));
    assert.deepEqual({ added: result.added, skipped: result.skipped }, { added: 1, skipped: 6 });

    assert.throws(() => store.importJSON('not json'), /JSON/);
    assert.throws(() => store.importJSON('{"foo": 1}'), /no contiene/);
    // A bare array of positions is accepted too
    assert.equal(store.importJSON('[{"name":"arr","angles":[1,1,1]}]').added, 1);
});

test('corrupted or tampered storage loads what is valid', () => {
    const key = SavedPositionsStore.STORAGE_KEY;

    assert.equal(createStore(new MemoryStorage({ [key]: '{broken' })).list().length, 0);

    const tampered = new MemoryStorage({
        [key]: JSON.stringify({
            version: 1,
            positions: [
                { id: 'same', name: 'A', angles: [0, 0, 0] },
                { id: 'same', name: 'B', angles: [1, 1, 1] },
                { id: 'x', name: 'a', angles: [2, 2, 2] },
                { id: 'y', name: 'C', angles: [0, 0, 999] }
            ]
        })
    });
    const store = createStore(tampered);
    assert.deepEqual(store.list().map(p => p.name), ['A', 'B']);
    const ids = store.list().map(p => p.id);
    assert.equal(new Set(ids).size, ids.length, 'ids are unique');
});

test('works in memory when storage is unavailable or full', () => {
    const noStorage = createStore(null);
    assert.equal(noStorage.isPersistent(), false);
    assert.equal(noStorage.save('A', [0, 0, 0]).persisted, false);
    assert.equal(noStorage.list().length, 1);

    const full = new MemoryStorage();
    full.setItem = () => {
        throw new Error('QuotaExceededError');
    };
    const store = createStore(full);
    const { persisted } = store.save('A', [0, 0, 0]);
    assert.equal(persisted, false);
    assert.equal(store.list().length, 1);
});

test('export can include motor settings and step counts', () => {
    const store = createStore();
    store.save('A', [45, -10, 90]);

    const data = JSON.parse(store.exportJSON({
        motor: { stepsPerRev: 200, microstepping: 16 },
        stepsFor: angles => angles.map(a => Math.round(a * 3200 / 360))
    }));
    assert.deepEqual(data.motor, { stepsPerRev: 200, microstepping: 16 });
    assert.deepEqual(data.positions[0].steps, [400, -89, 800]);

    // Files with extra fields import cleanly
    assert.equal(createStore().importJSON(JSON.stringify(data)).added, 1);
});

test('only real numbers are accepted as angles', () => {
    const store = createStore();
    for (const angles of [[null, null, null], ['', ' ', '\n'], [true, false, 0], [[45], 0, 0], ['10', 0, 0]]) {
        assert.throws(() => store.save('X', angles), `${JSON.stringify(angles)} must be rejected`);
    }
    const result = store.importJSON('[{"name":"nulls","angles":[null,null,null]}]');
    assert.equal(result.added, 0);
    assert.equal(result.skipped, 1);
});

test('angles keep full precision (rounding could push a floor pose below the floor)', () => {
    const storage = new MemoryStorage();
    const store = createStore(storage);
    const angles = [9.405371234567, 12.220134567891, -129.398961234567];
    const { position } = store.save('Mesa', angles);
    assert.deepEqual(position.angles, angles);
    // ...and survive a reload exactly
    assert.deepEqual(createStore(storage).findByName('Mesa').angles, angles);
});

test('names that differ only in Unicode composition are the same name', () => {
    const store = createStore();
    store.save('Posición', [0, 0, 0]);                 // precomposed ó
    const second = store.save('Posición', [1, 1, 1]);  // o + combining accent
    assert.equal(second.overwritten, true);
    assert.equal(store.list().length, 1);
});

test('import can skip poses the robot cannot reach', () => {
    const store = createStore();
    const file = JSON.stringify({ positions: [
        { name: 'ok', angles: [0, 0, 0] },
        { name: 'floor', angles: [0, -90, 0] }
    ] });
    const result = store.importJSON(file, { isReachable: angles => angles[1] > -30 });
    assert.deepEqual({ added: result.added, unreachable: result.unreachable }, { added: 1, unreachable: 1 });
    assert.deepEqual(store.list().map(p => p.name), ['ok']);
});
