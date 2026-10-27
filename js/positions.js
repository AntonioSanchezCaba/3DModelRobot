/**
 * =====================================================
 * SAVED POSITIONS MODULE
 * Named robot poses persisted in the browser (localStorage)
 * =====================================================
 *
 * Each saved position stores the three joint angles (degrees) under a
 * user-chosen name. Names are unique (case-insensitive); saving with an
 * existing name overwrites that position.
 */

class SavedPositionsStore {
    /**
     * @param {Object} options
     * @param {Storage|null} [options.storage] - Storage backend (defaults to window.localStorage)
     * @param {string} [options.storageKey] - Key used in the storage backend
     * @param {Object} [options.jointLimits] - {theta1:{min,max}, theta2:{...}, theta3:{...}}
     */
    constructor(options = {}) {
        this.storageKey = options.storageKey || SavedPositionsStore.STORAGE_KEY;
        this.storage = options.storage !== undefined ? options.storage : SavedPositionsStore.getDefaultStorage();
        this.jointLimits = options.jointLimits || null;
        this.positions = [];
        this.listeners = [];
        this.idCounter = 0;
        this.load();
    }

    /**
     * Get localStorage if it is usable (it throws in some private modes)
     */
    static getDefaultStorage() {
        try {
            const storage = window.localStorage;
            const probe = '__roboarm3d_probe__';
            storage.setItem(probe, '1');
            storage.removeItem(probe);
            return storage;
        } catch (err) {
            return null;
        }
    }

    /**
     * Collapse whitespace so "  Pick   A " and "Pick A" are the same name
     */
    static normalizeName(name) {
        // NFC: a pasted "o" + combining accent and a typed "ó" are the same name
        return String(name == null ? '' : name).normalize('NFC').trim().replace(/\s+/g, ' ');
    }

    static nameKey(name) {
        return SavedPositionsStore.normalizeName(name).toLocaleLowerCase();
    }

    /**
     * True when positions survive a page reload
     */
    isPersistent() {
        return this.storage !== null;
    }

    /**
     * Validate a name for saving or renaming
     * @param {string} name
     * @param {string|null} ignoreId - Position allowed to already own the name (rename)
     * @returns {Object} {ok, name, error, existing}
     */
    validateName(name, ignoreId = null) {
        const normalized = SavedPositionsStore.normalizeName(name);

        if (!normalized) {
            return { ok: false, name: normalized, error: 'Escribe un nombre para la posición', existing: null };
        }
        if (normalized.length > SavedPositionsStore.MAX_NAME_LENGTH) {
            return {
                ok: false,
                name: normalized,
                error: `Máximo ${SavedPositionsStore.MAX_NAME_LENGTH} caracteres`,
                existing: null
            };
        }

        const existing = this.findByName(normalized);
        return {
            ok: true,
            name: normalized,
            error: null,
            existing: existing && existing.id !== ignoreId ? existing : null
        };
    }

    /**
     * Validate joint angles: three finite numbers inside the joint limits
     *
     * Angles are kept at full precision: rounding a pose that touches the floor
     * could push it a hair below it and make it unreachable.
     *
     * @returns {Array|null} The angles, or null if invalid
     */
    sanitizeAngles(angles) {
        if (!Array.isArray(angles) || angles.length !== 3) return null;

        const keys = ['theta1', 'theta2', 'theta3'];
        const clean = [];
        for (let i = 0; i < 3; i++) {
            const value = angles[i];
            // Real numbers only: Number(null), Number('') and Number(true) would sneak in as 0 or 1
            if (typeof value !== 'number' || !Number.isFinite(value)) return null;

            if (this.jointLimits) {
                const limits = this.jointLimits[keys[i]];
                // Small tolerance for values that went through rounding
                if (value < limits.min - 1e-6 || value > limits.max + 1e-6) return null;
            }
            clean.push(value);
        }
        return clean;
    }

    /**
     * Turn raw stored/imported data into a valid entry, or null
     */
    sanitizeEntry(raw) {
        if (!raw || typeof raw !== 'object') return null;

        const name = SavedPositionsStore.normalizeName(raw.name);
        if (!name || name.length > SavedPositionsStore.MAX_NAME_LENGTH) return null;

        const angles = this.sanitizeAngles(raw.angles);
        if (!angles) return null;

        const now = Date.now();
        return {
            id: typeof raw.id === 'string' && raw.id ? raw.id : this.generateId(),
            name: name,
            angles: angles,
            createdAt: Number.isFinite(raw.createdAt) ? raw.createdAt : now,
            updatedAt: Number.isFinite(raw.updatedAt) ? raw.updatedAt : now
        };
    }

    generateId() {
        this.idCounter++;
        return `pos_${Date.now().toString(36)}_${this.idCounter}_${Math.random().toString(36).slice(2, 7)}`;
    }

    /**
     * Load positions from storage, dropping anything invalid or duplicated
     */
    load() {
        this.positions = [];
        if (!this.storage) return;

        let data = null;
        try {
            const text = this.storage.getItem(this.storageKey);
            data = text ? JSON.parse(text) : null;
        } catch (err) {
            console.warn('Saved positions could not be read, starting empty:', err);
            return;
        }

        const items = data && Array.isArray(data.positions) ? data.positions : [];
        const seenNames = new Set();
        const seenIds = new Set();
        for (const raw of items) {
            const entry = this.sanitizeEntry(raw);
            if (!entry) continue;

            const key = SavedPositionsStore.nameKey(entry.name);
            if (seenNames.has(key)) continue;
            if (seenIds.has(entry.id)) entry.id = this.generateId();

            seenNames.add(key);
            seenIds.add(entry.id);
            this.positions.push(entry);
        }
    }

    /**
     * Write positions to storage
     * @returns {boolean} False if the browser refused to store them (quota, private mode)
     */
    persist() {
        if (!this.storage) return false;
        try {
            this.storage.setItem(this.storageKey, JSON.stringify({
                version: 1,
                positions: this.positions
            }));
            return true;
        } catch (err) {
            console.warn('Saved positions could not be stored:', err);
            return false;
        }
    }

    /**
     * Subscribe to changes (add/update/rename/remove/import/reload)
     */
    onChange(listener) {
        this.listeners.push(listener);
    }

    emitChange() {
        this.listeners.forEach(listener => listener(this.list()));
    }

    /**
     * Copies of all positions, in creation order
     */
    list() {
        return this.positions.map(p => ({ ...p, angles: [...p.angles] }));
    }

    get(id) {
        const found = this.positions.find(p => p.id === id);
        return found ? { ...found, angles: [...found.angles] } : null;
    }

    findByName(name) {
        const key = SavedPositionsStore.nameKey(name);
        const found = this.positions.find(p => SavedPositionsStore.nameKey(p.name) === key);
        return found ? { ...found, angles: [...found.angles] } : null;
    }

    /**
     * Save angles under a name. An existing position with the same name is overwritten.
     * @returns {Object} {position, overwritten, persisted}
     */
    save(name, angles) {
        const check = this.validateName(name);
        if (!check.ok) throw new Error(check.error);

        const clean = this.sanitizeAngles(angles);
        if (!clean) throw new Error('Ángulos inválidos');

        const now = Date.now();
        const existing = this.positions.find(
            p => SavedPositionsStore.nameKey(p.name) === SavedPositionsStore.nameKey(check.name)
        );

        let position;
        if (existing) {
            // Replacing keeps the existing name: only the pose changes
            existing.angles = clean;
            existing.updatedAt = now;
            position = existing;
        } else {
            position = {
                id: this.generateId(),
                name: check.name,
                angles: clean,
                createdAt: now,
                updatedAt: now
            };
            this.positions.push(position);
        }

        const persisted = this.persist();
        this.emitChange();
        return { position: this.get(position.id), overwritten: !!existing, persisted };
    }

    /**
     * Rename a position. Fails if another position already has the new name.
     * @returns {Object} {position, persisted}
     */
    rename(id, newName) {
        const position = this.positions.find(p => p.id === id);
        if (!position) throw new Error('La posición ya no existe');

        const check = this.validateName(newName, id);
        if (!check.ok) throw new Error(check.error);
        if (check.existing) throw new Error(`Ya existe una posición llamada "${check.existing.name}"`);

        position.name = check.name;
        position.updatedAt = Date.now();

        const persisted = this.persist();
        this.emitChange();
        return { position: this.get(id), persisted };
    }

    /**
     * Delete a position
     * @returns {Object|null} The removed position, or null if it did not exist
     */
    remove(id) {
        const index = this.positions.findIndex(p => p.id === id);
        if (index === -1) return null;

        const [removed] = this.positions.splice(index, 1);
        this.persist();
        this.emitChange();
        return removed;
    }

    /**
     * Serialize positions for a downloadable backup file
     * @param {Object} [extras]
     * @param {Object} [extras.motor] - Motor settings to record (e.g. {stepsPerRev, microstepping})
     * @param {Function} [extras.stepsFor] - angles -> motor steps, added to each position for firmware use
     */
    exportJSON({ motor = null, stepsFor = null } = {}) {
        const data = {
            app: 'RoboArm 3D',
            type: 'saved-positions',
            version: 1,
            exportedAt: new Date().toISOString()
        };
        if (motor) data.motor = motor;
        data.positions = this.positions.map(p => {
            const entry = {
                name: p.name,
                angles: p.angles,
                createdAt: p.createdAt,
                updatedAt: p.updatedAt
            };
            if (stepsFor) entry.steps = stepsFor(p.angles);
            return entry;
        });
        return JSON.stringify(data, null, 2);
    }

    /**
     * Merge positions from a backup file. Positions with an existing name are overwritten.
     * @param {string} text - File contents
     * @param {Object} [options]
     * @param {Function} [options.isReachable] - angles -> boolean; poses it rejects are
     *   skipped and counted as unreachable (e.g. below the floor with the current robot)
     * @returns {Object} {added, updated, skipped, unreachable, persisted}
     */
    importJSON(text, { isReachable = null } = {}) {
        let data;
        try {
            data = JSON.parse(text);
        } catch (err) {
            throw new Error('El archivo no es un JSON válido');
        }

        const items = Array.isArray(data) ? data : (data && Array.isArray(data.positions) ? data.positions : null);
        if (!items) throw new Error('El archivo no contiene posiciones guardadas');

        const result = { added: 0, updated: 0, skipped: 0, unreachable: 0, persisted: true };
        const now = Date.now();

        for (const raw of items) {
            // Imported ids are ignored so they can never collide with local ones
            const entry = this.sanitizeEntry({ ...raw, id: null });
            if (!entry) {
                result.skipped++;
                continue;
            }
            if (isReachable && !isReachable(entry.angles)) {
                result.unreachable++;
                continue;
            }

            const existing = this.positions.find(
                p => SavedPositionsStore.nameKey(p.name) === SavedPositionsStore.nameKey(entry.name)
            );
            if (existing) {
                existing.angles = entry.angles;
                existing.updatedAt = now;
                result.updated++;
            } else {
                this.positions.push(entry);
                result.added++;
            }
        }

        if (result.added || result.updated) {
            result.persisted = this.persist();
            this.emitChange();
        }
        return result;
    }

    /**
     * Re-read storage (e.g. another tab changed it)
     */
    reload() {
        this.load();
        this.emitChange();
    }
}

SavedPositionsStore.STORAGE_KEY = 'roboarm3d.savedPositions.v1';
SavedPositionsStore.MAX_NAME_LENGTH = 40;

// Export
window.SavedPositionsStore = SavedPositionsStore;
