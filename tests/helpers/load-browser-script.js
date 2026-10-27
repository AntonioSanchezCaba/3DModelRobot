/**
 * Load one of the app's browser scripts (js/*.js) in Node.
 *
 * The scripts export their class with `window.X = X`. Each script runs in its own
 * function scope with a fake `window` object, and the requested export is returned.
 * (Not node:vm — objects from another realm break assert.deepStrictEqual.)
 */
const fs = require('node:fs');
const path = require('node:path');

const ROOT = path.resolve(__dirname, '..', '..');

function loadBrowserScript(relativePath, exportName, windowGlobals = {}) {
    const file = path.join(ROOT, relativePath);
    const code = fs.readFileSync(file, 'utf8');
    const windowObject = { ...windowGlobals };

    // eslint-disable-next-line no-new-func
    new Function('window', `${code}\n//# sourceURL=${file}`)(windowObject);

    if (!windowObject[exportName]) {
        throw new Error(`${relativePath} did not define window.${exportName}`);
    }
    return windowObject[exportName];
}

module.exports = { loadBrowserScript, ROOT };
