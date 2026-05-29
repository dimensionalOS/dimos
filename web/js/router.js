// Tiny view dispatcher. Views register themselves at import time (in main.js)
// so router.js doesn't have to import from views/* — that import direction is
// what caused the cycles in the single-file split.

const routes = {};

export function register(view, renderer) {
    routes[view] = renderer;
}

export function navigate(view) {
    const app = document.getElementById('app');
    const fn = routes[view];
    if (fn) fn(app);
    else console.warn('[router] unknown view:', view);
}
