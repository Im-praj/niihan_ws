// Run without ROS or browser dependencies: node test/test_web_viewer2d.js
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const Viewer2D = require('../web/js/viewer2d.js');

function close(actual, expected) {
    assert(Math.abs(actual - expected) < 1e-8, `${actual} != ${expected}`);
}

function fixture() {
    const calls = [];
    const events = {};
    const context = {
        createImageData: (width, height) => ({data: new Uint8ClampedArray(width * height * 4)}),
        putImageData: image => { context.image = image; },
        save() {}, restore() {},
        translate: (...args) => calls.push(['translate', ...args]),
        rotate: (...args) => calls.push(['rotate', ...args]),
        scale: (...args) => calls.push(['scale', ...args]),
        drawImage: (...args) => calls.push(['drawImage', ...args]),
    };
    const parentRect = {width: 800, height: 600};
    const canvasRect = {left: 10, top: 20, width: 400, height: 300};
    const canvas = {
        width: 800, height: 600,
        getContext: () => context,
        parentElement: {getBoundingClientRect: () => parentRect},
        getBoundingClientRect: () => canvasRect,
        addEventListener: (name, callback) => { events[name] = callback; },
        setPointerCapture() {},
    };
    global.document = {
        getElementById: () => canvas,
        createElement: () => ({getContext: () => context}),
    };
    global.window = {devicePixelRatio: 1, addEventListener() {}};
    global.requestAnimationFrame = () => {};
    const viewer = new Viewer2D('map-canvas');
    return {viewer, context, calls, events, parentRect, canvasRect};
}

function map(overrides = {}) {
    return Object.assign({width: 4, height: 2, resolution: 0.5,
        origin: {x: 0, y: 0, yaw: 0}, data: [-1, 0, 100, 50, 0, 0, 100, -1]}, overrides);
}

let count = 0;
function test(name, run) {
    run();
    count++;
    console.log(`PASS ${name}`);
}

test('zero map origin remains valid and free/unknown/occupied cells are distinct', () => {
    const {viewer, context} = fixture();
    assert.strictEqual(viewer.updateMap(map()), true);
    assert.deepStrictEqual(viewer.center, {x: 1, y: 0.5});
    assert(context.image.data[0] > context.image.data[8]); // unknown brighter than obstacle
    assert(context.image.data[4] > context.image.data[0]); // free brighter than unknown
    assert.strictEqual(context.image.data[3], 255);
});

test('rotated map corners, raster transform, and fit agree', () => {
    const {viewer, calls} = fixture();
    viewer.updateMap(map({origin: {x: 10, y: -7, yaw: Math.PI / 2}}));
    const corner = viewer.mapToWorld(2, 1);
    close(corner.x, 9);
    close(corner.y, -5);
    close(viewer.center.x, 9.5);
    close(viewer.center.y, -6);
    close(viewer.scale, 270);
    viewer.drawMap();
    close(calls.find(call => call[0] === 'rotate')[1], -Math.PI / 2);
    const scale = calls.find(call => call[0] === 'scale');
    close(scale[1], 135);
    close(scale[2], -135); // occupancy row 0 grows towards world +Y
    [[0, 0], [2, 0], [0, 1], [2, 1]].forEach(([x, y]) => {
        const world = viewer.mapToWorld(x, y);
        const pixel = viewer.worldToCanvas(world.x, world.y);
        assert(pixel.x >= 0 && pixel.x <= 800 && pixel.y >= 0 && pixel.y <= 600);
        const roundTrip = viewer.canvasToWorld(pixel.x, pixel.y);
        close(roundTrip.x, world.x);
        close(roundTrip.y, world.y);
    });
});

test('click on CSS-scaled canvas selects the displayed world point', () => {
    const {viewer} = fixture();
    viewer.center = {x: 12, y: -4};
    viewer.scale = 100;
    let goal;
    viewer.setOnClickCallback((x, y) => { goal = {x, y}; });
    viewer.handleClick({clientX: 10 + 300, clientY: 20 + 100});
    assert.deepStrictEqual(goal, {x: 14, y: -3});
});

test('map view remains fixed when robot moves so mission clicks do not drift', () => {
    const {viewer} = fixture();
    viewer.updateMap(map());
    const before = viewer.worldToCanvas(0.5, 0.5);
    viewer.updateRobotPose(20, 30, 0);
    assert.deepStrictEqual(viewer.worldToCanvas(0.5, 0.5), before);
});

test('hidden tab preserves buffer and visible retina resize refits', () => {
    const {viewer, parentRect} = fixture();
    viewer.updateMap(map());
    parentRect.width = 0;
    parentRect.height = 0;
    viewer.onWindowResize();
    assert.strictEqual(viewer.canvas.width, 800);
    parentRect.width = 1000;
    parentRect.height = 500;
    window.devicePixelRatio = 2;
    viewer.onWindowResize();
    assert.strictEqual(viewer.canvas.width, 2000);
    assert.strictEqual(viewer.canvas.height, 1000);
    close(viewer.scale, 900);
});

test('zoom preserves the world position beneath pointer', () => {
    const {viewer} = fixture();
    viewer.updateMap(map());
    const event = {clientX: 310, clientY: 120, deltaY: -100, preventDefault() {}};
    const point = viewer.eventPoint(event);
    const before = viewer.canvasToWorld(point.x, point.y);
    viewer.handleZoom(event);
    const after = viewer.canvasToWorld(point.x, point.y);
    close(before.x, after.x);
    close(before.y, after.y);
    assert.strictEqual(viewer.autoFit, false);
});

test('pan gestures never submit a waypoint or goal', () => {
    const {viewer, events} = fixture();
    let clicks = 0;
    viewer.setOnClickCallback(() => clicks++);
    events.pointerdown({button: 0, clientX: 110, clientY: 120, pointerId: 1});
    events.pointermove({clientX: 160, clientY: 120});
    events.pointerup();
    viewer.handleClick({clientX: 160, clientY: 120});
    assert.strictEqual(clicks, 0);
    assert.strictEqual(viewer.autoFit, false);
    events.pointerdown({button: 0, clientX: 160, clientY: 120, pointerId: 1});
    events.pointerup();
    viewer.handleClick({clientX: 160, clientY: 120});
    assert.strictEqual(clicks, 1);
});

test('invalid grids are rejected without losing the last good map', () => {
    const {viewer} = fixture();
    const valid = map();
    viewer.updateMap(valid);
    [map({resolution: 0}), map({data: []}), map({width: -1}),
        map({origin: {x: NaN, y: 0}}), map({origin: {x: 0, y: 0, yaw: NaN}})]
        .forEach(invalid => assert.strictEqual(viewer.updateMap(invalid), false));
    assert.strictEqual(viewer.mapData, valid);
});

test('2D tab has no inline display override', () => {
    const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
    const tag = html.match(/<div[^>]*id="view-2d"[^>]*>/)[0];
    assert(!/display\s*:\s*none/.test(tag));
});

console.log(`${count} web map regression checks passed`);
