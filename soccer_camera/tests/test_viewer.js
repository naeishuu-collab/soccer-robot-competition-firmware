// node tests/test_viewer.js
'use strict';
const assert = require('assert');
const { parseViz, current } = require('../tools/viewer.js');

const line = 'VIZ,1,1,1,90,77,20,45,160,-8,ATTACK,VISION_ONLY,0,0,66,180,50,0';
const d = parseViz(line + '\r');
assert.strictEqual(d.ball, 90);
assert.strictEqual(d.direction, 45);
assert.strictEqual(d.turn, -8);
assert.strictEqual(d.yellow, 0);
assert.strictEqual(d.blue, 180);
assert.strictEqual(d.bno, 'OFF');
assert.strictEqual(parseViz('VIZ,1,1,1,-1,0,0,-1,0,0,SAFE_STOP,DEGRADED,0,-1,0,-1,0,1').ball, null);
for (const bad of ['', 'cam=3 ready=1', line.slice(0, -4), line + ',1', line.replace(',90,', ',x,'), line.replace('VIZ,1', 'VIZ,2')]) {
  assert.strictEqual(parseViz(bad), null, bad);
}
assert.strictEqual(current(d, 100).ball, 90);
assert.strictEqual(current(d, 190).ball, null);      // 20 ms old at MAIN + 190 ms here
assert.strictEqual(current(d, 600), null);           // stale link: show nothing
assert.strictEqual(current(parseViz(line.replace('VIZ,1,1,', 'VIZ,1,0,')), 0).ball, null);  // IR link down
console.log('viewer tests passed');
