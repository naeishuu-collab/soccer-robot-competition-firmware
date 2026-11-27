// Parsing for viewer.html (MAIN USB "VIZ" lines). Display only.
'use strict';
const STALE_MS = 500;
const STALE_BALL_MS = 200;

function parseViz(line) {
  const f = line.trim().split(',');
  if (f.length !== 18 || f[0] !== 'VIZ' || f[1] !== '1') return null;
  const number = (i) => (/^-?\d+$/.test(f[i]) ? Number(f[i]) : NaN);
  const numeric = [2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17];
  if (numeric.some((i) => Number.isNaN(number(i)))) return null;
  const angle = (i) => (number(i) >= 0 && number(i) < 360 ? number(i) : null);
  return {
    irLink: number(2) === 1, cameraLink: number(3) === 1, ball: angle(4), ballConfidence: number(5),
    ballAgeMs: number(6), direction: angle(7), power: number(8), turn: number(9), tactic: f[10], mode: f[11],
    bno: ['OFF', 'NO DATA', 'USED'][number(12)] || '?', yellow: angle(13), yellowConfidence: number(14),
    blue: angle(15), blueConfidence: number(16), hold: number(17) === 1,
  };
}

// What may be shown as current, given how long ago the line arrived.
function current(data, ageMs) {
  if (!data || ageMs > STALE_MS) return null;
  const ballFresh = data.ball !== null && data.irLink && data.ballAgeMs + ageMs <= STALE_BALL_MS;
  return Object.assign({}, data, { ball: ballFresh ? data.ball : null });
}

if (typeof module !== 'undefined') module.exports = { parseViz, current };
