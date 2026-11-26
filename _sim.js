// Two-axle Pacejka model + automatic parameter scan for driftdeal.
// Units: px, s. Mass normalized to 1 => tire force == acceleration.
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const norm = a => Math.atan2(Math.sin(a), Math.cos(a));
const sgn = v => (v < 0 ? -1 : v > 0 ? 1 : 0);

function mf(x, B, C, D, E) {
  const Bx = B * x;
  return D * Math.sin(C * Math.atan(Bx - E * (Bx - Math.atan(Bx))));
}

function makeP(o) {
  return Object.assign({
    ENGINE: 720, MAX_SPEED: 900, BRAKE: 1300, REVERSE: 470, MAX_REVERSE: 240,
    DRAG: 0.14, ROLL: 14, HB_BRAKE: 50,
    STEER_MAX: 0.55, STEER_RATE: 5.5, STEER_RETURN: 8,
    AF: 15, AR: 14, IZ: 2600, SUB: 6,
    FRONT: { B: 4.2, C: 1.4, D: 1500, E: 0.3 },
    REAR:  { B: 4.0, C: 1.4, D: 1250, E: 0.3 },
    REAR_HB: { B: 1.8, C: 1.5, D: 430, E: 0.1 },
    OFF:   { B: 3.2, C: 1.4, D: 900, E: 0.3 },
    GRIP_SPEED: 120, YAW_DAMP: 2.0, MAX_YAW: 3.4
  }, o || {});
}

function makeCar() {
  return { x: 0, y: 0, angle: 0, vx: 0, vy: 0, w: 0, steer: 0,
    vLong: 0, vLat: 0, slip: 0, speed: 0 };
}

function step(c, inp, dt, P) {
  const h = dt / P.SUB;
  for (let s = 0; s < P.SUB; s++) substep(c, inp, h, P);
}

function substep(c, inp, dt, P) {
  const stT = inp.steer || 0;
  const rate = stT === 0 ? P.STEER_RETURN : P.STEER_RATE;
  c.steer += clamp(stT - c.steer, -rate * dt, rate * dt);
  const delta = c.steer * P.STEER_MAX;

  const ca = Math.cos(c.angle), sa = Math.sin(c.angle);
  let vLong = c.vx * ca + c.vy * sa;
  let vLat = -c.vx * sa + c.vy * ca;
  const speed = Math.hypot(vLong, vLat);

  let driveFx = 0;
  if (inp.throttle) {
    if (vLong < -10) driveFx = P.BRAKE * 0.7;
    else driveFx = P.ENGINE * (1 - clamp(vLong / P.MAX_SPEED, 0, 1) * 0.8);
  }
  if (inp.brake) driveFx = vLong > 15 ? -P.BRAKE : -P.REVERSE;
  if (inp.handbrake) driveFx -= sgn(vLong) * P.HB_BRAKE;

  const den = Math.max(Math.abs(vLong), 30);
  const aF = Math.atan2(vLat + c.w * P.AF, den) - delta;
  const aR = Math.atan2(vLat - c.w * P.AR, den);

  const sf = clamp(speed / P.GRIP_SPEED, 0, 1);
  const tF = inp.off ? P.OFF : P.FRONT;
  const tR = inp.handbrake ? P.REAR_HB : (inp.off ? P.OFF : P.REAR);
  const util = Math.abs(driveFx) / tR.D;
  const rMul = 1 / Math.sqrt(1 + util * util * 0.9);

  const FyF = -mf(aF, tF.B, tF.C, tF.D * sf, tF.E);
  const FyR = -mf(aR, tR.B, tR.C, tR.D * sf * rMul, tR.E);

  vLong += driveFx * dt;
  vLat += (FyF * Math.cos(delta) + FyR) * dt;
  c.w += ((P.AF * FyF * Math.cos(delta) - P.AR * FyR) / P.IZ) * dt;
  c.w -= c.w * P.YAW_DAMP * sf * dt;

  vLong -= vLong * P.DRAG * dt;
  vLong -= sgn(vLong) * Math.min(Math.abs(vLong), P.ROLL * dt);
  if (inp.off) vLong *= Math.exp(-1.6 * dt);
  vLong = clamp(vLong, -P.MAX_REVERSE, P.MAX_SPEED);
  c.w = clamp(c.w, -P.MAX_YAW, P.MAX_YAW);
  c.angle = norm(c.angle + c.w * dt);

  c.vx = vLong * ca - vLat * sa;
  c.vy = vLong * sa + vLat * ca;
  c.x += c.vx * dt;
  c.y += c.vy * dt;
  c.vLong = vLong; c.vLat = vLat;
  c.speed = Math.hypot(c.vx, c.vy);
  c.slip = Math.atan2(vLat, Math.abs(vLong) + 1);
}

// --- scenarios: [name, input fn, seconds, slipPeak target (deg) as [min,max]] ---
const SCEN = [
  ['straight',      () => ({ throttle: true }), 4, [0, 6]],
  ['steer 0.4',     t => ({ throttle: t < 3.5, steer: t > 2 ? 0.4 : 0 }), 7, [0, 12]],
  ['steer 0.7',     t => ({ throttle: t < 3.5, steer: t > 2 ? 0.7 : 0 }), 7, [5, 25]],
  ['full+gas',      t => ({ throttle: true, steer: t > 2 ? 1 : 0 }), 8, [15, 50]],
  ['full coast',    t => ({ throttle: t < 2.2, steer: t > 2.2 ? 1 : 0 }), 8, [12, 45]],
  ['HB pulse',      t => ({ throttle: true, steer: t > 2 ? 0.85 : 0, handbrake: t > 2 && t < 2.5 }), 8, [20, 60]],
  ['HB+counter',    t => ({ throttle: true, steer: t > 2 && t < 2.5 ? 0.85 : t >= 2.5 ? -0.4 : 0,
                            handbrake: t > 2 && t < 2.5 }), 9, [20, 60]],
  ['low speed',     t => ({ throttle: t > 1.2, steer: t > 1.6 ? 1 : 0 }), 3.5, [0, 40]],
  ['S-turn',        t => ({ throttle: true, steer: t < 2 ? 0 : t < 3.2 ? 0.9 : t < 4.4 ? -0.9 : 0.7 }), 8, [10, 50]]
];

function trial(P, verbose) {
  const dt = 1 / 120;
  let fails = [];
  for (const [name, fn, secs, [lo, hi]] of SCEN) {
    const c = makeCar();
    let peak = 0, spin = false, bad = false;
    for (let t = 0; t < secs; t += dt) {
      step(c, fn(t, c), dt, P);
      if (!isFinite(c.x) || !isFinite(c.w)) { bad = true; break; }
      if (t > 2.4) peak = Math.max(peak, Math.abs(c.slip));
      if (Math.abs(c.slip) > 1.35) spin = true;
    }
    const deg = peak * 180 / Math.PI;
    const ok = !bad && !spin && deg >= lo && deg <= hi;
    if (!ok) fails.push(name + ':' + (bad ? 'EXPLODE' : spin ? 'SPUN' : deg.toFixed(0) + 'deg[' + lo + '-' + hi + ']'));
    if (verbose) {
      console.log('  ' + name.padEnd(12), 'slipPk=' + deg.toFixed(1).padStart(6),
        'v=' + String(Math.round(c.speed)).padStart(4), ok ? 'ok' : 'FAIL');
    }
  }
  return fails;
}

// --- scan ---
let best = null;
for (const IZ of [500, 650, 800, 1000]) {
  for (const Dr of [900, 1000, 1100]) {
    for (const Bd of [4.0, 5.0, 6.0, 7.0]) {
      for (const C of [1.5, 1.65, 1.8]) {
        for (const YD of [1.5, 2.5, 4.0]) {
          for (const SM of [0.45, 0.55]) {
            const P = makeP({ IZ, SUB: 4, YAW_DAMP: YD, STEER_MAX: SM,
              FRONT: { B: Bd, C: C, D: 1600, E: 0.3 },
              REAR:  { B: Bd * 0.95, C: C, D: Dr, E: 0.3 } });
            const f = trial(P, false);
            if (!best || f.length < best.f.length) {
              best = { IZ, Dr, Bd, C, YD, SM, f };
            }
          }
        }
      }
    }
  }
}
console.log('\nbest:', JSON.stringify(best && { IZ: best.IZ, Dr: best.Dr, Bd: best.Bd, C: best.C, YD: best.YD, SM: best.SM }),
  'fails=' + best.f.length);
console.log(best.f.join(' | ') || 'all scenarios pass');

// verbose report of the winner
const BP = makeP({ IZ: best.IZ, SUB: 4, YAW_DAMP: best.YD, STEER_MAX: best.SM,
  FRONT: { B: best.Bd, C: best.C, D: 1600, E: 0.3 },
  REAR: { B: best.Bd * 0.95, C: best.C, D: best.Dr, E: 0.3 } });
console.log('\n--- winner detail ---');
trial(BP, true);
