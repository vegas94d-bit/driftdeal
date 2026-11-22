// Scratch simulation for drift physics tuning (deleted after tuning).
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const TAU = Math.PI * 2;
const norm = a => Math.atan2(Math.sin(a), Math.cos(a));
const lerp = (a, b, t) => a + (b - a) * t;

function makeCar() {
  return { x: 0, y: 0, a: 0, vx: 0, vy: 0, w: 0, steer: 0, grip: 13, breakaway: 0 };
}

function step(c, inp, dt, P) {
  const target = inp.steer || 0;
  const rate = target === 0 ? P.steerReturn : P.steerRate;
  c.steer += clamp(target - c.steer, -rate * dt, rate * dt);

  const ca = Math.cos(c.a), sa = Math.sin(c.a);
  let vLong = c.vx * ca + c.vy * sa;
  let vLat = -c.vx * sa + c.vy * ca;

  if (inp.throttle) vLong += P.engine * (1 - clamp(vLong / P.maxSpeed, 0, 1) * 0.8) * dt;
  if (inp.brake) {
    if (vLong > 15) vLong -= P.brake * dt;
    else vLong = Math.max(vLong - P.engine * 0.65 * dt, -P.maxRev);
  }
  if (inp.handbrake) vLong -= Math.sign(vLong) * P.hbBrake * dt;
  vLong -= vLong * P.drag * dt;
  vLong -= Math.sign(vLong) * Math.min(Math.abs(vLong), P.roll * dt);

  const speed = Math.hypot(vLong, vLat);
  const sf = clamp(speed / 150, 0, 1);

  // Срыв задней оси: нужно боковое ускорение больше предела сцепления
  const demand = Math.abs(c.w) * speed;
  const slide = clamp((demand - P.gripLat) / P.gripBand, 0, 1);
  const base = inp.handbrake ? P.gripHB : P.grip;
  let grip = Math.min(base, lerp(base, P.gripDrift, slide));
  const slipNow = Math.abs(Math.atan2(vLat, Math.abs(vLong) + 1));
  if (inp.throttle && speed > 280 && slipNow > 0.16) grip *= P.powerGrip;
  vLat *= Math.exp(-grip * dt);

  const yawTarget = c.steer * P.yawMax * sf * (inp.handbrake ? P.yawHB : 1) * (vLong < -20 ? -1 : 1);
  c.w += (yawTarget - c.w) * (1 - Math.exp(-P.yawResponse * dt));

  const slip = Math.atan2(vLat, Math.abs(vLong) + 1);
  if (Math.abs(slip) > P.slipLimit && Math.sign(slip) !== Math.sign(c.w)) {
    c.w *= Math.exp(-Math.min(1, (Math.abs(slip) - P.slipLimit) * 8) * 9 * dt);
  }
  const align = (inp.handbrake ? P.alignHB : P.align) * sf;
  c.w += slip * align * dt;
  c.w = clamp(c.w, -P.maxYawVel, P.maxYawVel);

  c.a = norm(c.a + c.w * dt);

  c.vx = vLong * ca - vLat * sa;
  c.vy = vLong * sa + vLat * ca;
  c.x += c.vx * dt;
  c.y += c.vy * dt;
  c.vLong = vLong;
  c.vLat = vLat;
  c.slip = Math.abs(Math.atan2(vLat, Math.abs(vLong) + 1));
  c.breakaway = slide;
}

const P = {
  engine: 720, maxSpeed: 900, brake: 1300, maxRev: 240,
  drag: 0.14, roll: 14, hbBrake: 50,
  grip: 13, gripHB: 2.0, gripDrift: 4.2,
  gripLat: 620, gripBand: 500, powerGrip: 0.85,
  yawMax: 2.2, yawHB: 1.15, yawResponse: 9, maxYawVel: 3.0,
  align: 3.0, alignHB: 0.8, slipLimit: 0.7,
  steerRate: 5.5, steerReturn: 8
};

function run(label, script, seconds, every) {
  const c = makeCar();
  const dt = 1 / 120;
  const log = [];
  let maxSlip = 0;
  for (let t = 0; t < seconds; t += dt) {
    step(c, script(t, c), dt, P);
    maxSlip = Math.max(maxSlip, t > 2.3 ? c.slip : 0);
    if (every && Math.abs((t * 120) % 60) < 1) {
      log.push([+t.toFixed(2), Math.round(Math.hypot(c.vx, c.vy)), +(c.slip * 180 / Math.PI).toFixed(1), +(c.w).toFixed(2), +c.breakaway.toFixed(2)].join('/'));
    }
  }
  console.log(label.padEnd(26), 'v=', String(Math.round(Math.hypot(c.vx, c.vy))).padStart(4),
    'slipDeg=', (c.slip * 180 / Math.PI).toFixed(1).padStart(5),
    'maxSlip=', (maxSlip * 180 / Math.PI).toFixed(1).padStart(5),
    'w=', c.w.toFixed(2), 'slide=', c.breakaway.toFixed(2));
  if (every) console.log('   ', log.join('  '));
}

// разгон прямо
run('ACCEL straight', () => ({ throttle: true }), 4, false);
// плавный набор скорости и умеренный руль
run('gentle steer 0.5', t => ({ throttle: true, steer: t > 2 ? 0.5 : 0 }), 5, false);
run('full steer no gas', t => ({ throttle: t < 2.2, steer: t > 2.2 ? 1 : 0 }), 5, false);
run('FULL STEER + gas', t => ({ throttle: true, steer: t > 2 ? 1 : 0 }), 5, false);
run('FULL STEER + gas long', t => ({ throttle: true, steer: t > 2 ? 1 : 0 }), 8, true);
run('steer 0.8 + gas', t => ({ throttle: true, steer: t > 2 ? 0.8 : 0 }), 6, true);
run('HANDBRAKE then release', t => ({ throttle: true, steer: t > 2 ? 1 : 0, handbrake: t > 2 && t < 2.6 }), 6, true);
run('HB + counter no gas', t => ({ throttle: t < 2.6, steer: t > 2 && t < 2.6 ? 1 : t >= 2.6 ? -0.7 : 0, handbrake: t > 2 && t < 2.6 }), 6, false);
run('low speed full steer', t => ({ throttle: t > 2 ? true : false, steer: t > 2.4 ? 1 : 0 }), 4, false);
