// Scratch simulation for drift physics tuning (deleted after tuning).
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const TAU = Math.PI * 2;
const norm = a => Math.atan2(Math.sin(a), Math.cos(a));

function makeCar() {
  return { x: 0, y: 0, a: 0, vx: 0, vy: 0, w: 0, steer: 0, grip: 13 };
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
  const targetGrip = inp.handbrake ? P.gripHB : P.grip;
  c.grip += (targetGrip - c.grip) * (1 - Math.exp(-(inp.handbrake ? 16 : 4.5) * dt));
  let grip = c.grip;
  const slipNow = Math.abs(Math.atan2(vLat, Math.abs(vLong) + 1));
  if (inp.throttle && speed > 280 && slipNow > 0.18) grip *= P.powerGrip;
  vLat *= Math.exp(-grip * dt);
  const yawTarget = c.steer * P.yawMax * sf * (inp.handbrake ? P.yawHB : 1) * (vLong < -20 ? -1 : 1);
  c.w += (yawTarget - c.w) * (1 - Math.exp(-P.yawResponse * dt));

  const slip = Math.atan2(vLat, Math.abs(vLong) + 1);
  // Drift stabilizer: stop feeding rotation once the slide gets too deep.
  if (Math.abs(slip) > P.slipLimit && Math.sign(slip) !== Math.sign(c.w)) {
    const over = Math.abs(slip) - P.slipLimit;
    c.w *= Math.exp(-Math.min(1, over * 8) * 9 * dt);
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
}

const P = {
  engine: 720, maxSpeed: 900, brake: 1300, maxRev: 240,
  drag: 0.14, roll: 14, hbBrake: 90,
  grip: 13, gripHB: 2.8, powerGrip: 0.55,
  yawMax: 2.2, yawHB: 1.15, yawResponse: 9, maxYawVel: 3.0,
  align: 3.0, alignHB: 0.8, slipLimit: 0.62, steerRate: 5.5, steerReturn: 8
};

function run(label, script, seconds) {
  const c = makeCar();
  const dt = 1 / 120;
  const log = [];
  for (let t = 0; t < seconds; t += dt) {
    const inp = script(t, c);
    step(c, inp, dt, P);
    if (Math.abs((t * 120) % 60) < 1) log.push([+t.toFixed(2), Math.round(Math.hypot(c.vx, c.vy)), +(c.slip * 180 / Math.PI).toFixed(1), +(c.w).toFixed(2)]);
  }
  console.log(label, 'v=', Math.round(Math.hypot(c.vx, c.vy)), 'slipDeg=', (c.slip * 180 / Math.PI).toFixed(1), 'w=', c.w.toFixed(2));
  console.log(log.map(r => r.join('/')).join('  '));
}

run('ACCEL 4s', () => ({ throttle: true }), 4);
run('TURN 3s', t => ({ throttle: true, steer: t > 2.2 ? 1 : 0 }), 5);
run('HANDBRAKE', t => ({ throttle: true, steer: t > 2.2 ? 1 : 0, handbrake: t > 2.2 && t < 3.1 }), 5);
run('HB + counter', t => ({ throttle: true, steer: t > 2.2 && t < 3.0 ? 1 : t >= 3.0 && t < 4.0 ? -0.6 : 0, handbrake: t > 2.2 && t < 3.05 }), 5);
run('HB + counter no gas', t => ({ throttle: t < 3.05, steer: t > 2.2 && t < 3.0 ? 1 : t >= 3.0 && t < 4.0 ? -0.7 : 0, handbrake: t > 2.2 && t < 3.05 }), 5);
run('LONG DRIFT', t => ({ throttle: true, steer: t > 2.2 ? 1 : 0, handbrake: t > 2.2 && t < 4.4 }), 6);
run('LONG DRIFT no gas', t => ({ throttle: t < 2.2, steer: t > 2.2 ? 1 : 0, handbrake: t > 2.2 && t < 4.4 }), 6);
run('HOLD DRIFT', t => ({ throttle: true, steer: t > 2.2 && t < 2.7 ? 1 : t >= 2.7 ? 0.35 : 0, handbrake: t > 2.2 && t < 2.7 }), 6);
run('HOLD DRIFT 0.15', t => ({ throttle: true, steer: t > 2.2 && t < 2.7 ? 1 : t >= 2.7 ? 0.15 : 0, handbrake: t > 2.2 && t < 2.7 }), 6);
run('HOLD DRIFT -0.1', t => ({ throttle: true, steer: t > 2.2 && t < 2.7 ? 1 : t >= 2.7 ? -0.1 : 0, handbrake: t > 2.2 && t < 2.7 }), 6);
