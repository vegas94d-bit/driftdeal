// Track validation for driftdeal physics (mirrors index.html, units px/s, mass = 1).
// Driver never touches the handbrake: the acceptance criterion is that a
// sustained drift (slip angle) is reachable with steering + throttle alone.
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const norm = a => Math.atan2(Math.sin(a), Math.cos(a));
const sgn = v => (v < 0 ? -1 : v > 0 ? 1 : 0);
const TAU = Math.PI * 2;

function tireForce(x, B, C, D, E) {
  const Bx = B * x;
  return D * Math.sin(C * Math.atan(Bx - E * (Bx - Math.atan(Bx))));
}

const PHYS = {
  ENGINE: 720, MAX_SPEED: 900, BRAKE: 1300, REVERSE: 470, MAX_REVERSE: 240,
  DRAG: 0.14, ROLL: 14, HB_BRAKE: 50,
  STEER_MAX: 0.55, STEER_RATE: 5.5, STEER_RETURN: 8,
  AF: 15, AR: 14, IZ: 500, SUB: 4,
  FRONT: { B: 6.0, C: 1.8, D: 1600, E: 0.3 },
  REAR: { B: 5.7, C: 1.8, D: 900, E: 0.3 },
  REAR_HB: { B: 1.8, C: 1.5, D: 430, E: 0.1 },
  OFF: { B: 3.2, C: 1.4, D: 900, E: 0.3 },
  GRIP_SPEED: 120, YAW_DAMP: 1.5, MAX_YAW: 3.4
};

// --- track: superellipse, same as index.html ---
const TRACK = { cx: 1850, cy: 1150, rx: 1250, ry: 700, n: 4.2, half: 100, runoff: 64 };
const N = 900;
const pts = [];
for (let i = 0; i < N; i++) {
  const t = (i / N) * TAU;
  const ct = Math.cos(t), st = Math.sin(t);
  pts.push({
    x: TRACK.cx + TRACK.rx * Math.sign(ct) * Math.pow(Math.abs(ct), 2 / TRACK.n),
    y: TRACK.cy + TRACK.ry * Math.sign(st) * Math.pow(Math.abs(st), 2 / TRACK.n)
  });
}
for (let i = 0; i < N; i++) {
  const a = pts[(i + 1) % N], b = pts[(i - 1 + N) % N];
  const tx = a.x - b.x, ty = a.y - b.y;
  const d = Math.hypot(tx, ty) || 1;
  pts[i].tx = tx / d; pts[i].ty = ty / d;
  pts[i].nx = -ty / d; pts[i].ny = tx / d;
}
// curvature radius along the centerline
const radius = new Array(N);
for (let i = 0; i < N; i++) {
  const p = pts[(i - 2 + N) % N], q = pts[i], r = pts[(i + 2) % N];
  const a = Math.hypot(q.x - p.x, q.y - p.y);
  const b = Math.hypot(r.x - q.x, r.y - q.y);
  const c = Math.hypot(r.x - p.x, r.y - p.y);
  const area = Math.abs((q.x - p.x) * (r.y - p.y) - (r.x - p.x) * (q.y - p.y)) / 2;
  radius[i] = area > 1e-6 ? (a * b * c) / (4 * area) : 1e6;
}

function locate(x, y, hint) {
  let best = Infinity, bi = hint;
  for (let k = -60; k <= 60; k++) {
    const i = (hint + k + N) % N;
    const dx = x - pts[i].x, dy = y - pts[i].y;
    const d = dx * dx + dy * dy;
    if (d < best) { best = d; bi = i; }
  }
  const p = pts[bi];
  return { idx: bi, lat: (x - p.x) * p.nx + (y - p.y) * p.ny, p };
}

// --- physics: identical to index.html substep/stepPhysics ---
const car = {
  x: 0, y: 0, angle: 0, vx: 0, vy: 0, w: 0, steer: 0,
  lat: 0, onTrack: true, offTrack: false, lastIdx: 0, hitCd: 0,
  vLong: 0, vLat: 0, slip: 0, speed: 0
};
let input = { throttle: false, brake: false, handbrake: false, steer: 0 };

function substep(dt, off) {
  const P = PHYS;
  const stT = input.steer;
  const rate = stT === 0 ? P.STEER_RETURN : P.STEER_RATE;
  car.steer += clamp(stT - car.steer, -rate * dt, rate * dt);
  const delta = car.steer * P.STEER_MAX;

  const ca = Math.cos(car.angle), sa = Math.sin(car.angle);
  let vLong = car.vx * ca + car.vy * sa;
  let vLat = -car.vx * sa + car.vy * ca;
  const speed = Math.hypot(vLong, vLat);

  let driveFx = 0;
  if (input.throttle) {
    if (vLong < -10) driveFx = P.BRAKE * 0.7;
    else driveFx = P.ENGINE * (1 - clamp(vLong / P.MAX_SPEED, 0, 1) * 0.8) * (off ? 0.45 : 1);
  }
  if (input.brake) driveFx = vLong > 15 ? -P.BRAKE : -P.REVERSE;
  if (input.handbrake) driveFx -= sgn(vLong) * P.HB_BRAKE;

  const den = Math.max(Math.abs(vLong), 30);
  const aF = Math.atan2(vLat + car.w * P.AF, den) - delta;
  const aR = Math.atan2(vLat - car.w * P.AR, den);

  const sf = clamp(speed / P.GRIP_SPEED, 0, 1);
  const tF = off ? P.OFF : P.FRONT;
  const tR = input.handbrake ? P.REAR_HB : (off ? P.OFF : P.REAR);
  const util = Math.abs(driveFx) / tR.D;
  const rMul = 1 / Math.sqrt(1 + util * util * 0.9);

  const FyF = -tireForce(aF, tF.B, tF.C, tF.D * sf, tF.E);
  const FyR = -tireForce(aR, tR.B, tR.C, tR.D * sf * rMul, tR.E);

  vLong += driveFx * dt;
  vLat += (FyF * Math.cos(delta) + FyR) * dt;
  car.w += ((P.AF * FyF * Math.cos(delta) - P.AR * FyR) / P.IZ) * dt;
  car.w -= car.w * P.YAW_DAMP * sf * dt;

  vLong -= vLong * P.DRAG * dt;
  vLong -= sgn(vLong) * Math.min(Math.abs(vLong), P.ROLL * dt);
  if (off) vLong *= Math.exp(-1.6 * dt);
  vLong = clamp(vLong, -P.MAX_REVERSE, P.MAX_SPEED);
  car.w = clamp(car.w, -P.MAX_YAW, P.MAX_YAW);
  car.angle = norm(car.angle + car.w * dt);

  car.vx = vLong * ca - vLat * sa;
  car.vy = vLong * sa + vLat * ca;
  car.x += car.vx * dt;
  car.y += car.vy * dt;
}

function stepPhysics(dt) {
  const P = PHYS;
  const dirt = car.offTrack;
  const h = dt / P.SUB;
  for (let s = 0; s < P.SUB; s++) substep(h, dirt);

  const loc = locate(car.x, car.y, car.lastIdx);
  car.lastIdx = loc.idx;
  car.lat = loc.lat;
  const off = Math.abs(loc.lat) > TRACK.half + 16;
  car.offTrack = off;
  car.onTrack = !off;

  const wallLimit = TRACK.half + TRACK.runoff - 20;
  if (Math.abs(loc.lat) > wallLimit) {
    const side = sgn(loc.lat);
    const push = Math.abs(loc.lat) - wallLimit;
    car.x -= loc.p.nx * side * push;
    car.y -= loc.p.ny * side * push;
    const nx = loc.p.nx * side, ny = loc.p.ny * side;
    const vn = car.vx * nx + car.vy * ny;
    if (vn > 0) {
      car.vx -= 1.35 * vn * nx;
      car.vy -= 1.35 * vn * ny;
      if (car.hitCd <= 0 && vn > P.MAX_SPEED * 0.08) {
        car.hitCd = 0.6;
        car.vx *= 0.72; car.vy *= 0.72; car.w *= 0.35;
      }
    }
    car.lat = side * wallLimit;
  }
  car.hitCd = Math.max(0, car.hitCd - dt);

  const c2 = Math.cos(car.angle), s2 = Math.sin(car.angle);
  car.vLong = car.vx * c2 + car.vy * s2;
  car.vLat = -car.vx * s2 + car.vy * c2;
  car.speed = Math.hypot(car.vx, car.vy);
  car.slip = Math.atan2(car.vLat, Math.abs(car.vLong) + 1);
}

// --- driver: pure pursuit + speed limit by curvature. NO handbrake. ---
function drive(aggr) {
  const ahead = aggr ? 30 : 42;
  const tgt = pts[(car.lastIdx + ahead) % N];
  const err = norm(Math.atan2(tgt.y - car.y, tgt.x - car.x) - car.angle);
  input.steer = clamp(err * (aggr ? 3.4 : 2.6) - car.lat / (aggr ? 160 : 260), -1, 1);

  // corner radius ~150 px ahead
  let R = 1e6;
  for (let k = 20; k < 90; k += 10) R = Math.min(R, radius[(car.lastIdx + k + N) % N]);
  const vLim = Math.sqrt(1250 * Math.max(R, 60));
  if (aggr) {
    input.throttle = true;
    input.brake = false;
  } else {
    input.throttle = car.speed < vLim;
    input.brake = car.speed > vLim * 1.12;
  }
  input.handbrake = false;
}

// --- run ---
function run(aggr, label) {
  const start = pts[0];
  Object.assign(car, {
    x: start.x, y: start.y,
    angle: Math.atan2(start.ty, start.tx),
    vx: 0, vy: 0, w: 0, steer: 0, lat: 0,
    onTrack: true, offTrack: false, lastIdx: 0, hitCd: 0,
    vLong: 0, vLat: 0, slip: 0, speed: 0
  });
  const dt = 1 / 120;
  let laps = 0, prevIdx = 0, driftT = 0, peakSlip = 0, offT = 0, wallHits = 0, spin = false;
  let speedSum = 0, steps = 0, maxLat = 0;
  for (let t = 0; t < 90; t += dt) {
    drive(aggr);
    stepPhysics(dt);
    if (!isFinite(car.x) || !isFinite(car.w)) { console.log('EXPLODED at t=' + t.toFixed(2)); break; }
    steps++; speedSum += car.speed;
    const ad = Math.abs(car.slip);
    peakSlip = Math.max(peakSlip, ad);
    maxLat = Math.max(maxLat, Math.abs(car.lat));
    if (ad > 0.3 && car.speed > 200) driftT += dt;
    if (car.offTrack) offT += dt;
    if (ad > 1.3 && car.speed > 150) spin = true;
    if (car.hitCd >= 0.59) wallHits++;
    if (prevIdx > N - 40 && car.lastIdx < 40) laps++;
    prevIdx = car.lastIdx;
  }
  console.log(label.padEnd(10),
    'laps=' + laps,
    'drift=' + driftT.toFixed(1) + 's (' + (driftT / 90 * 100).toFixed(0) + '%)',
    'peakSlip=' + (peakSlip * 180 / Math.PI).toFixed(1) + 'deg',
    'offT=' + offT.toFixed(1) + 's',
    'wall=' + wallHits,
    'spin=' + spin,
    'maxLat=' + Math.round(maxLat),
    'avgV=' + Math.round(speedSum / steps) + 'px/s=' + Math.round(speedSum / steps * 0.24) + 'km/h');
}

run(false, 'clean lap');
run(true, 'full gas');
