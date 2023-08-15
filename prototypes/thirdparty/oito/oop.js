class Vec3 extends Array {
  static UP = [0, 1, 0];
  static DOWN = [0, -1, 0];
  static LEFT = [-1, 0, 0];
  static RIGHT = [1, 0, 0];
  static FORWARD = [0, 0, 1];
  static BACK = [0, 0, -1];
  constructor(v, y, z) {
    super(3);
    if (v instanceof Vec3 || v instanceof Float32Array || v instanceof Array && v.length == 3) {
      this[0] = v[0];
      this[1] = v[1];
      this[2] = v[2];
    } else if (typeof v === "number" && typeof y === "number" && typeof z === "number") {
      this[0] = v;
      this[1] = y;
      this[2] = z;
    } else if (typeof v === "number") {
      this[0] = v;
      this[1] = v;
      this[2] = v;
    } else {
      this[0] = 0;
      this[1] = 0;
      this[2] = 0;
    }
  }
  get len() {
    return Math.sqrt(this[0] ** 2 + this[1] ** 2 + this[2] ** 2);
  }
  get lenSqr() {
    return this[0] ** 2 + this[1] ** 2 + this[2] ** 2;
  }
  get isZero() {
    return this[0] === 0 && this[1] === 0 && this[2] === 0;
  }
  clone() {
    return new Vec3(this);
  }
  minAxis() {
    if (this[0] < this[1] && this[0] < this[2])
      return 0;
    if (this[1] < this[2])
      return 1;
    return 2;
  }
  maxAxis() {
    if (this[0] > this[1] && this[0] > this[2])
      return 0;
    if (this[1] > this[2])
      return 1;
    return 2;
  }
  xyz(x, y, z) {
    this[0] = x;
    this[1] = y;
    this[2] = z;
    return this;
  }
  copy(a) {
    this[0] = a[0];
    this[1] = a[1];
    this[2] = a[2];
    return this;
  }
  copyTo(a) {
    a[0] = this[0];
    a[1] = this[1];
    a[2] = this[2];
    return this;
  }
  setInfinite(sign = 1) {
    this[0] = Infinity * sign;
    this[1] = Infinity * sign;
    this[2] = Infinity * sign;
    return this;
  }
  rnd(x0 = 0, x1 = 1, y0 = 0, y1 = 1, z0 = 0, z1 = 1) {
    let t;
    t = Math.random();
    this[0] = x0 * (1 - t) + x1 * t;
    t = Math.random();
    this[1] = y0 * (1 - t) + y1 * t;
    t = Math.random();
    this[2] = z0 * (1 - t) + z1 * t;
    return this;
  }
  fromAdd(a, b) {
    this[0] = a[0] + b[0];
    this[1] = a[1] + b[1];
    this[2] = a[2] + b[2];
    return this;
  }
  fromSub(a, b) {
    this[0] = a[0] - b[0];
    this[1] = a[1] - b[1];
    this[2] = a[2] - b[2];
    return this;
  }
  fromMul(a, b) {
    this[0] = a[0] * b[0];
    this[1] = a[1] * b[1];
    this[2] = a[2] * b[2];
    return this;
  }
  fromScale(a, s) {
    this[0] = a[0] * s;
    this[1] = a[1] * s;
    this[2] = a[2] * s;
    return this;
  }
  fromCross(a, b) {
    const ax = a[0], ay = a[1], az = a[2], bx = b[0], by = b[1], bz = b[2];
    this[0] = ay * bz - az * by;
    this[1] = az * bx - ax * bz;
    this[2] = ax * by - ay * bx;
    return this;
  }
  fromNegate(a) {
    this[0] = -a[0];
    this[1] = -a[1];
    this[2] = -a[2];
    return this;
  }
  fromInvert(a) {
    this[0] = 1 / a[0];
    this[1] = 1 / a[1];
    this[2] = 1 / a[2];
    return this;
  }
  fromPerpendicular(a) {
    this[0] = -a[1];
    this[1] = a[0];
    this[2] = a[2];
    return this;
  }
  fromQuat(q, v) {
    const qx = q[0], qy = q[1], qz = q[2], qw = q[3], vx = v[0], vy = v[1], vz = v[2], x1 = qy * vz - qz * vy, y1 = qz * vx - qx * vz, z1 = qx * vy - qy * vx, x2 = qw * x1 + qy * z1 - qz * y1, y2 = qw * y1 + qz * x1 - qx * z1, z2 = qw * z1 + qx * y1 - qy * x1;
    this[0] = vx + 2 * x2;
    this[1] = vy + 2 * y2;
    this[2] = vz + 2 * z2;
    return this;
  }
  fromPolar(lon, lat) {
    const phi = (90 - lat) * 0.01745329251, theta = lon * 0.01745329251, sp = Math.sin(phi);
    this[0] = -sp * Math.sin(theta);
    this[1] = Math.cos(phi);
    this[2] = sp * Math.cos(theta);
    return this;
  }
  fromStruct(v) {
    this[0] = v.x;
    this[1] = v.y;
    this[2] = v.z;
    return this;
  }
  toStruct(v) {
    v.x = this[0];
    v.y = this[1];
    v.z = this[2];
    return this;
  }
  fromVec2(v, isYUp = false) {
    this[0] = v[0];
    if (isYUp) {
      this[1] = 0;
      this[2] = v[1];
    } else {
      this[1] = v[1];
      this[2] = 0;
    }
    return this;
  }
  fromNorm(v) {
    let mag = Math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2);
    if (mag == 0)
      return this;
    mag = 1 / mag;
    this[0] = v[0] * mag;
    this[1] = v[1] * mag;
    this[2] = v[2] * mag;
    return this;
  }
  fromTriNorm(a, b, c) {
    const ab = new Vec3().fromSub(b, a);
    const ac = new Vec3().fromSub(c, a);
    return this.fromCross(ab, ac).norm();
  }
  fromAxisAngle(axis, rad, v = Vec3.FORWARD) {
    const cp = new Vec3().fromCross(axis, v), dot = Vec3.dot(axis, v), s = Math.sin(rad), c = Math.cos(rad), ci = 1 - c;
    this[0] = v[0] * c + cp[0] * s + axis[0] * dot * ci;
    this[1] = v[1] * c + cp[1] * s + axis[1] * dot * ci;
    this[2] = v[2] * c + cp[2] * s + axis[2] * dot * ci;
    return this;
  }
  fromOrthogonal(v) {
    if (v[0] >= 0.57735026919) {
      this[0] = v[1];
      this[1] = -v[0];
      this[2] = 0;
    } else {
      this[0] = 0;
      this[1] = v[2];
      this[2] = -v[1];
    }
    return this;
  }
  fromReflect(dir, norm) {
    const factor = -2 * Vec3.dot(norm, dir);
    this[0] = factor * norm[0] + dir[0];
    this[1] = factor * norm[1] + dir[1];
    this[2] = factor * norm[2] + dir[2];
    return this;
  }
  fromPlaneProj(v, planeNorm, planePos) {
    const planeConst = -Vec3.dot(planePos, planeNorm);
    const scl = Vec3.dot(planeNorm, v) + planeConst;
    this.fromScale(planeNorm, -scl).add(v);
    return this;
  }
  fromBuf(ary, idx) {
    this[0] = ary[idx];
    this[1] = ary[idx + 1];
    this[2] = ary[idx + 2];
    return this;
  }
  toBuf(ary, idx) {
    ary[idx] = this[0];
    ary[idx + 1] = this[1];
    ary[idx + 2] = this[2];
    return this;
  }
  pushTo(ary) {
    ary.push(this[0], this[1], this[2]);
    return this;
  }
  fromLerp(a, b, t) {
    const ti = 1 - t;
    this[0] = a[0] * ti + b[0] * t;
    this[1] = a[1] * ti + b[1] * t;
    this[2] = a[2] * ti + b[2] * t;
    return this;
  }
  fromNLerp(a, b, t) {
    const ti = 1 - t;
    this[0] = a[0] * ti + b[0] * t;
    this[1] = a[1] * ti + b[1] * t;
    this[2] = a[2] * ti + b[2] * t;
    this.norm();
    return this;
  }
  fromSlerp(a, b, t) {
    const angle = Math.acos(Math.min(Math.max(Vec3.dot(a, b), -1), 1));
    const sin = Math.sin(angle);
    const ta = Math.sin((1 - t) * angle) / sin;
    const tb = Math.sin(t * angle) / sin;
    this[0] = ta * a[0] + tb * b[0];
    this[1] = ta * a[1] + tb * b[1];
    this[2] = ta * a[2] + tb * b[2];
    return this;
  }
  fromHermite(a, b, c, d, t) {
    const tt = t * t;
    const f1 = tt * (2 * t - 3) + 1;
    const f2 = tt * (t - 2) + t;
    const f3 = tt * (t - 1);
    const f4 = tt * (3 - 2 * t);
    this[0] = a[0] * f1 + b[0] * f2 + c[0] * f3 + d[0] * f4;
    this[1] = a[1] * f1 + b[1] * f2 + c[1] * f3 + d[1] * f4;
    this[2] = a[2] * f1 + b[2] * f2 + c[2] * f3 + d[2] * f4;
    return this;
  }
  fromBezier(a, b, c, d, t) {
    const it = 1 - t;
    const it2 = it * it;
    const tt = t * t;
    const f1 = it2 * it;
    const f2 = 3 * t * it2;
    const f3 = 3 * tt * it;
    const f4 = tt * t;
    this[0] = a[0] * f1 + b[0] * f2 + c[0] * f3 + d[0] * f4;
    this[1] = a[1] * f1 + b[1] * f2 + c[1] * f3 + d[1] * f4;
    this[2] = a[2] * f1 + b[2] * f2 + c[2] * f3 + d[2] * f4;
    return this;
  }
  fromCubic(a, b, c, d, t) {
    const t2 = t * t, t3 = t * t2, a0 = d[0] - c[0] - a[0] + b[0], a1 = d[1] - c[1] - a[1] + b[1], a2 = d[2] - c[2] - a[2] + b[2];
    this[0] = a0 * t3 + (a[0] - b[0] - a0) * t2 + (c[0] - a[0]) * t + b[0];
    this[1] = a1 * t3 + (a[1] - b[1] - a1) * t2 + (c[1] - a[1]) * t + b[1];
    this[2] = a2 * t3 + (a[2] - b[2] - a2) * t2 + (c[2] - a[2]) * t + b[2];
    return this;
  }
  add(a) {
    this[0] += a[0];
    this[1] += a[1];
    this[2] += a[2];
    return this;
  }
  sub(v) {
    this[0] -= v[0];
    this[1] -= v[1];
    this[2] -= v[2];
    return this;
  }
  mul(v) {
    this[0] *= v[0];
    this[1] *= v[1];
    this[2] *= v[2];
    return this;
  }
  scale(v) {
    this[0] *= v;
    this[1] *= v;
    this[2] *= v;
    return this;
  }
  divScale(v) {
    this[0] /= v;
    this[1] /= v;
    this[2] /= v;
    return this;
  }
  addScaled(a, s) {
    this[0] += a[0] * s;
    this[1] += a[1] * s;
    this[2] += a[2] * s;
    return this;
  }
  invert() {
    this[0] = 1 / this[0];
    this[1] = 1 / this[1];
    this[2] = 1 / this[2];
    return this;
  }
  norm() {
    let mag = Math.sqrt(this[0] ** 2 + this[1] ** 2 + this[2] ** 2);
    if (mag != 0) {
      mag = 1 / mag;
      this[0] *= mag;
      this[1] *= mag;
      this[2] *= mag;
    }
    return this;
  }
  cross(b) {
    const ax = this[0], ay = this[1], az = this[2], bx = b[0], by = b[1], bz = b[2];
    this[0] = ay * bz - az * by;
    this[1] = az * bx - ax * bz;
    this[2] = ax * by - ay * bx;
    return this;
  }
  abs() {
    this[0] = Math.abs(this[0]);
    this[1] = Math.abs(this[1]);
    this[2] = Math.abs(this[2]);
    return this;
  }
  floor() {
    this[0] = Math.floor(this[0]);
    this[1] = Math.floor(this[1]);
    this[2] = Math.floor(this[2]);
    return this;
  }
  ceil() {
    this[0] = Math.ceil(this[0]);
    this[1] = Math.ceil(this[1]);
    this[2] = Math.ceil(this[2]);
    return this;
  }
  min(a) {
    this[0] = Math.min(this[0], a[0]);
    this[1] = Math.min(this[1], a[1]);
    this[2] = Math.min(this[2], a[2]);
    return this;
  }
  max(a) {
    this[0] = Math.max(this[0], a[0]);
    this[1] = Math.max(this[1], a[1]);
    this[2] = Math.max(this[2], a[2]);
    return this;
  }
  nearZero() {
    if (Math.abs(this[0]) <= 1e-6)
      this[0] = 0;
    if (Math.abs(this[1]) <= 1e-6)
      this[1] = 0;
    if (Math.abs(this[2]) <= 1e-6)
      this[2] = 0;
    return this;
  }
  negate() {
    this[0] = -this[0];
    this[1] = -this[1];
    this[2] = -this[2];
    return this;
  }
  snap(v) {
    this[0] = v[0] != 0 ? Math.floor(this[0] / v[0]) * v[0] : 0;
    this[1] = v[1] != 0 ? Math.floor(this[1] / v[1]) * v[1] : 0;
    this[2] = v[2] != 0 ? Math.floor(this[2] / v[2]) * v[2] : 0;
    return this;
  }
  clamp(min, max) {
    this[0] = Math.min(Math.max(this[0], min[0]), max[0]);
    this[1] = Math.min(Math.max(this[1], min[1]), max[1]);
    this[2] = Math.min(Math.max(this[2], min[2]), max[2]);
    return this;
  }
  damp(v, lambda, dt) {
    const t = Math.exp(-lambda * dt);
    const ti = 1 - t;
    this[0] = this[0] * t + v[0] * ti;
    this[1] = this[1] * t + v[1] * ti;
    this[2] = this[2] * t + v[2] * ti;
    return this;
  }
  dot(b) {
    return this[0] * b[0] + this[1] * b[1] + this[2] * b[2];
  }
  axisAngle(axis, rad) {
    const cp = new Vec3().fromCross(axis, this), dot = Vec3.dot(axis, this), s = Math.sin(rad), c = Math.cos(rad), ci = 1 - c;
    this[0] = this[0] * c + cp[0] * s + axis[0] * dot * ci;
    this[1] = this[1] * c + cp[1] * s + axis[1] * dot * ci;
    this[2] = this[2] * c + cp[2] * s + axis[2] * dot * ci;
    return this;
  }
  rotate(rad, axis = "x") {
    const sin = Math.sin(rad), cos = Math.cos(rad), x = this[0], y = this[1], z = this[2];
    switch (axis) {
      case "y":
        this[0] = z * sin + x * cos;
        this[2] = z * cos - x * sin;
        break;
      case "x":
        this[1] = y * cos - z * sin;
        this[2] = y * sin + z * cos;
        break;
      case "z":
        this[0] = x * cos - y * sin;
        this[1] = x * sin + y * cos;
        break;
    }
    return this;
  }
  transformQuat(q) {
    const qx = q[0], qy = q[1], qz = q[2], qw = q[3], vx = this[0], vy = this[1], vz = this[2], x1 = qy * vz - qz * vy, y1 = qz * vx - qx * vz, z1 = qx * vy - qy * vx, x2 = qw * x1 + qy * z1 - qz * y1, y2 = qw * y1 + qz * x1 - qx * z1, z2 = qw * z1 + qx * y1 - qy * x1;
    this[0] = vx + 2 * x2;
    this[1] = vy + 2 * y2;
    this[2] = vz + 2 * z2;
    return this;
  }
  transformMat3(m) {
    const x = this[0], y = this[1], z = this[2];
    this[0] = x * m[0] + y * m[3] + z * m[6];
    this[1] = x * m[1] + y * m[4] + z * m[7];
    this[2] = x * m[2] + y * m[5] + z * m[8];
    return this;
  }
  transformMat4(m) {
    const x = this[0], y = this[1], z = this[2], w = m[3] * x + m[7] * y + m[11] * z + m[15] || 1;
    this[0] = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
    this[1] = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
    this[2] = (m[2] * x + m[6] * y + m[10] * z + m[14]) / w;
    return this;
  }
  static len(a) {
    return Math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2);
  }
  static lenSqr(a) {
    return a[0] ** 2 + a[1] ** 2 + a[2] ** 2;
  }
  static dist(a, b) {
    return Math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2);
  }
  static distSqr(a, b) {
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2;
  }
  static dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }
  static cross(a, b, out = new Vec3()) {
    const ax = a[0], ay = a[1], az = a[2], bx = b[0], by = b[1], bz = b[2];
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
    return out;
  }
  static norm(a) {
    let mag = Math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2);
    if (mag != 0) {
      mag = 1 / mag;
      a[0] = a[0] * mag;
      a[1] = a[1] * mag;
      a[2] = a[2] * mag;
    }
    return a;
  }
  static angle(a, b) {
    const d = this.dot(a, b), c = new Vec3().fromCross(a, b);
    return Math.atan2(Vec3.len(c), d);
  }
  static scaleThenAdd(scale, a, b, out = new Vec3()) {
    out[0] = a[0] * scale + b[0];
    out[1] = a[1] * scale + b[1];
    out[2] = a[2] * scale + b[2];
    return out;
  }
  static fromQuat(q, v = [0, 0, 1]) {
    return new Vec3(v).transformQuat(q);
  }
}class Quat extends Array {
  static LOOKXP = [0, -0.7071067811865475, 0, 0.7071067811865475];
  static LOOKXN = [0, 0.7071067811865475, 0, 0.7071067811865475];
  static LOOKYP = [0.7071067811865475, 0, 0, 0.7071067811865475];
  static LOOKYN = [-0.7071067811865475, 0, 0, 0.7071067811865475];
  static LOOKZP = [0, -1, 0, 0];
  static LOOKZN = [0, 0, 0, 1];
  constructor(v) {
    super(4);
    if (v instanceof Quat || v instanceof Float32Array || v instanceof Array && v.length == 4) {
      this[0] = v[0];
      this[1] = v[1];
      this[2] = v[2];
      this[3] = v[3];
    } else {
      this[0] = 0;
      this[1] = 0;
      this[2] = 0;
      this[3] = 1;
    }
  }
  get x() {
    return this[0];
  }
  set x(v) {
    this[0] = v;
  }
  get y() {
    return this[1];
  }
  set y(v) {
    this[1] = v;
  }
  get z() {
    return this[2];
  }
  set z(v) {
    this[2] = v;
  }
  get w() {
    return this[3];
  }
  set w(v) {
    this[3] = v;
  }
  xyzw(x, y, z, w) {
    this[0] = x;
    this[1] = y;
    this[2] = z;
    this[3] = w;
    return this;
  }
  identity() {
    this[0] = 0;
    this[1] = 0;
    this[2] = 0;
    this[3] = 1;
    return this;
  }
  copy(a) {
    this[0] = a[0];
    this[1] = a[1];
    this[2] = a[2];
    this[3] = a[3];
    return this;
  }
  copyTo(a) {
    a[0] = this[0];
    a[1] = this[1];
    a[2] = this[2];
    a[3] = this[3];
    return this;
  }
  clone() {
    return new Quat(this);
  }
  toString(rnd = 0) {
    if (rnd == 0)
      return "[" + this.join(",") + "]";
    else {
      let s = "[";
      for (let i = 0; i < 4; i++) {
        switch (this[i]) {
          case 0:
            s += "0,";
            break;
          case 1:
            s += "1,";
            break;
          default:
            s += this[i].toFixed(rnd) + ",";
            break;
        }
      }
      return s.slice(0, -1) + "]";
    }
  }
  isZero() {
    return this[0] == 0 && this[1] == 0 && this[2] == 0 && this[3] == 0;
  }
  lenSqr() {
    return this[0] ** 2 + this[1] ** 2 + this[2] ** 2 + this[3] ** 2;
  }
  len() {
    return Math.sqrt(this[0] ** 2 + this[1] ** 2 + this[2] ** 2 + this[3] ** 2);
  }
  getAxisAngle() {
    if (this[3] > 1)
      this.norm();
    const angle = 2 * Math.acos(this[3]), s = Math.sqrt(1 - this[3] * this[3]);
    if (s < 1e-3)
      return [1, 0, 0, 0];
    return [this[0] / s, this[1] / s, this[2] / s, angle];
  }
  getAngle() {
    if (this[3] > 1)
      this.norm();
    return 2 * Math.acos(this[3]);
  }
  getAxis(out) {
    if (this[3] > 1)
      this.norm();
    const s = Math.sqrt(1 - this[3] ** 2);
    out = out || [0, 0, 0];
    if (s < 1e-3) {
      out[0] = 1;
      out[1] = 0;
      out[2] = 0;
    } else {
      out[0] = this[0] / s;
      out[1] = this[1] / s;
      out[2] = this[2] / s;
    }
    return out;
  }
  fromPolar(lon, lat, up) {
    lat = Math.max(Math.min(lat, 89.999999), -89.999999);
    const phi = (90 - lat) * 0.01745329251, theta = lon * 0.01745329251, phi_s = Math.sin(phi), v = [
      -(phi_s * Math.sin(theta)),
      Math.cos(phi),
      phi_s * Math.cos(theta)
    ];
    this.fromLook(v, up || Vec3.UP);
    return this;
  }
  toPolar() {
    const fwd = new Vec3().fromQuat(this, Vec3.FORWARD);
    const flat = new Vec3(fwd[0], 0, fwd[2]).norm();
    let lon = Vec3.angle(Vec3.FORWARD, flat);
    let lat = Vec3.angle(flat, fwd);
    const d_side = Vec3.dot(fwd, Vec3.RIGHT);
    const d_up = Vec3.dot(fwd, Vec3.UP);
    if (d_side < 0)
      lon = -lon;
    if (d_up < 0)
      lat = -lat;
    if (d_up > 0.999 || d_up <= -0.999)
      lon = 0;
    const to_deg = 180 / Math.PI;
    return [lon * to_deg, lat * to_deg];
  }
  fromBuf(ary, idx) {
    this[0] = ary[idx];
    this[1] = ary[idx + 1];
    this[2] = ary[idx + 2];
    this[3] = ary[idx + 3];
    return this;
  }
  toBuf(ary, idx) {
    ary[idx] = this[0];
    ary[idx + 1] = this[1];
    ary[idx + 2] = this[2];
    ary[idx + 3] = this[3];
    return this;
  }
  pushTo(ary) {
    ary.push(this[0], this[1], this[2], this[3]);
    return this;
  }
  fromMul(a, b) {
    const ax = a[0], ay = a[1], az = a[2], aw = a[3], bx = b[0], by = b[1], bz = b[2], bw = b[3];
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  fromAxisAngle(axis, rad) {
    const half = rad * 0.5;
    const s = Math.sin(half);
    this[0] = axis[0] * s;
    this[1] = axis[1] * s;
    this[2] = axis[2] * s;
    this[3] = Math.cos(half);
    return this;
  }
  fromSwing(a, b) {
    const dot = Vec3.dot(a, b);
    if (dot < -0.999999) {
      const tmp = new Vec3().fromCross(Vec3.LEFT, a);
      if (tmp.len < 1e-6)
        tmp.fromCross(Vec3.UP, a);
      this.fromAxisAngle(tmp.norm(), Math.PI);
    } else if (dot > 0.999999) {
      this[0] = 0;
      this[1] = 0;
      this[2] = 0;
      this[3] = 1;
    } else {
      const v = Vec3.cross(a, b, [0, 0, 0]);
      this[0] = v[0];
      this[1] = v[1];
      this[2] = v[2];
      this[3] = 1 + dot;
      this.norm();
    }
    return this;
  }
  fromInvert(q) {
    const a0 = q[0], a1 = q[1], a2 = q[2], a3 = q[3], dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
    if (dot == 0) {
      this[0] = this[1] = this[2] = this[3] = 0;
      return this;
    }
    const invDot = 1 / dot;
    this[0] = -a0 * invDot;
    this[1] = -a1 * invDot;
    this[2] = -a2 * invDot;
    this[3] = a3 * invDot;
    return this;
  }
  fromNegate(q) {
    this[0] = -q[0];
    this[1] = -q[1];
    this[2] = -q[2];
    this[3] = -q[3];
    return this;
  }
  fromLook(dir, up = [0, 1, 0]) {
    const zAxis = new Vec3(dir).norm();
    const xAxis = new Vec3().fromCross(up, zAxis).norm();
    const yAxis = new Vec3().fromCross(zAxis, xAxis).norm();
    const m00 = xAxis[0], m01 = xAxis[1], m02 = xAxis[2], m10 = yAxis[0], m11 = yAxis[1], m12 = yAxis[2], m20 = zAxis[0], m21 = zAxis[1], m22 = zAxis[2], t = m00 + m11 + m22;
    let x, y, z, w, s;
    if (t > 0) {
      s = Math.sqrt(t + 1);
      w = s * 0.5;
      s = 0.5 / s;
      x = (m12 - m21) * s;
      y = (m20 - m02) * s;
      z = (m01 - m10) * s;
    } else if (m00 >= m11 && m00 >= m22) {
      s = Math.sqrt(1 + m00 - m11 - m22);
      x = 0.5 * s;
      s = 0.5 / s;
      y = (m01 + m10) * s;
      z = (m02 + m20) * s;
      w = (m12 - m21) * s;
    } else if (m11 > m22) {
      s = Math.sqrt(1 + m11 - m00 - m22);
      y = 0.5 * s;
      s = 0.5 / s;
      x = (m10 + m01) * s;
      z = (m21 + m12) * s;
      w = (m20 - m02) * s;
    } else {
      s = Math.sqrt(1 + m22 - m00 - m11);
      z = 0.5 * s;
      s = 0.5 / s;
      x = (m20 + m02) * s;
      y = (m21 + m12) * s;
      w = (m01 - m10) * s;
    }
    this[0] = x;
    this[1] = y;
    this[2] = z;
    this[3] = w;
    return this;
  }
  fromNBlend(a, b, t) {
    const a_x = a[0];
    const a_y = a[1];
    const a_z = a[2];
    const a_w = a[3];
    const b_x = b[0];
    const b_y = b[1];
    const b_z = b[2];
    const b_w = b[3];
    const dot = a_x * b_x + a_y * b_y + a_z * b_z + a_w * b_w;
    const ti = 1 - t;
    const s = dot < 0 ? -1 : 1;
    this[0] = ti * a_x + t * b_x * s;
    this[1] = ti * a_y + t * b_y * s;
    this[2] = ti * a_z + t * b_z * s;
    this[3] = ti * a_w + t * b_w * s;
    return this.norm();
  }
  fromLerp(a, b, t) {
    const ti = 1 - t;
    this[0] = a[0] * ti + b[0] * t;
    this[1] = a[1] * ti + b[1] * t;
    this[2] = a[2] * ti + b[2] * t;
    this[3] = a[3] * ti + b[3] * t;
    return this;
  }
  fromNLerp(a, b, t) {
    const ti = 1 - t;
    this[0] = a[0] * ti + b[0] * t;
    this[1] = a[1] * ti + b[1] * t;
    this[2] = a[2] * ti + b[2] * t;
    this[3] = a[3] * ti + b[3] * t;
    return this.norm();
  }
  fromSlerp(a, b, t) {
    const ax = a[0], ay = a[1], az = a[2], aw = a[3];
    let bx = b[0], by = b[1], bz = b[2], bw = b[3];
    let omega, cosom, sinom, scale0, scale1;
    cosom = ax * bx + ay * by + az * bz + aw * bw;
    if (cosom < 0) {
      cosom = -cosom;
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
    }
    if (1 - cosom > 1e-6) {
      omega = Math.acos(cosom);
      sinom = Math.sin(omega);
      scale0 = Math.sin((1 - t) * omega) / sinom;
      scale1 = Math.sin(t * omega) / sinom;
    } else {
      scale0 = 1 - t;
      scale1 = t;
    }
    this[0] = scale0 * ax + scale1 * bx;
    this[1] = scale0 * ay + scale1 * by;
    this[2] = scale0 * az + scale1 * bz;
    this[3] = scale0 * aw + scale1 * bw;
    return this;
  }
  fromAxes(xAxis, yAxis, zAxis) {
    const m00 = xAxis[0], m01 = xAxis[1], m02 = xAxis[2], m10 = yAxis[0], m11 = yAxis[1], m12 = yAxis[2], m20 = zAxis[0], m21 = zAxis[1], m22 = zAxis[2], t = m00 + m11 + m22;
    let x, y, z, w, s;
    if (t > 0) {
      s = Math.sqrt(t + 1);
      w = s * 0.5;
      s = 0.5 / s;
      x = (m12 - m21) * s;
      y = (m20 - m02) * s;
      z = (m01 - m10) * s;
    } else if (m00 >= m11 && m00 >= m22) {
      s = Math.sqrt(1 + m00 - m11 - m22);
      x = 0.5 * s;
      s = 0.5 / s;
      y = (m01 + m10) * s;
      z = (m02 + m20) * s;
      w = (m12 - m21) * s;
    } else if (m11 > m22) {
      s = Math.sqrt(1 + m11 - m00 - m22);
      y = 0.5 * s;
      s = 0.5 / s;
      x = (m10 + m01) * s;
      z = (m21 + m12) * s;
      w = (m20 - m02) * s;
    } else {
      s = Math.sqrt(1 + m22 - m00 - m11);
      z = 0.5 * s;
      s = 0.5 / s;
      x = (m20 + m02) * s;
      y = (m21 + m12) * s;
      w = (m01 - m10) * s;
    }
    this[0] = x;
    this[1] = y;
    this[2] = z;
    this[3] = w;
    return this;
  }
  fromMat3(m) {
    let fRoot;
    const fTrace = m[0] + m[4] + m[8];
    if (fTrace > 0) {
      fRoot = Math.sqrt(fTrace + 1);
      this[3] = 0.5 * fRoot;
      fRoot = 0.5 / fRoot;
      this[0] = (m[5] - m[7]) * fRoot;
      this[1] = (m[6] - m[2]) * fRoot;
      this[2] = (m[1] - m[3]) * fRoot;
    } else {
      let i = 0;
      if (m[4] > m[0])
        i = 1;
      if (m[8] > m[i * 3 + i])
        i = 2;
      const j = (i + 1) % 3;
      const k = (i + 2) % 3;
      fRoot = Math.sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1);
      this[i] = 0.5 * fRoot;
      fRoot = 0.5 / fRoot;
      this[3] = (m[j * 3 + k] - m[k * 3 + j]) * fRoot;
      this[j] = (m[j * 3 + i] + m[i * 3 + j]) * fRoot;
      this[k] = (m[k * 3 + i] + m[i * 3 + k]) * fRoot;
    }
    return this;
  }
  fromMat4(mat) {
    const trace = mat[0] + mat[5] + mat[10];
    let S = 0;
    if (trace > 0) {
      S = Math.sqrt(trace + 1) * 2;
      this[3] = 0.25 * S;
      this[0] = (mat[6] - mat[9]) / S;
      this[1] = (mat[8] - mat[2]) / S;
      this[2] = (mat[1] - mat[4]) / S;
    } else if (mat[0] > mat[5] && mat[0] > mat[10]) {
      S = Math.sqrt(1 + mat[0] - mat[5] - mat[10]) * 2;
      this[3] = (mat[6] - mat[9]) / S;
      this[0] = 0.25 * S;
      this[1] = (mat[1] + mat[4]) / S;
      this[2] = (mat[8] + mat[2]) / S;
    } else if (mat[5] > mat[10]) {
      S = Math.sqrt(1 + mat[5] - mat[0] - mat[10]) * 2;
      this[3] = (mat[8] - mat[2]) / S;
      this[0] = (mat[1] + mat[4]) / S;
      this[1] = 0.25 * S;
      this[2] = (mat[6] + mat[9]) / S;
    } else {
      S = Math.sqrt(1 + mat[10] - mat[0] - mat[5]) * 2;
      this[3] = (mat[1] - mat[4]) / S;
      this[0] = (mat[8] + mat[2]) / S;
      this[1] = (mat[6] + mat[9]) / S;
      this[2] = 0.25 * S;
    }
    return this;
  }
  fromAngularVec(v) {
    let len = Vec3.len(v);
    if (len < 1e-6) {
      this.identity();
      return this;
    }
    const h = 0.5 * len;
    const s = Math.sin(h);
    const c = Math.cos(h);
    len = 1 / len;
    this[0] = s * (v[0] * len);
    this[1] = s * (v[1] * len);
    this[2] = s * (v[2] * len);
    this[3] = c;
    return this;
  }
  toAngularVec(out) {
    const v = this.getAxisAngle();
    out = out || new Vec3();
    return out.fromScale(v, v[3]);
  }
  fromEuler(x, y, z) {
    let xx = 0, yy = 0, zz = 0;
    if (x instanceof Vec3 || x instanceof Float32Array || x instanceof Array && x.length == 3) {
      xx = x[0] * 0.01745329251 * 0.5;
      yy = x[1] * 0.01745329251 * 0.5;
      zz = x[2] * 0.01745329251 * 0.5;
    } else if (typeof x === "number" && typeof y === "number" && typeof z === "number") {
      xx = x * 0.01745329251 * 0.5;
      yy = y * 0.01745329251 * 0.5;
      zz = z * 0.01745329251 * 0.5;
    }
    const c1 = Math.cos(xx), c2 = Math.cos(yy), c3 = Math.cos(zz), s1 = Math.sin(xx), s2 = Math.sin(yy), s3 = Math.sin(zz);
    this[0] = s1 * c2 * c3 + c1 * s2 * s3;
    this[1] = c1 * s2 * c3 - s1 * c2 * s3;
    this[2] = c1 * c2 * s3 - s1 * s2 * c3;
    this[3] = c1 * c2 * c3 + s1 * s2 * s3;
    return this.norm();
  }
  fromEulerXY(x, y) {
    const xx = x * 0.01745329251 * 0.5, yy = y * 0.01745329251 * 0.5, c1 = Math.cos(xx), c2 = Math.cos(yy), s1 = Math.sin(xx), s2 = Math.sin(yy);
    this[0] = s1 * c2;
    this[1] = c1 * s2;
    this[2] = -s1 * s2;
    this[3] = c1 * c2;
    return this.norm();
  }
  fromEulerOrder(x, y, z, order = "YXZ") {
    const c1 = Math.cos(x * 0.5), c2 = Math.cos(y * 0.5), c3 = Math.cos(z * 0.5), s1 = Math.sin(x * 0.5), s2 = Math.sin(y * 0.5), s3 = Math.sin(z * 0.5);
    switch (order) {
      case "XYZ":
        this[0] = s1 * c2 * c3 + c1 * s2 * s3;
        this[1] = c1 * s2 * c3 - s1 * c2 * s3;
        this[2] = c1 * c2 * s3 + s1 * s2 * c3;
        this[3] = c1 * c2 * c3 - s1 * s2 * s3;
        break;
      case "YXZ":
        this[0] = s1 * c2 * c3 + c1 * s2 * s3;
        this[1] = c1 * s2 * c3 - s1 * c2 * s3;
        this[2] = c1 * c2 * s3 - s1 * s2 * c3;
        this[3] = c1 * c2 * c3 + s1 * s2 * s3;
        break;
      case "ZXY":
        this[0] = s1 * c2 * c3 - c1 * s2 * s3;
        this[1] = c1 * s2 * c3 + s1 * c2 * s3;
        this[2] = c1 * c2 * s3 + s1 * s2 * c3;
        this[3] = c1 * c2 * c3 - s1 * s2 * s3;
        break;
      case "ZYX":
        this[0] = s1 * c2 * c3 - c1 * s2 * s3;
        this[1] = c1 * s2 * c3 + s1 * c2 * s3;
        this[2] = c1 * c2 * s3 - s1 * s2 * c3;
        this[3] = c1 * c2 * c3 + s1 * s2 * s3;
        break;
      case "YZX":
        this[0] = s1 * c2 * c3 + c1 * s2 * s3;
        this[1] = c1 * s2 * c3 + s1 * c2 * s3;
        this[2] = c1 * c2 * s3 - s1 * s2 * c3;
        this[3] = c1 * c2 * c3 - s1 * s2 * s3;
        break;
      case "XZY":
        this[0] = s1 * c2 * c3 - c1 * s2 * s3;
        this[1] = c1 * s2 * c3 - s1 * c2 * s3;
        this[2] = c1 * c2 * s3 + s1 * s2 * c3;
        this[3] = c1 * c2 * c3 + s1 * s2 * s3;
        break;
    }
    return this.norm();
  }
  mul(q) {
    const ax = this[0], ay = this[1], az = this[2], aw = this[3], bx = q[0], by = q[1], bz = q[2], bw = q[3];
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  pmul(q) {
    const ax = q[0], ay = q[1], az = q[2], aw = q[3], bx = this[0], by = this[1], bz = this[2], bw = this[3];
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  norm() {
    let len = this[0] ** 2 + this[1] ** 2 + this[2] ** 2 + this[3] ** 2;
    if (len > 0) {
      len = 1 / Math.sqrt(len);
      this[0] *= len;
      this[1] *= len;
      this[2] *= len;
      this[3] *= len;
    }
    return this;
  }
  invert() {
    const a0 = this[0], a1 = this[1], a2 = this[2], a3 = this[3], dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
    if (dot == 0) {
      this[0] = this[1] = this[2] = this[3] = 0;
      return this;
    }
    const invDot = 1 / dot;
    this[0] = -a0 * invDot;
    this[1] = -a1 * invDot;
    this[2] = -a2 * invDot;
    this[3] = a3 * invDot;
    return this;
  }
  negate() {
    this[0] = -this[0];
    this[1] = -this[1];
    this[2] = -this[2];
    this[3] = -this[3];
    return this;
  }
  conjugate() {
    this[0] = -this[0];
    this[1] = -this[1];
    this[2] = -this[2];
    return this;
  }
  scaleAngle(scl) {
    if (this[3] > 1)
      this.norm();
    const angle = 2 * Math.acos(this[3]), len = 1 / Math.sqrt(this[0] ** 2 + this[1] ** 2 + this[2] ** 2), half = angle * scl * 0.5, s = Math.sin(half);
    this[0] = this[0] * len * s;
    this[1] = this[1] * len * s;
    this[2] = this[2] * len * s;
    this[3] = Math.cos(half);
    return this;
  }
  transformVec3(v) {
    const qx = this[0], qy = this[1], qz = this[2], qw = this[3], vx = v[0], vy = v[1], vz = v[2], x1 = qy * vz - qz * vy, y1 = qz * vx - qx * vz, z1 = qx * vy - qy * vx, x2 = qw * x1 + qy * z1 - qz * y1, y2 = qw * y1 + qz * x1 - qx * z1, z2 = qw * z1 + qx * y1 - qy * x1;
    v[0] = vx + 2 * x2;
    v[1] = vy + 2 * y2;
    v[2] = vz + 2 * z2;
    return this;
  }
  rotX(rad) {
    rad *= 0.5;
    const ax = this[0], ay = this[1], az = this[2], aw = this[3], bx = Math.sin(rad), bw = Math.cos(rad);
    this[0] = ax * bw + aw * bx;
    this[1] = ay * bw + az * bx;
    this[2] = az * bw - ay * bx;
    this[3] = aw * bw - ax * bx;
    return this;
  }
  rotY(rad) {
    rad *= 0.5;
    const ax = this[0], ay = this[1], az = this[2], aw = this[3], by = Math.sin(rad), bw = Math.cos(rad);
    this[0] = ax * bw - az * by;
    this[1] = ay * bw + aw * by;
    this[2] = az * bw + ax * by;
    this[3] = aw * bw - ay * by;
    return this;
  }
  rotZ(rad) {
    rad *= 0.5;
    const ax = this[0], ay = this[1], az = this[2], aw = this[3], bz = Math.sin(rad), bw = Math.cos(rad);
    this[0] = ax * bw + ay * bz;
    this[1] = ay * bw - ax * bz;
    this[2] = az * bw + aw * bz;
    this[3] = aw * bw - az * bz;
    return this;
  }
  rotDeg(deg, axis = 0) {
    const rad = deg * Math.PI / 180;
    switch (axis) {
      case 0:
        this.rotX(rad);
        break;
      case 1:
        this.rotY(rad);
        break;
      case 2:
        this.rotZ(rad);
        break;
    }
    return this;
  }
  pmulInvert(q) {
    let ax = q[0], ay = q[1], az = q[2], aw = q[3];
    const dot = ax * ax + ay * ay + az * az + aw * aw;
    if (dot === 0) {
      ax = ay = az = aw = 0;
    } else {
      const dot_inv = 1 / dot;
      ax = -ax * dot_inv;
      ay = -ay * dot_inv;
      az = -az * dot_inv;
      aw = aw * dot_inv;
    }
    const bx = this[0], by = this[1], bz = this[2], bw = this[3];
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  pmulAxisAngle(axis, rad) {
    const half = rad * 0.5;
    const s = Math.sin(half);
    const ax = axis[0] * s;
    const ay = axis[1] * s;
    const az = axis[2] * s;
    const aw = Math.cos(half);
    const bx = this[0], by = this[1], bz = this[2], bw = this[3];
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  mulAxisAngle(axis, angle) {
    const half = angle * 0.5, s = Math.sin(half), bx = axis[0] * s, by = axis[1] * s, bz = axis[2] * s, bw = Math.cos(half), ax = this[0], ay = this[1], az = this[2], aw = this[3];
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  dotNegate(q, chk) {
    if (Quat.dot(q, chk) < 0)
      this.fromNegate(q);
    else
      this.copy(q);
    return this;
  }
  mirrorX() {
    this[1] = -this[1];
    this[2] = -this[2];
    return this;
  }
  random() {
    const u1 = Math.random(), u2 = Math.random(), u3 = Math.random(), r1 = Math.sqrt(1 - u1), r2 = Math.sqrt(u1);
    this[0] = r1 * Math.sin(6.283185307179586 * u2);
    this[1] = r1 * Math.cos(6.283185307179586 * u2);
    this[2] = r2 * Math.sin(6.283185307179586 * u3);
    this[3] = r2 * Math.cos(6.283185307179586 * u3);
    return this;
  }
  mulUnitVecs(a, b) {
    const dot = Vec3.dot(a, b);
    const ax = this[0], ay = this[1], az = this[2], aw = this[3];
    let bx, by, bz, bw;
    if (dot < -0.999999) {
      const axis = Vec3.cross(Vec3.LEFT, a);
      if (Vec3.len(axis) < 1e-6)
        Vec3.cross(Vec3.UP, a, axis);
      Vec3.norm(axis);
      const half = Math.PI * 0.5, s = Math.sin(half);
      bx = axis[0] * s;
      by = axis[1] * s;
      bz = axis[2] * s;
      bw = Math.cos(half);
    } else if (dot > 0.999999) {
      bx = 0;
      by = 0;
      bz = 0;
      bw = 1;
    } else {
      const v = Vec3.cross(a, b);
      bx = v[0];
      by = v[1];
      bz = v[2];
      bw = 1 + dot;
      let len = bx ** 2 + by ** 2 + bz ** 2 + bw ** 2;
      if (len > 0) {
        len = 1 / Math.sqrt(len);
        bx *= len;
        by *= len;
        bz *= len;
        bw *= len;
      }
    }
    this[0] = ax * bw + aw * bx + ay * bz - az * by;
    this[1] = ay * bw + aw * by + az * bx - ax * bz;
    this[2] = az * bw + aw * bz + ax * by - ay * bx;
    this[3] = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }
  static dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  }
  static lenSqr(a, b) {
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2;
  }
  static nblend(a, b, t, out) {
    const a_x = a[0];
    const a_y = a[1];
    const a_z = a[2];
    const a_w = a[3];
    const b_x = b[0];
    const b_y = b[1];
    const b_z = b[2];
    const b_w = b[3];
    const dot = a_x * b_x + a_y * b_y + a_z * b_z + a_w * b_w;
    const ti = 1 - t;
    const s = dot < 0 ? -1 : 1;
    out[0] = ti * a_x + t * b_x * s;
    out[1] = ti * a_y + t * b_y * s;
    out[2] = ti * a_z + t * b_z * s;
    out[3] = ti * a_w + t * b_w * s;
    return out.norm();
  }
  static slerp(a, b, t, out) {
    const ax = a[0], ay = a[1], az = a[2], aw = a[3];
    let bx = b[0], by = b[1], bz = b[2], bw = b[3];
    let omega, cosom, sinom, scale0, scale1;
    cosom = ax * bx + ay * by + az * bz + aw * bw;
    if (cosom < 0) {
      cosom = -cosom;
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
    }
    if (1 - cosom > 1e-6) {
      omega = Math.acos(cosom);
      sinom = Math.sin(omega);
      scale0 = Math.sin((1 - t) * omega) / sinom;
      scale1 = Math.sin(t * omega) / sinom;
    } else {
      scale0 = 1 - t;
      scale1 = t;
    }
    out[0] = scale0 * ax + scale1 * bx;
    out[1] = scale0 * ay + scale1 * by;
    out[2] = scale0 * az + scale1 * bz;
    out[3] = scale0 * aw + scale1 * bw;
    return out;
  }
  static shortest(from, to, out) {
    const ax = from[0], ay = from[1], az = from[2], aw = from[3];
    let bx = to[0], by = to[1], bz = to[2], bw = to[3];
    const dot = ax * bx + ay * by + az * bz + aw * bw;
    if (dot < 0) {
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
    }
    const d = bx * bx + by * by + bz * bz + bw * bw;
    if (d === 0) {
      bx = 0;
      by = 0;
      bz = 0;
      bw = 0;
    } else {
      const di = 1 / d;
      bx = -bx * di;
      by = -by * di;
      bz = -bz * di;
      bw = bw * di;
    }
    out[0] = ax * bw + aw * bx + ay * bz - az * by;
    out[1] = ay * bw + aw * by + az * bx - ax * bz;
    out[2] = az * bw + aw * bz + ax * by - ay * bx;
    out[3] = aw * bw - ax * bx - ay * by - az * bz;
    return out;
  }
  static swing(a, b) {
    return new Quat().fromSwing(a, b);
  }
  static axisAngle(axis, rad) {
    return new Quat().fromAxisAngle(axis, rad);
  }
}class Vec2 extends Array {
  constructor(v, y) {
    super(2);
    if (v instanceof Vec2 || v instanceof Float32Array || v instanceof Array && v.length == 2) {
      this[0] = v[0];
      this[1] = v[1];
    } else if (typeof v === "number" && typeof y === "number") {
      this[0] = v;
      this[1] = y;
    } else if (typeof v === "number") {
      this[0] = v;
      this[1] = v;
    } else {
      this[0] = 0;
      this[1] = 0;
    }
  }
  xy(x, y) {
    if (y != void 0) {
      this[0] = x;
      this[1] = y;
    } else
      this[0] = this[1] = x;
    return this;
  }
  get x() {
    return this[0];
  }
  set x(v) {
    this[0] = v;
  }
  get y() {
    return this[1];
  }
  set y(v) {
    this[1] = v;
  }
  clone() {
    return new Vec2(this);
  }
  copy(v) {
    this[0] = v[0];
    this[1] = v[1];
    return this;
  }
  reset() {
    this[0] = 0;
    this[1] = 0;
    return this;
  }
  toString(rnd = 0) {
    if (rnd == 0)
      return "[" + this.join(",") + "]";
    else {
      let s = "[";
      for (let i = 0; i < 2; i++) {
        switch (this[i]) {
          case 0:
            s += "0,";
            break;
          case 1:
            s += "1,";
            break;
          default:
            s += this[i].toFixed(rnd) + ",";
            break;
        }
      }
      return s.slice(0, -1) + "]";
    }
  }
  isZero() {
    return this[0] == 0 && this[1] == 0;
  }
  nearZero(x = 1e-6, y = 1e-6) {
    if (Math.abs(this[0]) <= x)
      this[0] = 0;
    if (Math.abs(this[1]) <= y)
      this[1] = 0;
    return this;
  }
  rnd(x0 = 0, x1 = 1, y0 = 0, y1 = 1) {
    let t;
    t = Math.random();
    this[0] = x0 * (1 - t) + x1 * t;
    t = Math.random();
    this[1] = y0 * (1 - t) + y1 * t;
    return this;
  }
  angle(v) {
    return v ? Math.atan2(v[1] * this[0] - v[0] * this[1], v[0] * this[0] + v[1] * this[1]) : Math.atan2(this[1], this[0]);
  }
  setLen(len) {
    return this.norm().scale(len);
  }
  len() {
    return Math.sqrt(this[0] * this[0] + this[1] * this[1]);
  }
  lenSqr() {
    return this[0] * this[0] + this[1] * this[1];
  }
  toVec3(isYUp = true, n = 0) {
    const v = [this[0], 0, 0];
    if (isYUp) {
      v[1] = n;
      v[2] = this[1];
    } else {
      v[1] = this[1];
      v[2] = n;
    }
    return v;
  }
  fromAngleLen(ang, len) {
    this[0] = len * Math.cos(ang);
    this[1] = len * Math.sin(ang);
    return this;
  }
  fromAdd(a, b) {
    this[0] = a[0] + b[0];
    this[1] = a[1] + b[1];
    return this;
  }
  fromSub(a, b) {
    this[0] = a[0] - b[0];
    this[1] = a[1] - b[1];
    return this;
  }
  fromMul(a, b) {
    this[0] = a[0] * b[0];
    this[1] = a[1] * b[1];
    return this;
  }
  fromScale(a, s) {
    this[0] = a[0] * s;
    this[1] = a[1] * s;
    return this;
  }
  fromLerp(a, b, t = 0.5) {
    const tt = 1 - t;
    this[0] = a[0] * tt + b[0] * t;
    this[1] = a[1] * tt + b[1] * t;
    return this;
  }
  fromMax(a, b) {
    this[0] = Math.max(a[0], b[0]);
    this[1] = Math.max(a[1], b[1]);
    return this;
  }
  fromMin(a, b) {
    this[0] = Math.min(a[0], b[0]);
    this[1] = Math.min(a[1], b[1]);
    return this;
  }
  fromFloor(v) {
    this[0] = Math.floor(v[0]);
    this[1] = Math.floor(v[1]);
    return this;
  }
  fromFract(v) {
    this[0] = v[0] - Math.floor(v[0]);
    this[1] = v[1] - Math.floor(v[1]);
    return this;
  }
  fromNegate(a) {
    this[0] = -a[0];
    this[1] = -a[1];
    return this;
  }
  fromInvert(a) {
    this[0] = a[0] != 0 ? 1 / a[0] : 0;
    this[1] = a[1] != 0 ? 1 / a[1] : 0;
    return this;
  }
  fromLineProjecton(from, to) {
    const denom = Vec2.dot(to, to);
    if (denom < 1e-6)
      return this;
    const scl = Vec2.dot(from, to) / denom;
    this.fromScale(to, scl);
    return this;
  }
  fromBuf(ary, idx) {
    this[0] = ary[idx];
    this[1] = ary[idx + 1];
    return this;
  }
  toBuf(ary, idx) {
    ary[idx] = this[0];
    ary[idx + 1] = this[1];
    return this;
  }
  pushTo(ary) {
    ary.push(this[0], this[1]);
    return this;
  }
  add(v) {
    this[0] += v[0];
    this[1] += v[1];
    return this;
  }
  sub(v) {
    this[0] -= v[0];
    this[1] -= v[1];
    return this;
  }
  mul(v) {
    this[0] *= v[0];
    this[1] *= v[1];
    return this;
  }
  div(v) {
    this[0] = v[0] != 0 ? this[0] / v[0] : 0;
    this[1] = v[1] != 0 ? this[1] / v[1] : 0;
    return this;
  }
  scale(v) {
    this[0] *= v;
    this[1] *= v;
    return this;
  }
  divScale(v) {
    this[0] /= v;
    this[1] /= v;
    return this;
  }
  scaleThenAdd(scale, a, b) {
    this[0] = a[0] * scale + b[0];
    this[1] = a[1] * scale + b[1];
    return this;
  }
  floor() {
    this[0] = Math.floor(this[0]);
    this[1] = Math.floor(this[1]);
    return this;
  }
  negate() {
    this[0] = -this[0];
    this[1] = -this[1];
    return this;
  }
  min(a) {
    this[0] = Math.min(this[0], a[0]);
    this[1] = Math.min(this[1], a[1]);
    return this;
  }
  max(a) {
    this[0] = Math.max(this[0], a[0]);
    this[1] = Math.max(this[1], a[1]);
    return this;
  }
  norm() {
    const mag = Math.sqrt(this[0] ** 2 + this[1] ** 2);
    if (mag == 0)
      return this;
    this[0] = this[0] / mag;
    this[1] = this[1] / mag;
    return this;
  }
  lerp(v, t) {
    const ti = 1 - t;
    this[0] = this[0] * ti + v[0] * t;
    this[1] = this[1] * ti + v[1] * t;
    return this;
  }
  rotate(rad) {
    const cos = Math.cos(rad), sin = Math.sin(rad), x = this[0], y = this[1];
    this[0] = x * cos - y * sin;
    this[1] = x * sin + y * cos;
    return this;
  }
  rotateDeg(deg) {
    const rad = deg * Math.PI / 180, cos = Math.cos(rad), sin = Math.sin(rad), x = this[0], y = this[1];
    this[0] = x * cos - y * sin;
    this[1] = x * sin + y * cos;
    return this;
  }
  invert() {
    this[0] = 1 / this[0];
    this[1] = 1 / this[1];
    return this;
  }
  rotP90() {
    const x = this[0];
    this[0] = this[1];
    this[1] = -x;
    return this;
  }
  rotN90() {
    const x = this[0];
    this[0] = -this[1];
    this[1] = x;
    return this;
  }
  static add(a, b) {
    return new Vec2().fromAdd(a, b);
  }
  static sub(a, b) {
    return new Vec2().fromSub(a, b);
  }
  static scale(v, s) {
    return new Vec2().fromScale(v, s);
  }
  static floor(v) {
    return new Vec2().fromFloor(v);
  }
  static fract(v) {
    return new Vec2().fromFract(v);
  }
  static lerp(v0, v1, t) {
    return new Vec2().fromLerp(v0, v1, t);
  }
  static len(v0) {
    return Math.sqrt(v0[0] ** 2 + v0[1] ** 2);
  }
  static lenSqr(v0) {
    return v0[0] ** 2 + v0[1] ** 2;
  }
  static dist(a, b) {
    return Math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2);
  }
  static distSqr(a, b) {
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2;
  }
  static dot(a, b) {
    return a[0] * b[0] + a[1] * b[1];
  }
  static det(a, b) {
    return a[0] * b[1] - a[1] * b[0];
  }
  static project(from, to) {
    const out = new Vec2();
    const denom = Vec2.dot(to, to);
    if (denom < 1e-6)
      return out;
    const scl = Vec2.dot(from, to) / denom;
    return out.fromScale(to, scl);
  }
  static scaleThenAdd(scale, a, b) {
    return new Vec2().scaleThenAdd(scale, a, b);
  }
  static projectPlane(from, to, planeNorm) {
    const out = new Vec2();
    const denom = Vec2.dot(to, planeNorm);
    if (denom < 1e-6 && denom > -1e-6)
      return out;
    const t = Vec2.dot(from, planeNorm) / denom;
    return out.fromScale(to, t);
  }
  static rotateDeg(v, deg) {
    const out = new Vec2();
    const rad = deg * Math.PI / 180, cos = Math.cos(rad), sin = Math.sin(rad), x = v[0], y = v[1];
    out[0] = x * cos - y * sin;
    out[1] = x * sin + y * cos;
    return out;
  }
  static rotP90(v) {
    const out = new Vec2();
    out[0] = v[1];
    out[1] = -v[0];
    return out;
  }
  static rotN90(v) {
    const out = new Vec2();
    out[0] = -v[1];
    out[1] = v[0];
    return out;
  }
  static angleTo(a, b) {
    return Math.atan2(b[1] * a[0] - b[0] * a[1], b[0] * a[0] + b[1] * a[1]);
  }
  static bufIter(buf) {
    let i = 0;
    const result = { value: new Vec2(), done: false }, len = buf.length, next = () => {
      if (i >= len)
        result.done = true;
      else {
        result.value.fromBuf(buf, i);
        i += 2;
      }
      return result;
    };
    return { [Symbol.iterator]() {
      return { next };
    } };
  }
}class Vec3Wizzy {
  static xp(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = x;
    o[1] = -z;
    o[2] = y;
    return o;
  }
  static xn(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = x;
    o[1] = z;
    o[2] = -y;
    return o;
  }
  static yp(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = -z;
    o[1] = y;
    o[2] = x;
    return o;
  }
  static yn(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = z;
    o[1] = y;
    o[2] = -x;
    return o;
  }
  static zp(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = y;
    o[1] = -x;
    o[2] = z;
    return o;
  }
  static zn(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = -y;
    o[1] = x;
    o[2] = z;
    return o;
  }
  static xp_yn(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = -y;
    o[1] = -z;
    o[2] = x;
    return o;
  }
  static xp_yp(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = y;
    o[1] = -z;
    o[2] = -x;
    return o;
  }
  static xp_yp_yp(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = -x;
    o[1] = -z;
    o[2] = -y;
    return o;
  }
  static xp_xp(v, o) {
    const x = v[0], y = v[1], z = v[2];
    o[0] = x;
    o[1] = -y;
    o[2] = -z;
    return o;
  }
}export{Quat,Vec2,Vec3,Vec3Wizzy};