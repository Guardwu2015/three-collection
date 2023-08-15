import {Vec2}from'./oop.js';class Line2D {
  static intersectingSegments(a0, a1, b0, b1, out = [0, 0]) {
    const denom = (b1[1] - b0[1]) * (a1[0] - a0[0]) - (b1[0] - b0[0]) * (a1[1] - a0[1]);
    if (denom === 0)
      return null;
    const ua = ((b1[0] - b0[0]) * (a0[1] - b0[1]) - (b1[1] - b0[1]) * (a0[0] - b0[0])) / denom;
    const ub = ((a1[0] - a0[0]) * (a0[1] - b0[1]) - (a1[1] - a0[1]) * (a0[0] - b0[0])) / denom;
    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
      out[0] = a0[0] + ua * (a1[0] - a0[0]);
      out[1] = a0[1] + ua * (a1[1] - a0[1]);
      return out;
    }
    return null;
  }
  static intersectingRays(ap, ad, bp, bd, out = [0, 0]) {
    const dx = bp[0] - ap[0];
    const dy = bp[1] - ap[1];
    const det = bd[0] * ad[1] - bd[1] * ad[0];
    if (det !== 0) {
      const u = (dy * bd[0] - dx * bd[1]) / det;
      const v = (dy * ad[0] - dx * ad[1]) / det;
      if (u >= 0 && v >= 0) {
        out[0] = ap[0] + ad[0] * u;
        out[1] = ap[1] + ad[1] * u;
        return out;
      }
    }
    return null;
  }
  static intersectingRaySegment(ro, rd, s0, s1, out = [0, 0]) {
    const denom = rd[0] * (s1[1] - s0[1]) - rd[1] * (s1[0] - s0[0]);
    if (Math.abs(denom) < 1e-4)
      return null;
    const t1 = ((s0[0] - ro[0]) * (s1[1] - s0[1]) - (s0[1] - ro[1]) * (s1[0] - s0[0])) / denom;
    const t2 = ((s0[0] - ro[0]) * rd[1] - (s0[1] - ro[1]) * rd[0]) / denom;
    if (t1 >= 0 && t2 >= 0 && t2 <= 1) {
      out[0] = ro[0] + rd[0] * t1;
      out[1] = ro[1] + rd[1] * t1;
      return out;
    }
    return null;
  }
  static isLeft(a, b, p) {
    return (p[0] - a[0]) * (b[1] - a[1]) - (p[1] - a[1]) * (b[0] - a[0]) >= 0;
  }
}class Polygon2D {
  points = [];
  constructor(pnts) {
    if (pnts)
      this.points = pnts;
  }
  addPoint(p) {
    this.points.push(new Vec2(p));
    return this;
  }
  addArray(ary) {
    for (const i of ary)
      this.points.push(new Vec2(i));
    return this;
  }
  get pointCount() {
    return this.points.length;
  }
  getEdge(i) {
    const cnt = this.points.length;
    const j = (cnt + i) % cnt;
    const k = (cnt + i + 1) % cnt;
    return [this.points[j], this.points[k]];
  }
  getLongestEdge() {
    const pnts = this.points;
    const cnt = pnts.length;
    let max = -Infinity;
    let ai = -1;
    let bi = -1;
    let d = 0;
    let ii;
    for (let i = 0; i < cnt; i++) {
      ii = (i + 1) % cnt;
      d = Vec2.distSqr(pnts[i], pnts[ii]);
      if (d > max) {
        ai = i;
        bi = ii;
        max = d;
      }
    }
    return [pnts[ai], pnts[bi], ai, bi];
  }
  centroid(out = [0, 0]) {
    for (const p of this.points) {
      out[0] += p[0];
      out[1] += p[1];
    }
    const inv = 1 / this.points.length;
    out[0] *= inv;
    out[1] *= inv;
    return out;
  }
  isClockwise() {
    const pnts = this.points;
    const end = pnts.length - 1;
    let sum = 0;
    let ii;
    for (let i = 0; i < end; i++) {
      ii = i + 1;
      sum += pnts[i][0] * pnts[ii][1] - pnts[i][1] * pnts[ii][0];
    }
    return sum >= 0;
  }
  toVec3Buffer(isType = true, isYUp = true, n = 0) {
    const cnt = this.points.length;
    const buf = isType ? new Float32Array(cnt * 3) : new Array(cnt * 3);
    let i = 0;
    if (isYUp) {
      for (const p of this.points) {
        buf[i++] = p[0];
        buf[i++] = n;
        buf[i++] = p[1];
      }
    } else {
      for (const p of this.points) {
        buf[i++] = p[0];
        buf[i++] = p[1];
        buf[i++] = n;
      }
    }
    return buf;
  }
  toFlatBuffer(isType = true) {
    const cnt = this.points.length;
    const buf = isType ? new Float32Array(cnt * 2) : new Array(cnt * 2);
    let i = 0;
    for (const p of this.points) {
      buf[i++] = p[0];
      buf[i++] = p[1];
    }
    return buf;
  }
  segmentCut(p0, p1) {
    const hits = [];
    const pnts = this.points;
    let p;
    let ii;
    for (let i = 0; i < pnts.length; i++) {
      ii = (i + 1) % pnts.length;
      p = Line2D.intersectingSegments(p0, p1, pnts[i], pnts[ii]);
      if (p)
        hits.push({ pos: new Vec2(p), i, ii });
    }
    if (hits.length !== 2)
      return null;
    const [a, b] = hits;
    const poly0 = new Polygon2D([
      a.pos,
      ...pnts.slice(a.ii, b.i + 1),
      b.pos
    ]);
    const poly1 = new Polygon2D(
      b.ii < a.i ? [b.pos, ...pnts.slice(b.ii, a.i + 1), a.pos] : [b.pos, ...pnts.slice(b.ii), ...pnts.slice(0, a.i + 1), a.pos]
    );
    return [poly0, poly1];
  }
  polyline(radius = 0.1, isClosed = true) {
    const pnts = this.points;
    const cnt = this.pointCount;
    const end = isClosed ? cnt : cnt - 1;
    const edgeDir = [];
    let v;
    let i, j;
    for (i = 0; i < end; i++) {
      j = (i + 1) % cnt;
      v = Vec2.sub(pnts[j], pnts[i]).norm();
      edgeDir.push(v);
    }
    const miterDir = new Vec2();
    const normDir = new Vec2();
    const outer = [];
    const inner = [];
    const prevDir = new Vec2(isClosed ? edgeDir[edgeDir.length - 1] : [0, 0, 0]);
    let curDir;
    let scl;
    for (i = 0; i < end; i++) {
      curDir = edgeDir[i];
      normDir.copy(curDir).rotN90();
      miterDir.fromAdd(prevDir, curDir).norm().rotN90();
      scl = radius / Vec2.dot(miterDir, normDir);
      outer.push(v = Vec2.scaleThenAdd(scl, miterDir, pnts[i]));
      inner.push(v = Vec2.scaleThenAdd(scl, miterDir.negate(), pnts[i]));
      prevDir.copy(curDir);
    }
    if (!isClosed) {
      i = cnt - 1;
      normDir.copy(edgeDir[i - 1]).rotN90();
      outer.push(v = Vec2.scaleThenAdd(radius, normDir, pnts[i]));
      inner.push(v = Vec2.scaleThenAdd(radius, normDir.negate(), pnts[i]));
    }
    return [outer, inner];
  }
  iterVec3(isYUp = true) {
    let i = 0;
    const idx = isYUp ? 2 : 1;
    const result = { value: [0, 0, 0], done: false }, len = this.points.length, next = () => {
      if (i >= len)
        result.done = true;
      else {
        result.value[0] = this.points[i][0];
        result.value[idx] = this.points[i][1];
        i++;
      }
      return result;
    };
    return { [Symbol.iterator]() {
      return { next };
    } };
  }
  iterEdges() {
    const v = { a: this.points[0], b: this.points[0], ai: 0, bi: 1 };
    const result = { value: v, done: false };
    const len = this.points.length;
    const next = () => {
      if (v.ai >= len)
        result.done = true;
      else {
        v.a = this.points[v.ai];
        v.b = this.points[v.bi];
        v.ai++;
        v.bi = (v.bi + 1) % len;
      }
      return result;
    };
    return { [Symbol.iterator]() {
      return { next };
    } };
  }
}export{Line2D,Polygon2D};