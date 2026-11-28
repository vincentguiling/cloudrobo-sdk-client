// 手柄位姿的变换校准（HUD 侧，2026-11-18 需求，模型 b：影响 running 跟随映射）：
// 网关的相对映射是 target = lastOutput · inv(ctrl@press) · ctrl(t)，因此校准必须
// 作用在相对 press 的"增量"上才不会在映射中被抵消：
//   posR（位置变换）：重定向手柄的位移方向（眼镜系表达后旋转）
//     dp_head = conj(q_head) · (p − p_base)；dp_adj = posR · dp_head
//     p_eff   = p_base + q_head · dp_adj
//   rotR（姿态变换）：右乘在手柄自身系，经网关共轭后重定向旋转增量
//     q_eff   = q_ctrl ⊗ rotR
// 按钮把对应矩阵旋转 90°：位置 posR ← R_axis(±90°)·posR（眼镜/世界轴，作用于运动
// 方向）；姿态 rotR ← rotR·R_axis(±90°)（手柄自身轴，右乘保持有效性）。
// 位置与姿态分开、左右手独立。RUNNING 中矩阵锁定为常数（pause 下可调，resume 生效）。
// 纯函数、无依赖，便于 node 直接单测（页面与 HUD 共用）。

// 角度制轴旋转 -> 3x3 行主序旋转矩阵
export function axisRot3(axis, deg) {
  const a = (deg * Math.PI) / 180;
  const c = Math.cos(a);
  const s = Math.sin(a);
  if (axis === 'X') return [1, 0, 0, 0, c, -s, 0, s, c];
  if (axis === 'Y') return [c, 0, s, 0, 1, 0, -s, 0, c];
  if (axis === 'Z') return [c, -s, 0, s, c, 0, 0, 0, 1];
  throw new Error('bad axis: ' + axis);
}

function mat3MulVec(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
  ];
}

function mat3Mul(a, b) {
  const o = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      for (let k = 0; k < 3; k++) {
        o[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
      }
    }
  }
  return o;
}

// 四元数乘法 [x,y,z,w]：a ⊗ b
export function quatMul(a, b) {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz
  ];
}

// 四元数共轭（单位四元数的逆）
export function quatConj(q) {
  return [-q[0], -q[1], -q[2], q[3]];
}

// 四元数 [x,y,z,w] -> 3x3 行主序旋转矩阵
export function quatFromMat3(m) {
  const trace = m[0] + m[4] + m[8];
  let x, y, z, w;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1.0) * 2;
    w = 0.25 * s;
    x = (m[7] - m[5]) / s;
    y = (m[2] - m[6]) / s;
    z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
    w = (m[7] - m[5]) / s;
    x = 0.25 * s;
    y = (m[1] + m[3]) / s;
    z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = Math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
    w = (m[2] - m[6]) / s;
    x = (m[1] + m[3]) / s;
    y = 0.25 * s;
    z = (m[5] + m[7]) / s;
  } else {
    const s = Math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
    w = (m[3] - m[1]) / s;
    x = (m[2] + m[6]) / s;
    y = (m[5] + m[7]) / s;
    z = 0.25 * s;
  }
  return [x, y, z, w];
}

// 四元数 [x,y,z,w] -> 3x3 行主序旋转矩阵（显示用，与 quatFromMat3 互逆）
export function mat3FromQuat(q) {
  const [x, y, z, w] = quatNormalize(q);
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)
  ];
}

function quatNormalize(q) {
  const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / l, q[1] / l, q[2] / l, q[3] / l];
}

// 用四元数旋转一个向量：q · v · conj(q)
export function quatRotateVec(q, v) {
  const [x, y, z, w] = q;
  // v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + w * v)
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)
  ];
}

export function identityAdj() {
  return { posR: [1, 0, 0, 0, 1, 0, 0, 0, 1], rotR: [1, 0, 0, 0, 1, 0, 0, 0, 1] };
}

// 重置为单位阵（就地修改并返回）
export function resetAdjust(adj) {
  adj.posR = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  adj.rotR = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  return adj;
}

// adj 是否为单位阵（未做任何调整）
export function isIdentityAdj(adj) {
  const i = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  return adj.posR.every((v, k) => Math.abs(v - i[k]) < 1e-6)
    && adj.rotR.every((v, k) => Math.abs(v - i[k]) < 1e-6);
}

// 把校准应用于一个手柄位姿（需要基准 base 与眼镜朝向 headQ）：
//   p_eff = p_base + q_head · posR · conj(q_head) · (p − p_base)
//   q_eff = q_ctrl ⊗ rotR            （rotR 右乘，网关相对映射中共轭重定向）
// 无基准（该手未在线）时透传。
export function applyAdjust(adj, p, q, base, headQ) {
  if (!base) {
    return { p: [p[0], p[1], p[2]], q: quatNormalize([q[0], q[1], q[2], q[3]]) };
  }
  const head = headQ || [0, 0, 0, 1];
  const dp = [p[0] - base.p[0], p[1] - base.p[1], p[2] - base.p[2]];
  const dpHead = quatRotateVec(quatConj(head), dp);
  const dpAdj = mat3MulVec(adj.posR, dpHead);
  const dpWorld = quatRotateVec(head, dpAdj);
  return {
    p: [base.p[0] + dpWorld[0], base.p[1] + dpWorld[1], base.p[2] + dpWorld[2]],
    q: quatNormalize(quatMul(q, quatFromMat3(adj.rotR)))
  };
}

// 按钮旋转：part 'pos' 旋转位置变换矩阵（世界/眼镜系轴，左乘）；
// 'rot' 旋转姿态变换矩阵（手柄自身系轴，右乘——这样校准在网关相对映射中有效）。
// axis 'X'|'Y'|'Z'；sign +1 = +90°、-1 = -90°。就地修改并返回。
export function rotateAdjust(adj, part, axis, sign) {
  const r = axisRot3(axis, 90 * (sign >= 0 ? 1 : -1));
  if (part === 'pos') {
    adj.posR = mat3Mul(r, adj.posR);
  } else if (part === 'rot') {
    adj.rotR = mat3Mul(adj.rotR, r);
  } else {
    throw new Error('bad part: ' + part);
  }
  return adj;
}

// RUNNING 锁定：仅非 RUNNING（Idle/Finished/Paused/未知）允许调整
export function canAdjust(state) {
  return String(state || '').toUpperCase() !== 'RUNNING';
}

// 四元数双覆盖连续性：q 与 -q 是同一旋转，但 WebXR 每帧给出的符号可能跳变；
// 数值符号跳变会让下游（机器人端）收到"反向"的目标而大幅摆动。
// 与上一帧四元数点积 < 0 时取反，使数值连续（旋转语义不变）。
// store 为跨帧保存的 {x,y,z,w,valid}（就地更新）。
export function quatContinuityFix(q, store) {
  if (store && store.valid) {
    const dot = q[0] * store.x + q[1] * store.y + q[2] * store.z + q[3] * store.w;
    if (dot < 0) {
      q = [-q[0], -q[1], -q[2], -q[3]];
    }
  }
  store.x = q[0];
  store.y = q[1];
  store.z = q[2];
  store.w = q[3];
  store.valid = true;
  return q;
}