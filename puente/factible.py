"""Can the real robot do this clip? Speed, torque and self-contact of each claw joint, against the servo's curve.

Rodhan (15-oct): «let us not make moves the robot will then not manage: very sudden moves might not run on the real
robot; mind inertia, execution time». A kinematic clip can ask for anything; this measures what it asks of the
servos before any training:

- speed: the fastest each joint moves, against the servo's corner speed (where its torque starts to fall);
- torque: inverse dynamics of the two front arms (in the air they carry their own inertia and gravity, nothing
  else), against `servo_torque_limit` at that speed, the 300 ms peak rating and the 1.2 N*m continuous one;
- lag: the torque over the PD's stiffness (kp = 10 N*m/rad) is how far the real joint trails the command;
- contact: rows where two parts of the robot touch (claw on claw, claw on body or phone), robot posed in the air,
  and how close any leg comes to the phone holder and phone, when the model has them (montar.py).

The legs are not judged here: they stand on the floor, and inverse dynamics without the floor's forces says
nothing about them -- the trained policy is their test. Exit 1 when something is over a limit.

    python factible.py <jumper checkout> <clip.npz> [--modelo <xml>] [--lag 0.10]
                       [--telefono-mesa X,Y,GIRO[,INCLINACION]]

`--telefono-mesa` (the same numbers as blender/escena.py) puts the phone on the table beside the robot, and its
stand when it leans, and checks every row with the robot where the clip has it: no part into them, and how close.
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

repo, clip = Path(sys.argv[1]).resolve(), Path(sys.argv[2])
arg = lambda n, d=None: sys.argv[sys.argv.index(n) + 1] if n in sys.argv else d  # noqa: E731
xml = Path(arg("--modelo", repo / "assets/jumper/jumper.xml"))
LAG_MAX = float(arg("--lag", 0.10))   # rad (~6 deg): beyond it the real move visibly smears
sys.path[:0] = [str(repo / "tools"), str(repo)]
import import_wbc_dances as dances  # noqa: E402
from tasks.jumper.common.actuator import CURVE  # noqa: E402
from tasks.jumper.common.constants import STIFFNESS  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

z = np.load(clip)
dt = float(z["dt"])
names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)
base = np.stack([z[f"body_{a}"] for a in dances.AXES], axis=1).astype(np.float64)
spec = mujoco.MjSpec.from_file(str(xml))
obstaculos = []
if arg("--telefono-mesa"):
    tx, ty, giro, *inc = (float(v) for v in arg("--telefono-mesa").split(","))
    inc = math.radians(inc[0]) if inc else 0.0
    gz, z0 = math.radians(giro), 0.0027 + (0.0738 * math.sin(inc) if inc else 0.0)
    # the phone: origin at the centre of its back, X across, Y along (top +y), Z out of the screen;
    # turned INCLINACION about x (top up), then GIRO about z, as escena.py does
    Rz = np.array([[math.cos(gz), -math.sin(gz), 0], [math.sin(gz), math.cos(gz), 0], [0, 0, 1]])
    R = Rz @ np.array([[1, 0, 0], [0, math.cos(inc), -math.sin(inc)], [0, math.sin(inc), math.cos(inc)]])
    origen = np.array([tx, ty, z0])
    cajas = [("telefono", origen + R @ [0, 0, 0.0039], [0.0358, 0.0738, 0.0039], R)]
    if inc > 0:   # the stand: under the back from the bottom end to a little past the middle, down to the table
        y0, y1 = -0.072, 0.030
        alto = z0 + y1 * math.sin(inc)
        c = origen + R @ [0, (y0 + y1) / 2, 0]
        cajas.append(("soporte", np.array([c[0], c[1], alto / 2]), [0.03, (y1 - y0) * math.cos(inc) / 2, alto / 2], Rz))
    for nombre, pos, half, rot in cajas:
        g = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=half, pos=pos.tolist())
        g.name = f"obst_{nombre}"
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.asarray(rot, float).reshape(-1))
        g.quat = quat.tolist()
        g.contype = g.conaffinity = 1
        g.margin = 0.03
        obstaculos.append(f"obst_{nombre}")
m = spec.compile()
d = mujoco.MjData(m)
free = next(j for j in range(m.njnt) if m.jnt_type[j] == 0)
qa, va = m.jnt_qposadr[free], m.jnt_dofadr[free]
jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
adr, dof = [m.jnt_qposadr[j] for j in jid], [m.jnt_dofadr[j] for j in jid]
arm = [i for i, n in enumerate(names) if n[:2] in ("LF", "RF")]


def smooth_diff(x, k=1):
    """Derivative along rows, central differences over a 3-row (60 ms) smoothing: what a servo would follow."""
    x = np.apply_along_axis(lambda c: np.convolve(np.pad(c, 2, mode="edge"), np.ones(5) / 5, mode="valid"), 0, x)
    for _ in range(k):
        x = np.gradient(x, dt, axis=0)
    return x


qd, qdd = smooth_diff(q), smooth_diff(q, 2)
# base pose as qpos rows (position + quaternion), and its velocity/acceleration
pos = []
for x, y, zz, r, p, yw in base:
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, [r, p, yw], "xyz")
    pos.append((x, y, zz, *quat))
pos = np.array(pos)
bv, ba = smooth_diff(base[:, :3]), smooth_diff(base[:, :3], 2)
bw, bwd = smooth_diff(base[:, 3:]), smooth_diff(base[:, 3:], 2)   # small angles: rpy rates ~ body rates

tau = np.zeros((len(q), len(names)))
touch = {}
m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
for k in range(len(q)):
    d.qpos[qa:qa + 7] = pos[k]
    d.qpos[adr] = q[k]
    d.qvel[:] = 0
    d.qacc[:] = 0
    d.qvel[va:va + 3], d.qvel[va + 3:va + 6] = bv[k], bw[k]
    d.qacc[va:va + 3], d.qacc[va + 3:va + 6] = ba[k], bwd[k]
    d.qvel[dof], d.qacc[dof] = qd[k], qdd[k]
    mujoco.mj_inverse(m, d)
    tau[k] = d.qfrc_inverse[dof]
m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
# the holder and phone (montar.py) get a 30 mm margin, so the contact pass also measures how close a leg comes
carga = [g for g in range(m.ngeom) if (m.geom(g).name or "").startswith(("cuna", "telefono")) and m.geom_contype[g]]
m.geom_margin[carga] = 0.03
cerca = {}
for k in range(len(q)):   # self-contact, in the air (no floor in the model): deeper than 1 mm
    d.qpos[:] = 0
    d.qpos[qa:qa + 7] = (0, 0, 1.0, *pos[k][3:])
    d.qpos[adr] = q[k]
    mujoco.mj_forward(m, d)
    for c in d.contact[:d.ncon]:
        if carga and (c.geom1 in carga) != (c.geom2 in carga):
            otro = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2 if c.geom1 in carga else c.geom1])
            cerca[otro] = min(cerca.get(otro, 1.0), c.dist)
        if c.dist < -0.001:
            pair = "/".join(sorted(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "" for g in (c.geom1, c.geom2)))
            touch.setdefault(pair, []).append(k * dt)

speed = np.abs(qd)
avail = CURVE.plateau_torque * np.exp(-np.clip(speed - CURVE.corner_speed, 0, None) / CURVE.decay_speed)
avail[speed >= CURVE.cutoff_speed] = 0
lag = np.abs(tau) / STIFFNESS
over = False
print(f"{clip.name}: {len(q) * dt:.1f} s; servo: {CURVE.plateau_torque} N*m peak (300 ms), {CURVE.continuous_torque} "
      f"continuous, torque falls above {CURVE.corner_speed:.1f} rad/s")
print(f"{'joint':14} {'rad/s':>6} {'rad/s2':>7} {'N*m':>6} {'%avail':>7} {'lag deg':>8}")
for i in arm:
    a = np.abs(tau[:, i]) / np.maximum(avail[:, i], 1e-6)
    # longest stretch above the continuous rating
    above, run, longest = np.abs(tau[:, i]) > CURVE.continuous_torque, 0, 0
    for b in above:
        run = run + 1 if b else 0
        longest = max(longest, run)
    bad = a.max() > 1 or longest * dt > CURVE.peak_duration or lag[:, i].max() > LAG_MAX
    over |= bad
    print(f"{names[i]:14} {speed[:, i].max():6.2f} {np.abs(qdd[:, i]).max():7.1f} {np.abs(tau[:, i]).max():6.3f} "
          f"{100 * a.max():6.0f}% {np.degrees(lag[:, i].max()):8.1f}{'  <-- over' if bad else ''}")
legs_v = max(speed[:, i].max() for i in range(len(names)) if i not in arm)
print(f"legs: fastest {legs_v:.2f} rad/s (torque: the policy's test)")
if touch:
    over = True
    for pair, ts in sorted(touch.items()):
        print(f"contact {pair}: {len(ts)} rows, {ts[0]:.2f}-{ts[-1]:.2f} s  <-- over")
else:
    print("contact: none (claws, body, cradle and phone, deeper than 1 mm)")
if carga:
    near = sorted(cerca.items(), key=lambda e: e[1])[:3]
    print("closest to the holder/phone: " + (", ".join(f"{n} {1000 * v:.0f} mm" for n, v in near) if near else "nothing within 30 mm"))
if obstaculos:
    ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in obstaculos]
    cerca_o, dentro = {}, {}
    for k in range(len(q)):   # the robot where the clip has it, not in the air
        d.qpos[:] = 0
        d.qpos[qa:qa + 7] = pos[k]
        d.qpos[adr] = q[k]
        mujoco.mj_forward(m, d)
        for c in d.contact[:d.ncon]:
            if (c.geom1 in ids) != (c.geom2 in ids):
                otro = c.geom2 if c.geom1 in ids else c.geom1
                o = m.geom(c.geom1 if c.geom1 in ids else c.geom2).name
                b = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[otro])
                cerca_o[(o, b)] = min(cerca_o.get((o, b), 1.0), c.dist)
                if c.dist < -0.001:
                    dentro.setdefault((o, b), []).append(k * dt)
    near = sorted(cerca_o.items(), key=lambda e: e[1])[:3]
    print("closest to the phone on the table: " + (", ".join(f"{b} {1000 * v:.0f} mm from {o[5:]}" for (o, b), v in near)
                                                  if near else "nothing within 30 mm"))
    for (o, b), ts in sorted(dentro.items()):
        over = True
        print(f"{b} into the {o[5:]}: {len(ts)} rows, {ts[0]:.2f}-{ts[-1]:.2f} s  <-- over")
print("OK: within the servo's limits" if not over else "NOT OK: see the marked lines")
sys.exit(1 if over else 0)
