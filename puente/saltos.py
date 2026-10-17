"""What jumps can Jumper do? Simulated, against the servo's measured curve: height, distance, turn, time, force.

Rodhan (17-oct): «the robot is called Jumper: first work out the jump's distance, duration, force; whether it can
jump sideways or forwards, jump sideways to roll... then give each case the moves it can do».

An open-loop jump in MuJoCo, the controller the robot has: a PD on every joint (kp 20, kd 0.5, as jumper.jump
validated its reference) whose torque is clipped every millisecond by `servo_torque_limit` at the joint's speed.
The jump is the reference's own phases (tasks/jumper/jump/ref/high_jump_flat.npz, meta): crouch, a hard push
towards an extended pose (aimed past it by PUSH_P, as the reference's `push_p`), legs drawn in while in the air,
reach for the ground, absorb. The poses are the body moved over feet that stay put (IK of each leg), so a jump is
aimed by where the push carries the body: up, forward (x), sideways (y), or turned (yaw).

A search over the push's size and aim keeps, for each kind of jump, the best one that lands upright on all six
feet with the shell off the floor. Open loop is the floor of what the robot can do: a trained policy (jumper.jump
reached 0.159 m against this reference's 0.141) does a little better.

    python saltos.py <jumper checkout> [--tipo vertical|adelante|lado|giro|saltito|rodar] [--json out.json]
    python saltos.py <jumper checkout> --grabar TIPO out.npz --de saltos.json [--espejo]

`--grabar` records the jump a search found (saltos.json) as a clip .npz (the same channels as coreo's clips, 50 Hz),
for montaje.py and the previews; `--espejo` jumps to the other side (or turns the other way).
"""

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

repo = Path(sys.argv[1]).resolve()
arg = lambda n, d=None: sys.argv[sys.argv.index(n) + 1] if n in sys.argv else d  # noqa: E731
sys.path[:0] = [str(repo)]
from tasks.jumper.common.actuator import CURVE  # noqa: E402
from tasks.jumper.common.constants import FEET, HOME, STAND_Z  # noqa: E402

KP, KD = 20.0, 0.5
PUSH_P = 1.53                      # the reference's push_p: the push aims past the extended pose
T_CROUCH, T_GO_HOLD = 0.34, 0.054  # the reference's crouch and push times
TILT_OK = math.radians(20)

spec = mujoco.MjSpec.from_file(str(repo / "assets/jumper/jumper.xml"))
floor = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
floor.name = "suelo"
m = spec.compile()
d = mujoco.MjData(m)
dt = m.opt.timestep
names = [m.joint(j).name for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in names])
dadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in names])
lo, hi = m.jnt_range[[m.joint(n).id for n in names]].T
home = np.array([HOME[n] for n in names])
feet = [m.body(f).id for f in FEET]
legs = {f: [i for i, n in enumerate(names) if n.startswith(f[:2]) and "J4" not in n] for f in FEET}
suelo = m.geom("suelo").id
shell = {m.body(b).id for b in ("base_link", "upper_shell_link", "display_module_link")}
mass = float(sum(m.body_mass))


def limite(w):
    """servo_torque_limit (tasks/jumper/common/actuator.py) for a numpy speed array."""
    w = np.abs(w)
    lim = CURVE.plateau_torque * np.exp(-np.clip(w - CURVE.corner_speed, 0, None) / CURVE.decay_speed)
    return np.where(w >= CURVE.cutoff_speed, 0.0, lim)


def quat(roll, pitch, yaw):
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, [roll, pitch, yaw], "xyz")
    return q


# the stand: HOME at STAND_Z, and where its feet are
d.qpos[:3] = (0, 0, STAND_Z)
d.qpos[3:7] = quat(0, 0, 0)
d.qpos[qadr] = home
mujoco.mj_kinematics(m, d)
pies0 = np.array([d.xpos[b] for b in feet])


def postura(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, roll=0.0):
    """Leg joints that put the body at (dx, dy, STAND_Z + dz, roll, yaw) over the stand's feet (IK, each leg)."""
    q = home.copy()
    jac = np.zeros((3, m.nv))
    for _ in range(60):
        d.qpos[:3] = (dx, dy, STAND_Z + dz)
        d.qpos[3:7] = quat(roll, 0, yaw)
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        err = 0.0
        for f, b, tgt in zip(FEET, feet, pies0):
            e = tgt - d.xpos[b]
            err = max(err, np.linalg.norm(e))
            mujoco.mj_jacBody(m, d, jac, None, b)
            J = jac[:, dadr[legs[f]]]
            q[legs[f]] += J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(3), 0.7 * e)
        q = np.clip(q, lo + 0.002, hi - 0.002)
        if err < 1e-4:
            break
    return q, err


def saltar(d_c, d_e, dx=0.0, dy=0.0, yaw=0.0, push_p=PUSH_P, grabar=False, roll=0.0):
    """One jump: crouch d_c, push to d_e up (and dx, dy, yaw), tuck, reach, absorb. Returns its measurements."""
    crouch, _ = postura(-0.4 * dx, -0.4 * dy, -d_c, -0.4 * yaw)
    ext, e_ext = postura(dx, dy, d_e, yaw, roll)
    tuck, _ = postura(0, 0, -d_c, 0)
    mujoco.mj_resetData(m, d)
    d.qpos[:3] = (0, 0, STAND_Z)
    d.qpos[3:7] = quat(0, 0, 0)
    d.qpos[qadr] = home
    mujoco.mj_forward(m, d)
    t_go = 0.3 + T_CROUCH
    fase, t_aire, t_vuelo0, t_vuelo = "quieto", None, None, 0.0
    z_max, tau_max, heat, frames, roll_max = 0.0, 0.0, 0.0, [], 0.0
    shell_hit = False
    peak_force = 0.0
    T = t_go + 1.4
    for k in range(int(T / dt)):
        t = k * dt
        q, qd = d.qpos[qadr], d.qvel[dadr]
        en_suelo = 0
        fz = 0.0
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if suelo in (c.geom1, c.geom2):
                b = m.geom_bodyid[c.geom2 if c.geom1 == suelo else c.geom1]
                if b in shell:
                    shell_hit = True
                mujoco.mj_contactForce(m, d, i, f6)
                fz += f6[0]          # normal force, contact frame
                en_suelo += 1
        if t < 0.3:
            tgt = home
        elif t < t_go:
            u = (t - 0.3) / T_CROUCH
            u = u * u * (3 - 2 * u)
            tgt = home + (crouch - home) * u
            fase = "agacharse"
        elif fase in ("agacharse", "empuje") and en_suelo > 0 and t < t_go + 0.25:
            tgt = np.clip(crouch + push_p * (ext - crouch), lo, hi)
            fase = "empuje"
        elif fase in ("empuje", "aire") and en_suelo == 0:
            if t_vuelo0 is None:
                t_vuelo0 = t
            # in the air: draw the legs in, then reach for the ground once falling
            tgt = tuck if d.qvel[2] > 0 else home
            fase = "aire"
        else:
            if fase == "aire" and t_vuelo0 is not None:
                t_vuelo = t - t_vuelo0
            fase = "aterrizaje"
            tgt = home
        tau = KP * (tgt - q) - KD * qd
        lim = limite(qd)
        tau = np.clip(tau, -lim, lim)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[dadr] = tau
        tau_max = max(tau_max, float(np.abs(tau).max()))
        heat += float((np.abs(tau) > CURVE.continuous_torque).any()) * dt if fase == "empuje" else 0.0
        mujoco.mj_step(m, d)
        z_max = max(z_max, d.qpos[2])
        w = d.qpos[3:7]
        roll_t = math.atan2(2 * (w[0] * w[1] + w[2] * w[3]), 1 - 2 * (w[1] ** 2 + w[2] ** 2))
        roll_max = max(roll_max, abs(roll_t)) if k else 0.0
        if fase in ("aterrizaje", "empuje"):
            peak_force = max(peak_force, fz)
        if grabar and k % int(round(1 / (50 * dt))) == 0:
            frames.append((t, d.qpos.copy()))
    w = d.qpos[3:7]
    tilt = math.acos(min(1.0, abs(1 - 2 * (w[1] ** 2 + w[2] ** 2))))
    yaw_f = math.atan2(2 * (w[0] * w[3] + w[1] * w[2]), 1 - 2 * (w[2] ** 2 + w[3] ** 2))
    pies_suelo = sum(1 for c in d.contact[:d.ncon] if suelo in (c.geom1, c.geom2)
                     and m.geom_bodyid[c.geom2 if c.geom1 == suelo else c.geom1] in feet)
    ok = tilt < TILT_OK and not shell_hit and abs(d.qpos[2] - STAND_Z) < 0.03
    return {"ok": ok, "subida_cm": round((z_max - STAND_Z) * 100, 1), "vuelo_s": round(t_vuelo, 3),
            "avance_cm": round(d.qpos[0] * 100, 1), "lado_cm": round(d.qpos[1] * 100, 1),
            "giro_deg": round(math.degrees(yaw_f), 1), "rodar_max_deg": round(math.degrees(roll_max), 1), "inclinacion_final_deg": round(math.degrees(tilt), 1),
            "caparazon_toca": shell_hit, "par_max_Nm": round(tau_max, 2), "ik_mm": round(e_ext * 1000, 1),
            "fuerza_suelo_max_N": round(peak_force, 0), "fuerza_en_pesos": round(peak_force / (mass * 9.81), 1),
            "duracion_s": round(T - 0.3, 2), "params": dict(d_c=d_c, d_e=d_e, dx=dx, dy=dy, yaw=yaw, push_p=push_p),
            **({"frames": frames} if grabar else {})}


def buscar(tipo):
    """The best jump of a kind that lands well, over a grid of push sizes and aims."""
    best = None
    for d_c in (0.025, 0.037):
        for d_e in (0.04, 0.045, 0.05, 0.055, 0.06):
            # the open-loop push drifts back ~10 cm (the claws push unlike the legs): aims include the correction
            aims = {"vertical": [(a, 0, 0) for a in (0.0, 0.02, 0.04)], "saltito": [(a, 0, 0) for a in (0.0, 0.02)],
                    "adelante": [(a, 0, 0) for a in (0.04, 0.07, 0.10, 0.13)],
                    "lado": [(0.02, a, 0) for a in (0.03, 0.06, 0.09, 0.12)],
                    "giro": [(0.02, 0, math.radians(a)) for a in (15, 30, 45, 60)]}[tipo]
            pps = (1.0, 1.15, 1.3) if tipo == "saltito" else (1.3, PUSH_P, 1.7)
            for dx, dy, yaw in aims:
                for pp in pps:
                    r = saltar(d_c, d_e, dx, dy, yaw, pp)
                    if not r["ok"]:
                        continue
                    drift = math.hypot(r["avance_cm"], r["lado_cm"])
                    key = {"vertical": r["subida_cm"] - 0.5 * drift, "adelante": r["avance_cm"] - abs(r["lado_cm"]),
                           "lado": abs(r["lado_cm"]) - abs(r["avance_cm"]),
                           "giro": abs(r["giro_deg"]) - drift,
                           "saltito": -abs(r["subida_cm"] - 4.0) - 0.5 * drift - 50 * (r["vuelo_s"] < 0.05)}[tipo]
                    if best is None or key > best[0]:
                        best = (key, r)
    return best[1] if best else None


def rodar():
    """How far can a lopsided push roll the body? The most roll reached, landing or not (landing on the back is
    what a roll would mean); it answers whether a sideways roll is within the servos' reach."""
    best = None
    for d_e in (0.05, 0.06):
        for roll in (0.3, 0.5, 0.7):
            for pp in (PUSH_P, 1.7, 2.0):
                r = saltar(0.037, d_e, 0.02, 0.0, 0.0, pp, roll=roll)
                if best is None or r["rodar_max_deg"] > best["rodar_max_deg"]:
                    best = r
    return best


def grabar(tipo, dest, de, espejo=False):
    """The found jump of `tipo` as a clip .npz (coreo's channels), mirrored in y and yaw with `espejo`."""
    global T_CROUCH
    sys.path[:0] = [str(repo / "tools")]
    import import_wbc_dances as dances
    from tasks.jumper.common.dance.motion import LEG_JOINTS

    p = json.loads(Path(de).read_text())[tipo]["params"]
    if tipo == "saltito":   # the smallest hop (17-oct): a short crouch and a small push
        T_CROUCH, p = 0.2, dict(d_c=0.015, d_e=0.04, dx=0.0, dy=0.0, yaw=0.0, push_p=0.9)
    sg = -1.0 if espejo else 1.0
    r = saltar(p["d_c"], p["d_e"], p["dx"], sg * p["dy"], sg * p["yaw"], p["push_p"], grabar=True)
    Q = np.array([q for _, q in r["frames"]])
    jn = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
    out = {"dt": np.array(0.02)}
    for k, n in zip(dances._keys(), jn):
        out[k] = Q[:, qadr[names.index(n)]]
    out["body_x"], out["body_y"], out["body_z"] = Q[:, 0], Q[:, 1], Q[:, 2]
    w = Q[:, 3:7]
    out["body_roll"] = np.arctan2(2 * (w[:, 0] * w[:, 1] + w[:, 2] * w[:, 3]), 1 - 2 * (w[:, 1] ** 2 + w[:, 2] ** 2))
    out["body_pitch"] = np.arcsin(np.clip(2 * (w[:, 0] * w[:, 2] - w[:, 3] * w[:, 1]), -1, 1))
    out["body_yaw"] = np.arctan2(2 * (w[:, 0] * w[:, 3] + w[:, 1] * w[:, 2]), 1 - 2 * (w[:, 2] ** 2 + w[:, 3] ** 2))
    np.savez(dest, **out)
    r.pop("frames")
    print(f"{dest}: {tipo}{' (mirrored)' if espejo else ''}, {len(Q)} rows; " + json.dumps(r, default=lambda o: o.item()))


if __name__ == "__main__":
    if arg("--grabar"):
        i = sys.argv.index("--grabar")
        grabar(sys.argv[i + 1], sys.argv[i + 2], arg("--de"), "--espejo" in sys.argv)
        sys.exit(0)
    tipos = [arg("--tipo")] if arg("--tipo") else ["vertical", "saltito", "adelante", "lado", "giro", "rodar"]
    out = {}
    for t in tipos:
        r = rodar() if t == "rodar" else buscar(t)
        out[t] = r
        print(t, json.dumps(r, default=lambda o: o.item()) if r else "nothing lands upright", flush=True)
    if arg("--json"):
        Path(arg("--json")).write_text(json.dumps(out, indent=1, default=lambda o: o.item()))
