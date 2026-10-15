"""Create a posture by where things should be, not by joint angles.

    python crear_postura.py <jumper> patas  <nombre> [--dx mm] [--dy mm] [--dz mm] [--roll deg] [--pitch deg] [--yaw deg]
    python crear_postura.py <jumper> tenaza <nombre> --punta X Y Z [--desde <postura>] [--pinza RAD]
                                         (mm, base frame; the LEFT claw, coreo mirrors it)

`patas`: the body moved by (dx, dy, dz) and turned by roll/pitch/yaw over feet that stay where they stand at
HOME -- sway, crouch, lean -- by IK of the four legs' three joints. `tenaza`: the left claw's tip taken to a
point (J0..J3). `--pinza` opens the pincer: J4 of the left claw, 0 closed (HOME), negative open, -1.6 at most
(checked by rendering, 15-oct); without it J4 stays at HOME. `--desde` starts the IK from an existing claw posture: a point a few mm from a
known pose is found from that pose, where random starts can land in another fold of the arm. Reference points at HOME, base frame, mm: claw tips (187, +-73, -98),
the top of the shell z = 72 at x -112..94, the face screen x 66..91, z 32..65.

Refuses (writes nothing) when the IK misses by more than 1 mm, or when the posed claw touches the body.
Writes or replaces `[familia.nombre]` in coreo/posturas/propias.toml.
"""

import re
import sys
from pathlib import Path

import numpy as np

COREO = Path(__file__).resolve().parents[1]
DEST = COREO / "posturas" / "propias.toml"
LEGS = ("LM", "RM", "LR", "RR")
REACH = 0.001


def opt(name, n=1, default=None):
    if name not in sys.argv:
        return default
    i = sys.argv.index(name)
    v = [float(x) for x in sys.argv[i + 1:i + 1 + n]]
    return v if n > 1 else v[0]


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def main() -> None:
    repo, familia, nombre = Path(sys.argv[1]).resolve(), sys.argv[2], sys.argv[3]
    sys.path[:0] = [str(repo / "tools"), str(repo)]
    import import_wbc_gestures as G

    from tasks.jumper.common.constants import HOME

    R = G._Robot()
    mj, m, d = R.mujoco, R.model, R.data
    q = np.array([HOME[n] for n in R.names])
    feet, _ = R.feet_and_com(q[None])
    feet = feet[0]
    jac = np.zeros((3, m.nv))

    def site(leg):
        return R.feet[G.FEET.index(leg)]

    def cols(idx):
        return [m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, R.names[i])] for i in idx]

    def touches(row):
        """Pairs of bodies in contact (deeper than 1 mm) with the robot posed at `row`, in the air."""
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        d.qpos[2] = 1.0  # far above any floor
        d.qpos[R.q_adr] = row
        mj.mj_forward(m, d)
        hits = set()
        for c in d.contact[:d.ncon]:
            if c.dist < -0.001:
                n1, n2 = (mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "" for g in (c.geom1, c.geom2))
                hits.add(f"{n1}/{n2}")
        return hits

    def ik(row, goals):
        """goals: [(site id, joint indices, target)] -> (row, worst error m)."""
        err_all = 0.0
        for _ in range(200):
            d.qpos[:] = 0.0
            d.qpos[3] = 1.0
            d.qpos[R.q_adr] = row
            mj.mj_kinematics(m, d)
            err_all = 0.0
            for s, idx, tgt in goals:
                err = tgt - d.site_xpos[s]
                err_all = max(err_all, float(np.linalg.norm(err)))
                mj.mj_jacSite(m, d, jac, None, s)
                J = jac[:, cols(idx)]
                row[idx] += J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(3), 0.5 * err)
            row = np.clip(row, R.range[:, 0], R.range[:, 1])
            if err_all < 1e-5:
                break
        return row, err_all

    if familia == "patas":
        dx, dy, dz = (opt(k, default=0.0) / 1000 for k in ("--dx", "--dy", "--dz"))
        rr, pp, yy = (np.radians(opt(k, default=0.0)) for k in ("--roll", "--pitch", "--yaw"))
        Rb, tb = rot(rr, pp, yy), np.array([dx, dy, dz])
        goals = []
        for leg in LEGS:
            idx = [i for i, n in enumerate(R.names) if n.startswith(leg + "_")]
            # The foot stays put in the world; seen from the moved body it is at Rb^T (p - t).
            goals.append((site(leg), idx, Rb.T @ (feet[G.FEET.index(leg)] - tb)))
        row, err = ik(q.copy(), goals)
        values = {R.names[i]: row[i] for g in goals for i in g[1]}
        cabecera = f"patas.{nombre}"
        nota = (f"crear_postura: cuerpo {dx * 1000:+.0f} {dy * 1000:+.0f} {dz * 1000:+.0f} mm, "
                f"roll {np.degrees(rr):+.0f} pitch {np.degrees(pp):+.0f} yaw {np.degrees(yy):+.0f} deg")
    elif familia == "tenaza":
        punta = np.array(opt("--punta", 3)) / 1000
        idx = [i for i, n in enumerate(R.names) if n.startswith("LF_") and not n.startswith("LF_J4")]
        # Several starting poses: the claw can fold several ways, keep the closest solve.
        # The claw folds several ways and the IK finds the nearest: start from HOME and from
        # fixed random poses (80), keep the closest solve that does not touch the body.
        rng = np.random.default_rng(0)
        starts = [q.copy()]
        desde = sys.argv[sys.argv.index("--desde") + 1] if "--desde" in sys.argv else None
        if desde:
            import tomllib

            known = {}
            for f in sorted((COREO / "posturas").glob("*.toml")):
                known.update(tomllib.loads(f.read_text()).get("tenaza", {}))
            if desde not in known:
                raise SystemExit(f"--desde {desde}: no such claw posture; there are {', '.join(known)}")
            row = q.copy()
            for j in range(5):
                row[R.names.index(f"LF_J{j}_joint")] = known[desde][f"J{j}"]
            starts.insert(0, row)
        for _ in range(80):
            row = q.copy()
            row[idx] = [rng.uniform(*R.range[i]) for i in idx]
            starts.append(row)
        best = None
        chocan = 0
        for row in starts:
            row, err = ik(row, [(site("LF"), idx, punta)])
            if err <= REACH and touches(row):
                chocan += 1
                continue
            if best is None or err < best[1]:
                best = (row, err)
        row, err = best
        if "--pinza" in sys.argv:
            j4 = R.names.index("LF_J4_joint")
            row[j4] = float(np.clip(opt("--pinza"), *R.range[j4]))
        values = {f"J{j}": row[R.names.index(f'LF_J{j}_joint')] for j in range(5)}
        cabecera = f"tenaza.{nombre}"
        nota = f"crear_postura: punta de la tenaza izquierda en {np.round(punta * 1000).astype(int).tolist()} mm"
    else:
        raise SystemExit("familia: patas | tenaza")

    if err > REACH:
        why = (f"; {chocan} solve(s) reached it but put the claw into the body"
               if familia == "tenaza" and chocan else " (joint ranges)")
        raise SystemExit(f"{cabecera}: the IK misses by {err * 1000:.1f} mm{why}; nothing written")
    hits = touches(row)
    if hits:
        raise SystemExit(f"{cabecera}: the posed robot touches itself ({', '.join(sorted(hits))}); nothing written")

    body = "\n".join(f"{k} = {v:.4f}" for k, v in values.items())
    entry = f"# {nota}\n[{cabecera}]\n{body}\n"
    text = DEST.read_text() if DEST.exists() else "# Posturas creadas con puente/crear_postura.py (por posición, no por ángulos).\n"
    pat = re.compile(rf"(# [^\n]*\n)?\[{re.escape(cabecera)}\]\n(?:[^\n\[]*\n)*", re.M)
    text = pat.sub("", text).rstrip("\n") + "\n\n" + entry
    DEST.write_text(text)
    print(f"[{cabecera}] -> {DEST.name}  ({nota}; IK error {err * 1000:.2f} mm)")


if __name__ == "__main__":
    main()
