"""The other end of a stroke: a claw posture whose tip moves (DX, DY, DZ) mm from <desde>, the arm in the same fold.

    python golpe.py <jumper checkout> <desde> <nombre> DX DY DZ

A strum, a key press or a drum hit is two postures a short move apart. crear_postura.py places a tip anywhere, but
the arm has more than one way to fold onto a point, and the IK may pick another fold for the second end: then the
move between them swings a joint through radians and runs past the servo's speed (16-oct: the guitar's strum at
37 rad/s). This samples small joint changes around <desde> instead, and keeps the one that moves the tip as asked
with the least joint change. Refuses (writes nothing) if that pose touches the body. Writes or replaces
`[tenaza.<nombre>]` in coreo/posturas/propias.toml, the left claw (coreo mirrors it), J4 as in <desde>.
"""

import re
import sys
import tomllib
from pathlib import Path

import numpy as np

COREO = Path(__file__).resolve().parents[1]
DEST = COREO / "posturas" / "propias.toml"


def main() -> None:
    repo, desde, nombre = Path(sys.argv[1]).resolve(), sys.argv[2], sys.argv[3]
    mov = np.array([float(v) for v in sys.argv[4:7]]) / 1000
    sys.path[:0] = [str(repo / "tools"), str(repo)]
    import import_wbc_gestures as G

    from tasks.jumper.common.constants import HOME

    R = G._Robot()
    mj, m, d = R.mujoco, R.model, R.data
    known = {}
    for f in sorted((COREO / "posturas").glob("*.toml")):
        known.update(tomllib.loads(f.read_text()).get("tenaza", {}))
    if desde not in known:
        raise SystemExit(f"{desde}: no such claw posture")
    q = np.array([HOME[n] for n in R.names])
    for j in range(5):
        q[R.names.index(f"LF_J{j}_joint")] = known[desde][f"J{j}"]
    idx = [R.names.index(f"LF_J{j}_joint") for j in range(4)]
    site = R.feet[G.FEET.index("LF")]

    def tip(row):
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        d.qpos[R.q_adr] = row
        mj.mj_kinematics(m, d)
        return d.site_xpos[site].copy()

    def touches(row):
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        d.qpos[2] = 1.0
        d.qpos[R.q_adr] = row
        mj.mj_forward(m, d)
        return any(c.dist < -0.001 for c in d.contact[:d.ncon])

    p0 = tip(q)
    rng = np.random.default_rng(0)
    lo, hi = R.range[idx, 0] + 0.002, R.range[idx, 1] - 0.002
    best = None
    for _ in range(20000):
        row = q.copy()
        row[idx] = np.clip(q[idx] + rng.uniform(-0.8, 0.8, 4) * rng.uniform(0.05, 1), lo, hi)
        e = np.linalg.norm(tip(row) - p0 - mov)
        score = e * 1000 + 8 * np.abs(row[idx] - q[idx]).max()   # mm off, plus 8 mm per radian of the largest change
        if best is None or score < best[0]:
            best = (score, row, e)
    _, row, e = best
    if touches(row):
        raise SystemExit(f"tenaza.{nombre}: the posed claw touches the body; nothing written")
    body = "\n".join(f"J{j} = {row[R.names.index(f'LF_J{j}_joint')]:.4f}" for j in range(5))
    nota = f"# golpe.py: desde {desde}, la punta {np.round(mov * 1000).astype(int).tolist()} mm (error {e * 1000:.1f} mm)"
    text = DEST.read_text()
    text = re.sub(rf"(# [^\n]*\n)?\[tenaza\.{re.escape(nombre)}\]\n(?:[^\n\[]*\n)*\n?", "", text).rstrip("\n")
    DEST.write_text(f"{text}\n\n{nota}\n[tenaza.{nombre}]\n{body}\n")
    print(f"[tenaza.{nombre}] -> {DEST.name}  (tip off by {e * 1000:.1f} mm, "
          f"largest joint change {np.abs(row[idx] - q[idx]).max():.2f} rad)")


if __name__ == "__main__":
    main()
