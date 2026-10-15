"""Detour the claws over the top wherever a transition runs them into each other or into the body.

Going straight from one claw pose to another in joint space can sweep a claw through the other one or through
the shell -- reposo -> reverencia did, both ways (15-oct), and the fix by hand was a stop at `arriba` written into
the step. This pass does that by itself: it finds the stretches of the clip where the robot touches itself (the
same check as clip_a_npz.py, every row), and over each one pulls both claws towards `tenazas arriba` with a
raised-cosine bump that is zero at both ends of the stretch plus PAD. The bump is made as small as works (A from
0.3 up to 1.0); if even A = 1 leaves contact, or the contact is in a pose the guion holds (the pose itself
collides, not the path), it says so and stops.

    python desviar.py <jumper checkout> <clip.json> <out.json>
"""

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

COREO = Path(__file__).resolve().parents[1]
#: Extra time each side of a contact stretch over which the detour rises and falls, seconds.
PAD = 0.4
DEEP = 0.001  # contact deeper than 1 mm, as clip_a_npz.py


def arriba_row(names):
    """Both claws at `arriba`, as coreo itself poses it (the right claw mirrored from HOME)."""
    with tempfile.TemporaryDirectory() as d:
        g, j = Path(d) / "a.guion", Path(d) / "a.json"
        g.write_text("pulso 60\n@0 tenazas arriba en 1\n@2 fin\n")
        subprocess.run([COREO / "target" / "release" / "coreo", "compilar", g, "-o", j],
                       check=True, capture_output=True)
        c = json.loads(j.read_text())
        return np.asarray(c["q"][-1])[[c["joints"].index(n) for n in names]]


def main(repo: Path, src: Path, dest: Path) -> None:
    sys.path[:0] = [str(repo / "tools"), str(repo)]
    import import_wbc_gestures as G

    clip = json.loads(src.read_text())
    R = G._Robot()
    mj, m, d = R.mujoco, R.model, R.data
    order = [clip["joints"].index(n) for n in R.names]
    q = np.asarray(clip["q"], dtype=np.float64)[:, order]
    dt = 1.0 / clip["hz"]
    claw = np.array([n.startswith(("LF_", "RF_")) for n in R.names])
    up = arriba_row(R.names)

    def touches(row):
        d.qpos[:] = 0.0
        d.qpos[2], d.qpos[3] = 1.0, 1.0
        d.qpos[R.q_adr] = row
        mj.mj_forward(m, d)
        return any(c.dist < -DEEP for c in d.contact[:d.ncon])

    hit = np.array([touches(r) for r in q])
    if not hit.any():
        out = q
        detours = []
    else:
        idx = np.nonzero(hit)[0]
        windows = np.split(idx, np.nonzero(np.diff(idx) > 1)[0] + 1)
        out = q.copy()
        detours = []
        pad = round(PAD / dt)
        for w in windows:
            a, b = max(0, w[0] - pad), min(len(q) - 1, w[-1] + pad)
            # A held pose that collides is the posture's problem, not the path's.
            held = np.abs(np.diff(q[w[0]:w[-1] + 1], axis=0)).max(initial=0.0) < 1e-6 and len(w) > 5
            if held:
                raise SystemExit(f"{src.name}: the robot touches itself in a held pose at {w[0] * dt:.2f} s; "
                                 "that posture collides (fix it with crear_postura), a detour cannot")
            u = (np.arange(a, b + 1) - a) / max(1, b - a)
            bump = 0.5 * (1.0 - np.cos(2.0 * np.pi * u))
            for amp in (0.3, 0.5, 0.7, 0.9, 1.0):
                seg = q[a:b + 1].copy()
                seg[:, claw] += (amp * bump)[:, None] * (up[claw] - seg[:, claw])
                if not any(touches(r) for r in seg):
                    out[a:b + 1] = seg
                    detours.append((round(a * dt, 2), round(b * dt, 2), amp))
                    break
            else:
                raise SystemExit(f"{src.name}: the claws still touch at {w[0] * dt:.2f}-{w[-1] * dt:.2f} s even "
                                 "going all the way up; change the guion there")

    clip["q"] = out[:, [R.names.index(n) for n in clip["joints"]]].tolist()
    clip.setdefault("informe", {})["desvios"] = detours
    dest.write_text(json.dumps(clip))
    print(f"{src.name} -> {dest.name}: " + ("no contact, nothing to detour" if not detours else
          "; ".join(f"claws over the top {a:.2f}-{b:.2f} s (A {amp})" for a, b, amp in detours)))


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve(), Path(sys.argv[2]), Path(sys.argv[3]))
