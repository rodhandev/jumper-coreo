"""Support margin of the centre of mass, the rule coreo's balance uses, measured on clips.

The floor is the plane through the four legs' feet (LM RM LR RR); a claw supports when
its tip is within 5 mm of it. The margin is the signed distance (mm, + inside) of the
COM, projected on that plane, to the convex hull of the supporting feet.

    python margen.py <jumper checkout> <clip.npz> [...]
"""
import sys
from pathlib import Path

import numpy as np

repo = Path(sys.argv[1]).resolve()
sys.path[:0] = [str(repo / "tools"), str(repo)]
import import_wbc_dances as D  # noqa: E402
import import_wbc_gestures as G  # noqa: E402

DOWN = 0.005


def hull(p):
    p = sorted(map(tuple, p))
    cr = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for x in p:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], x) <= 0:
            lo.pop()
        lo.append(x)
    for x in reversed(p):
        while len(up) >= 2 and cr(up[-2], up[-1], x) <= 0:
            up.pop()
        up.append(x)
    return np.array(lo[:-1] + up[:-1])


def margin(feet, com):
    legs = feet[2:]
    c = legs.mean(0)
    n = np.linalg.svd(legs - c)[2][2]
    n = n if n[2] > 0 else -n
    u = np.cross(n, [0, 1, 0]); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    down = np.abs((feet - c) @ n) < DOWN
    down[2:] = True
    P = np.stack([(feet[down] - c) @ u, (feet[down] - c) @ v], 1)
    pc = np.array([(com - c) @ u, (com - c) @ v])
    h = hull(P)
    m = np.inf
    for i in range(len(h)):
        e = h[(i + 1) % len(h)] - h[i]
        nn = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        m = min(m, (h[i] - pc) @ nn)
    return m, down[:2]


if __name__ == "__main__":
    R = G._Robot()
    for f in sys.argv[2:]:
        z = np.load(f)
        q = np.stack([z[k] for k in D._keys()], 1).astype(float)[::10]
        feet, com = R.feet_and_com(q)
        res = [margin(feet[k], com[k]) for k in range(len(q))]
        ms = np.array([r[0] for r in res]) * 1000
        lifted = np.array([not r[1].all() for r in res])
        both = np.array([not r[1].any() for r in res])
        print(f"{Path(f).stem:10s} claws down: min {ms[~lifted].min():6.1f} | one or both lifted: "
              f"min {ms[lifted].min() if lifted.any() else np.nan:6.1f} p10 "
              f"{np.percentile(ms[lifted], 10) if lifted.any() else np.nan:6.1f} | both lifted rows {both.sum()}")
