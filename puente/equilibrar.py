"""Automatic balance: move the body over the four legs while a claw is up.

The claws are the robot's front legs: at HOME the centre of mass sits 17 mm ahead of
the middle feet, so lifting both claws as a guion says tips it onto its face (51
degrees, measured 14-oct). This pass makes the guion not have to know that. Wherever
a claw is off the floor, the centre of mass has to lie MARGIN inside the polygon of
the four legs' feet; where it does not, the four feet are moved (IK on their three
joints, the base held) so that the body shifts towards the polygon's centre. The
shift is spread out in time -- widened by LEAD on both sides and smoothed -- so the
body is already there when the claw leaves the floor, and it is back to zero
wherever the claws are down, so a clip that starts and ends at HOME still does.

"Off the floor" is decided against the four legs' plane, not by the repository's
resting-floor solver: that solver rests a tipping robot on its claws and then reports
them down, which hid every lifted claw from this pass. A claw is up when its tip
stands more than LIFT above where it stands, over the same plane, at HOME. The
resting-floor solver then judges the result (the bridge's tilt check).

    python equilibrar.py <jumper checkout> <clip.json> <out.json>
"""

import json
import sys
from pathlib import Path

import numpy as np

#: Required distance of the COM inside the four-leg polygon, metres. 15 mm is about a
#: ninth of the polygon's 135 mm depth; a starting value, checked afterwards by the
#: resting-floor solver (tilt) rather than trusted.
MARGIN = 0.015
#: How far ahead (and behind) of a lifted claw the shift starts, seconds.
LEAD = 0.5
#: Smoothing of the shift, seconds (Gaussian sigma).
SMOOTH = 0.2
#: The IK has reached a target when every foot is within this of it, metres.
REACH = 0.001
#: A claw tip this far above its HOME height over the legs' plane is up, metres.
LIFT = 0.005
LEGS = ("LM", "RM", "LR", "RR")


def claw_heights(feet):
    """[T, 2] the claw tips' heights over the plane through the four legs' feet."""
    out = np.zeros((len(feet), 2))
    for k, f in enumerate(feet):
        legs = f[2:]
        c = legs.mean(axis=0)
        n = np.linalg.svd(legs - c)[2][2]
        n = n if n[2] > 0 else -n
        out[k] = (f[:2] - c) @ n
    return out


def hull(p):
    p = sorted(map(tuple, p))
    cr = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])  # noqa: E731
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


def margin(poly, c):
    """Signed distance of c inside the convex CCW polygon (+ inside)."""
    m = np.inf
    for i in range(len(poly)):
        e = poly[(i + 1) % len(poly)] - poly[i]
        n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        m = min(m, float((poly[i] - c) @ n))
    return m


def main(repo: Path, src: Path, dest: Path) -> None:
    sys.path[:0] = [str(repo / "tools"), str(repo)]
    import import_wbc_gestures as G

    clip = json.loads(src.read_text())
    R = G._Robot()
    mj, m, d = R.mujoco, R.model, R.data
    col = [clip["joints"].index(n) for n in R.names]
    q = np.asarray(clip["q"], dtype=np.float64)[:, col]
    dt = 1.0 / clip["hz"]

    from tasks.jumper.common.constants import HOME

    feet, com = R.feet_and_com(q)
    home_feet, _ = R.feet_and_com(np.array([[HOME[n] for n in R.names]]))
    lifted = (claw_heights(feet) > claw_heights(home_feet)[0] + LIFT).any(axis=1)  # FEET: LF, RF first

    # The shift each row needs, in the base frame's xy.
    need = np.zeros((len(q), 2))
    for k in np.nonzero(lifted)[0]:
        poly = hull(feet[k, 2:, :2])
        c = com[k, :2]
        mg = margin(poly, c)
        if mg < MARGIN:
            to_centre = poly.mean(axis=0) - c
            need[k] = (MARGIN - mg) * to_centre / np.linalg.norm(to_centre)

    # Widen (the largest shift within LEAD either side) and smooth.
    w = round(LEAD / dt)
    wide = np.zeros_like(need)
    for k in range(len(q)):
        seg = need[max(0, k - w):k + w + 1]
        wide[k] = seg[np.argmax(np.linalg.norm(seg, axis=1))]
    s = round(SMOOTH / dt)
    kern = np.exp(-0.5 * (np.arange(-3 * s, 3 * s + 1) / s) ** 2)
    kern /= kern.sum()
    shift = np.stack([np.convolve(np.pad(wide[:, i], 3 * s, mode="edge"), kern, "valid")
                      for i in range(2)], axis=1)
    # Zero at both ends, so a clip that starts and ends at HOME still does: a smoothstep
    # ramp over LEAD at each end (a claw that rises sooner than that gets less lead).
    t = np.arange(len(q)) * dt
    u = np.clip(np.minimum(t, t[-1] - t) / LEAD, 0.0, 1.0)
    shift *= (u * u * (3.0 - 2.0 * u))[:, None]

    # IK: each leg's foot to where it was minus the shift; the base stays at the origin.
    sites = [R.feet[G.FEET.index(leg)] for leg in LEGS]
    dofs = {leg: [i for i, n in enumerate(R.names) if n.startswith(leg + "_")] for leg in LEGS}
    cols = {leg: [m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, R.names[i])] for i in dofs[leg]]
            for leg in LEGS}
    jac = np.zeros((3, m.nv))

    def solve(k, sh):
        """Row k's joints with the feet moved by -sh, and the worst foot error (m)."""
        row = q[k].copy()
        target = [feet[k, 2 + i] - np.array([*sh, 0.0]) for i in range(len(LEGS))]
        err_all = 0.0
        for _ in range(30):
            d.qpos[:] = 0.0
            d.qpos[3] = 1.0
            d.qpos[R.q_adr] = row
            mj.mj_kinematics(m, d)
            err_all = 0.0
            for leg, site, tgt in zip(LEGS, sites, target):
                err = tgt - d.site_xpos[site]
                err_all = max(err_all, float(np.linalg.norm(err)))
                mj.mj_jacSite(m, d, jac, None, site)
                J = jac[:, cols[leg]]
                row[dofs[leg]] += J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(3), err)
            row = np.clip(row, R.range[:, 0], R.range[:, 1])
            if err_all < 1e-5:
                break
        return row, err_all

    # How much of each row's shift the legs can take (their joint ranges end somewhere):
    # the largest fraction the IK solves within REACH. Then the fraction is narrowed to
    # the smallest nearby and smoothed, so it cannot jump from row to row -- clamping the
    # IK in place moved a leg at 13 rad/s on the saludo (14-oct).
    frac = np.ones(len(q))
    for k in np.nonzero(np.linalg.norm(shift, axis=1) > 1e-5)[0]:
        for f in (1.0, 0.8, 0.6, 0.4, 0.2, 0.0):
            if solve(k, f * shift[k])[1] < REACH:
                frac[k] = f
                break
    low = np.array([frac[max(0, k - w):k + w + 1].min() for k in range(len(q))])
    frac = np.convolve(np.pad(low, 3 * s, mode="edge"), kern, "valid")
    applied = shift * frac[:, None]

    out = q.copy()
    worst = 0.0
    for k in np.nonzero(np.linalg.norm(applied, axis=1) > 1e-5)[0]:
        out[k], e = solve(k, applied[k])
        worst = max(worst, e)
    short = np.linalg.norm(shift - applied, axis=1).max()

    clip["q"] = out[:, [R.names.index(n) for n in clip["joints"]]].tolist()
    clip.setdefault("informe", {})["equilibrio"] = {
        "margen_mm": MARGIN * 1000,
        "filas_con_tenaza_arriba": int(lifted.sum()),
        "desplazamiento_max_mm": float(np.linalg.norm(applied, axis=1).max() * 1000),
        "falto_mm": float(short * 1000),
        "error_ik_max_mm": worst * 1000,
    }
    dest.write_text(json.dumps(clip))
    print(f"{src.name} -> {dest.name}: claw up in {lifted.sum()} of {len(q)} rows, "
          f"body shifted up to {np.linalg.norm(applied, axis=1).max() * 1000:.1f} mm "
          f"({short * 1000:.1f} mm short of what was asked, where the legs ran out), "
          f"IK error at most {worst * 1000:.2f} mm")


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve(), Path(sys.argv[2]), Path(sys.argv[3]))
