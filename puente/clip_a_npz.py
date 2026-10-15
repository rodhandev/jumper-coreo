"""coreo clip (.json) -> the .npz a Jumper imitation task trains on.

Reuses the repository's own gesture importer rather than re-deriving anything:
`tools/import_wbc_gestures.py`'s `_Robot` and `solve_resting_pose` put the floor
under whichever feet the posture rests on and solve the base pose from it, and the
output follows the same schema, float32 per-sample channels, commanded == meas_
(as the gesture importer does: the tracking reference is the table itself).

The guion starts and ends at HOME (coreo starts every joint there), so no lead-in
is added; HOLD_S of standing still is appended so the hand-back to walking happens
from a still stance, as the importer does.

    python clip_a_npz.py <jumper checkout> <clip.json> <dest.npz>
"""

import hashlib
import json
import sys
from pathlib import Path

import numpy as np

#: Tilt of the base the bridge accepts, degrees. Measured 14-oct on the repo's clips:
#: the crab dance reaches 16.4 (trained and deployed), the gestures 6.7-9.9. Past it
#: the guion is asking for a pose no clip a policy already tracks has gone near.
TILT_MAX = 17.0


def main(repo: Path, src: Path, dest: Path) -> None:
    sys.path[:0] = [str(repo / "tools"), str(repo)]
    import import_wbc_dances as dances
    import import_wbc_gestures as gestures

    from tasks.jumper.common.constants import HOME

    raw = src.read_bytes()
    clip = json.loads(raw)
    robot = gestures._Robot()
    col = [clip["joints"].index(n) for n in robot.names]
    q = np.asarray(clip["q"], dtype=np.float64)[:, col]
    dt = 1.0 / clip["hz"]

    home = np.array([HOME[n] for n in robot.names])
    for end, row in (("first", q[0]), ("last", q[-1])):
        off = float(np.abs(row - home).max())
        if off > 1e-6:
            raise SystemExit(f"{src}: the {end} row is {off:.3f} rad from HOME; "
                             "end the guion with every group back at reposo / de_pie")
    hold = np.repeat(home[None], round(gestures.HOLD_S / dt), axis=0)
    q = np.concatenate([q, hold])

    pose, height = gestures.solve_resting_pose(robot, q)
    lifted = {f: float((height[:, i] >= gestures.DOWN_TOL).mean())
              for i, f in enumerate(gestures.FEET)}

    # Self-collision over the whole clip (every 5th row, the robot posed in the air): crear_postura checks one claw
    # at a time, so two claws posed together -- `tenazas besito` -- can still meet only here.
    mj, m, d = robot.mujoco, robot.model, robot.data
    hits = {}
    for k in range(0, len(q), 5):
        d.qpos[:] = 0.0
        d.qpos[2], d.qpos[3] = 1.0, 1.0
        d.qpos[robot.q_adr] = q[k]
        mj.mj_forward(m, d)
        for c in d.contact[:d.ncon]:
            if c.dist < -0.001:
                pair = "/".join(sorted(mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "" for g in (c.geom1, c.geom2)))
                hits.setdefault(pair, k * dt)
    if hits:
        first = ", ".join(f"{p} at {t:.2f} s" for p, t in sorted(hits.items(), key=lambda x: x[1])[:4])
        raise SystemExit(f"{src}: the robot runs into itself: {first}")

    tilt = np.degrees(np.hypot(pose[:, 3], pose[:, 4]))
    bad = np.nonzero(tilt > TILT_MAX)[0]
    if len(bad):
        cuts = np.split(bad, np.nonzero(np.diff(bad) > 1)[0] + 1)
        spans = ", ".join(f"{c[0] * dt:.2f}-{c[-1] * dt:.2f} s (max {tilt[c].max():.0f} deg)"
                          for c in cuts)
        raise SystemExit(f"{src}: the base tilts past {TILT_MAX} deg at {spans}; "
                         "those postures do not go together")

    n = len(q)
    f32 = np.float32
    out = {"time": (np.arange(n) * dt).astype(f32), "dt": np.array(dt),
           "phase": np.ones(n, dtype=f32)}
    for prefix in ("", "meas_"):
        for i, a in enumerate(dances.AXES):
            out[f"{prefix}body_{a}"] = pose[:, i].astype(f32)
        for k, key in enumerate(dances._keys()):
            out[f"{prefix}{key}"] = q[:, k].astype(f32)
    if clip.get("pulso"):
        out["bpm"] = np.array(float(clip["pulso"]))
        out["beat_times_sim"] = np.asarray(clip["pulsos"], dtype=np.float64)
    out["source"] = np.array(f"coreo:{src.name}")
    out["source_sha256"] = np.array(hashlib.sha256(raw).hexdigest())
    out["hold_s"] = np.array(gestures.HOLD_S)

    dest.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(dest, **out)
    print(f"{src.name} -> {dest}: {n} rows = {n * dt:.2f} s")
    print(f"  base z {pose[:, 2].min() * 1000:.0f}..{pose[:, 2].max() * 1000:.0f} mm, "
          f"tilt at most {np.degrees(np.abs(pose[:, 3:5]).max()):.1f} deg, "
          f"travels {np.hypot(*np.ptp(pose[:, :2], axis=0)) * 1000:.0f} mm")
    print("  off the floor: " + ", ".join(f"{f} {v * 100:.0f}%" for f, v in lifted.items()))


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve(), Path(sys.argv[2]), Path(sys.argv[3]))
