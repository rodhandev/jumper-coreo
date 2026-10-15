"""Record what a trained policy did as a clip .npz, so ver_clip.py can draw it with faces, music and any camera.

play.py records a video with one fixed look; the eyes' expressions follow the guion's `cara` orders, which only
ver_clip.py plays. This runs play.py unchanged and, each time its recorder writes a frame, keeps the pose it
drew (the base and the 22 joints): the physics, not the reference.

    python grabar_politica.py <jumper checkout> <out.npz> -- <play.py arguments, --video included>

The `--video` play.py writes is a by-product (keep it small: --video-width 320 --video-height 240). The .npz has
ver_clip.py's keys at the video's frame rate: `dt`, `body_x` ... `body_yaw` and the joint channels.
"""

import runpy
import sys
from pathlib import Path

import mujoco
import numpy as np

sep = sys.argv.index("--")
repo, dest = Path(sys.argv[1]).resolve(), Path(sys.argv[2])
play_args = sys.argv[sep + 1:]
sys.path[:0] = [str(repo / "scripts"), str(repo / "tools"), str(repo), str(repo / "rl")]
import import_wbc_dances as dances  # noqa: E402
from mjrl.viewer import video as replay_video  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
frames: list[np.ndarray] = []
layout: dict = {}
fps = float(play_args[play_args.index("--video-fps") + 1]) if "--video-fps" in play_args else 30.0

_write = replay_video.ReplayVideo._write_frames


def _keep(self, count):
    _write(self, count)
    model, data = self._renderer._model, self._renderer._data
    if not layout:
        # the render model is the scene: the robot's joints may carry a prefix ("robot/...")
        by_name = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): j for j in range(model.njnt)}
        def find(n):
            return next(j for k, j in by_name.items() if k == n or k.endswith("/" + n))
        layout["adr"] = [model.jnt_qposadr[find(n)] for n in names]
        layout["free"] = model.jnt_qposadr[next(j for j in range(model.njnt) if model.jnt_type[j] == 0)]
    for _ in range(count):
        frames.append(data.qpos.copy())


replay_video.ReplayVideo._write_frames = _keep
sys.argv = [str(repo / "scripts" / "play.py"), *play_args]
try:
    runpy.run_path(sys.argv[0], run_name="__main__")
except SystemExit as e:
    if e.code not in (None, 0):
        raise
q = np.stack(frames)
f = layout["free"]
out = {"dt": np.float64(1.0 / fps)}
for a, v in zip(("x", "y", "z"), q[:, f:f + 3].T):
    out[f"body_{a}"] = v
# ver_clip.py rebuilds the quaternion with mju_euler2Quat(..., "xyz"): MuJoCo's lowercase sequence is intrinsic,
# which scipy spells "XYZ".
from scipy.spatial.transform import Rotation  # noqa: E402

rpy = Rotation.from_quat(q[:, f + 3:f + 7][:, [1, 2, 3, 0]]).as_euler("XYZ")
for a, v in zip(("roll", "pitch", "yaw"), rpy.T):
    out[f"body_{a}"] = v
for k, adr in zip(dances._keys(), layout["adr"]):
    out[k] = q[:, adr]
np.savez(dest, **out)
assert list(dances.AXES) == ["x", "y", "z", "roll", "pitch", "yaw"]
# the round trip ver_clip.py will make, checked on the first frame
chk = np.zeros(4)
mujoco.mju_euler2Quat(chk, rpy[0], "xyz")
assert min(np.abs(chk - q[0, f + 3:f + 7]).max(), np.abs(chk + q[0, f + 3:f + 7]).max()) < 1e-6, "euler round trip"
print(f"{dest}: {len(q)} frames at {fps:g} fps ({len(q) / fps:.1f} s)")
