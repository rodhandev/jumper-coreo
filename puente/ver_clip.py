"""Render a clip .npz kinematically: the robot posed at every row, no physics, no policy.

What the guion asks for, before any training: the base pose from the resting-floor
solver and the 22 joints, drawn with any MJCF (the Zymbot colours, for example).

    python ver_clip.py <jumper checkout> <clip.npz> <model.xml> <out.mp4> [--fps 30]
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

repo, clip, xml, dest = (Path(a) for a in sys.argv[1:5])
fps = int(sys.argv[sys.argv.index("--fps") + 1]) if "--fps" in sys.argv else 30
sys.path[:0] = [str(repo.resolve() / "tools"), str(repo.resolve())]
import import_wbc_dances as dances  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

z = np.load(clip)
dt = float(z["dt"])
names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)

spec = mujoco.MjSpec.from_file(str(xml))
floor = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1])
floor.rgba = [0.86, 0.88, 0.9, 1]
spec.worldbody.add_light(pos=[0.5, -0.5, 1.5], dir=[-0.3, 0.3, -1], castshadow=True)
spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
model = spec.compile()
data = mujoco.MjData(model)
adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
free = model.jnt_qposadr[next(j for j in range(model.njnt) if model.jnt_type[j] == 0)]

W, H = 1024, 600
model.vis.global_.offwidth, model.vis.global_.offheight = W, H
r = mujoco.Renderer(model, H, W)
cam = mujoco.MjvCamera()
cam.distance, cam.azimuth, cam.elevation = 0.75, 135, -18
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                       "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", str(dest)],
                      stdin=subprocess.PIPE)
step = max(1, round(1.0 / (fps * dt)))
for k in range(0, len(q), step):
    x, y, zz, roll, pitch, yaw = (float(z[f"body_{a}"][k]) for a in dances.AXES)
    data.qpos[free:free + 3] = (x, y, zz)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], "xyz")
    data.qpos[free + 3:free + 7] = quat
    data.qpos[adr] = q[k]
    mujoco.mj_kinematics(model, data)
    cam.lookat[:] = (x, y, 0.08)
    r.update_scene(data, cam)
    ff.stdin.write(r.render().tobytes())
ff.stdin.close()
ff.wait()
print(f"{dest}: {len(range(0, len(q), step))} frames at {fps} fps")
