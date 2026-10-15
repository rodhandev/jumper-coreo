"""Render a clip .npz kinematically: the robot posed at every row, no physics, no policy.

What the guion asks for, before any training: the base pose from the resting-floor
solver and the 22 joints, drawn with any MJCF (the Zymbot colours, for example).

    python ver_clip.py <jumper checkout> <clip.npz> <model.xml> <out.mp4> [--fps 30]
                       [--caras <dir de PNG> --guion <clip.json>] [--musica <pista>]
                       [--tam 1080x1920] [--camara 0.75,135,-18]

With `--caras` and a model that has a `cara` geom (recolorear.py --cara), the face
screen follows the guion's `cara` orders: each PNG in the directory is one
expression, named as the guion names it. With `--musica` (and `--guion`, for the
clip's `musica_desfase`) the track is laid under the video from the clip's first beat.
"""

import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

repo, clip, xml, dest = (Path(a) for a in sys.argv[1:5])
def arg(name):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else None


fps = int(arg("--fps") or 30)
caras = Path(arg("--caras")) if arg("--caras") else None
guion = json.loads(Path(arg("--guion")).read_text()) if arg("--guion") else {}
ordenes = guion.get("cara", []) if caras is not None else []
musica = arg("--musica")
desfase = guion.get("musica_desfase") or 0.0
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
def _material(name, file):
    spec.add_texture(name=name, type=mujoco.mjtTexture.mjTEXTURE_2D, file=str(file))
    mat = spec.add_material(name=name, emission=0.8, specular=0.3)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name


# Two kinds of face: the older flat `cara` panel, or the two round `ojo_der`/`ojo_izq` displays on the visor
# (recolorear.py --ojos), which take each face image cut in two (the viewer's left half is the robot's right eye).
ojos = {g.name for g in spec.geoms} >= {"ojo_der", "ojo_izq"}
if caras is not None:
    if ojos:
        sys.path.insert(0, str(Path(xml).resolve().parent))
        from recolorear import eye_crops  # noqa: E402
    for png in sorted(caras.glob("*.png")):
        if ojos:
            der, izq = eye_crops(png.resolve(), caras.resolve() / ".ojos")
            _material(f"ojo_der_{png.stem}", der)
            _material(f"ojo_izq_{png.stem}", izq)
        else:
            _material(f"cara_{png.stem}", png.resolve())
spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
model = spec.compile()
data = mujoco.MjData(model)
adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
free = model.jnt_qposadr[next(j for j in range(model.njnt) if model.jnt_type[j] == 0)]

W, H = (int(v) for v in (arg("--tam") or "1024x600").split("x"))
model.vis.global_.offwidth, model.vis.global_.offheight = W, H
r = mujoco.Renderer(model, H, W)
cam = mujoco.MjvCamera()
# --camara DIST,AZIMUT,ELEVACION (m, grados): 160 o 180 de azimut mira a la cara; 135 de tres cuartos.
cam.distance, cam.azimuth, cam.elevation = (float(v) for v in (arg("--camara") or "0.75,135,-18").split(","))
audio = ["-ss", f"{desfase:.3f}", "-i", musica, "-shortest", "-c:a", "aac"] if musica else []
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                       "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", *audio, "-pix_fmt", "yuv420p", str(dest)],
                      stdin=subprocess.PIPE)
gid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
mid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, n)  # noqa: E731
# (geom, material prefix) pairs the face orders drive.
pantallas = [(gid("ojo_der"), "ojo_der_"), (gid("ojo_izq"), "ojo_izq_")] if ojos else [(gid("cara"), "cara_")]
pantallas = [(g, p) for g, p in pantallas if g >= 0]
for _, e in ordenes:
    if any(mid(p + e) < 0 for _, p in pantallas):
        raise SystemExit(f"the guion asks for face «{e}» and {caras} has no {e}.png")


def face_at(t):
    now = [e for te, e in ordenes if te <= t + 1e-9]
    return now[-1] if now else None


# One frame every 1/fps s, each the row nearest its time: a fixed row stride (round(1 / (fps * dt))) is 2 for a
# 50 Hz clip at 30 fps, which played the clip 20 % fast and drifted it off the music.
rows = np.minimum(np.round(np.arange(0.0, len(q) * dt, 1.0 / fps) / dt).astype(int), len(q) - 1)
for k in rows:
    x, y, zz, roll, pitch, yaw = (float(z[f"body_{a}"][k]) for a in dances.AXES)
    data.qpos[free:free + 3] = (x, y, zz)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], "xyz")
    data.qpos[free + 3:free + 7] = quat
    data.qpos[adr] = q[k]
    mujoco.mj_kinematics(model, data)
    if (e := face_at(k * dt)) is not None:
        for g, p in pantallas:
            model.geom_matid[g] = mid(p + e)
    cam.lookat[:] = (x, y, 0.08)
    r.update_scene(data, cam)
    ff.stdin.write(r.render().tobytes())
ff.stdin.close()
ff.wait()
print(f"{dest}: {len(rows)} frames at {fps} fps")
