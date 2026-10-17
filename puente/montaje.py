"""Put clips on a timeline, for one robot or two: what each robot does when, as one scene.

Rodhan (17-oct): the video for social media has Jumper dancing and jumping, a second Jumper (black, pink eyes)
that plays a duet with it, and a battle. Each move is a clip already: a trained policy's recording, a coreo clip,
or a jump simulated by saltos.py. This lays them end to end per robot -- each clip carried from where the robot
stands when it starts, so a jump that moves or turns it leaves it there for the next -- and the robots side by
side in one MuJoCo model (the second attached with its names prefixed), frame by frame.

    python montaje.py <jumper checkout> <plan.json> <out dir>

plan.json:
    {"fps": 30, "duracion": 55,
     "robots": [{"prefijo": "",    "modelo": "a.xml", "caras": "caras/",      "en": [x, y, yaw_deg]},
                {"prefijo": "r2_", "modelo": "b.xml", "caras": "caras-rosa/", "en": [x, y, yaw_deg]}],
     "tramos": [{"robot": "", "clip": "x.npz", "t": 0.0, "desde": 0.0, "hasta": 3.0}, ...],
     "cara": {"": [[t, "sonrisa"], ...], "r2_": [...]},
     "telefono": [[t, "musica"], ...], "camara": [[t, "dist,azim,elev,height,dx,dy"], ...],
     "centro": "" | "r2_" | "medio",           # whom the camera's look-at point follows
     "sonidos": [[t, "celebracion"], ...], "rotulos": [[texto, a, b], ...], "musica": [[t, pista, desde_s]]}

The first robot's model is the scene (the desk phone comes with it, montar.py --version mesa); the others join it.
Writes <out>/modelo.xml, <out>/montaje.npz (qpos per frame, dt, robots' base names) and <out>/guion.json
(faces per robot, phone, camera, sounds, captions, music), which esqueleto.py and exportar_blender.py read as they
read a clip and its guion.
"""

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

repo, plan_f, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]), Path(sys.argv[3])
plan = json.loads(plan_f.read_text())
base_dir = plan_f.resolve().parent
ruta = lambda p: (base_dir / p).resolve() if not Path(p).is_absolute() else Path(p)  # noqa: E731
sys.path[:0] = [str(repo / "tools"), str(repo)]
import import_wbc_dances as dances  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

out.mkdir(parents=True, exist_ok=True)
fps = int(plan.get("fps", 30))
robots = plan["robots"]

# ── the scene: the first robot's model, the others attached ─────────────────────────────────────────────────────
spec = mujoco.MjSpec.from_file(str(ruta(robots[0]["modelo"])))
for r in robots[1:]:
    otro = mujoco.MjSpec.from_file(str(ruta(r["modelo"])))
    spec.worldbody.add_frame().attach_body(otro.body("base_link"), r["prefijo"], "")
m = spec.compile()
(out / "modelo.xml").write_text(spec.to_xml())
names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]


def adr(prefijo):
    free = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, prefijo + "floating_base")]
    return free, [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, prefijo + n)] for n in names]


def leer(clip):
    """A clip as (times, base x y z roll pitch yaw, joints) at its own rate."""
    z = np.load(clip)
    q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)
    b = np.stack([z[f"body_{a}"] for a in dances.AXES], axis=1).astype(np.float64)
    return float(z["dt"]), b, q


T = float(plan["duracion"])
times = np.arange(0.0, T, 1.0 / fps)
qpos = np.tile(m.qpos0, (len(times), 1))
from tasks.jumper.common.constants import HOME, STAND_Z  # noqa: E402

home = np.array([HOME[n] for n in names])
def carry(b, k, b0, pose):
    """Clip row k's base, carried from the clip's start b0 onto the robot's pose (x, y, heading)."""
    px, py, pyaw = pose
    dyaw = pyaw - b0[5]
    c, s_ = math.cos(dyaw), math.sin(dyaw)
    dx, dy = b[k, 0] - b0[0], b[k, 1] - b0[1]
    return np.array([px + c * dx - s_ * dy, py + s_ * dx + c * dy, b[k, 2], b[k, 3], b[k, 4], b[k, 5] + dyaw])


for r in robots:
    p = r["prefijo"]
    free, ja = adr(p)
    x0, y0, yaw0 = r.get("en", [0, 0, 0])
    pose = [x0, y0, math.radians(yaw0)]                      # where it stands: x, y, heading
    estado = (np.array([x0, y0, STAND_Z, 0, 0, pose[2]]), home.copy())
    filas = [None] * len(times)
    for seg in sorted((t for t in plan["tramos"] if t["robot"] == p), key=lambda t: t["t"]):
        dt_c, b, q = leer(ruta(seg["clip"]))
        a = seg.get("desde", 0.0)
        z = min(seg.get("hasta", len(q) * dt_c), len(q) * dt_c)
        k0 = min(int(round(a / dt_c)), len(q) - 1)
        b0 = b[k0].copy()
        for i, t in enumerate(times):
            if seg["t"] <= t < seg["t"] + (z - a):
                k = min(int(round((a + t - seg["t"]) / dt_c)), len(q) - 1)
                filas[i] = (carry(b, k, b0, pose), q[k].copy())
        kz = max(min(int(round(z / dt_c)) - 1, len(q) - 1), k0)
        fin = carry(b, kz, b0, pose)
        pose = [fin[0], fin[1], fin[5]]                      # the next clip starts where this one ended
    for i in range(len(times)):
        if filas[i] is not None:
            estado = filas[i]
        base, joints = estado                                # between clips the robot holds its last frame
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, base[3:].tolist(), "xyz")
        qpos[i, free:free + 3] = base[:3]
        qpos[i, free + 3:free + 7] = quat
        qpos[i, ja] = joints

np.savez(out / "montaje.npz", qpos=qpos, dt=np.array(1.0 / fps),
         prefijos=np.array([r["prefijo"] for r in robots]))
guion = {k: plan.get(k) for k in ("telefono", "camara", "sonidos", "rotulos", "musica", "centro") if plan.get(k)}
guion["caras"] = plan.get("cara", {})
guion["caras_dir"] = {r["prefijo"]: str(ruta(r["caras"])) for r in robots if r.get("caras")}
(out / "guion.json").write_text(json.dumps(guion, indent=1, ensure_ascii=False))
print(f"{out}: {len(robots)} robot(s), {len(times)} frames at {fps} fps, {T:.1f} s")
