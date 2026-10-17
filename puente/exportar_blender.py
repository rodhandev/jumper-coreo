"""Hand a clip to Blender: the robot's visible parts as meshes, where each one is every frame, and the pictures its
two eyes and the phone's screen show, frame by frame. `blender/escena.py` builds the scene from this.

Rodhan (16-oct): the published video wants a real set (a desk), and Blender with free assets gives it the quality
2D backgrounds would take away. MuJoCo stays the source of the motion -- the trained policy's recording or a
kinematic clip --; Blender only lights and films it.

    python exportar_blender.py <jumper checkout> <clip.npz> <model.xml> <out dir> [--guion <clip.json>]
                               [--caras <dir de PNG>] [--fps 30] [--desde s] [--hasta s]
                               [--camara DIST,AZIM,ELEV[,HEIGHT[,DX,DY]]]

Writes into <out dir>:
    mallas/<geom>.obj      each visible geom (group 2) once, in its own frame, metres (texture coordinates for
                           the phone's screen); boxes (the eyes) as their size
    escena.json            per geom: mesh, colour, which picture it shows; fps, frames
    poses.npy              [frame, geom, 7]: position (3) and quaternion w x y z (4), world frame
    camara.npy             [frame, 7]: the camera's eye (3), look-at point (3) and distance, from --camara and the
                           guion's `camara` orders (puente/camara.py), as ver_clip.py moves it
    ojo_der/, ojo_izq/, pantalla/   one PNG per frame (only when the guion asks for faces / phone states)
"""

import json
import shutil
import sys
from pathlib import Path

import mujoco
import numpy as np

repo, clip, xml, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4])
arg = lambda n, d=None: sys.argv[sys.argv.index(n) + 1] if n in sys.argv else d  # noqa: E731
fps = int(arg("--fps", 30))
guion = json.loads(Path(arg("--guion")).read_text()) if arg("--guion") else {}
caras = Path(arg("--caras")) if arg("--caras") else None
sys.path[:0] = [str(repo / "tools"), str(repo)]
import import_wbc_dances as dances  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

z = np.load(clip)
dt = float(z["dt"])
names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)
m = mujoco.MjModel.from_xml_path(str(xml))
d = mujoco.MjData(m)
free = m.jnt_qposadr[next(j for j in range(m.njnt) if m.jnt_type[j] == 0)]
adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
(out / "mallas").mkdir(parents=True, exist_ok=True)

# ── the meshes, once ─────────────────────────────────────────────────────────────────────────────────────────
geoms = []
for g in range(m.ngeom):
    if m.geom_group[g] != 2 or m.geom_rgba[g][3] == 0:
        continue
    name = m.geom(g).name or f"geom{g}"
    entry = {"geom": name, "rgba": [round(float(v), 4) for v in m.geom_rgba[g]]}
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        k = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[k], m.mesh_vertnum[k]
        fa, fn = m.mesh_faceadr[k], m.mesh_facenum[k]
        v = m.mesh_vert[va:va + vn]
        f = m.mesh_face[fa:fa + fn] + 1
        lines = [f"v {a:.6f} {b:.6f} {c:.6f}" for a, b, c in v]
        ta, tn = m.mesh_texcoordadr[k], m.mesh_texcoordnum[k]
        if tn > 0:   # MuJoCo keeps texture coordinates per face corner (mesh_facetexcoord), v flipped on load
            lines += [f"vt {a:.6f} {1 - b:.6f}" for a, b in m.mesh_texcoord[ta:ta + tn]]
            ft = m.mesh_facetexcoord[fa:fa + fn] + 1
            lines += [f"f {a}/{x} {b}/{y} {c}/{w}" for (a, b, c), (x, y, w) in zip(f, ft)]
        else:
            lines += [f"f {a} {b} {c}" for a, b, c in f]
        (out / "mallas" / f"{name}.obj").write_text("\n".join(lines) + "\n")
        entry["malla"] = f"mallas/{name}.obj"
    elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
        entry["caja"] = [float(v) for v in m.geom_size[g]]   # half sizes
    else:
        continue
    if name in ("ojo_der", "ojo_izq", "telefono_pantalla"):
        entry["imagen"] = {"telefono_pantalla": "pantalla"}.get(name, name)
    geoms.append((g, entry))

# ── where every geom is, every frame ─────────────────────────────────────────────────────────────────────────
t0 = float(arg("--desde", 0.0))
t1 = min(float(arg("--hasta", len(q) * dt)), len(q) * dt)
times = np.arange(t0, t1, 1.0 / fps)
rows = np.minimum(np.round(times / dt).astype(int), len(q) - 1)
poses = np.zeros((len(rows), len(geoms), 7), np.float32)
for i, k in enumerate(rows):
    x, y, zz, roll, pitch, yaw = (float(z[f"body_{a}"][k]) for a in dances.AXES)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], "xyz")
    d.qpos[free:free + 7] = (x, y, zz, *quat)
    d.qpos[adr] = q[k]
    mujoco.mj_kinematics(m, d)
    for j, (g, _) in enumerate(geoms):
        qg = np.zeros(4)
        mujoco.mju_mat2Quat(qg, d.geom_xmat[g])
        poses[i, j, :3] = d.geom_xpos[g]
        poses[i, j, 3:] = qg
np.save(out / "poses.npy", poses)
sys.path.insert(0, str(Path(__file__).resolve().parent))
from camara import base as camara_base, camara_en  # noqa: E402

BASE = camara_base(arg("--camara") or "1.00,176,-32")
cams = np.zeros((len(rows), 7), np.float32)
for i, (t, k) in enumerate(zip(times, rows)):
    dist, azim, elev, alto, dx, dy = camara_en(t, BASE, guion.get("camara", []))
    look = np.array([z["body_x"][k] + dx, z["body_y"][k] + dy, alto])
    el, az = np.radians(elev), np.radians(azim)
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    cams[i] = [*(look - dist * fwd), *look, dist]
np.save(out / "camara.npy", cams)


def at(orders, t):
    now = [(te, e) for te, e in orders if te <= t + 1e-9]
    return now[-1] if now else (None, None)


# ── the pictures, frame by frame ─────────────────────────────────────────────────────────────────────────────
imagenes = {}
if caras is not None and guion.get("cara"):
    sys.path.insert(0, str(Path(xml).resolve().parent))
    from recolorear import eye_crops  # noqa: E402

    for lado in ("ojo_der", "ojo_izq"):
        shutil.rmtree(out / lado, ignore_errors=True)
        (out / lado).mkdir()
    cache = {}
    for i, t in enumerate(times):
        _, e = at(guion["cara"], t)
        e = e or guion["cara"][0][1]
        if e not in cache:
            cache[e] = eye_crops((caras / f"{e}.png").resolve(), caras.resolve() / ".ojos")
        der, izq = cache[e]
        shutil.copyfile(der, out / "ojo_der" / f"{i + 1:04d}.png")
        shutil.copyfile(izq, out / "ojo_izq" / f"{i + 1:04d}.png")
    imagenes.update({"ojo_der": "ojo_der", "ojo_izq": "ojo_izq"})
if guion.get("telefono"):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "modulos" / "cuna"))
    from pantallas import imagen  # noqa: E402

    shutil.rmtree(out / "pantalla", ignore_errors=True)
    (out / "pantalla").mkdir()
    for i, t in enumerate(times):
        te, e = at(guion["telefono"], t)
        imagen(e or "reposo", t - (te or 0.0)).save(out / "pantalla" / f"{i + 1:04d}.png")
    imagenes["pantalla"] = "pantalla"

(out / "escena.json").write_text(json.dumps({
    "fps": fps, "frames": len(rows), "desde": t0, "clip": str(clip.resolve()),
    "geoms": [e for _, e in geoms], "imagenes": imagenes,
    "musica_desfase": guion.get("musica_desfase"),
}, indent=1))
print(f"{out}: {len(geoms)} parts, {len(rows)} frames at {fps} fps, pictures: {', '.join(imagenes) or 'none'}")
