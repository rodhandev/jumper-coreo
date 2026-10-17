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
    rayos.npy              [frame, shot, 8]: the app's virtual shots (guion `disparos`), start, end, visible, impact
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
m = mujoco.MjModel.from_xml_path(str(xml))
d = mujoco.MjData(m)
# a clip (one robot) or a montage (montaje.py: qpos per frame, its own model, robots by name prefix)
MONTAJE = "qpos" in z.files
if MONTAJE:
    qp = z["qpos"]
    n_rows = len(qp)
    frees = {p: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, p + "floating_base")]
             for p in z["prefijos"]}
else:
    q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)
    n_rows = len(q)
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
    if name.endswith(("ojo_der", "ojo_izq")) or name == "telefono_pantalla":
        entry["imagen"] = {"telefono_pantalla": "pantalla"}.get(name, name)
    geoms.append((g, entry))

# ── where every geom is, every frame ─────────────────────────────────────────────────────────────────────────
t0 = float(arg("--desde", 0.0))
t1 = min(float(arg("--hasta", n_rows * dt)), n_rows * dt)
times = np.arange(t0, t1, 1.0 / fps)
rows = np.minimum(np.round(times / dt).astype(int), n_rows - 1)
poses = np.zeros((len(rows), len(geoms), 7), np.float32)
centros = np.zeros((len(rows), 2))
for i, k in enumerate(rows):
    if MONTAJE:
        d.qpos[:] = qp[k]
        c = guion.get("centro", "")
        centros[i] = np.mean([qp[k, f:f + 2] for p, f in frees.items() if c in ("medio", p)], axis=0)
    else:
        x, y, zz, roll, pitch, yaw = (float(z[f"body_{a}"][k]) for a in dances.AXES)
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], "xyz")
        d.qpos[free:free + 7] = (x, y, zz, *quat)
        d.qpos[adr] = q[k]
        centros[i] = (x, y)
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
    look = np.array([centros[i, 0] + dx, centros[i, 1] + dy, alto])
    el, az = np.radians(elev), np.radians(azim)
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    cams[i] = [*(look - dist * fwd), *look, dist]
np.save(out / "camara.npy", cams)

# ── the app's virtual shots (montage guion `disparos`: [t, shooter prefix, target prefix, hit]) ─────────────────
# a beam from the shooter's aiming claw to the target's body, 0.12 s in flight and 0.10 s held; a miss flies past
VUELO, QUEDA = 0.12, 0.10
disparos = guion.get("disparos", [])
NB = max(1, len(disparos))                            # one beam per shot
rayos = np.zeros((len(rows), NB, 8), np.float32)     # x0 y0 z0 x1 y1 z1 visible impact
if disparos:
    for i, k in enumerate(rows):
        t = times[i]
        if MONTAJE:
            d.qpos[:] = qp[k]
            mujoco.mj_kinematics(m, d)
        for j, (ts, quien, blanco, acierta) in enumerate(disparos[:]):
            u = t - ts
            if not 0 <= u <= VUELO + QUEDA:
                continue
            slot = j
            a = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, quien + "LF_finger_tip_link")].copy()
            bz = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, blanco + "base_link")] + [0, 0, 0.03]
            if not acierta:   # past the target, 15 cm to its side
                lado = np.cross(bz - a, [0, 0, 1])
                bz = bz + 0.15 * lado / (np.linalg.norm(lado) + 1e-9) + 0.5 * (bz - a)
            f = min(u / VUELO, 1.0)
            rayos[i, slot] = [*a, *(a + (bz - a) * f), 1.0, float(acierta and u >= VUELO)]
np.save(out / "rayos.npy", rayos)


def at(orders, t):
    now = [(te, e) for te, e in orders if te <= t + 1e-9]
    return now[-1] if now else (None, None)


# ── the pictures, frame by frame ─────────────────────────────────────────────────────────────────────────────
imagenes = {}
# faces per robot: a clip's guion has `cara` (and --caras); a montage's has `caras` and `caras_dir` by prefix
por_robot = guion.get("caras") or ({"": guion["cara"]} if guion.get("cara") else {})
dirs = {p: Path(v) for p, v in guion.get("caras_dir", {}).items()}
if caras is not None:
    dirs.setdefault("", caras)
for p, ordenes in por_robot.items():
    if not ordenes or p not in dirs:
        continue
    sys.path.insert(0, str(dirs[p].resolve().parent))   # recolorear.py lives beside the faces' folder
    from recolorear import eye_crops  # noqa: E402

    for lado in ("ojo_der", "ojo_izq"):
        shutil.rmtree(out / (p + lado), ignore_errors=True)
        (out / (p + lado)).mkdir()
    cache = {}
    for i, t in enumerate(times):
        _, e = at(ordenes, t)
        e = e or ordenes[0][1]
        if e not in cache:
            cache[e] = eye_crops((dirs[p] / f"{e}.png").resolve(), dirs[p].resolve() / ".ojos")
        der, izq = cache[e]
        shutil.copyfile(der, out / (p + "ojo_der") / f"{i + 1:04d}.png")
        shutil.copyfile(izq, out / (p + "ojo_izq") / f"{i + 1:04d}.png")
    imagenes.update({p + "ojo_der": p + "ojo_der", p + "ojo_izq": p + "ojo_izq"})
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
    "disparos": disparos,
}, indent=1))
print(f"{out}: {len(geoms)} parts, {len(rows)} frames at {fps} fps, pictures: {', '.join(imagenes) or 'none'}")
