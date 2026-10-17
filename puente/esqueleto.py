"""Preview a clip .npz as a flat silhouette, on the CPU: for checking a move before rendering it.

Rodhan (15-oct): «what if we only use the skeleton to see how it is turning out, so no time goes on renders that
then do not convince?». Each part of the robot is drawn as the outline (convex hull) of its mesh, far to near, so
the pose reads like the render, at a fraction of the cost and without the GPU (shared, one job at a time). The two
claws are coloured by SCREEN side as the camera sees them (orange on the right, blue on the left), to talk about
moves in screen quadrants; with --guion the current face, sound and phone screen are written on the frame, and with --musica the
track goes under it.

    python esqueleto.py <jumper checkout> <clip.npz> <model.xml> <out.mp4> [--fps 30] [--tam 540x960]
                        [--camara 0.80,168,-24] [--guion <clip.json>] [--musica <pista>]
"""

import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import ConvexHull

repo, clip, xml, dest = (Path(a) for a in sys.argv[1:5])


def arg(name):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else None


fps = int(arg("--fps") or 30)
W, H = (int(v) for v in (arg("--tam") or "540x960").split("x"))
dist0, azim0, elev0 = (float(v) for v in (arg("--camara") or "0.80,168,-24").split(","))
dist, azim, elev = dist0, azim0, elev0
guion = json.loads(Path(arg("--guion")).read_text()) if arg("--guion") else {}
musica = arg("--musica")
sys.path[:0] = [str(repo.resolve() / "tools"), str(repo.resolve())]
import import_wbc_dances as dances  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

z = np.load(clip)
dt = float(z["dt"])
names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)
model = mujoco.MjModel.from_xml_path(str(xml))
data = mujoco.MjData(model)
adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
free = model.jnt_qposadr[next(j for j in range(model.njnt) if model.jnt_type[j] == 0)]

# Each visible geom as a few hundred points in its own frame (a mesh's vertices, a box's corners).
FILL = {"base": (226, 230, 234), "dark": (60, 66, 76), "right": (255, 150, 60), "left": (70, 150, 255),
        "phone": (40, 44, 52), "screen": (40, 90, 130), "cradle": (245, 246, 248), "eye": (0, 195, 227)}
geoms = []
for g in range(model.ngeom):
    if model.geom_rgba[g][3] == 0 or model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
        continue
    t = model.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        m = model.geom_dataid[g]
        a, n = model.mesh_vertadr[m], model.mesh_vertnum[m]
        pts = model.mesh_vert[a:a + n]
        pts = pts[np.linspace(0, n - 1, min(n, 400)).astype(int)]
    elif t in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        s = model.geom_size[g]
        pts = np.array([[x, y, zz] for x in (-1, 1) for y in (-1, 1) for zz in (-1, 1)]) * s
    else:
        continue
    gname = model.geom(g).name or ""
    if gname == "telefono_camara":   # on the phone's back: an outline would show it through the phone
        continue
    bname = model.body(model.geom_bodyid[g]).name
    rgba = model.geom_rgba[g]
    if gname.startswith("ojo"):
        kind = "eye"
    elif "pantalla" in gname:
        kind = "screen"
    elif gname.startswith("telefono"):
        kind = "phone"
    elif gname.startswith("cuna"):
        kind = "cradle"
    elif bname[:2] in ("LF", "RF") and bname.split("_")[1] in ("palm", "finger", "forearm"):
        kind = bname[:2]   # resolved to a screen side per frame
    else:
        kind = "base" if rgba[:3].mean() > 0.5 else "dark"
    geoms.append((g, pts, kind))

def ejes(azim, elev):
    el, az = np.radians(elev), np.radians(azim)
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    return fwd, right, np.cross(right, fwd)


fwd, right, up = ejes(azim, elev)
f_px = (H / 2) / np.tan(np.radians(model.vis.global_.fovy) / 2)
try:
    FONT = ImageFont.truetype("/usr/share/fonts/opentype/inter/Inter-Bold.otf", max(14, H // 40))
except OSError:
    FONT = ImageFont.load_default()


def project(p, eye):
    v = p - eye
    d = v @ fwd
    return np.stack([W / 2 + f_px * (v @ right) / d, H / 2 - f_px * (v @ up) / d], axis=-1), d


def camara_en(t, base, ordenes, paso=0.6):
    """The camera at time t: the guion's `camara` orders ([t, "dist,azim,elev[,height]"]) each glide there over
    `paso` s (smoothstep) from where the camera was; before the first, `base`. Height is the look-at point's z (m)."""
    cur = list(base)
    for te, v in ordenes:
        if te > t:
            break
        goal = [float(x) for x in v.split(",")]
        goal += cur[len(goal):]
        u = min(max((t - te) / paso, 0.0), 1.0)
        u = u * u * (3 - 2 * u)
        cur = [a + (b - a) * u for a, b in zip(cur, goal)]
    return cur


def at(orders, t):
    now = [e for te, e in orders if te <= t + 1e-9]
    return now[-1] if now else None


audio = ["-ss", f"{guion.get('musica_desfase') or 0.0:.3f}", "-i", musica, "-shortest", "-c:a", "aac"] if musica else []
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                       "-r", str(fps), "-i", "-", *audio, "-pix_fmt", "yuv420p", str(dest)], stdin=subprocess.PIPE)
rows = np.minimum(np.round(np.arange(0.0, len(q) * dt, 1.0 / fps) / dt).astype(int), len(q) - 1)
for k in rows:
    x, y, zz, roll, pitch, yaw = (float(z[f"body_{a}"][k]) for a in dances.AXES)
    data.qpos[free:free + 3] = (x, y, zz)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], "xyz")
    data.qpos[free + 3:free + 7] = quat
    data.qpos[adr] = q[k]
    mujoco.mj_kinematics(model, data)
    dist, azim, elev, alto = camara_en(k * dt, [dist0, azim0, elev0, 0.08], guion.get("camara", []))
    fwd, right, up = ejes(azim, elev)
    look = np.array([x, y, alto])
    eye = look - dist * fwd
    im = Image.new("RGB", (W, H), (150, 156, 162))
    dr = ImageDraw.Draw(im)
    for gx in np.arange(-1.0, 1.01, 0.1):   # floor grid, 10 cm
        for a, b in (([gx, -1, 0], [gx, 1, 0]), ([-1, gx, 0], [1, gx, 0])):
            seg = np.linspace(a, b, 40) + [x, y, 0]
            p, d = project(seg, eye)
            p = p[d > 0.05]
            if len(p) > 1:
                dr.line([tuple(v) for v in p], fill=(138, 144, 150), width=1)
    # which front claw is on the screen's right this frame
    side = {}
    for leg in ("LF", "RF"):
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_shoulder_link")
        side[leg] = "right" if project(data.xpos[b][None], eye)[0][0, 0] > project(np.array([[x, y, zz]]), eye)[0][0, 0] else "left"
    polys = []
    for g, pts, kind in geoms:
        w = pts @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        p, d = project(w, eye)
        if (d < 0.05).any():
            continue
        try:
            hull = p[ConvexHull(p).vertices]
        except Exception:
            continue
        polys.append((d.mean(), hull, side.get(kind, kind)))
    for _, hull, kind in sorted(polys, key=lambda e: -e[0]):
        dr.polygon([tuple(v) for v in hull], fill=FILL[kind], outline=(20, 24, 30))
    t = k * dt
    cara, son = at(guion.get("cara", []), t), [n for ts, n in guion.get("sonidos", []) if 0 <= t - ts < 0.4]
    # at the bottom, out of the way of the video's captions at the top
    y0 = H - 12 - FONT.size * 5.4
    dr.text((12, y0), f"{t:5.2f} s", font=FONT, fill=(255, 255, 255))
    if cara:
        dr.text((12, y0 + FONT.size * 1.3), f"face: {cara}", font=FONT, fill=FILL["eye"])
    if son:
        dr.text((12, y0 + FONT.size * 2.6), f"♪ {son[-1]}", font=FONT, fill=(255, 220, 120))
    if tel := at(guion.get("telefono", []), t):
        dr.text((12, y0 + FONT.size * 3.9), f"phone: {tel}", font=FONT, fill=(150, 200, 255))
    dr.text((W - 12, H - 12 - FONT.size * 1.3), "esqueleto", font=FONT, fill=(230, 230, 230), anchor="ra")
    ff.stdin.write(im.tobytes())
ff.stdin.close()
ff.wait()
print(f"{dest}: {len(rows)} frames at {fps} fps")
