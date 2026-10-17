"""Put a phone holder and its phone on a Jumper MJCF, with their masses, for training and for video.

    python montar.py <in.xml> <out.xml> [--version vertical|plana] [--sin-telefono]
    python montar.py <in.xml> <out.xml> --version mesa --telefono-mesa X,Y,GIRO[,INCLINACION]

The STLs (construir.sh) are already in base_link's frame, the phone placed in its holder, so every geom goes in a
body `cuna_link` at base_link's origin, colliding with the legs and claws: a dance whose claw would hit the phone
has to learn not to. A body of its own because base_link carries an explicit <inertial>, which makes MuJoCo ignore
the masses of geoms added to it; welded, it moves with the shell and MuJoCo skips contacts between it and its parent.

Masses (15-oct): the holder as PLA (1.24 g/cm^3) at 0.85 for walls of 2.5-3.5 mm, which a slicer prints almost
solid -- it is a showpiece, but it is made to be printable and to weigh what it would. The phone: 172 g, today's
common 6.1" phone (147.6 x 71.6 x 7.8 mm), all in its body; the screen, the camera pill and the camera block are
for the eye only. MuJoCo collides a mesh as its convex hull: conservative, nothing should be inside it anyway.

The screen gets texture coordinates (next to <out.xml>: <out>-pantalla.obj, .png) so it can show something:
ver_clip.py draws what the guion's `telefono <estado>` orders ask for (pantallas.py) into texture `pantalla`, every
frame. Its «up» is towards the camera pill, whatever way the holder lays the phone.
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

HERE = Path(__file__).resolve().parent
PHONE_KG = 0.172
PLA = 1.24 * 0.85   # g/cm^3
#: part -> (rgba, collides): titanium frame, dark-blue lock screen, black pill and camera
PARTES = {"cuerpo": ("0.62 0.64 0.67 1", True), "pantalla": ("0.10 0.20 0.33 1", False),
          "isla": ("0.02 0.02 0.03 1", False), "camara": ("0.10 0.11 0.13 1", False)}


def pantalla_uv(stl, dst: Path) -> Path:
    """The screen mesh with planar texture coordinates: u across (left to right as you face it), v along (top = 1,
    the camera pill's end), and the lock screen as its first picture."""
    import numpy as np
    import trimesh

    from pantallas import imagen

    scr, body, pill = (trimesh.load(stl(p)) for p in ("pantalla", "cuerpo", "isla"))
    c = scr.bounds.mean(axis=0)
    n = c - body.bounds.mean(axis=0)
    n /= np.linalg.norm(n)
    up = pill.bounds.mean(axis=0) - c
    up -= (up @ n) * n
    up /= np.linalg.norm(up)
    right = np.cross(up, n)
    a, b = (scr.vertices - c) @ right, (scr.vertices - c) @ up
    uv = np.stack([(a - a.min()) / np.ptp(a), (b - b.min()) / np.ptp(b)], axis=1)
    obj = dst.with_name(f"{dst.stem}-pantalla.obj").resolve()
    obj.write_text("".join(f"v {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in scr.vertices)
                   + "".join(f"vt {u:.5f} {v:.5f}\n" for u, v in uv)
                   + "".join(f"f {i + 1}/{i + 1} {j + 1}/{j + 1} {k + 1}/{k + 1}\n" for i, j, k in scr.faces))
    imagen("reposo", 0.0).save(dst.with_name(f"{dst.stem}-pantalla.png"))
    return obj


def mesa(src: Path, dst: Path, donde: str) -> None:
    """The phone on the table beside the robot (Rodhan, 17-oct), as part of the set: a fixed body in the world, so
    ver_clip, esqueleto, factible and exportar_blender all see the same phone. X, Y in metres (world), GIRO and
    INCLINACION in degrees, as blender/escena.py takes them: turned INCLINACION about its own x (top end up), then
    GIRO about z, its bottom end on the table. Leaning, it gets a plain desk stand under its back. It collides: a
    dance that would run into it shows in factible.py and, trained with it, the policy learns to keep clear."""
    import math

    import numpy as np
    import trimesh

    tx, ty, giro, *inc = (float(v) for v in donde.split(","))
    inc = inc[0] if inc else 0.0
    text = src.read_text()
    if 'meshdir="./"' in text:
        text = text.replace('meshdir="./"', f'meshdir="{src.parent.resolve()}/"', 1)
    stl = lambda p: (HERE / "stl" / f"suelto-{p}.stl").resolve()  # noqa: E731
    files = {p: stl(p) for p in PARTES}
    files["pantalla"] = pantalla_uv(stl, dst)
    assets = "".join(f'<mesh name="mesa_{p}" file="{f}" scale="0.001 0.001 0.001"/>' for p, f in files.items())
    png = dst.with_name(f"{dst.stem}-pantalla.png").resolve()
    assets += (f'<texture name="pantalla" type="2d" file="{png}"/>'
               '<material name="pantalla" texture="pantalla" emission="0.85" specular="0.4" shininess="0.8"/>')
    geoms = ""
    for p, (rgba, collides) in PARTES.items():
        look = 'material="pantalla"' if p == "pantalla" else f'rgba="{rgba}"'
        extra = 'contype="1" conaffinity="1"' if collides else 'contype="0" conaffinity="0"'
        geoms += f'<geom name="telefono_{p}" type="mesh" mesh="mesa_{p}" {look} {extra} group="2"/>'
    if inc > 0:   # the stand: a wedge from the table to the phone's back, in the phone's frame (mm)
        # built where it stands (phone's bottom end on the table), then brought into the phone's frame
        a = math.radians(inc)
        R = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        z0 = 2.7 + 73.8 * math.sin(a)
        back = [R @ np.array([x, y, 0.0]) + [0, 0, z0] for x in (-30, 30) for y in (-72, 30)]
        drop = [np.array([b[0], b[1], 0.0]) for b in back[1::2]]
        world = np.array(back + drop)                       # 4 on the back, 2 on the table under the high edge
        local = (R.T @ (world - [0, 0, z0]).T).T
        cuña = trimesh.convex.convex_hull(local)
        f = dst.with_name(f"{dst.stem}-soporte.stl").resolve()
        cuña.export(f)
        assets += f'<mesh name="mesa_soporte" file="{f}" scale="0.001 0.001 0.001"/>'
        geoms += ('<geom name="soporte_mesa" type="mesh" mesh="mesa_soporte" rgba="0.85 0.86 0.87 1" '
                  'contype="1" conaffinity="1" group="2"/>')
    text = text.replace("<asset>", "<asset>" + assets, 1)
    g, a = math.radians(giro), math.radians(inc)
    Rz = np.array([[math.cos(g), -math.sin(g), 0], [math.sin(g), math.cos(g), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    import mujoco

    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, (Rz @ Rx).reshape(-1))
    pos = (tx, ty, 0.0027 + 0.0738 * math.sin(a))
    body = (f'<body name="telefono_mesa" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" '
            f'quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}">{geoms}</body>')
    anchor = re.search(r"<worldbody>", text)
    text = text[:anchor.end()] + body + text[anchor.end():]
    dst.write_text(text)
    print(f"wrote {dst}: phone on the table at {donde}" + (" on a stand" if inc > 0 else ""))


def main(src: Path, dst: Path, version: str, phone: bool) -> None:
    import trimesh

    text = src.read_text()
    if 'meshdir="./"' in text:
        text = text.replace('meshdir="./"', f'meshdir="{src.parent.resolve()}/"', 1)
    stl = lambda p: (HERE / "stl" / f"{version}-{p}.stl").resolve()  # noqa: E731
    if not stl("cuna").exists():
        raise SystemExit(f"{stl('cuna')} missing: run construir.sh first")
    partes = ["cuna"] + (list(PARTES) if phone else [])
    files = {p: stl(p) for p in partes}
    if phone:
        files["pantalla"] = pantalla_uv(stl, dst)
    assets = "".join(f'<mesh name="{version}_{p}" file="{files[p]}" scale="0.001 0.001 0.001"/>' for p in partes)
    if phone:
        png = dst.with_name(f"{dst.stem}-pantalla.png").resolve()
        assets += (f'<texture name="pantalla" type="2d" file="{png}"/>'
                   '<material name="pantalla" texture="pantalla" emission="0.85" specular="0.4" shininess="0.8"/>')
    text = text.replace("<asset>", "<asset>" + assets, 1)
    kg = trimesh.load(stl("cuna")).volume / 1000 * PLA / 1000
    geoms = (f'<geom name="cuna" type="mesh" mesh="{version}_cuna" mass="{kg:.4f}" rgba="0.95 0.96 0.97 1" '
             'contype="1" conaffinity="1" group="2"/>')
    for p in partes[1:]:
        rgba, collides = PARTES[p]
        extra = f'mass="{PHONE_KG}" contype="1" conaffinity="1"' if collides else 'contype="0" conaffinity="0" density="0"'
        look = 'material="pantalla"' if p == "pantalla" else f'rgba="{rgba}"'
        geoms += f'<geom name="telefono_{p}" type="mesh" mesh="{version}_{p}" {look} {extra} group="2"/>'
    anchor = re.search(r'<body name="base_link"[^>]*>', text)
    text = text[:anchor.end()] + f'<body name="cuna_link">{geoms}</body>' + text[anchor.end():]
    dst.write_text(text)
    print(f"wrote {dst}: holder «{version}» {kg * 1000:.0f} g" + (f" + phone {PHONE_KG * 1000:.0f} g" if phone else ""))


if __name__ == "__main__":
    v = sys.argv[sys.argv.index("--version") + 1] if "--version" in sys.argv else "vertical"
    if v == "mesa":
        if "--telefono-mesa" not in sys.argv:
            raise SystemExit("--version mesa needs --telefono-mesa X,Y,GIRO[,INCLINACION]")
        mesa(Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[sys.argv.index("--telefono-mesa") + 1])
    elif v in ("vertical", "plana"):
        main(Path(sys.argv[1]), Path(sys.argv[2]), v, "--sin-telefono" not in sys.argv)
    else:
        raise SystemExit("--version vertical | plana | mesa")
