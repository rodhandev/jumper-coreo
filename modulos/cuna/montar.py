"""Put a phone holder and its phone on a Jumper MJCF, with their masses, for training and for video.

    python montar.py <in.xml> <out.xml> [--version vertical|plana] [--sin-telefono]

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
    if v not in ("vertical", "plana"):
        raise SystemExit("--version vertical | plana")
    main(Path(sys.argv[1]), Path(sys.argv[2]), v, "--sin-telefono" not in sys.argv)
