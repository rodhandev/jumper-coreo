"""Put the cradle and a phone on a Jumper MJCF, with their masses, for training and for video.

    python montar.py <in.xml> <out.xml> [--sin-telefono]

Both go in a body `cuna_link` welded to base_link at its origin (cuna.scad's frame), colliding with the legs and
claws: a dance whose claw would hit the phone has to learn not to. A body of its own because base_link carries an
explicit <inertial>, which makes MuJoCo ignore the masses of geoms added to it; welded, it moves with the shell and
MuJoCo skips contacts between bodies welded together.

Masses (15-oct): the cradle is 64.6 cm^3; its walls are 3-4 mm, which a slicer prints almost solid, so 0.85 of
PLA's 1.24 g/cm^3 -> 68 g, plus the N52 ring (~15 g): 83 g. The phone: 200 g, 160 x 75 x 8 mm (a mid-size phone;
a heavy one is ~240 g). MuJoCo collides a mesh as its convex hull, which for the cradle is a block over the shell:
conservative, nothing should be inside it anyway.
"""

import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CUNA_G, PHONE_G = 0.083, 0.200
# The plate's frame, as in cuna.scad: origin (x0, 0, z0), turned -tilt about y.
X0, Z0, TILT = 0.0, 0.078, 30.0
PHONE = (0.160, 0.075, 0.008)  # long (y), short (up the plate), thick


def plate(c, a, b):
    """cuna.scad's plate frame (X out of the face, Y along, Z up the plate) -> base_link, metres."""
    t = math.radians(TILT)
    return (X0 + c * math.cos(t) - b * math.sin(t), a, Z0 + c * math.sin(t) + b * math.cos(t))


def main(src: Path, dst: Path, phone: bool = True) -> None:
    text = src.read_text()
    if 'meshdir="./"' in text:
        text = text.replace('meshdir="./"', f'meshdir="{src.parent.resolve()}/"', 1)
    stl = (HERE / "cuna.stl").resolve()
    text = text.replace("<asset>", f'<asset><mesh name="cuna" file="{stl}" scale="0.001 0.001 0.001"/>', 1)
    half = math.radians(-TILT) / 2
    quat = f"{math.cos(half):.6f} 0 {math.sin(half):.6f} 0"
    geoms = (f'<geom name="cuna" type="mesh" mesh="cuna" mass="{CUNA_G}" rgba="0.95 0.96 0.97 1" '
             'contype="1" conaffinity="1" group="2"/>')
    if phone:
        x, y, z = plate(PHONE[2] / 2, 0.0, PHONE[1] / 2)
        size = f"{PHONE[2] / 2} {PHONE[0] / 2} {PHONE[1] / 2}"
        geoms += (f'<geom name="telefono" type="box" pos="{x:.5f} {y:.5f} {z:.5f}" quat="{quat}" size="{size}" '
                  f'mass="{PHONE_G}" rgba="0.08 0.09 0.11 1" contype="1" conaffinity="1" group="2"/>')
        # the screen, facing out of the plate: visual only
        x, y, z = plate(PHONE[2] + 0.0002, 0.0, PHONE[1] / 2)
        geoms += (f'<geom name="telefono_pantalla" type="box" pos="{x:.5f} {y:.5f} {z:.5f}" quat="{quat}" '
                  f'size="0.0002 {PHONE[0] / 2 - 0.004} {PHONE[1] / 2 - 0.004}" rgba="0.10 0.22 0.30 1" '
                  'contype="0" conaffinity="0" density="0" group="2"/>')
    anchor = re.search(r'<body name="base_link"[^>]*>', text)
    text = text[:anchor.end()] + f'<body name="cuna_link">{geoms}</body>' + text[anchor.end():]
    dst.write_text(text)
    print(f"wrote {dst}")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]), "--sin-telefono" not in sys.argv)
