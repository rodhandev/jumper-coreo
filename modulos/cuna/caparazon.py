"""Write caparazon.stl: the top of Jumper's shell under the cradle, as a closed solid, for cuna.scad to cut from.

The shell's own mesh does not do: joined to the display module it is not closed (CGAL refuses it), and 19 000
faces make the cut slow. What the cradle needs is the surface it sits on, so this raycasts the shell from above
on a 2 mm grid over the footprint of both holders and closes it down to z = 20 mm: a height field, closed by
construction. base_link frame, millimetres, robot at its model pose.

    python caparazon.py <jumper checkout> [caparazon.stl]
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

repo = Path(sys.argv[1])
dest = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent / "caparazon.stl"
X, Y, STEP, FLOOR = (-94.0, 62.0), (-62.0, 62.0), 2.0, 20.0

m = mujoco.MjModel.from_xml_path(str(repo / "assets/jumper/jumper.xml"))
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
R, p = d.xmat[b].reshape(3, 3), d.xpos[b]
gid = np.zeros(1, np.int32)
xs = np.arange(X[0], X[1] + 1e-9, STEP)
ys = np.arange(Y[0], Y[1] + 1e-9, STEP)
top = np.empty((len(xs), len(ys)))
for i, x in enumerate(xs):
    for j, y in enumerate(ys):
        dist = mujoco.mj_ray(m, d, p + R @ (np.array([x, y, 300.0]) / 1000), -R[:, 2], None, 1, -1, gid)
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(gid[0])) or ""
        if dist < 0 or not ("upper_shell" in name or "display_module" in name):
            raise SystemExit(f"no shell under ({x}, {y}): hit {name or 'nothing'} -- shrink X/Y")
        top[i, j] = 300.0 - dist * 1000

# vertices: the top grid, then the bottom grid
nx, ny = top.shape
gx, gy = np.meshgrid(xs, ys, indexing="ij")
v = np.concatenate([np.stack([gx, gy, top], -1).reshape(-1, 3),
                    np.stack([gx, gy, np.full_like(top, FLOOR)], -1).reshape(-1, 3)])
idx = np.arange(nx * ny).reshape(nx, ny)
bot = idx + nx * ny
f = []
for i in range(nx - 1):
    for j in range(ny - 1):
        a, b_, c, e = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
        f += [(a, b_, c), (a, c, e)]                       # top, facing up
        a, b_, c, e = bot[i, j], bot[i + 1, j], bot[i + 1, j + 1], bot[i, j + 1]
        f += [(a, c, b_), (a, e, c)]                       # bottom, facing down
ring = [idx[i, 0] for i in range(nx)] + [idx[nx - 1, j] for j in range(1, ny)] + \
       [idx[i, ny - 1] for i in range(nx - 2, -1, -1)] + [idx[0, j] for j in range(ny - 2, 0, -1)]
for k in range(len(ring)):                                 # walls
    t0, t1 = ring[k], ring[(k + 1) % len(ring)]
    f += [(t0, t1 + nx * ny, t1), (t0, t0 + nx * ny, t1 + nx * ny)]
f = np.array(f)

import trimesh  # noqa: E402

mesh = trimesh.Trimesh(v, f, process=True)
mesh.fix_normals()
assert mesh.is_watertight and mesh.volume > 0, "height field not closed"
mesh.export(dest)
print(f"{dest}: {len(mesh.faces)} faces, top {top.min():.1f}-{top.max():.1f} mm")
