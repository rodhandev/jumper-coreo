"""The camera of a clip over time, shared by ver_clip.py, esqueleto.py and exportar_blender.py.

MuJoCo's free camera: distance, azimuth, elevation (degrees) around a look-at point. The look-at point follows the
robot's base in x and y, at a fixed height, shifted by (dx, dy) metres -- towards the phone on the table, say.
A guion's `camara` orders, [t, "dist,azim,elev[,height[,dx,dy]]"], each glide the camera there over `paso`
seconds (smoothstep) from where it was; values left out stay as they were. Before the first order, `base`.
"""

#: dist, azim, elev, height, dx, dy
DEFECTO = [0.75, 135.0, -18.0, 0.08, 0.0, 0.0]


def base(texto):
    """`--camara DIST,AZIM,ELEV[,HEIGHT[,DX,DY]]` -> the six values, the rest from DEFECTO."""
    v = [float(x) for x in texto.split(",")] if texto else []
    return v + DEFECTO[len(v):]


def camara_en(t, inicio, ordenes, paso=0.6):
    cur = list(inicio)
    for te, v in ordenes:
        if te > t:
            break
        goal = [float(x) for x in v.split(",")]
        goal += cur[len(goal):]
        u = min(max((t - te) / paso, 0.0), 1.0)
        u = u * u * (3 - 2 * u)
        cur = [a + (b - a) * u for a, b in zip(cur, goal)]
    return cur
