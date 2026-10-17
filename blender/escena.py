"""Film a clip on a desk in Blender: the scene `puente/exportar_blender.py` wrote, on a set of free (CC0) assets.

    blender -b -P escena.py -- <export dir> <assets dir> <out> [--frame N] [--motor cycles|eevee]
                               [--muestras 64] [--tam 1080x1920] [--escala 100] [--camara DIST,AZIM,ELEV,ALTO]
                               [--gpu] [--blend escena.blend]

<assets dir> holds what Poly Haven gives under CC0 (https://polyhaven.com/license): the HDRI
poly_haven_studio_2k.exr and the glTF folders WoodenTable_01, desk_lamp_arm_01, potted_plant_04,
book_encyclopedia_set_01, alarm_clock_01 (see coreo/README.md, «Filming on a desk»). With --frame it renders one
still (<out> a .png), otherwise the whole clip as numbered PNGs in <out>/ (ffmpeg joins them with the sound).

Why a desk (16/17-oct): Jumper is sold as a desk companion and its spec says indoors only; a park would contradict
both. The robot stands on the table top at z = 0, as on the floor in MuJoCo; the camera is MuJoCo's free camera
(distance, azimuth, elevation around a look-at point over the robot) so framings carry over from ver_clip.py.
"""

import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
exp, assets, dest = Path(argv[0]), Path(argv[1]), Path(argv[2])
arg = lambda n, d=None: argv[argv.index(n) + 1] if n in argv else d  # noqa: E731
frame = int(arg("--frame")) if arg("--frame") else None
W, H = (int(v) for v in arg("--tam", "1080x1920").split("x"))
escena = json.loads((exp / "escena.json").read_text())
poses = np.load(exp / "poses.npy")
DIST, AZIM, ELEV, ALTO = (float(v) for v in arg("--camara", "1.00,176,-32,0.08").split(","))

# ── empty scene, units in metres ──────────────────────────────────────────────────────────────────────────────
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.render.fps = escena["fps"]
sc.frame_start, sc.frame_end = 1, escena["frames"]


def material(name, rgba, rough=0.4, imagen=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    p = nt.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = rgba
    p.inputs["Roughness"].default_value = rough
    if rgba[0] > 0.3:   # the white shell: a satin plastic with a hint of sheen
        p.inputs["Coat Weight"].default_value = 0.15
    if imagen:          # a screen: the picture lights itself
        tex = nt.nodes.new("ShaderNodeTexImage")
        first = sorted((exp / imagen).glob("*.png"))[0]
        img = bpy.data.images.load(str(first))
        img.source = "SEQUENCE"
        tex.image = img
        tex.image_user.frame_duration = escena["frames"]
        tex.image_user.use_auto_refresh = True
        nt.links.new(tex.outputs["Color"], p.inputs["Base Color"])
        nt.links.new(tex.outputs["Color"], p.inputs["Emission Color"])
        p.inputs["Emission Strength"].default_value = 1.6
        p.inputs["Roughness"].default_value = 0.15
    return mat


def caja(name, half):
    """A box whose every face shows the whole picture, as MuJoCo maps a 2D texture onto a box."""
    bpy.ops.mesh.primitive_cube_add(size=2)
    ob = bpy.context.active_object
    ob.name = name
    ob.data.transform(np.diag([*half, 1.0]).tolist())
    uv = ob.data.uv_layers.active.data
    for poly in ob.data.polygons:
        for li, c in zip(poly.loop_indices, ((0, 0), (1, 0), (1, 1), (0, 1))):
            uv[li].uv = c
    return ob


# ── the robot ─────────────────────────────────────────────────────────────────────────────────────────────────
partes = []
for j, g in enumerate(escena["geoms"]):
    if "malla" in g:
        bpy.ops.wm.obj_import(filepath=str(exp / g["malla"]), forward_axis="Y", up_axis="Z")
        ob = bpy.context.selected_objects[0]
        ob.name = g["geom"]
        ob.data.set_sharp_from_angle(angle=math.radians(35))   # smooth curved shells, crisp edges
    else:
        ob = caja(g["geom"], g["caja"])
    # MuJoCo's rgba are display (sRGB) values; Blender's colours are linear
    rgba = tuple([c ** 2.2 for c in g["rgba"][:3]] + [g["rgba"][3]])
    rough = 0.3 if g["rgba"][0] > 0.6 else 0.5
    if g["geom"].startswith(("display_module", "tof_sensor", "camera_link")):
        rough = 0.12   # the visor is glossy black glass: crisp reflections, not a milky sheen
    im = g.get("imagen")
    ob.data.materials.clear()
    ob.data.materials.append(material(g["geom"], rgba, rough, im if im in escena["imagenes"] else None))
    ob.rotation_mode = "QUATERNION"
    partes.append(ob)


def poner(i):
    for j, ob in enumerate(partes):
        ob.location = poses[i, j, :3].tolist()
        ob.rotation_quaternion = poses[i, j, 3:].tolist()


if frame is None:
    for i in range(len(poses)):
        poner(i)
        for ob in partes:
            ob.keyframe_insert("location", frame=i + 1)
            ob.keyframe_insert("rotation_quaternion", frame=i + 1)
else:
    poner(frame - 1)
    sc.frame_set(frame)


# ── the set ───────────────────────────────────────────────────────────────────────────────────────────────────
def importar(nombre, pos, rot_z=0.0, escala=1.0, apoyo=0.0):
    """A Poly Haven glTF, its lowest point on z = apoyo, at pos (x, y), turned rot_z degrees."""
    antes = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(assets / nombre / f"{nombre}.gltf"))
    nuevos = [o for o in bpy.data.objects if o not in antes]
    raiz = bpy.data.objects.new(nombre, None)
    sc.collection.objects.link(raiz)
    for o in nuevos:
        if o.parent is None:
            o.parent = raiz
    raiz.scale = (escala,) * 3
    raiz.rotation_euler[2] = math.radians(rot_z)
    bpy.context.view_layer.update()
    pts = [o.matrix_world @ Vector(c) for o in nuevos if o.type == "MESH" for c in o.bound_box]
    zmin = min(p.z for p in pts)
    raiz.location = (pos[0], pos[1], apoyo - zmin)
    bpy.context.view_layer.update()
    return raiz, [o.matrix_world @ Vector(c) for o in nuevos if o.type == "MESH" for c in o.bound_box]


# the table: its top at z = 0, under the robot
mesa, pts = importar("WoodenTable_01", (0, 0), escala=1.0, apoyo=0.0)
top = max(p.z for p in pts)
mesa.location.z -= top
cx = sum(p.x for p in pts) / len(pts)
cy = sum(p.y for p in pts) / len(pts)
mesa.location.x -= cx + 0.15   # the robot a little in front of the table's centre, towards the camera
mesa.location.y -= cy
# props behind and beside it, as on a real desk; the robot faces +x, the camera looks from +x
# (a laptop was tried first: open, it filled half the background as a pale block)
importar("desk_lamp_arm_01", (-0.42, 0.30), rot_z=-60)
importar("potted_plant_04", (-0.40, -0.30))
importar("book_encyclopedia_set_01", (-0.62, 0.02), rot_z=90)
importar("alarm_clock_01", (-0.18, 0.40), rot_z=200)

# the phone beside the robot instead of in a holder (Rodhan, 17-oct): `--telefono-mesa X,Y,GIRO[,INCLINACION]`,
# metres and degrees; INCLINACION raises its top end, as if leant on something. modulos/cuna/stl/suelto-*.stl.
if arg("--telefono-mesa"):
    tx, ty, giro, *inc = (float(v) for v in arg("--telefono-mesa").split(","))
    stl = Path(__file__).resolve().parents[1] / "modulos" / "cuna" / "stl"
    color = {"cuerpo": (0.62, 0.64, 0.67, 1), "isla": (0.02, 0.02, 0.03, 1), "camara": (0.1, 0.11, 0.13, 1)}
    tel = bpy.data.objects.new("telefono", None)
    sc.collection.objects.link(tel)
    for p in ("cuerpo", "pantalla", "isla", "camara"):
        bpy.ops.wm.stl_import(filepath=str(stl / f"suelto-{p}.stl"), global_scale=0.001)
        ob = bpy.context.selected_objects[0]
        ob.name = f"telefono_{p}"
        ob.parent = tel
        if p == "pantalla":
            # flat texture coordinates over the screen, its top (the camera pill's end) at +y
            ob.data.uv_layers.new(name="UV")
            xs = [v.co.x for v in ob.data.vertices]
            ys = [v.co.y for v in ob.data.vertices]
            uv = ob.data.uv_layers.active.data
            for poly in ob.data.polygons:
                for li in poly.loop_indices:
                    co = ob.data.vertices[ob.data.loops[li].vertex_index].co
                    uv[li].uv = ((co.x - min(xs)) / (max(xs) - min(xs)), (co.y - min(ys)) / (max(ys) - min(ys)))
            con = "pantalla" in escena["imagenes"]
            ob.data.materials.append(material(ob.name, (0.1, 0.2, 0.33, 1), imagen="pantalla" if con else None))
        else:
            c = color[p]
            ob.data.materials.append(material(ob.name, tuple([v ** 2.2 for v in c[:3]] + [1]), 0.35))
    tel.rotation_euler = (math.radians(inc[0] if inc else 0.0), 0, math.radians(giro))
    tel.location = (tx, ty, 0.0027 + (0.0738 * math.sin(math.radians(inc[0])) if inc else 0.0))
    if inc and inc[0] > 0:
        # a leaning phone needs something under it: a plain white desk stand, a wedge from the table to its back
        import bmesh

        bpy.context.view_layer.update()
        M = tel.matrix_world
        bm = bmesh.new()
        vs = []
        for x in (-0.03, 0.03):
            a = M @ Vector((x, -0.072, 0.0))      # the back, at the bottom end
            b = M @ Vector((x, 0.030, 0.0))       # the back, a little past the middle
            vs += [bm.verts.new(a), bm.verts.new(b), bm.verts.new((b.x, b.y, 0.0))]
        bm.faces.new(vs[0:3])
        bm.faces.new(vs[3:6][::-1])
        for i, j in ((0, 1), (1, 2), (2, 0)):
            bm.faces.new((vs[i], vs[j], vs[j + 3], vs[i + 3])[::-1])
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        me = bpy.data.meshes.new("soporte")
        bm.to_mesh(me)
        ob = bpy.data.objects.new("soporte", me)
        sc.collection.objects.link(ob)
        ob.data.materials.append(material("soporte", (0.85, 0.86, 0.87, 1), 0.35))

# light: the studio HDRI (a room with a window and a desk) plus a warm key from the lamp's side
world = bpy.data.worlds.new("mundo")
sc.world = world
world.use_nodes = True
nt = world.node_tree
env = nt.nodes.new("ShaderNodeTexEnvironment")
env.image = bpy.data.images.load(str(assets / "poly_haven_studio_2k.exr"))
mapping = nt.nodes.new("ShaderNodeMapping")
coord = nt.nodes.new("ShaderNodeTexCoord")
mapping.inputs["Rotation"].default_value[2] = math.radians(float(arg("--hdri-giro", 120)))
nt.links.new(coord.outputs["Generated"], mapping.inputs["Vector"])
nt.links.new(mapping.outputs["Vector"], env.inputs["Vector"])
nt.links.new(env.outputs["Color"], nt.nodes["Background"].inputs["Color"])
nt.nodes["Background"].inputs["Strength"].default_value = 1.0
key = bpy.data.lights.new("clave", "AREA")
key.energy, key.size, key.color = 60, 0.6, (1.0, 0.92, 0.82)
ko = bpy.data.objects.new("clave", key)
sc.collection.objects.link(ko)
ko.location = (0.6, -0.7, 0.9)
ko.rotation_euler = (math.radians(50), 0, math.radians(40))

# ── the camera: MuJoCo's free camera ─────────────────────────────────────────────────────────────────────────
cam = bpy.data.cameras.new("camara")
cam.sensor_fit = "VERTICAL"
cam.angle_y = math.radians(45)
cam.dof.use_dof = True
cam.dof.aperture_fstop = float(arg("--fstop", 2.8))
co = bpy.data.objects.new("camara", cam)
sc.collection.objects.link(co)
sc.camera = co
look = Vector(poses[0, 0, :2].tolist() + [ALTO])
el, az = math.radians(ELEV), math.radians(AZIM)
fwd = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
co.location = look - DIST * fwd
co.rotation_euler = fwd.to_track_quat("-Z", "Y").to_euler()
cam.dof.focus_distance = DIST

# ── render ────────────────────────────────────────────────────────────────────────────────────────────────────
r = sc.render
r.resolution_x, r.resolution_y = W, H
r.resolution_percentage = int(arg("--escala", 100))
if arg("--motor", "cycles") == "cycles":
    r.engine = "CYCLES"
    sc.cycles.samples = int(arg("--muestras", 64))
    sc.cycles.use_denoising = True
    sc.cycles.device = "GPU" if "--gpu" in argv else "CPU"
    if "--gpu" in argv:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.compute_device_type = "OPTIX"
        prefs.get_devices()
        for dev in prefs.devices:
            dev.use = True
else:
    r.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items.keys() else "BLENDER_EEVEE"
sc.view_settings.view_transform = "AgX"
sc.view_settings.look = "AgX - Medium High Contrast"
if arg("--blend"):
    bpy.ops.wm.save_as_mainfile(filepath=str(Path(arg("--blend")).resolve()))
if frame is not None:
    r.filepath = str(dest.resolve())
    bpy.ops.render.render(write_still=True)
else:
    dest.mkdir(parents=True, exist_ok=True)
    r.filepath = str(dest.resolve()) + "/"
    bpy.ops.render.render(animation=True)
print(f"rendered -> {dest}")
