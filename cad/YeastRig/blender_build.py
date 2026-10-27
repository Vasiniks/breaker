"""Builds the yeast rig in Blender from rig_recipe.py, and exports it.

In Blender's Python console or Text Editor:
    import sys; sys.path.insert(0, '<this folder>')
    import blender_build as bb
    bb.build()            # (re)build the scene; reads "Rig Settings" overrides
    bb.export_fusion()    # writes rig_export.json for YeastRig.py (Fusion)
    bb.export_stls()      # print-ready STLs in ./print

Move the "Carriage height" empty up/down to set the height (export snaps it to
the nearest 10 mm pin step). Rotate "Rock hinge" / "Pour hinge" about X to preview the motion;
export always uses the rest pose. Change sizes on "Rig Settings" (custom
properties) and run build() again.
"""

import importlib
import json
import math
import os
import struct

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = 'YeastRig'
SETTINGS = 'Rig Settings'
EDITABLE = ('tip_z', 'pitch', 'upper_back', 'upper_rise', 'pour_angle', 'pour_drop',
            'rock_angle', 'clip_squeeze', 'clip_open', 'hotplate_half', 'tower_gap')


# ---------------------------------------------------------------- primitives

def _segments(r):
    return 64 if r >= 6 else 32 if r >= 2 else 20


def _prim_mesh(prim, name):
    bm = bmesh.new()
    kind = prim[0]
    if kind == 'box':
        (x0, y0, z0), (x1, y1, z1) = prim[1], prim[2]
        bmesh.ops.create_cube(bm, size=1.0)
        m = Matrix.Translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) @ \
            Matrix.Diagonal((x1 - x0, y1 - y0, z1 - z0, 1))
    elif kind == 'obox':
        c, u, v, (L, W, H) = prim[1], Vector(prim[2]), Vector(prim[3]), prim[4]
        w = u.cross(v)
        bmesh.ops.create_cube(bm, size=1.0)
        m = Matrix(((u.x * L, v.x * W, w.x * H, c[0]),
                    (u.y * L, v.y * W, w.y * H, c[1]),
                    (u.z * L, v.z * W, w.z * H, c[2]),
                    (0, 0, 0, 1)))
    else:  # cyl
        p, q, r1, r2 = Vector(prim[1]), Vector(prim[2]), prim[3], prim[4]
        d = q - p
        bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=_segments(max(r1, r2)),
                              radius1=r1, radius2=r2, depth=d.length)
        rot = Vector((0, 0, 1)).rotation_difference(d.normalized()).to_matrix().to_4x4()
        m = Matrix.Translation((p + q) / 2) @ rot
    bmesh.ops.transform(bm, matrix=m, verts=bm.verts)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    return me


class _Builder:
    def __init__(self, scratch):
        self.scratch = scratch
        self.n = 0

    def obj(self, prim):
        if prim[0] == 'group':
            return self.solid(prim[1])
        self.n += 1
        o = bpy.data.objects.new('tmp%d' % self.n, _prim_mesh(prim, 'tmp%d' % self.n))
        self.scratch.objects.link(o)
        return o

    def solid(self, ops):
        """Evaluate a list of ops into one object. Runs of the same sign share
        one boolean against a collection, which keeps it quick."""
        target = self.obj(ops[0][1])
        i = 1
        while i < len(ops):
            sign = ops[i][0]
            run = []
            while i < len(ops) and ops[i][0] == sign:
                run.append(self.obj(ops[i][1]))
                i += 1
            coll = bpy.data.collections.new('run%d' % self.n)
            self.scratch.children.link(coll)
            for o in run:
                for c in o.users_collection:
                    c.objects.unlink(o)
                coll.objects.link(o)
            mod = target.modifiers.new('b', 'BOOLEAN')
            mod.operation = {'+': 'UNION', '-': 'DIFFERENCE', '&': 'INTERSECT'}[sign]
            mod.operand_type = 'COLLECTION'
            mod.collection = coll
            try:
                mod.solver = 'MANIFOLD'
            except TypeError:
                mod.solver = 'EXACT'
            _apply(target)
            for o in run:
                _delete(o)
            bpy.data.collections.remove(coll)
        return target


def _apply(obj):
    deps = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(obj.evaluated_get(deps))
    old = obj.data
    obj.modifiers.clear()
    obj.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)


def _delete(obj):
    me = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    if me and me.users == 0:
        bpy.data.meshes.remove(me)


# ---------------------------------------------------------------- scene

def _material(name, hexcol, glass=False, metal=0.0, rough=0.45):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    col = tuple(int(hexcol[i:i + 2], 16) / 255 for i in (1, 3, 5)) + (1.0,)
    bsdf = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Base Color'].default_value = col
    bsdf.inputs['Roughness'].default_value = rough
    bsdf.inputs['Metallic'].default_value = metal
    if glass:
        for key in ('Transmission Weight', 'Transmission'):
            if key in bsdf.inputs:
                bsdf.inputs[key].default_value = 1.0
        bsdf.inputs['Roughness'].default_value = 0.05
        mat.diffuse_color = col[:3] + (0.25,)
        for attr, val in (('surface_render_method', 'BLENDED'), ('blend_method', 'BLEND')):
            try:
                setattr(mat, attr, val)
            except (AttributeError, TypeError):
                pass
    else:
        mat.diffuse_color = col
    return mat


def _collection(name, parent):
    coll = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if coll.name not in parent.children:
        parent.children.link(coll)
    return coll


def _clear():
    root = bpy.data.collections.get(ROOT)
    if not root:
        return
    for coll in [root] + list(root.children_recursive):
        for o in list(coll.objects):
            _delete(o) if o.type == 'MESH' else bpy.data.objects.remove(o, do_unlink=True)
    for coll in list(root.children_recursive):
        bpy.data.collections.remove(coll)


def _settings_overrides():
    s = bpy.data.objects.get(SETTINGS)
    if not s:
        return {}
    return {k: float(s[k]) for k in EDITABLE if k in s.keys()}


def _place_matrix(pl):
    x, y, z, o = Vector(pl['x']), Vector(pl['y']), Vector(pl['z']), pl['origin']
    return Matrix(((x[0], y[0], z[0], o[0]), (x[1], y[1], z[1], o[1]),
                   (x[2], y[2], z[2], o[2]), (0, 0, 0, 1)))


def build(params=None):
    import rig_recipe
    importlib.reload(rig_recipe)
    over = _settings_overrides()
    over.update(params or {})
    rig = rig_recipe.build(over)
    P = rig['params']

    _clear()
    scene = bpy.context.scene
    root = _collection(ROOT, scene.collection)
    scratch = _collection('_scratch', scene.collection)
    builder = _Builder(scratch)

    s = bpy.data.objects.get(SETTINGS)
    if not s:
        s = bpy.data.objects.new(SETTINGS, None)
        s.empty_display_size = 20
    if SETTINGS not in root.objects:
        for c in s.users_collection:
            c.objects.unlink(s)
        root.objects.link(s)
    for k in EDITABLE:
        if k in P:
            s[k] = float(P[k])

    colls = {a: _collection(a, root) for a in
             ('Base assembly', 'Carriage assembly', 'Lower rack assembly', 'Upper rack assembly')}
    info = rig['info']

    carriage = bpy.data.objects.new('Carriage height', None)
    carriage.empty_display_type = 'ARROWS'
    carriage.empty_display_size = 30
    carriage.location = (info['frame_front_x'] + 24, 0, info['carriage_z'][0])
    colls['Carriage assembly'].objects.link(carriage)
    lim = carriage.constraints.new('LIMIT_LOCATION')
    for ax in ('x', 'y'):
        setattr(lim, 'use_min_' + ax, True)
        setattr(lim, 'use_max_' + ax, True)
        setattr(lim, 'min_' + ax, carriage.location['xy'.index(ax)])
        setattr(lim, 'max_' + ax, carriage.location['xy'.index(ax)])
    lim.use_min_z = lim.use_max_z = True
    lim.min_z = carriage.location.z + min(info['heights'])
    lim.max_z = carriage.location.z + max(info['heights'])
    carriage['rest_z'] = carriage.location.z
    carriage['step'] = info['height_step']
    bpy.context.view_layer.update()

    hinge_obj, empties = {}, {}
    for h in rig['hinges']:
        e = bpy.data.objects.new(h['name'], None)
        e.empty_display_type = 'SINGLE_ARROW'
        e.empty_display_size = 25
        e.rotation_mode = 'XYZ'
        e.location = h.get('origin') or (h['bore'][0], h['pivot'][0], h['pivot'][1])
        colls[h['asm']].objects.link(e)
        parent = empties.get(h.get('parent')) or carriage
        bpy.context.view_layer.update()
        e.parent = parent
        e.matrix_parent_inverse = parent.matrix_world.inverted()
        axis = h.get('axis', 'x')
        c = e.constraints.new('LIMIT_ROTATION')
        c.use_limit_x = c.use_limit_y = c.use_limit_z = True
        lo, hi = (math.radians(a) for a in h['limits'])
        setattr(c, 'min_' + axis, lo)
        setattr(c, 'max_' + axis, hi)
        empties[h['name']] = e
        if h.get('members'):
            for m in h['members']:
                hinge_obj[m] = e
        else:
            hinge_obj.setdefault(h['asm'], e)
        bpy.context.view_layer.update()

    mats = {}
    for part in rig['parts']:
        o = builder.solid(part['ops'])
        o.name = part['name']
        o.data.name = part['name']
        for c in o.users_collection:
            c.objects.unlink(o)
        colls[part['asm']].objects.link(o)
        mat = mats.get(part['color']) or _material('Rig ' + part['color'], part['color'])
        mats[part['color']] = mat
        o.data.materials.append(mat)
        o.color = mat.diffuse_color
        bev = o.modifiers.new('Rounded edges', 'BEVEL')
        bev.width = 0.8
        bev.segments = 3
        bev.limit_method = 'ANGLE'
        bev.angle_limit = math.radians(35)
        bev.use_clamp_overlap = True
        _centre_origin(o)
        o['recipe_part'] = part['name']
        o['stl'] = part.get('stl', True)
        o['print_rot'] = json.dumps(part['print_rot'])
        o['note'] = part['note']
        _parent(o, hinge_obj.get(part['name']) or hinge_obj.get(part['asm']) or
                (carriage if part['asm'] == 'Carriage assembly' else None))

    glass = _material('Rig tube', '#cfe6f2', glass=True)
    metal = _material('Rig motor', '#2b2b2e', metal=0.6, rough=0.35)
    for b in rig['bought']:
        o = builder.solid(b['ops'])
        o.data.transform(_place_matrix(b['place']))
        o.name = b['name']
        o.data.name = b['name']
        for c in o.users_collection:
            c.objects.unlink(o)
        colls[b['asm']].objects.link(o)
        mat = glass if b['kind'] == 'tube' else metal
        o.data.materials.append(mat)
        o.color = mat.diffuse_color
        _centre_origin(o)
        o['bought'] = b['kind']
        o.hide_select = False
        _parent(o, hinge_obj.get(b['asm']) or (carriage if b['asm'] == 'Carriage assembly' else None))

    bpy.data.collections.remove(scratch)
    scene['yeastrig'] = json.dumps(rig)
    for area in bpy.context.screen.areas if bpy.context.screen else []:
        if area.type == 'VIEW_3D':
            sp = area.spaces.active
            sp.shading.type = 'SOLID'
            sp.shading.color_type = 'OBJECT'
            sp.clip_end = 10000
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = 'MILLIMETERS'
    return rig['info']


def _centre_origin(o):
    """Put the object's origin at its middle (easier to move); export undoes it."""
    vs = [v.co for v in o.data.vertices]
    c = Vector(((min(v.x for v in vs) + max(v.x for v in vs)) / 2,
                (min(v.y for v in vs) + max(v.y for v in vs)) / 2,
                (min(v.z for v in vs) + max(v.z for v in vs)) / 2))
    o.data.transform(Matrix.Translation(-c))
    o.matrix_world = Matrix.Translation(c)
    o['origin_offset'] = list(c)


def _parent(o, parent):
    if parent is None:
        return
    o.parent = parent
    o.matrix_parent_inverse = parent.matrix_world.inverted()


# ---------------------------------------------------------------- export

def _rest_pose():
    """Put both hinges at rest; returns a function that restores them."""
    saved = []
    for name in ('Rock hinge', 'Pour hinge', 'Funnel hinge'):
        e = bpy.data.objects.get(name)
        if e:
            saved.append((e, e.rotation_euler.copy()))
            e.rotation_euler = (0, 0, 0)
    bpy.context.view_layer.update()

    def restore():
        for e, r in saved:
            e.rotation_euler = r
        bpy.context.view_layer.update()
    return restore


def export_fusion(path=None):
    """Writes the rest pose, with the carriage snapped to the nearest pin step."""
    path = path or os.path.join(HERE, 'rig_export.json')
    rig = json.loads(bpy.context.scene['yeastrig'])
    c = bpy.data.objects.get('Carriage height')
    z_was = c.location.z
    steps = rig['info']['heights']
    dz = min(steps, key=lambda h: abs(c.location.z - c['rest_z'] - h))
    c.location.z = c['rest_z'] + dz
    rig['info']['height_offset'] = dz
    restore = _rest_pose()
    try:
        mats = {}
        for item in rig['parts'] + rig['bought']:
            o = bpy.data.objects.get(item['name'])
            if o:
                delta = o.matrix_world @ Matrix.Translation(-Vector(o['origin_offset']))
                mats[item['name']] = [v for row in delta for v in row]
    finally:
        restore()
        c.location.z = z_was
        bpy.context.view_layer.update()
    rig['matrices'] = mats
    with open(path, 'w') as f:
        json.dump(rig, f)
    return path


def export_stls(folder=None):
    folder = folder or os.path.join(HERE, 'print')
    os.makedirs(folder, exist_ok=True)
    written = []
    for o in [o for o in bpy.data.collections[ROOT].all_objects if o.get('stl')]:
        deps = bpy.context.evaluated_depsgraph_get()
        me = bpy.data.meshes.new_from_object(o.evaluated_get(deps))   # includes rounded edges
        me.transform(Matrix.Translation(Vector(o['origin_offset'])))
        rot = json.loads(o['print_rot'])
        if rot:
            me.transform(Matrix.Rotation(math.radians(rot[1]), 4, rot[0].upper()))
        xs = [v.co for v in me.vertices]
        lo = Vector((min(c.x for c in xs), min(c.y for c in xs), min(c.z for c in xs)))
        hi = Vector((max(c.x for c in xs), max(c.y for c in xs), max(c.z for c in xs)))
        me.transform(Matrix.Translation((-(lo.x + hi.x) / 2, -(lo.y + hi.y) / 2, -lo.z)))
        me.calc_loop_triangles()
        path = os.path.join(folder, o.name.replace(' ', '_') + '.stl')
        with open(path, 'wb') as f:
            f.write(b'YeastRig'.ljust(80, b' '))
            f.write(struct.pack('<I', len(me.loop_triangles)))
            for t in me.loop_triangles:
                f.write(struct.pack('<3f', *t.normal))
                for vi in t.vertices:
                    f.write(struct.pack('<3f', *me.vertices[vi].co))
                f.write(b'\0\0')
        size = hi - lo
        written.append((o.name, path, tuple(round(c, 1) for c in size)))
        bpy.data.meshes.remove(me)
    return written
