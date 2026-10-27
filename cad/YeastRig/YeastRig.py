"""Yeast rig: builds the Fusion assembly from what you set up in Blender.

Run in Fusion: Utilities > Add-Ins > Scripts and Add-Ins (Shift+S), +,
"Script or add-in from device", pick this YeastRig folder, Run.

It reads rig_export.json (written by blender_build.export_fusion(); falls back
to rig_recipe.json) and makes:
    Base assembly                Base + Base tail + 3 tail pins
    Carriage assembly            Carriage + 2 NEMA17 motors + 4 height pins
    Lower rack assembly          Lower rack + Funnel strip + 3 tubes
    Upper rack assembly          Upper rack + 3 tubes
Every printed part is its own component with a real solid body. The motors
and tubes are exact-size stand-ins (see USE_REAL_FILES). The two
hinges are revolute joints (Drive Joints shows the rock and the pour). It
saves YeastRig.f3d next to this file. A progress bar shows each step.
"""

import json
import os
import time
import traceback

try:
    import adsk.core
    import adsk.fusion
except ImportError:
    adsk = None

HERE = os.path.dirname(os.path.realpath(__file__))
BOUGHT_FILES = {'motor': 'NEMA17 Motor.f3d', 'tube': 'Test Tube.f3d'}
BOUGHT_NAMES = {'motor': 'NEMA17 motor', 'tube': 'Tube 50 mL'}
# Importing the .f3d files from ./parts hung Fusion, so by default the motors and
# tubes are built as exact-size stand-ins. Set True to try the real files again.
USE_REAL_FILES = False
MM = 0.1  # the Fusion API works in centimetres


# ---------------------------------------------------------------- 4x4 maths (row-major, mm)

def _identity():
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def _mul(a, b):
    return [sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4)) for r in range(4) for c in range(4)]


def _place(pl):
    x, y, z, o = pl['x'], pl['y'], pl['z'], pl['origin']
    return [x[0], y[0], z[0], o[0], x[1], y[1], z[1], o[1], x[2], y[2], z[2], o[2], 0, 0, 0, 1]


def _apply(m, p):
    return [m[r * 4] * p[0] + m[r * 4 + 1] * p[1] + m[r * 4 + 2] * p[2] + m[r * 4 + 3] for r in range(3)]


# ---------------------------------------------------------------- Fusion helpers

def _matrix(m):
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(
        adsk.core.Point3D.create(m[3] * MM, m[7] * MM, m[11] * MM),
        adsk.core.Vector3D.create(m[0], m[4], m[8]),
        adsk.core.Vector3D.create(m[1], m[5], m[9]),
        adsk.core.Vector3D.create(m[2], m[6], m[10]))
    return mat


def _pt(p):
    return adsk.core.Point3D.create(p[0] * MM, p[1] * MM, p[2] * MM)


def _brep(tbm, prim):
    kind = prim[0]
    if kind == 'group':
        return _solid(tbm, prim[1])
    if kind == 'box':
        (x0, y0, z0), (x1, y1, z1) = prim[1], prim[2]
        obb = adsk.core.OrientedBoundingBox3D.create(
            _pt(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)),
            adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
            (x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM)
        return tbm.createBox(obb)
    if kind == 'obox':
        c, u, v, (L, W, H) = prim[1], prim[2], prim[3], prim[4]
        obb = adsk.core.OrientedBoundingBox3D.create(
            _pt(c), adsk.core.Vector3D.create(*u), adsk.core.Vector3D.create(*v), L * MM, W * MM, H * MM)
        return tbm.createBox(obb)
    p, q, r1, r2 = prim[1:]
    return tbm.createCylinderOrCone(_pt(p), r1 * MM, _pt(q), r2 * MM)


def _solid(tbm, ops):
    """Runs of adds or cuts are merged into one tool first, so the part
    itself only goes through a few booleans (much faster)."""
    kinds = {'+': adsk.fusion.BooleanTypes.UnionBooleanType,
             '-': adsk.fusion.BooleanTypes.DifferenceBooleanType,
             '&': adsk.fusion.BooleanTypes.IntersectionBooleanType}
    union = kinds['+']
    body = _brep(tbm, ops[0][1])
    i = 1
    while i < len(ops):
        sign = ops[i][0]
        if sign == '&':
            tbm.booleanOperation(body, _brep(tbm, ops[i][1]), kinds['&'])
            i += 1
            continue
        tool = None
        while i < len(ops) and ops[i][0] == sign:
            b = _brep(tbm, ops[i][1])
            if tool is None:
                tool = b
            else:
                tbm.booleanOperation(tool, b, union)
            i += 1
        tbm.booleanOperation(body, tool, kinds[sign])
    return body


def _add_body(comp, tbm, ops, name):
    base = comp.features.baseFeatures.add()
    base.startEdit()
    body = comp.bRepBodies.add(_solid(tbm, ops), base)
    base.finishEdit()
    body.name = name
    return body


def _standin(comp, kind, ops, tbm):
    """Build one stand-in for a bought part; copies are placed from it."""
    occ = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = BOUGHT_NAMES[kind]
    _add_body(occ.component, tbm, ops, BOUGHT_NAMES[kind])
    return occ.component, occ


def _template(comp, kind, notes):
    """Import the real bought part once; returns (component, occurrence to delete)."""
    path = os.path.join(HERE, 'parts', BOUGHT_FILES[kind])
    im = adsk.core.Application.get().importManager
    try:
        made = im.importToTarget2(im.createFusionArchiveImportOptions(path), comp)
        for i in range(made.count):
            occ = adsk.fusion.Occurrence.cast(made.item(i))
            if occ:
                return occ.component, occ
    except Exception:  # pylint:disable=broad-except
        pass
    notes.append('Could not import %s, used a stand-in.' % BOUGHT_FILES[kind])
    return None, None


def _bore_edge(asm_occ, centre, radius):
    """Circular edge of a hub bore anywhere inside an assembly occurrence."""
    best = None
    for child in asm_occ.childOccurrences:
        for b in child.bRepBodies:
            for edge in b.edges:
                geo = edge.geometry
                if geo.curveType != adsk.core.Curve3DTypes.Circle3DCurveType:
                    continue
                circle = adsk.core.Circle3D.cast(geo)
                if abs(circle.radius - radius * MM) > 2e-3:
                    continue
                c = circle.center
                d = ((c.x - centre[0] * MM) ** 2 + (c.y - centre[1] * MM) ** 2 + (c.z - centre[2] * MM) ** 2) ** 0.5
                if d < 0.05 and (best is None or d < best[0]):
                    best = (d, edge)
    return best[1] if best else None


def load_rig():
    for name in ('rig_export.json', 'rig_recipe.json'):
        path = os.path.join(HERE, name)
        if os.path.exists(path):
            with open(path) as f:
                rig = json.load(f)
            rig.setdefault('matrices', {})
            return rig, name
    raise FileNotFoundError('rig_export.json not found next to the script')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        rig, source = load_rig()
        mats = rig['matrices']
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()
        notes = []
        steps = len(rig['parts']) + 4
        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = False
        progress.show('Yeast rig', 'Building... %v of %m', 0, steps)
        t0 = time.time()

        def tick(label):
            progress.progressValue = progress.progressValue + 1
            progress.message = label
            app.log('YeastRig %5.1fs  %s' % (time.time() - t0, label))
            adsk.doEvents()

        # one assembly component per group, one part component per printed part
        asm = {}
        for name in [p['asm'] for p in rig['parts']] + [b['asm'] for b in rig['bought']]:
            if name not in asm:
                occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                occ.component.name = name
                asm[name] = occ

        def parent_comp(a):
            return asm[a].component

        # printed parts: one component each
        for part in rig['parts']:
            m = mats.get(part['name'], _identity())
            occ = parent_comp(part['asm']).occurrences.addNewComponent(_matrix(m))
            occ.component.name = part['name']
            _add_body(occ.component, tbm, part['ops'], part['name'])
            tick('Built ' + part['name'])

        # bought parts: import each file once, then place copies
        templates = {}
        for b in rig['bought']:
            comp = parent_comp(b['asm'])
            key = b['kind']
            if key not in templates:
                found = _template(comp, key, notes) if USE_REAL_FILES else (None, None)
                templates[key] = found if found[0] else _standin(comp, key, b['ops'], tbm)
            tcomp, _ = templates[key]
            m = _mul(mats.get(b['name'], _identity()), _place(b['place']))
            comp.occurrences.addExistingComponent(tcomp, _matrix(m))
        for tcomp, first in templates.values():
            if first:
                first.deleteMe()  # the copy at the file's own origin
        tick('Placed motors and tubes')

        asm['Base assembly'].isGrounded = True
        asm['Carriage assembly'].isGrounded = True

        # hinges
        for h in rig['hinges']:
            if 'bore' not in h:
                continue  # the funnel flap: leave it free in Fusion
            try:
                m = mats.get(h['part'], _identity())
                centre = _apply(m, (h['bore'][0], h['pivot'][0], h['pivot'][1]))
                edge = _bore_edge(asm[h['asm']], centre, h['bore'][1])
                geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
                joint_in = root.asBuiltJoints.createInput(asm[h['asm']], asm['Carriage assembly'], geo)
                joint_in.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
                joint = root.asBuiltJoints.add(joint_in)
                joint.name = h['name']
            except Exception:  # pylint:disable=broad-except
                notes.append('%s was skipped; add it by hand (As-built Joint, Revolute, on the hub hole).' % h['name'])
        tick('Added joints')

        saved = []
        exporter = design.exportManager
        path = os.path.join(HERE, 'YeastRig.f3d')
        try:
            if exporter.execute(exporter.createFusionArchiveExportOptions(path)):
                saved.append(os.path.basename(path))
        except Exception:  # pylint:disable=broad-except
            notes.append('Could not save %s.' % path)
        tick('Saved')
        progress.hide()

        app.activeViewport.fit()
        ui.messageBox('\n'.join(['Yeast rig built from %s in %.0f s.' % (source, time.time() - t0),
                                 'Saved: ' + (', '.join(saved) or 'nothing')] + notes))
    except Exception:  # pylint:disable=broad-except
        app.log('Failed:\n' + traceback.format_exc())
        ui.messageBox('Failed:\n' + traceback.format_exc())
