"""Yeast rig recipe: every printed part as a list of solid operations.

Pure Python, no CAD library. Blender (blender_build.py) turns the recipe into
meshes you can look at and move; Fusion (YeastRig.py) turns the same recipe
into real solids, one component per part. Units are mm.

Axes: X along the row of tubes, Y front to back (camera on the -Y side),
Z up. The hot plate sits in front of the frame, centred on x = 0.

Primitives:
    ('box', (x0, y0, z0), (x1, y1, z1))           axis-aligned box
    ('obox', centre, u, v, (L, W, H))              box along u, v and u x v
    ('cyl', p, q, r1, r2)                          cylinder or cone from p to q
    ('group', [ops])                               evaluated on its own first
Ops are (sign, prim) with sign '+' union, '-' cut, '&' intersect.
"""

import json
import math
import os

P = dict(
    # bought parts, measured from NEMA17 Motor.f3d and Test Tube.f3d
    tube_r0=14.58, tube_r1=13.90, tube_cyl=98.0, tube_len=113.7, tube_rtip=3.25,
    tube_wall=1.0,
    motor_w=42.3, motor_len=40.0, boss_d=22.0, boss_h=2.0, shaft_d=5.0, shaft_len=24.0,
    hole_pitch=31.0,
    # layout
    hotplate_half=100,  # everything mechanical stays behind x = this + tower_gap
    tower_gap=10,
    tip_z=107,          # lower tube tips, with the carriage in the middle of its travel
    pitch=40,           # tube spacing
    upper_back=58,      # Y of the upper tubes
    upper_rise=211,     # upper tube mouths sit this far above the lower tube tips
    pour_angle=135,
    pour_drop=16,       # tipped mouths end up this far above the funnel rim
    rock_angle=15,
    # height: printed pins drop through the post into the carriage, 10 mm steps
    height_step=10, steps_down=4, steps_up=2,
    ratchet_z=160,      # top of the click tooth on the post
    # funnels: click into the lower tube mouths
    funnel_top_d=38, funnel_h=22,
    # printed details
    print_max=180,      # every printed part fits in this cube
    wall=5, plate_t=4, clip_t=2.6, clip_h=14, clip_squeeze=0.25, clip_open=118,
    web_t=8, flange_t=5, flange_depth=48,
    base_t=8,
    base_extra_x=50,    # flat space at the back of the base for weights
)


# ---------------------------------------------------------------- primitives

def box(x0, y0, z0, x1, y1, z1):
    return ('box', (min(x0, x1), min(y0, y1), min(z0, z1)), (max(x0, x1), max(y0, y1), max(z0, z1)))


def obox(centre, u, v, size):
    return ('obox', tuple(centre), tuple(u), tuple(v), tuple(size))


def cyl(p, q, r, r2=None):
    return ('cyl', tuple(p), tuple(q), r, r if r2 is None else r2)


def group(ops):
    return ('group', ops)


def xcyl(x0, x1, y, z, r):
    return cyl((x0, y, z), (x1, y, z), r)


def ycyl(x, y0, y1, z, r):
    return cyl((x, y0, z), (x, y1, z), r)


def zcyl(x, y, z0, z1, r, r2=None):
    return cyl((x, y, z0), (x, y, z1), r, r2)


def rbox(x0, y0, z0, x1, y1, z1, r):
    """Box with its four vertical edges rounded."""
    return group([
        ('+', box(x0 + r, y0, z0, x1 - r, y1, z1)),
        ('+', box(x0, y0 + r, z0, x1, y1 - r, z1)),
    ] + [('+', zcyl(x, y, z0, z1, r)) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)])


def rbox_x(x0, x1, y0, y1, z0, z1, r):
    """Window through X: rounded rectangle in Y-Z."""
    r = min(r, (y1 - y0) / 2, (z1 - z0) / 2)
    return group([('+', box(x0, y0 + r, z0, x1, y1 - r, z1)),
                  ('+', box(x0, y0, z0 + r, x1, y1, z1 - r))] +
                 [('+', xcyl(x0, x1, y, z, r)) for y in (y0 + r, y1 - r) for z in (z0 + r, z1 - r)])


def rwin_y(x0, x1, y0, y1, z0, z1, r):
    """Window through Y: rounded rectangle in X-Z."""
    r = min(r, (x1 - x0) / 2, (z1 - z0) / 2)
    return group([('+', box(x0 + r, y0, z0, x1 - r, y1, z1)),
                  ('+', box(x0, y0, z0 + r, x1, y1, z1 - r))] +
                 [('+', ycyl(x, y0, y1, z, r)) for x in (x0 + r, x1 - r) for z in (z0 + r, z1 - r)])


def norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def wedge_xz(cx, cz, y0, y1, ang, half):
    """Pie slice in the X-Z plane around direction ang (degrees from +X towards +Z)."""
    ops = []
    bis = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
    for side in (1, -1):
        a = math.radians(ang + side * half)
        b = (math.cos(a), 0.0, math.sin(a))
        n = (-b[2], 0.0, b[0])
        if n[0] * bis[0] + n[2] * bis[1] < 0:
            n = (-n[0], 0.0, -n[2])
        c = (cx + n[0] * 40, (y0 + y1) / 2, cz + n[2] * 40)
        ops.append(('+' if not ops else '&', obox(c, b, n, (200, 80, y1 - y0))))
    return group(ops)


def wedge_xy(cx, cy, z0, z1, ang, half):
    """Pie slice around direction ang (degrees, in XY), half-angle half."""
    ops = []
    for side in (1, -1):
        a = math.radians(ang + side * half)
        b = (math.cos(a), math.sin(a), 0.0)                 # boundary line
        n = (-side * b[1], side * b[0], 0.0)                 # towards the inside
        # make sure n points towards the bisector
        bis = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
        if n[0] * bis[0] + n[1] * bis[1] < 0:
            n = (-n[0], -n[1], 0.0)
        c = (cx + n[0] * 40, cy + n[1] * 40, (z0 + z1) / 2)
        ops.append(('+' if not ops else '&', obox(c, b, n, (200, 80, z1 - z0))))
    return group(ops)


def c_clip(cx, cy, z0, z1, r_in, t, opening_deg, half):
    """C-shaped snap clip around a tube, open towards opening_deg."""
    ring = group([
        ('+', zcyl(cx, cy, z0, z1, r_in + t)),
        ('-', zcyl(cx, cy, z0 - 1, z1 + 1, r_in)),
        ('-', wedge_xy(cx, cy, z0 - 1, z1 + 1, opening_deg, half)),
    ])
    ops = [('+', ring)]
    for side in (1, -1):  # rounded tips
        a = math.radians(opening_deg + side * half)
        rm = r_in + t / 2
        ops.append(('+', zcyl(cx + rm * math.cos(a), cy + rm * math.sin(a), z0, z1, t / 2)))
    return ops


def gusset_xz(x0, z0, a, b, y0, y1):
    """Right-angle gusset in the X-Z plane: legs a along +X and b along +Z."""
    h = norm((-a, 0, b))
    n = norm((-b, 0, -a))  # points back towards the corner
    mid = (x0 + a / 2, (y0 + y1) / 2, z0 + b / 2)
    c = (mid[0] + n[0] * 100, mid[1], mid[2] + n[2] * 100)
    return group([('+', box(x0, y0, z0, x0 + a, y1, z0 + b)),
                  ('&', obox(c, h, n, (600, 200, y1 - y0)))])


def gusset_yz(y0, z0, a, b, x0, x1):
    """Right-angle gusset in the Y-Z plane: legs a along Y (sign gives direction) and b along +Z."""
    s = 1 if a > 0 else -1
    a = abs(a)
    h = norm((0, -s * a, b))
    n = norm((0, -s * b, -a))
    mid = ((x0 + x1) / 2, y0 + s * a / 2, z0 + b / 2)
    c = (mid[0], mid[1] + n[1] * 100, mid[2] + n[2] * 100)
    return group([('+', box(x0, y0, z0, x1, y0 + s * a, z0 + b)),
                  ('&', obox(c, h, n, (600, 200, x1 - x0)))])


def teardrop_x(x0, x1, y, z, r, up=1):
    """Horizontal hole along X that prints without support (point towards up)."""
    d = r * math.sqrt(2)
    diamond = obox(((x0 + x1) / 2, y, z), (1, 0, 0),
                   (0, math.cos(math.pi / 4), math.sin(math.pi / 4)), (x1 - x0, 2 * r, 2 * r))
    top = box(x0 - 1, y - d, z, x1 + 1, y + d, z + up * (d + 1))
    return group([('+', xcyl(x0, x1, y, z, r)),
                  ('+', group([('+', diamond), ('&', top)]))])


def vgroove_x(x0, x1, y, z, depth):
    """V-groove along X on a face whose normal is +-Y (diamond rod)."""
    s = depth * math.sqrt(2)
    return obox(((x0 + x1) / 2, y, z), (1, 0, 0),
                (0, math.cos(math.pi / 4), math.sin(math.pi / 4)), (x1 - x0, s, s))


def pour_hinge(m, f, deg):
    """Hinge point that carries the tube mouth m onto the pour point f
    when the rack turns by deg about X. Solves f - A = R(m - A)."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    rm = (c * m[0] - s * m[1], s * m[0] + c * m[1])
    bx, by = f[0] - rm[0], f[1] - rm[1]
    a11, a12, a21, a22 = 1 - c, s, -s, 1 - c
    det = a11 * a22 - a12 * a21
    return ((a22 * bx - a12 * by) / det, (-a21 * bx + a11 * by) / det)


# ---------------------------------------------------------------- the rig

def tube_r_at(depth):
    """Outside radius of the tube this far below the mouth (straight part)."""
    return P['tube_r0'] - (P['tube_r0'] - P['tube_r1']) * depth / P['tube_cyl']


def motor_matrix(origin):
    """Motor file: face at z=0, shaft +Z. Placed with the shaft pointing -X."""
    return dict(origin=origin, x=(0, 1, 0), y=(0, 0, -1), z=(-1, 0, 0))


def tube_matrix(origin):
    """Tube file: mouth at the origin, body along -Y. Placed mouth up."""
    return dict(origin=origin, x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0))


def bought_ops(kind):
    """Stand-in geometry for a bought part, in that part's own file coordinates."""
    if kind == 'motor':
        h = P['motor_w'] / 2
        ops = [('+', box(-h, -h, -P['motor_len'], h, h, 0))]
        for sx in (-1, 1):  # chamfered corners
            for sy in (-1, 1):
                ops.append(('-', obox((sx * h, sy * h, -P['motor_len'] / 2), (sx * 0.7071, sy * 0.7071, 0),
                                      (-sy * 0.7071, sx * 0.7071, 0), (6.4, 20, P['motor_len'] + 2))))
        ops.append(('+', zcyl(0, 0, -0.1, P['boss_h'], P['boss_d'] / 2)))
        ops.append(('+', zcyl(0, 0, 0, P['shaft_len'], P['shaft_d'] / 2)))
        return ops
    r0, r1, rt = P['tube_r0'], P['tube_r1'], P['tube_rtip']
    c, L, wl = P['tube_cyl'], P['tube_len'], P['tube_wall']
    return [('+', cyl((0, -c, 0), (0, 0, 0), r1, r0)),
            ('+', cyl((0, -L, 0), (0, -c + 0.01, 0), rt, r1)),
            ('-', cyl((0, -c, 0), (0, 1, 0), r1 - wl, r0 - wl)),
            ('-', cyl((0, -L + wl, 0), (0, -c + 0.02, 0), max(rt - wl, 0.5), r1 - wl))]


def build(params=None):
    if params:
        P.update(params)
    R = P['tube_r0']
    w = P['wall']
    xs = [-P['pitch'], 0.0, P['pitch']]
    k = (P['tube_r1'] - P['tube_rtip']) / (P['tube_len'] - P['tube_cyl'])  # cone slope
    half = P['clip_open'] / 2
    mh = P['motor_w'] / 2

    tip = P['tip_z']
    mouth = tip + P['tube_len']
    xt0 = P['hotplate_half'] + P['tower_gap']      # carriage web, front face
    xt1 = xt0 + P['web_t']
    bore_r = (P['shaft_d'] + 0.2) / 2
    parts, bought = [], []

    def hub(pivot, y_floor=None):
        """Hub on the motor shaft. Its nose reaches into the motor hole in the
        carriage so it grips about 21 mm of the 24 mm shaft."""
        y, z = pivot
        nose = xcyl(xt0 - 4, xt0 + 5.5, y, z, 10)
        if y_floor is not None:  # flatten the nose where it would stick out of the part
            nose = group([('+', nose), ('&', box(xt0 - 5, y_floor, z - 11, xt0 + 6, y + 11, z + 11))])
        return [('+', nose),
                ('-', xcyl(xt0 - 20, xt0 + 6, y, z, bore_r)),
                ('-', zcyl(xt0 - 10, y, z, z + 12, 1.3)),     # M3 set screw, from above
                ('-', ycyl(xt0 - 10, y - 12, y, z, 1.3))]     # and from the front

    k = (P['tube_r1'] - P['tube_rtip']) / (P['tube_len'] - P['tube_cyl'])  # tube cone slope
    pt = P['plate_t']
    hx = P['pitch'] + R + 4                                             # rack half length

    def seat(x, y, z_tip):
        """Cone seat for the tube tip, open at the bottom."""
        cl = 0.4
        return [('-', zcyl(x, y, z_tip - 1, z_tip + 11, P['tube_rtip'] + cl - k, P['tube_rtip'] + cl + 11 * k)),
                ('-', zcyl(x, y, z_tip - 6, z_tip + 2, 6.5))]

    def cup(x, y, z_tip, y_plate):
        """Ring under each tube tip, braced to the plate like the clips."""
        r_out, z0, z1 = 13.0, z_tip - 3, z_tip + 8
        brace = 2 * r_out * math.sin(math.radians(40))
        return [('+', zcyl(x, y, z0, z1, r_out)),
                ('+', box(x - brace / 2, min(y, y_plate), z0, x + brace / 2, max(y, y_plate), z1))]

    def clip(x, y, z0, r_in, opening, y_plate):
        """Snap clip: thin at the open tips, thicker towards the back, braced to
        the plate on both sides so it can't splay."""
        h, t, e = P['clip_h'], P['clip_t'], 1.2
        s_ = 1 if y_plate > y else -1
        r_out = r_in + t + e
        t_tip = t + e * (1 + math.cos(math.radians(180 - half)))
        brace = 2 * r_out * math.sin(math.radians(40))
        ops = [('+', group([('+', zcyl(x, y + s_ * e, z0, z0 + h, r_out)),
                            ('+', box(x - brace / 2, min(y, y_plate), z0, x + brace / 2, max(y, y_plate), z0 + h)),
                            ('-', zcyl(x, y, z0 - 1, z0 + h + 1, r_in)),
                            ('-', wedge_xy(x, y, z0 - 1, z0 + h + 1, opening, half))]))]
        for side in (1, -1):                                            # rounded tips
            a = math.radians(opening + side * half)
            rm = r_in + t_tip / 2
            ops.append(('+', zcyl(x + rm * math.cos(a), y + rm * math.sin(a), z0, z0 + h, t_tip / 2)))
        return ops

    # -- lower rack: back plate, one clip row, open seats; the funnel hinges on its arm
    lp = (0.0, mouth - 9)
    ys0, ys1 = R + 1.5, R + 1.5 + pt
    zb = tip - 3
    row = mouth - 27
    ops = [('+', rwin_y(-hx, hx, ys0, ys1, zb, mouth - 2, 8)),           # back plate
           ('+', rbox_x(hx - 14, xt0 - 3, -10, ys1, lp[1] - 10, lp[1] + 10, 6))]   # arm
    for x in xs:
        ops += cup(x, 0.0, tip, ys0 + 0.5)
    for x in xs:
        ops += clip(x, 0.0, row, tube_r_at(19) - P['clip_squeeze'], -90, ys0 + 0.5)
    ops += hub(lp)
    for x in xs:
        ops += seat(x, 0.0, tip)
    # funnel hinge: a pin on two posts on top of the arm
    fx, fz = hx + 7, lp[1] + 16                                         # hinge axis (along Y)
    for y0_, y1_ in ((-10, -5), (15, 20)):
        ops.append(('+', box(fx - 4.5, y0_, lp[1] + 9, fx + 4.5, y1_, fz)))
        ops.append(('+', ycyl(fx, y0_, y1_, fz, 4.5)))
    ops.append(('+', ycyl(fx, -5.5, 15.5, fz, 2.5)))
    for xg in (-P['pitch'] / 2, P['pitch'] / 2):                         # mm scale on the plate
        z, n = tip + 12, 0
        while z < row - 2:
            L = 7 if n % 2 == 0 else 4
            ops.append(('-', box(xg - L / 2, ys0 - 0.1, z - 0.45, xg + L / 2, ys0 + 0.7, z + 0.45)))
            z += 5
            n += 1
    parts.append(dict(name='Lower rack', asm='Lower rack assembly', color='#1d1d1f', ops=ops,
                      print_rot=('x', -90), note='print lying on its back'))

    # -- funnel strip: rests on the tube rims, flips up on the arm hinge to load tubes
    ft = mouth + P['funnel_h']
    f_in_bot, f_in_top = 10.0, P['funnel_top_d'] / 2
    f_out_top = f_in_top + 1.6
    slope = (f_in_top - f_in_bot) / P['funnel_h']
    fl_x0, fl_x1 = -hx - 4, fx
    ops = [('+', zcyl(x, 0, mouth, ft, R + 1.2, f_out_top)) for x in xs]
    ops.append(('+', rbox(fl_x0, -f_out_top - 2, ft - 2.5, fl_x1 - 4, f_out_top + 2, ft, 6)))
    ops.append(('+', rbox(fl_x0 - 12, -9, ft - 2.5, fl_x0 + 2, 9, ft, 8)))           # lift tab
    ops.append(('+', box(fx - 9, -4.4, fz, fx + 3, 14.4, ft)))                       # hinge web
    ops.append(('+', ycyl(fx, -4.4, 14.4, fz, 5.4)))                                 # hinge knuckle
    ops += [('-', zcyl(x, 0, mouth - 1, ft + 0.5, f_in_bot - slope, f_in_top + 0.5 * slope)) for x in xs]
    ops.append(('-', ycyl(fx, -5, 15, fz, 2.7)))                                     # pin hole
    ops.append(('-', wedge_xz(fx, fz, -5, 15, -90, 52)))                             # snap opening
    parts.append(dict(name='Funnel strip', asm='Lower rack assembly', color='#e8e8e3', ops=ops,
                      print_rot=('x', 180), note='print upside down'))

    for i, x in enumerate(xs):
        bought.append(dict(name='Lower tube %d' % (i + 1), asm='Lower rack assembly',
                           kind='tube', place=tube_matrix((x, 0.0, mouth))))

    # -- upper rack: same style, two clip rows, clips open to the back
    uy, um = P['upper_back'], tip + P['upper_rise']
    utip = um - P['tube_len']
    pour_point = (0.0, ft + P['pour_drop'])
    up = pour_hinge((uy, um), pour_point, P['pour_angle'])
    uys1 = uy - R - 1.5
    uys0 = uys1 - pt
    ops = [('+', rwin_y(-hx, hx, uys0, uys1, utip - 3, um - 10, 8)),     # back plate
           ('+', rbox_x(hx - 14, xt0 - 3, uys0, up[0] + 10, up[1] - 10, up[1] + 10, 6))]   # arm
    for x in xs:
        ops += cup(x, uy, utip, uys1 - 0.5)
    for z0 in (um - 27, um - 82):
        rc = tube_r_at(um - z0 - P['clip_h'] / 2) - P['clip_squeeze']
        for x in xs:
            ops += clip(x, uy, z0, rc, 90, uys1 - 0.5)
    ops += hub(up, y_floor=uys0)
    for x in xs:
        ops += seat(x, uy, utip)
    parts.append(dict(name='Upper rack', asm='Upper rack assembly', color='#f2f2ee', ops=ops,
                      print_rot=('x', 90), note='print lying on its front'))

    for i, x in enumerate(xs):
        bought.append(dict(name='Upper tube %d' % (i + 1), asm='Upper rack assembly',
                           kind='tube', place=tube_matrix((x, uy, um))))

    # -- carriage: U-channel holding both motors, slides inside the post
    y_lo = min(lp[0], up[0]) - mh - 3.85 - P['flange_t']
    y_hi = max(lp[0], up[0]) + mh + 3.85 + P['flange_t']
    xf = xt0 + P['flange_depth']
    zc1 = up[1] + mh + 5
    zc0 = zc1 - (P['print_max'] - 3)
    ft_ = P['flange_t']
    ops = [
        ('+', box(xt0, y_lo, zc0, xt1, y_hi, zc1)),                     # web
        ('+', box(xt0, y_lo, zc0, xf, y_lo + ft_, zc1)),                # flanges
        ('+', box(xt0, y_hi - ft_, zc0, xf, y_hi, zc1)),
        ('+', box(xt0, y_lo, zc0, xf, y_hi, zc0 + 4)),                  # floor, makes it a box
    ]
    # motor sockets: the motor pushes in from the back and is held by crush ribs
    sock_in, sock_out, sock_d = mh + 0.25, mh + 3.25, 22
    for y, z in (lp, up):
        for a0, a1 in ((sock_in, sock_out), (-sock_out, -sock_in)):
            ops.append(('+', box(xt1 - 0.5, y + a0, z - sock_out, xt1 + sock_d, y + a1, z + sock_out)))
            ops.append(('+', box(xt1 - 0.5, y - sock_out, z + a0, xt1 + sock_d, y + sock_out, z + a1)))
        rc = sock_in + 1.0 - 0.45                                       # ribs bite 0.45 mm into the motor
        for sgn in (1, -1):
            for off in (-10, 10):
                for py, pz in ((y + sgn * rc, z + off), (y + off, z + sgn * rc)):
                    ops.append(('+', xcyl(xt1 + 1, xt1 + sock_d - 3, py, pz, 1.0)))
                    ops.append(('+', cyl((xt1 + sock_d - 3, py, pz), (xt1 + sock_d, py, pz), 1.0, 0.4)))  # lead-in
    for y, z in (lp, up):
        ops.append(('-', xcyl(xt0 - 1, xt1 + 1, y, z, P['boss_d'] / 2 + 0.3)))
        for dz in (-17, 17):                                            # push-out holes to remove the motor
            ops.append(('-', xcyl(xt0 - 1, xt1 + 1, y, z + dz, 2.5)))
        for dy in (-1, 1):
            for dz in (-1, 1):
                hy, hz = y + dy * P['hole_pitch'] / 2, z + dz * P['hole_pitch'] / 2
                ops.append(('-', xcyl(xt0 - 1, xt1 + 1, hy, hz, 1.7)))
                ops.append(('-', xcyl(xt0 - 1, xt0 + 3, hy, hz, 3.2)))   # counterbore
    step = P['height_step']
    heights = [k * step for k in range(-P['steps_down'], P['steps_up'] + 1)]
    rx, rz = (xt0 + xf) / 2, P['ratchet_z']
    for fy0, fy1 in ((y_lo - 1, y_lo + ft_ + 1), (y_hi - ft_ - 1, y_hi + 1)):
        for dz in heights:                                              # click windows
            ops.append(('-', box(rx - 6, fy0, rz - dz - 6.4, rx + 6, fy1, rz - dz + 0.4)))
        top, bot = rz - min(heights) + 8, zc1 - 14                       # rounded windows above them,
        n_w, gap = 3, 6                                                 # split in 3 so each prints flat
        hw = (bot - top - gap * (n_w - 1)) / n_w
        for i in range(n_w):
            z0 = top + i * (hw + gap)
            ops.append(('-', rwin_y(xt0 + 14, xf - 8, fy0, fy1, z0, z0 + hw, 6)))
    for (y, z), (oy, oz) in ((lp, up), (up, lp)):                       # web windows beside motors
        wy0 = (y + mh + 4) if oy > y else (oy - mh + 4)
        wy1 = (oy + mh - 4) if oy > y else (y - mh - 4)
        wz0, wz1 = (z - mh + 4, oz - mh - 4) if oz > z else (oz + mh + 4, z + mh - 4)
        if wy1 - wy0 > 12 and wz1 - wz0 > 12:
            ops.append(('-', rbox_x(xt0 - 1, xt1 + 1, max(wy0, y_lo + ft_ + 3), min(wy1, y_hi - ft_ - 3),
                                    wz0, wz1, 5)))
    sk0, sk1 = zc0 + 14, lp[1] - mh - 10                                # skirt window in the web
    if sk1 - sk0 > 16:
        ops.append(('-', rbox_x(xt0 - 1, xt1 + 1, y_lo + ft_ + 6, y_hi - ft_ - 6, sk0, sk1, 7)))
    parts.append(dict(name='Carriage', asm='Carriage assembly', color='#3a3f45', ops=ops,
                      print_rot=('y', -90), note='print lying on the motor plate'))

    for name, (y, z) in (('Rock motor', lp), ('Pour motor', up)):
        bought.append(dict(name=name, asm='Carriage assembly', kind='motor',
                           place=motor_matrix((xt1, y, z))))

    cl = 0.3
    cy0, cy1 = y_lo - cl - 6, y_hi + cl + 6                             # post cheeks, outer faces

    # -- base and post, one print. Lift the carriage and it clicks up a step;
    #    pull both tabs out to let it slide down.
    post_top = P['print_max'] - 0.5
    xb = xf + 2                                                         # back web front face
    yc = (y_lo + y_hi) / 2
    bt = P['base_t']
    pm = P['print_max'] - 4
    bx0, bx1 = xt0 - 5, xt0 - 5 + pm + P['base_extra_x']
    g_len = xt0 - 5 + pm - xb - 12                                      # braces keep their size
    by0, by1 = yc - pm / 2, yc + pm / 2
    ops = [
        ('+', rbox(bx0, by0, 0, bx1, by1, bt, 12)),                     # base plate
        ('+', box(xt0, cy0, 0, xb + 8, y_lo - cl, post_top)),           # cheeks
        ('+', box(xt0, y_hi + cl, 0, xb + 8, cy1, post_top)),
        ('+', box(xb, cy0, 0, xb + 8, cy1, post_top)),                  # back web
        ('+', gusset_xz(xb + 7, bt - 1, g_len, 95, cy0, cy0 + 5)),
        ('+', gusset_xz(xb + 7, bt - 1, g_len, 95, cy1 - 5, cy1)),
        ('+', gusset_xz(xb + 7, bt - 1, g_len, 95, yc - 2.5, yc + 2.5)),
    ]
    ops.append(('-', box(xt0 - 1, y_lo - cl, bt, xb, y_hi + cl, post_top + 1)))       # carriage room
    for gy0, gy1, ty0, ty1 in ((y_lo - cl - 0.1, y_lo + P['flange_t'] + 1, cy0, y_lo + P['flange_t'] + 4),
                               (y_hi - P['flange_t'] - 1, y_hi + cl + 0.1, y_hi - P['flange_t'] - 4, cy1)):
        ops.append(('+', box(xf + cl, gy0, bt, xb + 0.5, gy1, post_top)))           # back guides
        ops.append(('+', box(xt0 - 5, ty0, post_top - 14, xt0 - cl, ty1, post_top)))  # front lips
    for wy0, wy1 in ((y_lo + P['flange_t'] + 6, yc - 8), (yc + 8, y_hi - P['flange_t'] - 6)):
        ops.append(('-', rbox_x(xb - 1, xb + 9, wy0, wy1, bt + 30, post_top - 55, 6)))   # back web windows
    for cyy0, cyy1 in ((cy0 - 1, y_lo - cl + 0.5), (y_hi + cl - 0.5, cy1 + 1)):
        ops.append(('-', rwin_y(xt0 + 8, xb - 4, cyy0, cyy1, bt + 12, rz - 62, 6)))  # cheek windows
    low = zc0 + min(heights)                                            # bottom stop: one small ledge
    n45 = norm((1, 0, 1))                                               # 45 deg underside, prints clean
    ops.append(('+', group([('+', box(xf - 10, yc - 9, low - 13, xb + 1, yc + 9, low - 0.2)),
                            ('&', obox((xf - 10 + n45[0] * 200, yc, low - 0.2 + n45[2] * 200),
                                       norm((1, 0, -1)), n45, (400, 400, 30)))])))
    for side in (-1, 1):                                                # click pawls
        face = y_lo - cl if side < 0 else y_hi + cl                     # cheek inner face
        outer = cy0 if side < 0 else cy1
        out = side                                                      # direction away from carriage
        ops.append(('-', box(rx - 8, min(face, outer) - 1, rz - 55, rx + 8, max(face, outer) + 1, post_top + 1)))
        t0 = face + out * 3                                             # flexible tongue, 3 thick
        ops.append(('+', box(rx - 6.5, min(face, t0), rz - 57, rx + 6.5, max(face, t0), rz + 6)))
        tip = face - out * 4.8                                          # tooth reaches into the carriage
        n = norm((0, out, 1))
        ops.append(('+', group([('+', box(rx - 5, min(face + out * 0.5, tip), rz - 6, rx + 5, max(face + out * 0.5, tip), rz)),
                                ('&', obox((rx, face + n[1] * 20, rz - 6 + n[2] * 20), (1, 0, 0), n, (30, 40, 40)))])))
        tab_out = outer + out * 7                                       # finger tab past the cheek
        n2 = norm((0, -out, 1))
        ops.append(('+', group([('+', box(rx - 7, min(face, tab_out), rz + 6, rx + 7, max(face, tab_out), post_top - 0.5)),
                                ('&', obox((rx, t0 + n2[1] * 20, rz + 6 + n2[2] * 20), (1, 0, 0), n2, (30, 40, 40)))])))
    for wy0, wy1 in ((y_lo + 9, y_hi - 9),):                            # base plate windows
        ops.append(('-', rbox(xt0 + 6, wy0, -1, xf - 6, wy1, bt + 1, 6)))
    for wy0, wy1 in ((by0 + 10, cy0 - 8), (cy1 + 8, by1 - 10)):
        if wy1 - wy0 > 14:
            ops.append(('-', rbox(bx0 + 10, wy0, -1, xb, wy1, bt + 1, 6)))
    parts.append(dict(name='Base', asm='Base assembly', color='#3a3f45', ops=ops,
                      print_rot=None, note='print flat, one piece with the post'))

    hinges = [
        dict(name='Rock hinge', asm='Lower rack assembly', part='Lower rack', pivot=lp,
             bore=(xt0 + 5.5, bore_r), limits=(-P['rock_angle'], P['rock_angle'])),
        dict(name='Pour hinge', asm='Upper rack assembly', part='Upper rack', pivot=up,
             bore=(xt0 + 5.5, bore_r), limits=(0, P['pour_angle'])),
        dict(name='Funnel hinge', asm='Lower rack assembly', part='Funnel strip', axis='y',
             origin=(fx, 5.0, fz), limits=(0, 100), parent='Rock hinge', members=['Funnel strip']),
    ]
    info = dict(lower_pivot=lp, upper_pivot=up, pour_point=pour_point, mouth=mouth,
                funnel_top=ft, carriage_z=(zc0, zc1), post_top=post_top, heights=heights,
                height_step=step, frame_front_x=xt0)
    for b in bought:
        b['ops'] = bought_ops(b['kind'])
    return dict(params=dict(P), parts=parts, bought=bought, hinges=hinges, info=info)


if __name__ == '__main__':
    rig = build()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rig_recipe.json')
    with open(out, 'w') as f:
        json.dump(rig, f, indent=1)
    print('wrote', out)
    print(json.dumps(rig['info'], indent=1))
