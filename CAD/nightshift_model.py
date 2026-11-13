"""
NIGHTSHIFT -- Rev Max powered 2-seat sport UTV, concept layout model.
Units: millimeters.  Axes: X = forward, Y = left (driver side), Z = up.
Rear axle centerline at X=0, ground at Z=0.

Change the numbers in PARAMS and re-run to resize the whole machine.
"""
import math
import cadquery as cq
import nightshift_geometry as geometry

# ---------------------------------------------------------------- PARAMS
IN = 25.4
P = dict(
    wheelbase=2400,          # 94.5 in
    track=64 * IN,           # 64 in tire center to center -> 74 in overall (RZR Pro R width)
    tire_dia=32 * IN,        # 32 x 10-15: the biggest tire that clears the frame at full steering lock
    tire_w=10 * IN,
    rim_dia=15 * IN,
    tube_main=1.75 * IN,     # 1.75 x .120 DOM main frame/cage
    tube_sec=1.5 * IN,       # 1.5 in secondary tubes
    tube_wall=0.120 * IN,
    floor_z=330,             # bottom of lower frame rails (ground clearance)
)
# Stretch: extra length added ahead of the seats (cabin) and in the front clip (nose).
# Design coordinates below stay the same; SX() slides everything forward of each
# break point when the shapes are built.
CABIN_STRETCH = 12 * IN
NOSE_STRETCH = 2 * IN
CABIN_BREAK, NOSE_BREAK = 1750, 2100


def SX(x):
    if x >= NOSE_BREAK:
        return x + CABIN_STRETCH + NOSE_STRETCH
    if x >= CABIN_BREAK:
        return x + CABIN_STRETCH
    return x


def X(p):
    return (SX(p[0]), p[1], p[2])


# Ride height: the whole chassis is raised by LIFT; wheels, hubs and rear axle stay put,
# so every suspension point that rides with a wheel is written as (height - LIFT).
LIFT = 3 * IN
R_TIRE = P["tire_dia"] / 2
WB = P["wheelbase"]
HT = P["track"] / 2
FZ = P["floor_z"] + P["tube_main"] / 2   # lower rail centerline height

assert abs(geometry.R - R_TIRE) < 1 and abs(geometry.HALF_TRACK - HT) < 1 and abs(geometry.H - (P["floor_z"] + LIFT)) < 1
HP, GEO = geometry.final()                # front suspension hardpoints (ground coords, left corner)
ENG_Y = -8 * IN                           # the whole drivetrain sits 8 in right of center
PIV = (340, 440)                          # swingarm pivot = jackshaft centerline (x, z)


def G(p, s=1):
    """Front-corner ground coordinates -> model coordinates (s = 1 left, -1 right)."""
    return (WB + p[0], s * p[1], p[2] - LIFT)


UPZ = HP["PUf"][2] - LIFT - 28            # upper nose rail height: upper arm pivots sit on its outer face
TUBE_LEN, PANEL_AREA, _area = {}, {}, [0.0]   # running totals for the weight estimate
CUT, _pending, _quiet = [], [], [0]           # every frame/cage tube, for the cut list

parts = {}   # name -> (shape, color)


def add(name, shape, color):
    parts.setdefault(name, ([], color))[0].append(shape)
    if _area[0]:
        PANEL_AREA[name] = PANEL_AREA.get(name, 0.0) + _area[0]
        _area[0] = 0.0
    if name in ("frame", "cage"):
        CUT.extend((name, od, pts) for od, pts in _pending)
    del _pending[:]


# ---------------------------------------------------------------- helpers
def tube(p1, p2, od, wall=None, solid=False, mapped=False):
    """Round tube between two 3D points."""
    if not mapped:
        p1, p2 = X(p1), X(p2)
    v = cq.Vector(*p2) - cq.Vector(*p1)
    L = v.Length
    TUBE_LEN[round(od, 2)] = TUBE_LEN.get(round(od, 2), 0.0) + L
    if not _quiet[0]:
        _pending.append((od, [tuple(p1), tuple(p2)]))
    d = v.normalized()
    plane = cq.Plane(origin=cq.Vector(*p1), xDir=_perp(d), normal=d)
    w = cq.Workplane(plane).circle(od / 2)
    if not solid:
        w = w.circle(od / 2 - (wall or P["tube_wall"]))
    return w.extrude(L).val()


def _perp(d):
    a = cq.Vector(0, 0, 1) if abs(d.z) < 0.9 else cq.Vector(1, 0, 0)
    return d.cross(a).normalized()


def path(points, od, wall=None):
    """Bent tube through a list of points. Each corner gets a real bend
    (about 3.2 x tube OD centerline radius, like a DOM bender die)."""
    pts = _bend([X(p) for p in points], 3.2 * od)
    _quiet[0] += 1
    s = [tube(a, b, od, wall, solid=True, mapped=True) for a, b in zip(pts[:-1], pts[1:])]
    _quiet[0] -= 1
    _pending.append((od, [X(p) for p in points]))
    s += [cq.Solid.makeSphere(od / 2, cq.Vector(*p)) for p in pts[1:-1]]
    return cq.Compound.makeCompound(s)


def _bend(points, r, steps=7):
    V = [cq.Vector(*p) for p in points]
    out = [V[0]]
    for i in range(1, len(V) - 1):
        A, B, C = V[i - 1], V[i], V[i + 1]
        u, v = (A - B), (C - B)
        la, lc = u.Length, v.Length
        u, v = u.normalized(), v.normalized()
        cosang = max(-1.0, min(1.0, u.dot(v)))
        ang = math.acos(cosang)
        if ang > math.radians(175):          # basically straight
            out.append(B)
            continue
        t = min(r / math.tan(ang / 2), 0.45 * la, 0.45 * lc)
        P1, P2 = B + u * t, B + v * t
        for k in range(steps + 1):            # quadratic Bezier ~ circular arc
            s = k / steps
            out.append(P1 * (1 - s) ** 2 + B * (2 * s * (1 - s)) + P2 * s ** 2)
    out.append(V[-1])
    # drop zero-length steps
    clean = [out[0]]
    for p in out[1:]:
        if (p - clean[-1]).Length > 0.5:
            clean.append(p)
    return [(p.x, p.y, p.z) for p in clean]


def mirror_y(pts):
    return [(x, -y, z) for (x, y, z) in pts]


def both(name, pts, od, color):
    add(name, path(pts, od), color)
    add(name, path(mirror_y(pts), od), color)


def box(cx, cy, cz, lx, ly, lz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((SX(cx), cy, cz)).val()


def ycyl(cx, cy, cz, r, w):
    """Cylinder whose axis runs along Y (wheels, sprockets, axle)."""
    return (cq.Workplane("XZ").circle(r).extrude(w / 2, both=True)
            .translate((SX(cx), cy, cz)).val())


# ================================================================ FRAME
MAIN, SEC = P["tube_main"], P["tube_sec"]
C_FRAME = (0.06, 0.06, 0.07)        # satin black
C_ORANGE = (0.95, 0.33, 0.05)       # Nightshift orange
C_CAGE = (0.08, 0.08, 0.09)

# Lower main rails: rear kick-up over axle -> cabin floor -> footwell taper -> raised nose rails
lower = [(-300, 350, 720), (150, 350, 720), (520, 430, FZ), (1050, 600, FZ),
         (1880, 600, FZ), (2060, 400, FZ), (2190, 330, UPZ), (2780, 330, 450)]
both("frame", lower, MAIN, C_FRAME)
# Nose keel: two low tubes that carry the lower arm pivots. Narrow, so the lower arms can be long.
KEEL_Y = 165
both("frame", [(2060, KEEL_Y, FZ), (2640, KEEL_Y, FZ), (2780, KEEL_Y, 450)], SEC, C_FRAME)
for xs in (WB + HP["PLr"][0], WB + HP["PLf"][0]):
    add("frame", tube((xs, -KEEL_Y, FZ), (xs, KEEL_Y, FZ), SEC, solid=True), C_FRAME)
    both("frame", [(xs, KEEL_Y, FZ), (xs, 330, UPZ)], SEC, C_FRAME)          # keel-to-rail uprights

# Upper front frame (dash bar height -> front clip)
upper_front = [(1900, 560, 720), (2250, 360, 820), (2780, 330, 720)]
both("frame", upper_front, SEC, C_FRAME)
both("frame", [(2780, 330, 450), (2780, 330, 720)], SEC, C_FRAME)
both("frame", [(1880, 600, FZ), (1900, 560, 720)], MAIN, C_FRAME)     # lower A-pillar

# Front shock towers, triangulated front and back
both("frame", [(2060, 400, FZ), (2330, 380, 900), (2250, 360, 820)], SEC, C_FRAME)
both("frame", [(2330, 380, 900), (2640, 330, UPZ)], SEC, C_FRAME)

# Cross members
for x, y, z in [(-300, 350, 720), (150, 350, 720), (520, 430, FZ), (1050, 600, FZ),
                (1460, 600, FZ), (1880, 600, FZ), (2060, 400, FZ),
                (2780, 330, 450), (2780, 330, 720), (1900, 560, 720), (2330, 380, 900),
                (WB + HP["PUr"][0], 330, UPZ), (2640, 330, UPZ)]:
    add("frame", tube((x, -y, z), (x, y, z), SEC, solid=True), C_FRAME)

# Seat-base / firewall crossbar and engine cradle rails (cradle follows the offset engine)
for yy in (ENG_Y - 150, ENG_Y + 150):
    add("frame", tube((560, yy, FZ), (1080, yy, FZ), SEC, solid=True), C_FRAME)
add("frame", tube((560, -430, FZ), (560, 430, FZ), SEC, solid=True), C_FRAME)
# swingarm pivot mounts: tie the jackshaft/pivot into the floor rail and the rear kick-up rail
both("frame", [(560, 430, FZ), (PIV[0], 345, PIV[1])], SEC, C_FRAME)
both("frame", [(PIV[0], 345, PIV[1]), (300, 383, 565)], SEC, C_FRAME)

# ================================================================ ROLL CAGE
WS_RAKE = 35                                   # windshield hoop lean back from vertical, degrees
WS_TOP_X = round(SX(1900) - (1510 - 720) * math.tan(math.radians(WS_RAKE)))
ROOF_MID_X = round((1070 + WS_TOP_X) / 2)
# Main hoop: one continuous bent tube, narrows toward the roof (tumblehome)
add("cage", path([(1100, 600, FZ), (1095, 595, 1150), (1070, 470, 1530),
                  (1070, -470, 1530), (1095, -595, 1150), (1100, -600, FZ)], MAIN), C_CAGE)
add("cage", tube((1096, -594, 1000), (1096, 594, 1000), SEC, solid=True), C_CAGE)  # harness bar
# Front hoop: raked windshield hoop, one bent tube
add("cage", path([(1900, 560, 720), (WS_TOP_X, 470, 1510),
                  (WS_TOP_X, -470, 1510), (1900, -560, 720)], MAIN), C_CAGE)
# Roof rails with a slight crown, plus an X in the roof
both("cage", [(1070, 470, 1530), (ROOF_MID_X, 490, 1560), (WS_TOP_X, 470, 1510)], MAIN, C_CAGE)
add("cage", tube((1070, 470, 1530), (WS_TOP_X, -470, 1510), SEC, solid=True), C_CAGE)
add("cage", tube((1070, -470, 1530), (WS_TOP_X, 470, 1510), SEC, solid=True), C_CAGE)
# Rear exo: sweeps from the roof down over the swingarm to the rear bumper
both("cage", [(1070, 470, 1530), (380, 410, 1040), (-300, 350, 780), (-300, 350, 720)], MAIN, C_CAGE)
add("cage", tube((330, -405, 1020), (330, 405, 1020), SEC, solid=True), C_CAGE)    # rear shock mount bar
add("cage", tube((-300, -350, 780), (-300, 350, 780), SEC, solid=True), C_CAGE)
# Rear bumper hoop
add("cage", path([(-300, 350, 720), (-415, 300, 910), (-415, -300, 910), (-300, -350, 720)], SEC), C_CAGE)  # taller rear hoop
# Side exoskeleton over the skins (both sides)
both("cage", [(760, 458, 1300), (1095, 600, FZ + 20)], SEC, C_CAGE)            # rear diagonal off the exo
both("cage", [(150, 352, 735), (1095, 600, FZ + 20)], SEC, C_CAGE)             # rear lower, shock to main hoop base
both("cage", [(1900, 566, 720), (1975, 545, 560), (2040, 428, FZ + 25)], SEC, C_CAGE)  # front, dash corner to lower rail
# Bulged door bars (elbow room) and rock sliders
both("cage", [(1100, 600, FZ + 120), (1300, 690, FZ + 330), (1660, 690, FZ + 330),
              (1880, 600, FZ + 120)], SEC, C_CAGE)
both("cage", [(1095, 595, 1150), (1300, 690, FZ + 330)], SEC, C_CAGE)
both("cage", [(1000, 600, FZ), (1080, 700, FZ + 40), (1860, 700, FZ + 40), (1940, 590, FZ + 20)], MAIN, C_CAGE)
for xx in (1300, 1600):
    both("cage", [(xx, 600, FZ), (xx, 700, FZ + 40)], SEC, C_CAGE)
# Front stinger bumper + winch plate
add("cage", path([(2780, 330, 720), (2950, 240, 650), (2950, -240, 650), (2780, -330, 720)], SEC), C_CAGE)
both("cage", [(2780, 330, 450), (2960, 240, 520), (2950, 240, 650)], SEC, C_CAGE)
add("cage", tube((2960, -240, 520), (2960, 240, 520), SEC, solid=True), C_CAGE)
add("cage", box(2930, 0, 585, 120, 480, 8), (0.45, 0.47, 0.50))                    # winch plate
add("winch", box(2930, 0, 640, 140, 380, 110), (0.10, 0.10, 0.11))
add("winch", ycyl(2930, 0, 640, 55, 250), (0.55, 0.56, 0.58))
# Roof light bar
add("lights", box(WS_TOP_X + 30, 0, 1580, 60, 860, 75), (0.10, 0.10, 0.11))
add("lights", box(WS_TOP_X + 62, 0, 1580, 6, 820, 50), (0.85, 0.88, 0.92))
# Small light bar on top of the front stinger bumper tube (about 16 in wide)
add("lights", box(2950, 0, 760, 50, 410, 55), (0.10, 0.10, 0.11))
add("lights", box(2977, 0, 760, 5, 380, 38), (0.85, 0.88, 0.92))
for yy in (-150, 150):                                    # mounting feet to the bumper tube
    add("lights", box(2950, yy, 695, 30, 20, 76), (0.10, 0.10, 0.11))   # risers off the bumper hoop
# Yellow LED cubes (3 in pods) on top of each front shock tower, aimed forward
C_AMBER = (1.0, 0.78, 0.05)
for sgn in (1, -1):
    add("lights", box(2330, sgn * 380, 975, 76, 76, 76), (0.10, 0.10, 0.11))
    add("pod_lenses", box(2370, sgn * 380, 975, 5, 64, 64), C_AMBER)
    add("lights", box(2330, sgn * 380, 925, 30, 30, 26), (0.10, 0.10, 0.11))   # mount stud

# ================================================================ BODY PANELS (0.080 aluminum)
C_BODY = (0.105, 0.11, 0.12)        # carbon fiber panels (2 mm)
C_TRIM = (0.10, 0.10, 0.11)


def panel(pts, t=3):
    """Sheet panel through a list of 3D points (fan-triangulated, so it can be bent)."""
    V = [cq.Vector(*X(p)) for p in pts]
    faces = []
    for i in range(1, len(V) - 1):
        _area[0] += (V[i] - V[0]).cross(V[i + 1] - V[0]).Length / 2
        f = cq.Face.makeFromWires(cq.Wire.makePolygon([V[0], V[i], V[i + 1]], close=True))
        faces.append(f.thicken(t))
    return cq.Compound.makeCompound(faces)


def panel_lr(name, pts, color, t=3):
    add(name, panel(pts, t), color)
    add(name, panel(mirror_y(pts), t), color)


TOP = 24   # sit panels on top of the tubes
# Hood: two facets with a peak at the front shock-tower bar
add("body", panel([(1900, -560, 720 + TOP), (1900, 560, 720 + TOP),
                   (2250, 360, 820 + TOP), (2250, -360, 820 + TOP)]), C_BODY)
add("body", panel([(2250, -360, 820 + TOP), (2250, 360, 820 + TOP),
                   (2780, 330, 720 + TOP), (2780, -330, 720 + TOP)]), C_BODY)
# Hood side cheeks down to the lower front rails
# (the lower edge stops above the upper arms, so the arms and tie rods swing in the open)
panel_lr("body", [(1900, 585, 720), (2250, 385, 820), (2780, 355, 720),
                  (2780, 355, UPZ + 80), (2190, 355, UPZ + 80), (2060, 425, FZ), (1880, 625, FZ)], C_BODY)
# Side skins below the bulged door bars
panel_lr("body", [(1100, 622, FZ), (1880, 622, FZ), (1880, 622, FZ + 120),
                  (1660, 712, FZ + 330), (1300, 712, FZ + 330), (1100, 622, FZ + 120)], C_BODY)
# Roof skin
add("body", panel([(1070, -470, 1530 + TOP), (1070, 470, 1530 + TOP), (ROOF_MID_X, 490, 1560 + TOP),
                   (WS_TOP_X, 470, 1510 + TOP), (WS_TOP_X, -470, 1510 + TOP), (ROOF_MID_X, -490, 1560 + TOP)]), C_BODY)
# ---- Accent stripes (orange) on the roof, hood and side skins
ACC = TOP + 4
for y1, y2 in ((70, 110), (-110, -70)):
    add("accents", panel([(1070, y1, 1530 + ACC), (ROOF_MID_X, y1, 1560 + ACC), (WS_TOP_X, y1, 1510 + ACC),
                          (WS_TOP_X, y2, 1510 + ACC), (ROOF_MID_X, y2, 1560 + ACC), (1070, y2, 1530 + ACC)], 2), C_ORANGE)
    add("accents", panel([(1900, y1, 720 + ACC), (2250, y1, 820 + ACC), (2780, y1, 720 + ACC),
                          (2780, y2, 720 + ACC), (2250, y2, 820 + ACC), (1900, y2, 720 + ACC)], 2), C_ORANGE)
panel_lr("accents", [(1200, 630, FZ + 60), (1290, 630, FZ + 60), (1730, 708, FZ + 300), (1640, 708, FZ + 300)],
         C_ORANGE, 2)

# Side graphic: three orange slashes fanning back across each rear quarter panel
QP = [(1085, 604, FZ + 20), (1085, 594, 1150), (720, 446, 1280), (380, 418, 1046), (150, 356, 742), (520, 436, FZ + 20)]


def _on_panel(i, e, r, sgn):
    """Point on quarter-panel fan triangle i: e = position along its far edge, r = distance out from the front-bottom corner."""
    a, b, c = (cq.Vector(*QP[k]) for k in (0, i, i + 1))
    p = a + ((b + (c - b) * e) - a) * r
    return (p.x, sgn * (p.y + 6), p.z)


for sgn in (1, -1):
    for i, e1, e2, r1, r2 in [(2, 0.30, 0.48, 0.34, 0.97), (3, 0.08, 0.24, 0.30, 0.95), (3, 0.52, 0.66, 0.36, 0.90)]:
        add("accents", panel([_on_panel(i, e1, r1, sgn), _on_panel(i, e1 + 0.04, r2, sgn),
                              _on_panel(i, e2, r2, sgn), _on_panel(i, e2 - 0.05, r1, sgn)], 2), C_ORANGE)

# ---- Quick-release hood pins at every panel corner, so the skins pop off by hand
C_PIN = (0.85, 0.86, 0.88)


def pin(p, d):
    """Hood pin standing off a panel at point p in direction d (unit tuple), with its clip ring."""
    a = cq.Vector(*p)
    v = cq.Vector(*d)
    add("hood_pins", tube(tuple(a), tuple(a + v * 28), 10, solid=True), C_PIN)
    add("hood_pins", tube(tuple(a + v * 3), tuple(a + v * 6), 30, wall=4), C_PIN)          # flush cup
    add("hood_pins", tube(tuple(a + v * 18), tuple(a + v * 21), 24, wall=3), C_ORANGE)    # clip ring


for sgn in (1, -1):
    for p in [(1950, 520, 720 + TOP), (2250, 330, 820 + TOP), (2730, 300, 720 + TOP),   # hood
              (1110, 440, 1530 + TOP), (WS_TOP_X - 40, 440, 1510 + TOP)]:              # roof
        pin((p[0], sgn * p[1], p[2]), (0, 0, 1))
    for p in [(1150, 624, FZ + 60), (1840, 624, FZ + 60), (1330, 708, FZ + 300), (1630, 708, FZ + 300)]:  # side skins
        pin((p[0], sgn * p[1], p[2]), (0, sgn, 0))

# ---- Full-armor extras (only in the ARMOR version)
# half doors filling the door-bar opening up to shoulder height
panel_lr("armor", [(1110, 604, 1000), (1110, 606, FZ + 140), (1300, 698, FZ + 330),
                   (1660, 698, FZ + 330), (1690, 640, 1000)], C_BODY)
# rear quarter panels around the engine bay, rail up to the rear exo
panel_lr("armor", [(1085, 604, FZ + 20), (1085, 594, 1150), (720, 446, 1280), (380, 418, 1046),
                   (150, 356, 742), (520, 436, FZ + 20)], C_BODY)
# rear triangle skins behind the rear shock, exo tube down to the rear frame rail
panel_lr("armor", [(380, 422, 1046), (150, 360, 742), (-300, 358, 722), (-300, 358, 782)], C_BODY)
for sgn in (1, -1):
    for p in [(250, 392, 900), (-250, 362, 760)]:
        pin((p[0], sgn * p[1], p[2]), (0, sgn, 0))
# nose armor plate and rear bash plate
add("armor", panel([(2780, -330, 470), (2780, 330, 470), (2780, 330, 700), (2780, -330, 700)], 5),
    (0.45, 0.47, 0.50))
add("armor", panel([(-405, -290, 770), (-405, 290, 770), (-405, 290, 880), (-405, -290, 880)], 5),
    (0.45, 0.47, 0.50))
for sgn in (1, -1):
    for p in [(1180, 650, 960), (1620, 680, 960), (1060, 600, 600), (480, 432, 980)]:
        pin((p[0], sgn * p[1], p[2]), (0, sgn, 0))

# Firewall behind the seats (keeps heat and noise off your back)
add("interior", panel([(1080, -590, FZ), (1080, 590, FZ), (1080, 590, 1000), (1080, -590, 1000)]), C_TRIM)
# Dash: angled panel back from the dash bar, with a round gauge pod on the driver side
add("interior", panel([(1900, -560, 720), (1900, 560, 720), (1790, 545, 860), (1790, -545, 860)]), C_TRIM)
gauge = (cq.Workplane("YZ").circle(62).extrude(-45).rotate((0, 0, 0), (0, 1, 0), 40)
         .translate((SX(1785), 300, 905)).val())
add("dash", gauge, C_TRIM)
add("dash", (cq.Workplane("YZ").circle(52).extrude(3).rotate((0, 0, 0), (0, 1, 0), 40)
             .translate((SX(1752), 300, 878)).val()), (0.25, 0.45, 0.55))

# Skid plate (3/16 in) under the cabin and engine
skid = (cq.Workplane("XY").polyline([(SX(x), y) for x, y in
                                     [(520, 400), (1050, 560), (1880, 560), (2060, 400),
                                      (2060, -400), (1880, -560), (1050, -560), (520, -400)]])
        .close().extrude(5).translate((0, 0, P["floor_z"] - 5)).val())
add("skid", skid, (0.45, 0.47, 0.50))
add("skid", box(2780, 0, 470, 10, 640, 80), (0.45, 0.47, 0.50))
add("skid", box(2350, 0, P["floor_z"] - 2.5, 580, 2 * KEEL_Y + 40, 5), (0.45, 0.47, 0.50))   # keel skid
SKID_AREA = 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(
    [(SX(x), y) for x, y in [(520, 400), (1050, 560), (1880, 560), (2060, 400), (2060, -400), (1880, -560), (1050, -560), (520, -400)]],
    [(SX(x), y) for x, y in [(1050, 560), (1880, 560), (2060, 400), (2060, -400), (1880, -560), (1050, -560), (520, -400), (520, 400)]]))) \
    + 580 * (2 * KEEL_Y + 40)

# ================================================================ WHEELS
C_TIRE, C_RIM = (0.06, 0.06, 0.06), (0.55, 0.42, 0.22)
for x in (0, WB):
    for s in (1, -1):
        t = (cq.Workplane("XZ").circle(R_TIRE).circle(P["rim_dia"] / 2 - 5)
             .extrude(P["tire_w"] / 2, both=True).edges().fillet(70)
             .translate((SX(x), s * HT, R_TIRE)).val())
        add("tires", t, C_TIRE)
        # tread lugs
        for i in range(18):
            a = i * 20 * math.pi / 180
            lug = (cq.Workplane("XY").box(60, P["tire_w"] * 0.42, 22)
                   .translate((0, (0.22 if i % 2 else -0.22) * P["tire_w"], R_TIRE))
                   .rotate((0, 0, 0), (0, 1, 0), math.degrees(a))
                   .translate((SX(x), s * HT, R_TIRE)).val())
            add("tires", lug, C_TIRE)
        # Original lightweight race-style wheel: deep barrel, 8 straight spokes, 5 lugs
        RR = P["rim_dia"] / 2
        FACE = s * (HT + 58)                                   # outboard face of the spokes
        add("rims", (cq.Workplane("XZ").circle(RR).circle(RR - 22).extrude(100, both=True)
                     .translate((SX(x), s * HT, R_TIRE)).val()), C_RIM)                      # barrel
        add("rims", (cq.Workplane("XZ").circle(RR + 6).circle(RR - 20).extrude(7, both=True)
                     .translate((SX(x), s * (HT + 96), R_TIRE)).val()), C_RIM)               # outer lip
        add("rims", ycyl(x, FACE - s * 6, R_TIRE, 74, 30), C_RIM)                            # hub pad
        for k in range(8):
            spoke = (cq.Workplane("XY").box(RR - 22 - 50, 22, 30).translate((50 + (RR - 72) / 2, 0, 0))
                     .rotate((0, 0, 0), (0, 1, 0), k * 45 + 22.5).translate((SX(x), FACE - s * 6, R_TIRE)).val())
            add("rims", spoke, C_RIM)
        add("lug_nuts", ycyl(x, FACE + s * 14, R_TIRE, 30, 14), (0.08, 0.08, 0.09))          # center cap
        for k in range(5):
            a = math.radians(k * 72 + 18)
            nut = (cq.Workplane("XZ").polygon(6, 21).extrude(9, both=True)
                   .translate((SX(x) + 57 * math.cos(a), FACE + s * 16, R_TIRE + 57 * math.sin(a))).val())
            add("lug_nuts", nut, (0.08, 0.08, 0.09))
C_DISC = (0.55, 0.56, 0.58)
def disc(x, y, z, r):
    return (cq.Workplane("XZ").circle(r).circle(r - 62).extrude(5, both=True).translate((SX(x), y, z)).val()
            .fuse(cq.Workplane("XZ").circle(r - 55).circle(40).extrude(3, both=True).translate((SX(x), y, z)).val()))


for s in (1, -1):                                                                # 11.4 in front discs, 4-piston style calipers
    add("brakes", disc(WB, s * (HT - 40), R_TIRE - LIFT, 145), C_DISC)
    add("calipers", box(WB - 126, s * (HT - 40), R_TIRE - LIFT, 54, 64, 100), C_ORANGE)
add("brakes", disc(0, -260, R_TIRE - LIFT, 150), C_DISC)                          # 11.8 in rear disc on the solid axle
add("calipers", (cq.Workplane("XY").box(54, 64, 100).rotate((0, 0, 0), (0, 1, 0), -49)
                 .translate((96, -260, R_TIRE - LIFT + 110)).val()), C_ORANGE)       # rear caliper, hung off the swingarm
add("calipers", tube((96, -260, R_TIRE - LIFT + 110), (150, -400, R_TIRE - LIFT + 95), 24, solid=True), C_FRAME)

# ================================================================ REAR AXLE (solid, chain driven)
C_AXLE, C_SUSP = (0.35, 0.36, 0.38), C_ORANGE
AX_Z = R_TIRE
add("rear_axle", ycyl(0, 0, AX_Z, 45, 2 * (HT - 110)), C_AXLE)          # housing tube
for s in (1, -1):
    add("rear_axle", ycyl(0, s * (HT - 115), AX_Z, 85, 30), C_AXLE)     # hub/brake flanges
    add("rear_axle", box(0, s * 520, AX_Z + 25, 140, 40, 140), C_AXLE)  # link brackets
add("rear_axle", ycyl(0, 30, AX_Z, 75, 70), C_AXLE)                      # sprocket carrier, near the middle of the axle

# Cross-kart swingarm: pivots on the jackshaft centerline, so chain length never changes.
# Each side is a triangle (pivot -> above and below the axle); an arched brace ties the two
# sides together over the chain so the arm can carry the twist a solid axle feeds into it.
AXZ = AX_Z - LIFT
for s in (1, -1):
    pv = (PIV[0], s * 300, PIV[1])
    top, bot = (35, s * 520, AXZ + 60), (35, s * 520, AXZ - 50)
    add("rear_susp", tube(pv, top, 50, solid=True), C_SUSP)
    add("rear_susp", tube(pv, bot, 50, solid=True), C_SUSP)
    add("rear_susp", tube(bot, top, 50, solid=True), C_SUSP)
    add("rear_susp", ycyl(PIV[0], s * 300, PIV[1], 45, 70), C_AXLE)            # poly-bushed pivot
add("rear_susp", path([(250, 365, 425), (250, 300, 560), (250, -300, 560), (250, -365, 425)], 45), C_SUSP)

# ================================================================ FRONT SUSPENSION (double A-arm)
# Every point here comes from geometry.py (kingpin 10 deg, caster 7 deg, 30 mm scrub).
C_ROD = (0.7, 0.7, 0.72)
HUBZ = R_TIRE - LIFT
for s in (1, -1):
    LBJ, UBJ, TRO, RE = G(HP["LBJ"], s), G(HP["UBJ"], s), G(HP["T"], s), G(HP["E"], s)
    for a, b, od in [("PLr", LBJ, 38), ("PLf", LBJ, 38), ("PUr", UBJ, 32), ("PUf", UBJ, 32)]:
        pv = G(HP[a], s)
        add("front_susp", tube(pv, b, od, solid=True), C_SUSP)
        add("front_susp", tube((pv[0] - 28, pv[1], pv[2]), (pv[0] + 28, pv[1], pv[2]), 46, solid=True), C_AXLE)  # pivot bushing
    for bj, od in ((LBJ, 38), (UBJ, 32)):
        add("front_susp", cq.Solid.makeSphere(od / 2 + 8, cq.Vector(*X(bj))), C_AXLE)      # ball joint
    # gusset tube across each arm
    for a, b, bj, od in [("PLr", "PLf", LBJ, 32), ("PUr", "PUf", UBJ, 25)]:
        p1, p2 = G(HP[a], s), G(HP[b], s)
        m1 = tuple(p1[k] + 0.55 * (bj[k] - p1[k]) for k in range(3))
        m2 = tuple(p2[k] + 0.55 * (bj[k] - p2[k]) for k in range(3))
        add("front_susp", tube(m1, m2, od, solid=True), C_SUSP)
    kp = lambda z: tuple(LBJ[k] + (UBJ[k] - LBJ[k]) * (z - LBJ[2]) / (UBJ[2] - LBJ[2]) for k in range(3))
    add("knuckles", tube(LBJ, UBJ, 64, solid=True), C_AXLE)                              # upright
    add("knuckles", tube(kp(HUBZ), (WB, s * (HT + 40), HUBZ), 58, solid=True), C_AXLE)   # spindle
    add("knuckles", ycyl(WB, s * (HT - 95), HUBZ, 78, 50), C_AXLE)                       # hub
    add("knuckles", tube(kp(TRO[2]), TRO, 30, solid=True), C_AXLE)                       # steering arm
    add("steering", tube(RE, TRO, 22, solid=True), C_ROD)                                # tie rod

RKX, RKY, RKZ = G(HP["E"])
add("steering", tube((RKX, -RKY, RKZ), (RKX, RKY, RKZ), 26, solid=True), C_ROD)             # rack bar
add("steering", tube((RKX, -RKY + 90, RKZ), (RKX, RKY - 90, RKZ), 54, solid=True), (0.55, 0.56, 0.58))   # rack housing
add("steering", box(RKX, 120, RKZ + 30, 60, 60, 70), (0.55, 0.56, 0.58))                    # pinion
add("steering", path([(RKX, 120, RKZ + 60), (2050, 250, 700), (1720, 300, 960)], 25), (0.6, 0.6, 0.62))
add("steering", (cq.Workplane("YZ").circle(180).circle(160).extrude(20)
                 .rotate((0, 0, 0), (0, 1, 0), -25)
                 .translate((SX(1700), 300, 980)).val()), (0.1, 0.1, 0.1))

# Paddle shifters behind the steering wheel (right = upshift, left = downshift). They fire an
# electric actuator on the gearbox shift shaft; the foot clutch is only needed to pull away and stop.
for dy in (-150, 150):
    add("shifter", (cq.Workplane("XY").box(8, 95, 62).translate((48, dy, 25))
                    .rotate((0, 0, 0), (0, 1, 0), -25).translate((SX(1700), 300, 980)).val()), C_ORANGE)
    add("shifter", (cq.Workplane("XY").box(40, 14, 14).translate((28, dy * 0.55, 25))
                    .rotate((0, 0, 0), (0, 1, 0), -25).translate((SX(1700), 300, 980)).val()), C_ORANGE)
add("shifter", box(660, ENG_Y + 215, 500, 130, 50, 60), C_ORANGE)                           # shift actuator on the trans

# ================================================================ PEDAL BOX (clutch, brake, gas)
C_PEDAL = (0.75, 0.76, 0.78)
PX = 1990
add("pedals", box(PX, 290, FZ + 30, 90, 300, 8), C_PEDAL)                               # pedal box plate
add("pedals", tube((PX, 150, FZ + 45), (PX, 430, FZ + 45), 20, solid=True), C_PEDAL)    # pivot shaft
for py, padw, padh, name in [(385, 70, 55, "clutch"), (290, 80, 55, "brake"), (195, 45, 120, "gas")]:
    add("pedals", tube((PX, py, FZ + 45), (PX - 70, py, FZ + 250), 20, solid=True), C_PEDAL)      # pedal arm
    add("pedals", (cq.Workplane("XY").box(10, padw, padh).rotate((0, 0, 0), (0, 1, 0), -19)
                   .translate((SX(PX - 70), py, FZ + 250)).val()), C_PEDAL)                        # pad
for py in (265, 315):                                                                    # dual brake master cylinders
    add("pedals", tube((PX + 20, py, FZ + 120), (PX + 150, py, FZ + 120), 34, solid=True), (0.10, 0.10, 0.11))
    add("pedals", tube((PX + 110, py, FZ + 137), (PX + 110, py, FZ + 175), 30, solid=True), (0.10, 0.10, 0.11))
# clutch cable (or hydraulic line) from the foot pedal back to the engine
add("pedals", path([(PX, 385, FZ + 110), (1750, 130, FZ + 50), (1150, 60, FZ + 50), (940, ENG_Y + 195, 560)], 10), (0.10, 0.10, 0.11))

# ================================================================ COILOVERS
C_SPRING, C_BODY = C_ORANGE, (0.08, 0.08, 0.09)


def coilover(lo, hi, name):
    """King-style coilover: black body, chrome shaft, piggyback reservoir,
    orange main spring over a black tender spring."""
    A, B = cq.Vector(*X(lo)), cq.Vector(*X(hi))
    v = B - A
    L = v.Length
    d = v.normalized()
    # aim the reservoir side (local +X) outboard, away from the car's centerline
    side = 1 if (A.y + B.y) >= 0 else -1
    out = cq.Vector(0, side, 0)
    out = out - d * out.dot(d)
    xdir = out.normalized() if out.Length > 1e-6 else _perp(d)
    pl = cq.Plane(origin=A, xDir=xdir, normal=d)

    def place(shape):
        return shape.transformShape(pl.rG)

    def zcyl(r, z0, z1, rin=0):
        w = cq.Workplane("XY").circle(r)
        if rin:
            w = w.circle(rin)
        return place(w.extrude(z1 - z0).translate((0, 0, z0)).val())

    def spring(r, wire, z0, z1, coils):
        h = z1 - z0
        helix = cq.Wire.makeHelix(h / coils, h, r)
        sol = (cq.Workplane("XZ", origin=(r, 0, 0)).circle(wire)
               .sweep(cq.Workplane().add(helix), isFrenet=True).val())
        return place(sol.translate((0, 0, z0)))

    C_SHOCK, C_CHR = (0.07, 0.07, 0.08), (0.85, 0.86, 0.88)
    add("shock_shafts", zcyl(9, 0, 0.55 * L), C_CHR)                  # shaft
    add("shock_shafts", zcyl(16, 0, 22), C_CHR)                       # lower eye
    add("coilovers", zcyl(31, 0.42 * L, L - 18), C_SHOCK)              # body
    add("coilovers", zcyl(36, L - 30, L), C_SHOCK)                    # top cap
    add("coilovers", zcyl(50, 0.12 * L, 0.12 * L + 9), C_SHOCK)       # lower spring seat
    add("coilovers", zcyl(46, 0.63 * L, 0.63 * L + 10), C_SHOCK)      # crossover ring
    add("coilovers", zcyl(46, L - 48, L - 38), C_SHOCK)               # top hat
    add("springs", spring(43, 7, 0.12 * L + 9, 0.63 * L, 6), C_ORANGE)    # main spring
    add("tender_springs", spring(43, 5, 0.63 * L + 10, L - 48, 5), C_SHOCK)  # tender spring
    # piggyback reservoir alongside the body, with orange end caps (King style)
    res = cq.Workplane("XY").circle(25).extrude(0.42 * L).translate((70, 0, 0.5 * L)).val()
    add("coilovers", place(res), C_SHOCK)
    add("coilovers", place(cq.Workplane("XY").box(60, 30, 34).translate((38, 0, L - 45)).val()), C_SHOCK)
    for z in (0.5 * L - 6, 0.92 * L):
        cap = cq.Workplane("XY").circle(27).extrude(12).translate((70, 0, z)).val()
        add("shock_caps", place(cap), C_ORANGE)
    add("shock_caps", zcyl(38, L - 64, L - 54), C_ORANGE)             # adjuster ring

for s in (1, -1):
    _mid = (HP["PLf"] + HP["PLr"]) / 2
    _low = _mid + 0.62 * (HP["LBJ"] - _mid) + (0, 0, 25)                               # 62 % out along the lower arm
    coilover(G(_low, s), (2330, s * 390, 890), "coilovers")       # front
    coilover((40, s * 560, AX_Z - LIFT + 70), (330, s * 400, 1000), "coilovers")  # rear



# ================================================================ REV MAX ENGINE (envelope)
# 975T bottom end today; plan is a 1250 top end / Pan America spec. Outside size is about the same.
C_CASE, C_CYL, C_CHROME = (0.70, 0.71, 0.73), (0.12, 0.12, 0.12), (0.82, 0.82, 0.85)
CRANK = (800, ENG_Y, 560)                   # crank centerline, 8 in right of the car's centerline
case = (cq.Workplane("XZ").polyline([(-230, -150), (180, -150), (210, 20), (90, 120),
                                     (-260, 110), (-330, 40), (-330, -90)]).close()
        .extrude(130, both=True).edges("|Y").fillet(30)
        .translate(CRANK).val())
add("engine", case, C_CASE)
# primary/clutch cover
prim = (cq.Workplane("XZ").polyline([(-290, -110), (120, -110), (150, 30), (-290, 60)]).close()
        .extrude(55).edges("|Y").fillet(25).translate((CRANK[0], ENG_Y + 130 + 55, CRANK[2])).val())
add("engine", prim, C_CHROME)
# water pump on right
add("engine", ycyl(CRANK[0] + 90, ENG_Y - 150, CRANK[2] + 40, 70, 40), C_CHROME)

for ang in (20, -40):                       # 60-deg V, tilted back 10 deg to clear the firewall
    a = math.radians(ang)
    def along(r, a=a):
        return (CRANK[0] + r * math.sin(a), ENG_Y, CRANK[2] + r * math.cos(a))
    cyl = (cq.Workplane("XY").box(150, 190, 230).edges("|Z").fillet(25)
           .rotate((0, 0, 0), (0, 1, 0), ang).translate(along(215)).val())
    head = (cq.Workplane("XY").box(185, 230, 130).edges("|Z").fillet(30)
            .rotate((0, 0, 0), (0, 1, 0), ang).translate(along(395)).val())
    cams = (cq.Workplane("XY").box(150, 215, 45).edges("|Z").fillet(20)
            .rotate((0, 0, 0), (0, 1, 0), ang).translate(along(480)).val())
    add("engine", cyl, C_CYL)
    add("engine", head, C_CASE)
    add("engine", cams, C_CYL)
# airbox sits in the V
add("engine", box(CRANK[0] - 82, ENG_Y, CRANK[2] + 463, 170, 220, 120), C_CYL)

# ================================================================ COOLING (radiator behind seats, up high)
C_RAD, C_FAN = (0.32, 0.34, 0.36), (0.12, 0.12, 0.13)
RAD_C = (560, 0, 1160)
rad = (cq.Workplane("XY").box(55, 600, 400).rotate((0, 0, 0), (0, 1, 0), -40)
       .translate(RAD_C).val())
fan = (cq.Workplane("YZ").circle(165).extrude(60).rotate((0, 0, 0), (0, 1, 0), -40)
       .translate((RAD_C[0] - 75, 0, RAD_C[2] - 60)).val())
add("radiator", rad, C_RAD)
add("radiator", fan, C_FAN)
# coolant hoses down to the engine
add("radiator", path([(RAD_C[0] + 80, 100, RAD_C[2] - 120), (CRANK[0] + 150, ENG_Y + 150, CRANK[2] + 300)], 38), C_FAN)
add("radiator", path([(RAD_C[0] - 80, -220, RAD_C[2] - 150), (CRANK[0] + 90, ENG_Y - 170, CRANK[2] + 60)], 38), C_FAN)

# ================================================================ EXHAUST: 2-into-1, pie-cut shotgun pipe, SuperTrapp tip
C_HOT = (0.42, 0.30, 0.22)
C_PIPE_CH, C_WELD = (0.80, 0.81, 0.83), (0.55, 0.47, 0.40)
PIPE_OD = 2.5 * IN


def piecut(points, od, r, cuts=4):
    """Pie-cut pipe: each bend is built from a few short straight wedges with weld seams."""
    pts = _bend([X(p) for p in points], r, steps=cuts)
    s = [tube(a, b, od, solid=True, mapped=True) for a, b in zip(pts[:-1], pts[1:])]
    for i in range(1, len(pts) - 1):
        a, b, c = (cq.Vector(*p) for p in pts[i - 1:i + 2])
        d = ((b - a).normalized() + (c - b).normalized()).normalized()
        p0 = b - d * 2.5
        s.append(tube(tuple(p0), tuple(b + d * 2.5), od + 5, solid=True, mapped=True))   # weld bead
        s.append(cq.Solid.makeSphere(od / 2, b))
    return cq.Compound.makeCompound(s)


COLL = (520, -385, 705)                      # where the two headers merge, tucked between the motor and the quarter panel
add("headers", path([(1000, ENG_Y - 60, 880), (1030, ENG_Y - 170, 770), (910, -425, 640), (600, -410, 690), COLL], 45), C_HOT)
add("headers", path([(440, ENG_Y - 60, 850), (420, -350, 830), (470, -385, 760), COLL], 45), C_HOT)
add("headers", cq.Solid.makeSphere(36, cq.Vector(*COLL)), C_HOT)                    # merge collector
PRE_TIP = (200, -310, 935)
KICK = (-120, -325, 945)                     # where the pipe starts to kick out and up
TIP = (-260, -410, 1010)
shotgun = [COLL, (400, -370, 735), (330, -330, 880), PRE_TIP, KICK, TIP]
add("exhaust", piecut(shotgun, PIPE_OD, 4 * IN, cuts=3), C_PIPE_CH)

# SuperTrapp-style megaphone with disc pack
tip_d = (cq.Vector(*X(TIP)) - cq.Vector(*X(KICK))).normalized()
m0 = cq.Vector(*X(TIP))
MEG_L = 210
add("exhaust", cq.Solid.makeCone(PIPE_OD / 2, 50, MEG_L, m0, tip_d), C_PIPE_CH)
add("exhaust", tube(tuple(m0 - tip_d * 3), tuple(m0 + tip_d * 3), PIPE_OD + 5, solid=True), C_WELD)
c = m0 + tip_d * MEG_L
for k in range(9):                                   # stacked diffuser discs
    p = c + tip_d * (4 + k * 8)
    add("exhaust", tube(tuple(p), tuple(p + tip_d * 3), 118, wall=40), C_WELD)
end = c + tip_d * (4 + 9 * 8)
add("exhaust", tube(tuple(end), tuple(end + tip_d * 6), 122, solid=True), C_PIPE_CH)   # end cap
add("exhaust", tube(tuple(c), tuple(end), 16, solid=True), C_PIPE_CH)                  # through-bolt
for ang in (0, 120, 240):                                                             # disc-pack bolts
    off = cq.Vector(0, 46 * math.cos(math.radians(ang)), 46 * math.sin(math.radians(ang)))
    add("exhaust", tube(tuple(c + off), tuple(end + off), 7, solid=True), C_PIPE_CH)
# hanger rods from the pipe to the rear exo tube
add("exhaust", tube((250, -316, 925), (250, -399, 990), 14, solid=True), C_FRAME)
add("exhaust", tube((0, -319, 925), (0, -372, 897), 14, solid=True), C_FRAME)

# ================================================================ DRIVELINE: trans -> jackshaft (with electric reverse) -> axle
C_SPROCKET, C_CHAIN = (0.55, 0.55, 0.57), (0.30, 0.25, 0.20)
TRANS_OUT = (CRANK[0] - 260, ENG_Y + 165, CRANK[2] - 20)    # trans output; stock is a belt pulley, drawn as a chain sprocket
JACK = (PIV[0], 0, PIV[1])
JS_IN_Y, JS_OUT_Y = TRANS_OUT[1], 30                       # stage 2 runs near the middle of the axle
r_trans, r_js_in, r_js_out, r_axle = 45, 105, 45, 150

add("driveline", ycyl(*TRANS_OUT, r_trans, 20), C_SPROCKET)
add("driveline", ycyl(JACK[0], 0, JACK[2], 20, 690), C_AXLE)                    # jackshaft, runs through both swingarm pivots
add("driveline", ycyl(JACK[0], JS_IN_Y, JACK[2], r_js_in, 20), C_SPROCKET)
add("driveline", ycyl(JACK[0], JS_OUT_Y, JACK[2], r_js_out, 20), C_SPROCKET)
add("driveline", ycyl(0, JS_OUT_Y, AX_Z - LIFT, r_axle, 20), C_SPROCKET)
for yy in (-235, 235):                                                           # jackshaft pillow blocks
    add("driveline", box(JACK[0], yy, JACK[2], 90, 40, 80), (0.25, 0.30, 0.45))
# Electric reverse: a starter motor spins a ring gear on the jackshaft (same idea as a trike reverse kit)
C_REV = (0.20, 0.45, 0.75)
add("reverse", ycyl(JACK[0], -150, JACK[2], 110, 14), C_SPROCKET)                # ring gear
add("reverse", ycyl(JACK[0] + 64, -190, JACK[2] + 111, 40, 170), C_REV)           # starter motor
add("reverse", ycyl(JACK[0] + 64, -150, JACK[2] + 111, 18, 22), C_SPROCKET)      # starter pinion
add("reverse", ycyl(JACK[0] + 64, -225, JACK[2] + 165, 22, 90), C_REV)            # solenoid


def chain(c1, r1, c2, r2, y):
    """Top and bottom runs of a chain between two sprockets (outer tangents)."""
    x1, z1 = c1
    x2, z2 = c2
    dx, dz = x2 - x1, z2 - z1
    d = math.hypot(dx, dz)
    ang = math.atan2(dz, dx)
    beta = math.asin((r1 - r2) / d)
    out = None
    for sgn in (1, -1):
        th = ang + sgn * (math.pi / 2 - beta) if sgn == 1 else ang - (math.pi / 2 - beta)
        p1 = (x1 + r1 * math.cos(th), y, z1 + r1 * math.sin(th))
        p2 = (x2 + r2 * math.cos(th), y, z2 + r2 * math.sin(th))
        run = tube(p1, p2, 16, solid=True)
        out = run if out is None else out.fuse(run)
    return out


add("chains", chain((TRANS_OUT[0], TRANS_OUT[2]), r_trans, (JACK[0], JACK[2]), r_js_in, JS_IN_Y), C_CHAIN)
add("chains", chain((JACK[0], JACK[2]), r_js_out, (0, AX_Z - LIFT), r_axle, JS_OUT_Y), C_CHAIN)

# ================================================================ SEATS, TANK
C_SEAT = (0.10, 0.10, 0.11)
for s in (1, -1):
    seat = (cq.Workplane("YZ").polyline([(-230, 0), (230, 0), (230, 110), (-230, 110)]).close()
            .extrude(480).translate((1170, s * 300, FZ + 40)).val())
    back = (cq.Workplane("XY").box(110, 460, 720).edges("|X").fillet(40)
            .rotate((0, 0, 0), (0, 1, 0), -12).translate((1200, s * 300, FZ + 470)).val())
    add("seats", seat.fuse(back), C_SEAT)
    for dy in (-70, 70):                                   # twin orange stripes on the seat back
        stripe = (cq.Workplane("XY").box(6, 26, 600).translate((-57, dy, 0))
                  .rotate((0, 0, 0), (0, 1, 0), -12).translate((1200, s * 300, FZ + 470)).val())
        add("seat_stripes", stripe, C_ORANGE)
# Fuel: pony keg (quarter barrel, 7.75 gal, 16.1 in dia x 13.9 in tall), standing up behind the
# driver's seat. With the engine moved right, the keg now sits fully inside the frame rails.
KEG = (870, 250, 650)                       # center of the keg bottom
KEG_R, KEG_H = 16.1 * IN / 2, 13.875 * IN
KEG_TOP = KEG[2] + KEG_H
C_STEEL = (0.78, 0.79, 0.81)
add("fuel_tank", (cq.Workplane("XY").circle(KEG_R).extrude(KEG_H).edges().fillet(30)
                  .translate(KEG).val()), C_STEEL)
for zr in (0.22, 0.78):                     # rolling rings
    add("fuel_tank", (cq.Workplane("XY").circle(KEG_R + 8).circle(KEG_R - 2).extrude(14)
                      .translate((KEG[0], KEG[1], KEG[2] + KEG_H * zr - 7)).val()), C_STEEL)
add("fuel_tank", (cq.Workplane("XY").circle(KEG_R - 10).circle(KEG_R - 30).extrude(70)   # top chime / handles
                  .translate((KEG[0], KEG[1], KEG_TOP - 20)).val()), C_STEEL)
add("fuel_tank", (cq.Workplane("XY").circle(14).extrude(30)                                 # rollover vent valve
                  .translate((KEG[0] - 70, KEG[1] - 40, KEG_TOP)).val()), (0.10, 0.10, 0.11))
add("fuel_tank", (cq.Workplane("XY").circle(70).extrude(-12)                                # EFI pump insert flange (bottom)
                  .translate(KEG).val()), (0.15, 0.15, 0.16))
for yy in (KEG[1] - 140, KEG[1] + 140):                                                    # cradle straps to the frame
    add("fuel_tank", box(KEG[0], yy, KEG[2] - 20, 380, 30, 8), C_FRAME)
for xx in (KEG[0] - 150, KEG[0] + 150):                                                    # cradle legs down to the floor
    for yy in (KEG[1] - 140, KEG[1] + 140):
        add("fuel_tank", tube((xx, yy, KEG[2] - 24), (xx, yy, FZ), 25, solid=True), C_FRAME)
add("fuel_tank", tube((KEG[0] - 150, ENG_Y + 150, FZ), (KEG[0] - 150, 494, FZ), 25, solid=True), C_FRAME)   # cradle cross tubes,
add("fuel_tank", tube((KEG[0] + 150, ENG_Y + 150, FZ), (KEG[0] + 150, 590, FZ), 25, solid=True), C_FRAME)   # engine rail to floor rail
# Remote fill: flip-up motorcycle (Monza) cap on a bracket outside the quarter panel, above the keg,
# with a fill hose down to the keg's top bung. Filling from above the keg means all 7.75 gal are usable.
C_CAP = (0.88, 0.89, 0.91)
CAP_C = cq.Vector(835, 548, 1132)
CAP_N = cq.Vector(0, 0.707, 0.707)
add("fuel_tank", path([(835, 520, 1108), (848, 430, 1112), (870, 300, 1085), (870, KEG[1], KEG_TOP + 5)], 45),
    (0.10, 0.10, 0.11))                                                                     # fill hose
add("fuel_tank", (cq.Workplane("XY").circle(34).extrude(20)
                  .translate((KEG[0], KEG[1], KEG_TOP)).val()), C_STEEL)                     # top bung
add("gas_cap", tube(tuple(CAP_C - CAP_N * 30), tuple(CAP_C - CAP_N * 12), 124, solid=True), C_CAP)   # flange
add("gas_cap", tube(tuple(CAP_C - CAP_N * 12), tuple(CAP_C + CAP_N * 4), 110, solid=True), C_CAP)    # cap
add("gas_cap", tube(tuple(CAP_C + CAP_N * 4), tuple(CAP_C + CAP_N * 8), 86, solid=True), (0.10, 0.10, 0.11))
add("gas_cap", tube(tuple(CAP_C + CAP_N * 12 + cq.Vector(-32, 0, 0)), tuple(CAP_C + CAP_N * 12 + cq.Vector(32, 0, 0)),
                    14, solid=True), C_CAP)                                                 # flip lever
add("gas_cap", tube((835, 498, 1082), tuple(CAP_C - CAP_N * 30), 20, solid=True), C_FRAME)  # bracket off the exo tube
add("battery", box(1600, 0, FZ + 100, 220, 140, 180), (0.2, 0.2, 0.2))   # center tunnel

# ================================================================ SHOW + EXPORT
import json, os
try:
    OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
except NameError:                                  # some editors don't set __file__
    OUT = os.path.join(os.path.expanduser("~"), "Documents", "nightshift_out")
os.makedirs(OUT, exist_ok=True)
parts = {n: (cq.Compound.makeCompound(lst), c) for n, (lst, c) in parts.items()}
NO_LIFT = {"tires", "rims", "rear_axle", "lug_nuts"}
parts = {n: ((sh if n in NO_LIFT else sh.translate(cq.Vector(0, 0, LIFT))), c) for n, (sh, c) in parts.items()}

# Show every part group in CQ-editor, each in its own color (uncheck a part in the Objects panel to hide it).
if "show_object" in globals():
    for name, (shape, col) in parts.items():
        show_object(shape, name=name, options={"color": tuple(int(c * 255) for c in col), })

# Set SAVE_FILES = True to also write the STEP / GLB / STL files into the "out" folder (slow).
SAVE_FILES = "show_object" not in globals()
if SAVE_FILES:
    assy = cq.Assembly(name="sportster_crawler")
    for name, (shape, col) in parts.items():
        assy.add(cq.Workplane().add(shape), name=name, color=cq.Color(*col, 1.0))
    assy.save(os.path.join(OUT, "sportster_crawler.step"))
    SKIN = {"body", "accents", "hood_pins", "armor"}
    for tag, drop in (("skeleton", SKIN), ("armor", set())):
        a2 = cq.Assembly(name=f"nightshift_{tag}")
        for name, (shape, col) in parts.items():
            if name not in drop:
                a2.add(cq.Workplane().add(shape), name=name, color=cq.Color(*col, 1.0))
        a2.save(os.path.join(OUT, f"nightshift_{tag}.step"))
    assy.export(os.path.join(OUT, "sportster_crawler.glb"), tolerance=1.0, angularTolerance=0.5)
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    for name, (shape, col) in parts.items():
        cq.exporters.export(cq.Workplane().add(shape), os.path.join(OUT, "stl", f"{name}.stl"))

bb = cq.Compound.makeCompound([s for s, _ in parts.values()]).BoundingBox()
json.dump({"names": list(parts), "bbox_mm": [bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax]},
          open(os.path.join(OUT, "info.json"), "w"), indent=1)
print("bbox", bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)

# ================================================================ CUT LIST + WEIGHT ESTIMATE
import csv


def lb_ft(od_in, wall_in):
    return 10.68 * (od_in - wall_in) * wall_in       # steel tube, pounds per foot


def develop(od, pts):
    """Cut length of a bent tube (centerline radius 3.2 x OD), plus its bend angles and the twist between bends."""
    V = [cq.Vector(*p) for p in pts]
    segs = [(b - a).Length for a, b in zip(V[:-1], V[1:])]
    total, angles, normals = sum(segs), [], []
    r = 3.2 * od
    for i in range(1, len(V) - 1):
        u, v = (V[i] - V[i - 1]).normalized(), (V[i + 1] - V[i]).normalized()
        th = math.acos(max(-1.0, min(1.0, u.dot(v))))
        if th < math.radians(5):
            continue
        t = min(r * math.tan(th / 2), 0.45 * segs[i - 1], 0.45 * segs[i])
        total -= 2 * t - (t / math.tan(th / 2)) * th
        angles.append(math.degrees(th))
        normals.append(u.cross(v).normalized())
    twists = [math.degrees(math.acos(max(-1.0, min(1.0, a.dot(b))))) for a, b in zip(normals[:-1], normals[1:])]
    return total, segs, angles, twists


MATERIAL = {round(MAIN, 1): ("1.75 x .120 DOM", lb_ft(1.75, 0.120)), round(SEC, 1): ("1.50 x .120 DOM", lb_ft(1.50, 0.120))}
rows, frame_lb, feet = {}, 0.0, {}
for group, od, pts in CUT:
    if round(od, 1) not in MATERIAL:
        continue
    L, segs, angles, twists = develop(od, pts)
    mat, w = MATERIAL[round(od, 1)]
    frame_lb += L / 304.8 * w
    feet[mat] = feet.get(mat, 0.0) + L / 304.8
    key = (group, mat, tuple((round(x), round(abs(y)), round(z)) for x, y, z in pts))
    if key in rows:
        rows[key]["qty"] += 1
    else:
        rows[key] = dict(group=group, mat=mat, qty=1, L=L, segs=segs, angles=angles, twists=twists, pts=pts)
with open(os.path.join(OUT, "nightshift_cut_list_DRAFT.csv"), "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["DRAFT - concept geometry, not for cutting until the real engine, seats and axle parts are measured in"])
    wr.writerow(["Tube", "Where", "Material", "Qty", "Cut length (in)", "Bends", "Bend angles (deg)", "Twist between bends (deg)",
                 "Node-to-node lengths (in)", "Start point x/y/z (in)", "End point x/y/z (in)"])
    for n, r in enumerate(sorted(rows.values(), key=lambda r: (r["group"], r["mat"], -r["L"])), 1):
        wr.writerow(["T%02d" % n, r["group"], r["mat"], r["qty"], "%.2f" % (r["L"] / IN), len(r["angles"]),
                     " / ".join("%.0f" % a for a in r["angles"]), " / ".join("%.0f" % a for a in r["twists"]),
                     " / ".join("%.1f" % (x / IN) for x in r["segs"]),
                     " / ".join("%.1f" % (v / IN) for v in (r["pts"][0][0], r["pts"][0][1], r["pts"][0][2] + LIFT)),
                     " / ".join("%.1f" % (v / IN) for v in (r["pts"][-1][0], r["pts"][-1][1], r["pts"][-1][2] + LIFT))])
    wr.writerow([])
    for mat, ft in feet.items():
        wr.writerow(["TOTAL", "", mat, "", "%.0f ft" % ft, "", "", "", "order about %.0f ft with 10%% extra" % (ft * 1.1)])

FT2 = 92903.0
carbon_ft2 = (PANEL_AREA.get("body", 0) + PANEL_AREA.get("armor", 0)) / FT2
alu_ft2 = PANEL_AREA.get("interior", 0) / FT2
arm_lb = 2 * sum(float(((HP[a] - HP[b]) ** 2).sum() ** 0.5) / 304.8 * w for a, b, w in
                 [("PLf", "LBJ", lb_ft(1.5, .120)), ("PLr", "LBJ", lb_ft(1.5, .120)),
                  ("PUf", "UBJ", lb_ft(1.25, .095)), ("PUr", "UBJ", lb_ft(1.25, .095))]) * 1.25
WEIGHT = [
    ("Frame and cage tube (from the model)", frame_lb, "calc"),
    ("Tabs, gussets, brackets, weld, powder coat", 45, "est"),
    ("Carbon fiber skins, 2 mm (%.0f sq ft)" % carbon_ft2, carbon_ft2 * 0.635, "calc"),
    ("Aluminum firewall and dash, .063 (%.0f sq ft)" % alu_ft2, alu_ft2 * 0.885, "calc"),
    ("Aluminum skid plates, 3/16 (%.0f sq ft)" % (SKID_AREA / FT2), SKID_AREA / FT2 * 2.634, "calc"),
    ("Rev Max engine and trans with fluids", 190, "est"),
    ("Radiator, fan, hoses, coolant", 22, "est"),
    ("Exhaust: headers, pipe, SuperTrapp", 22, "est"),
    ("Jackshaft, sprockets, chains, electric reverse", 48, "est"),
    ("Wheels and tires, 32x10-15 (4)", 4 * 62, "est"),
    ("Coilovers with reservoirs and springs (4)", 4 * 22, "est"),
    ("Front arms (from the model)", arm_lb, "calc"),
    ("Front knuckles, hubs, brakes", 60, "est"),
    ("Steering rack, column, wheel, tie rods", 24, "est"),
    ("Rear axle, hubs, bearing carriers, brake", 78, "est"),
    ("Swingarm", 38, "est"),
    ("Seats and harnesses (2)", 50, "est"),
    ("Keg tank, pump, lines (empty)", 30, "est"),
    ("Battery, harness, ECM, gauge", 32, "est"),
    ("Pedals, master cylinders, brake lines, shifter and clutch linkage", 28, "est"),
    ("Winch and lights", 34, "est"),
]
dry = sum(w for _, w, _ in WEIGHT)
alu_skins = carbon_ft2 * 1.124
json.dump(dict(items=WEIGHT, dry_lb=dry, fuel_lb=7.75 * 6.1, tube_ft=feet, carbon_ft2=carbon_ft2,
               carbon_saves_lb=alu_skins - carbon_ft2 * 0.635, geometry={k: v for k, v in GEO.items()},
               overall=dict(length_in=(bb.xmax - bb.xmin) / IN, width_in=(bb.ymax - bb.ymin) / IN, height_in=bb.zmax / IN,
                            swingarm_len_in=math.hypot(PIV[0], PIV[1] - AXZ) / IN,
                            swingarm_deg=math.degrees(math.atan2(PIV[1] - AXZ, PIV[0])))),
          open(os.path.join(OUT, "weight.json"), "w"), indent=1, default=float)
for name, w, kind in WEIGHT:
    print("%-66s %6.0f lb  %s" % (name, w, kind))
print("DRY TOTAL %.0f lb   with fuel %.0f lb   tube %s   carbon vs .080 aluminum saves %.0f lb" %
      (dry, dry + 7.75 * 6.1, {k: round(v) for k, v in feet.items()}, alu_skins - carbon_ft2 * 0.635))
