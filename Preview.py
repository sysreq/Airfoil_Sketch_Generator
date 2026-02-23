"""
Preview — Pure Python PNG Renderer
------------------------------------
Generates small PNG preview images of airfoil geometry for use in
Fusion 360 ImageCommandInput widgets.

Pure Python — only struct, zlib, math, os. No Fusion 360 dependencies,
no PIL/Pillow.
"""

import struct
import zlib
import math
import os

import Geometry


# -- Colors -------------------------------------------------------------------

_CLR_BACKGROUND  = (240, 240, 240)
_CLR_AIRFOIL     = (30, 60, 120)
_CLR_OFFSET      = (100, 140, 200)
_CLR_SPLIT       = (200, 50, 50)
_CLR_SPAR        = (50, 160, 50)
_CLR_WING        = (80, 130, 180)
_CLR_WING_FILL   = (200, 215, 235)
_CLR_CS_FILL     = (230, 180, 180)

# Mapping constants for _norm_to_pixel: vertical center and scale
_Y_CENTER_FRAC = 0.57   # Y=0 at 57% from top (camber bias)
_Y_SCALE = 2.85         # vertical stretch to fill drawing area


# -- Canvas -------------------------------------------------------------------

class Canvas:
    """RGBA pixel buffer with basic drawing primitives."""

    __slots__ = ("width", "height", "_buf")

    def __init__(self, width, height):
        self.width = width
        self.height = height
        # Flat list: [R, G, B, A, R, G, B, A, ...] row-major
        self._buf = bytearray(width * height * 4)

    def fill(self, r, g, b, a=255):
        pixel = bytes((r, g, b, a))
        self._buf[:] = pixel * (self.width * self.height)

    def set_pixel(self, x, y, r, g, b, a=255):
        if 0 <= x < self.width and 0 <= y < self.height:
            offset = (y * self.width + x) * 4
            self._buf[offset] = r
            self._buf[offset + 1] = g
            self._buf[offset + 2] = b
            self._buf[offset + 3] = a

    def draw_line(self, x0, y0, x1, y1, color, thickness=1):
        """Bresenham line with optional thickness."""
        r, g, b = color
        if thickness <= 1:
            self._bresenham(x0, y0, x1, y1, r, g, b)
        else:
            # Draw parallel lines offset perpendicular to the line direction
            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)
            if length < 0.5:
                self.set_pixel(int(x0), int(y0), r, g, b)
                return
            # Unit perpendicular
            px = -dy / length
            py = dx / length
            half = (thickness - 1) / 2.0
            steps = int(math.ceil(half))
            for s in range(-steps, steps + 1):
                frac = s
                ox = px * frac
                oy = py * frac
                self._bresenham(
                    int(round(x0 + ox)), int(round(y0 + oy)),
                    int(round(x1 + ox)), int(round(y1 + oy)),
                    r, g, b,
                )

    def _bresenham(self, x0, y0, x1, y1, r, g, b):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.set_pixel(x0, y0, r, g, b)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def draw_circle(self, cx, cy, radius, color, thickness=1):
        """Midpoint circle algorithm."""
        r_col, g_col, b_col = color
        rad = int(round(radius))
        if rad < 1:
            self.set_pixel(int(round(cx)), int(round(cy)),
                           r_col, g_col, b_col)
            return

        icx = int(round(cx))
        icy = int(round(cy))

        def _plot_circle_points(ox, oy):
            for px, py in (
                (icx + ox, icy + oy), (icx - ox, icy + oy),
                (icx + ox, icy - oy), (icx - ox, icy - oy),
                (icx + oy, icy + ox), (icx - oy, icy + ox),
                (icx + oy, icy - ox), (icx - oy, icy - ox),
            ):
                self.set_pixel(px, py, r_col, g_col, b_col)

        if thickness <= 1:
            x, y = rad, 0
            d = 1 - rad
            while x >= y:
                _plot_circle_points(x, y)
                y += 1
                if d <= 0:
                    d += 2 * y + 1
                else:
                    x -= 1
                    d += 2 * (y - x) + 1
        else:
            # Draw circles at multiple radii for thickness
            r_inner = max(0, rad - thickness // 2)
            r_outer = rad + (thickness - 1) // 2
            for ri in range(r_inner, r_outer + 1):
                x, y = ri, 0
                d = 1 - ri
                if ri == 0:
                    self.set_pixel(icx, icy, r_col, g_col, b_col)
                    continue
                while x >= y:
                    _plot_circle_points(x, y)
                    y += 1
                    if d <= 0:
                        d += 2 * y + 1
                    else:
                        x -= 1
                        d += 2 * (y - x) + 1

    def draw_polyline(self, points, color, thickness=1, closed=False):
        """Draw connected line segments from a list of (px, py) pixel coords."""
        n = len(points)
        if n < 2:
            return
        end = n if closed else n - 1
        for i in range(end):
            x0, y0 = points[i]
            x1, y1 = points[(i + 1) % n]
            self.draw_line(
                int(round(x0)), int(round(y0)),
                int(round(x1)), int(round(y1)),
                color, thickness,
            )

    def to_png_bytes(self):
        return _write_png(self.width, self.height, self._buf)


# -- PNG Writer ---------------------------------------------------------------

def _make_chunk(chunk_type, data):
    """Build a single PNG chunk: length(4) + type(4) + data + crc32(4)."""
    raw = chunk_type + data
    return struct.pack(">I", len(data)) + raw + struct.pack(">I",
        zlib.crc32(raw) & 0xFFFFFFFF)


def _write_png(width, height, rgba_buf):
    """Minimal PNG writer. Returns bytes of a valid PNG file."""
    # PNG signature
    signature = b"\x89PNG\r\n\x1a\n"

    # IHDR: width, height, bit depth 8, color type 6 (RGBA)
    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    ihdr = _make_chunk(b"IHDR", ihdr_data)

    # Build raw scanlines with filter byte 0 (None) per row
    raw_rows = bytearray()
    row_bytes = width * 4
    for y in range(height):
        raw_rows.append(0)  # filter type None
        start = y * row_bytes
        raw_rows.extend(rgba_buf[start:start + row_bytes])

    # IDAT: zlib-compressed scanlines
    compressed = zlib.compress(bytes(raw_rows), 1)
    idat = _make_chunk(b"IDAT", compressed)

    # IEND
    iend = _make_chunk(b"IEND", b"")

    return signature + ihdr + idat + iend


# -- Coordinate Mapping -------------------------------------------------------

def _norm_to_pixel(x_norm, y_norm, width, height, margin):
    """
    Map normalised airfoil coords [0..1] to pixel coords.

    X maps left to right across (width - 2*margin).
    Y is flipped (airfoil Y-up to pixel Y-down) and centered vertically.
    """
    draw_w = width - 2 * margin
    draw_h = height - 2 * margin

    px = margin + x_norm * draw_w
    py = margin + draw_h * _Y_CENTER_FRAC - y_norm * draw_h * _Y_SCALE
    return px, py


def _coords_to_pixels(coords, width, height, margin, ox=0, oy=0):
    """Convert a list of normalised (x,y) tuples to pixel coords."""
    if ox == 0 and oy == 0:
        return [_norm_to_pixel(x, y, width, height, margin) for x, y in coords]
    return [(px + ox, py + oy)
            for px, py in (_norm_to_pixel(x, y, width, height, margin)
                           for x, y in coords)]


# -- Main Render Functions ----------------------------------------------------

def render_preview(coords, chord_cm=10.0, split_pct=0.0,
                   spars=None, lightening_holes=None,
                   cs_coords=None, offset_pts=None,
                   wing=None, width=360, height=180):
    """
    Render a preview PNG of the airfoil with optional features.

    All geometry inputs (coords, offset_pts, spars, lightening_holes,
    cs_coords) are in NORMALIZED [0..1] space.

    Parameters
    ----------
    split_pct : float
        Chord split percentage; 0 means no split.
    spars, lightening_holes, cs_coords, offset_pts :
        Pass None to disable each feature (presence implies enabled).
    wing : dict or None
        When provided, draws a planform view. Expected keys:
        half_span_mm, tip_chord_mm, sweep_deg, sections,
        mirror, solid, cs_inboard_pct, cs_outboard_pct.

    Returns PNG bytes.
    """
    canvas = Canvas(width, height)
    canvas.fill(*_CLR_BACKGROUND)

    margin = 12
    half_span = wing["half_span_mm"] if wing else 0.0

    if wing and half_span > 0:
        # Split canvas: left half = airfoil cross-section, right half = planform
        half_w = width // 2
        _draw_cross_section(canvas, coords, split_pct,
                            offset_pts, cs_coords, spars,
                            lightening_holes, 0, 0, half_w, height, margin)
        _draw_planform(canvas, chord_cm,
                       half_span,
                       wing["tip_chord_mm"],
                       wing["sweep_deg"],
                       half_w, 0, half_w, height, margin,
                       wing.get("sections", 2),
                       wing.get("mirror", False),
                       wing.get("solid", False),
                       split_pct=split_pct,
                       cs_inboard_pct=wing.get("cs_inboard_pct", 0.0),
                       cs_outboard_pct=wing.get("cs_outboard_pct", 0.0))
    else:
        _draw_cross_section(canvas, coords, split_pct,
                            offset_pts, cs_coords, spars,
                            lightening_holes, 0, 0, width, height, margin)

    return canvas.to_png_bytes()


def _draw_cross_section(canvas, coords, split_pct, offset_pts, cs_coords,
                        spars, lightening_holes, ox, oy, w, h, margin):
    """Draw the airfoil cross-section view within a sub-region of the canvas."""
    # Main airfoil contour
    if coords:
        pixels = _coords_to_pixels(coords, w, h, margin, ox, oy)
        canvas.draw_polyline(pixels, _CLR_AIRFOIL, thickness=2, closed=True)

    # Offset contour
    if offset_pts:
        off_pixels = _coords_to_pixels(offset_pts, w, h, margin, ox, oy)
        canvas.draw_polyline(off_pixels, _CLR_OFFSET, thickness=1, closed=True)

    # Control surface contour
    if cs_coords:
        cs_pixels = _coords_to_pixels(cs_coords, w, h, margin, ox, oy)
        canvas.draw_polyline(cs_pixels, _CLR_AIRFOIL, thickness=2, closed=True)

    # Split line
    if split_pct > 0 and coords:
        split_x = split_pct / 100.0
        upper_y, lower_y = _find_surface_y(coords, split_x)
        px0, py0 = _norm_to_pixel(split_x, upper_y, w, h, margin)
        px1, py1 = _norm_to_pixel(split_x, lower_y, w, h, margin)
        canvas.draw_line(int(round(px0 + ox)), int(round(py0 + oy)),
                         int(round(px1 + ox)), int(round(py1 + oy)),
                         _CLR_SPLIT, thickness=1)

    # Spar circles
    if spars:
        draw_h = h - 2 * margin
        for cx, cy, thickness in spars:
            px, py = _norm_to_pixel(cx, cy, w, h, margin)
            r_px = (thickness / 2.0) * draw_h * _Y_SCALE
            canvas.draw_circle(px + ox, py + oy, r_px, _CLR_SPAR, thickness=1)

    # Lightening holes
    if lightening_holes:
        draw_h = h - 2 * margin
        for cx, cy, diam in lightening_holes:
            px, py = _norm_to_pixel(cx, cy, w, h, margin)
            r_px = (diam / 2.0) * draw_h * _Y_SCALE
            canvas.draw_circle(px + ox, py + oy, r_px, _CLR_SPAR, thickness=1)


def _planform_to_pixel(x_mm, y_span_mm, ox, oy, w, h, margin,
                       root_chord_mm, half_span_mm, sweep_offset_mm):
    """
    Map planform mm coords to pixel coords within a sub-region.

    x_mm:       chordwise position (0 = LE, root_chord_mm = TE)
    y_span_mm:  spanwise position (0 = root, half_span_mm = tip)
    ox, oy:     pixel origin of the sub-region
    w, h:       sub-region dimensions in pixels
    """
    draw_w = w - 2 * margin
    draw_h = h - 2 * margin

    # Total chordwise extent includes sweep offset
    extent_x = max(root_chord_mm, abs(sweep_offset_mm) + root_chord_mm,
                   sweep_offset_mm + root_chord_mm)
    min_x = min(0.0, sweep_offset_mm)
    extent_x = extent_x - min_x

    # Scale to fit both halves (mirrored planform: total span = 2 * half_span)
    total_span = 2.0 * half_span_mm
    if extent_x <= 0 or total_span <= 0:
        return ox + margin, oy + margin

    scale = min(draw_w / extent_x, draw_h / total_span) * 0.90

    # Center horizontally and vertically
    used_w = extent_x * scale
    used_h = total_span * scale
    pad_x = (draw_w - used_w) / 2.0
    pad_y = (draw_h - used_h) / 2.0

    px = ox + margin + pad_x + (x_mm - min_x) * scale
    # y_span_mm=0 is root (center), positive goes up in the top half
    # In pixel space, root is at vertical center, tip-top is above, tip-bottom below
    py = oy + margin + pad_y + (half_span_mm - y_span_mm) * scale
    return px, py


def _fill_quad(canvas, quad, color):
    """
    Fill a convex quadrilateral with a solid color using scanline fill.

    quad: list of 4 (px, py) vertices in order.
    """
    r, g, b = color
    # Find vertical pixel bounds
    ys = [p[1] for p in quad]
    y_min = int(math.floor(min(ys)))
    y_max = int(math.ceil(max(ys)))
    y_min = max(0, y_min)
    y_max = min(canvas.height - 1, y_max)

    n = len(quad)
    for y in range(y_min, y_max + 1):
        # Find all x-intercepts of this scanline with the polygon edges
        xs = []
        for i in range(n):
            x0, y0 = quad[i]
            x1, y1 = quad[(i + 1) % n]
            if y0 == y1:
                continue
            if (y0 <= y < y1) or (y1 <= y < y0):
                t = (y - y0) / (y1 - y0)
                xs.append(x0 + t * (x1 - x0))
        if len(xs) >= 2:
            xs.sort()
            x_start = int(round(xs[0]))
            x_end = int(round(xs[-1]))
            x_start = max(0, x_start)
            x_end = min(canvas.width - 1, x_end)
            for x in range(x_start, x_end + 1):
                canvas.set_pixel(x, y, r, g, b)


def _draw_dashed_line(canvas, p0, p1, color, dash=6, gap=4):
    """Draw a dashed line from p0 to p1."""
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length < 1:
        return
    step = dash + gap
    pos = 0.0
    while pos < length:
        seg_end = min(pos + dash, length)
        frac_s = pos / length
        frac_e = seg_end / length
        sx = int(round(x0 + dx * frac_s))
        sy = int(round(y0 + dy * frac_s))
        ex = int(round(x0 + dx * frac_e))
        ey = int(round(y0 + dy * frac_e))
        canvas.draw_line(sx, sy, ex, ey, color, thickness=1)
        pos += step


def _draw_planform(canvas, chord_cm, half_span_mm, tip_chord_mm,
                   sweep_deg, ox, oy, w, h, margin,
                   sections=2, mirror=False, solid=False,
                   split_pct=0.0,
                   cs_inboard_pct=0.0, cs_outboard_pct=0.0):
    """
    Draw a mirrored top-down planform view within a sub-region.

    Root chord is drawn at the vertical center; both wing halves extend
    up and down from it.

    sections:  number of span-wise loft sections (>=2); intermediate
               section lines are drawn at evenly spaced span stations.
    mirror:    when True, draw a dashed center line to indicate the
               mirror/symmetry plane.
    solid:     when True, fill the planform with _CLR_WING_FILL;
               when False, outline only.
    split_pct: chord split percentage; 0 means no split.
    cs_inboard_pct / cs_outboard_pct:  CS span extents as fractions [0..1]
               of half-span.  When split_pct > 0 and outboard > inboard,
               draws the CS region, hinge line, and boundary lines.
    """
    root_chord_mm = chord_cm * 10.0  # cm -> mm
    if half_span_mm <= 0 or root_chord_mm <= 0:
        return

    sweep_offset_mm = half_span_mm * math.tan(math.radians(sweep_deg))

    def _to_px(x_mm, y_span_mm):
        return _planform_to_pixel(x_mm, y_span_mm, ox, oy, w, h, margin,
                                  root_chord_mm, half_span_mm,
                                  sweep_offset_mm)

    # Root chord (center, y_span=0)
    root_le = _to_px(0.0, 0.0)
    root_te = _to_px(root_chord_mm, 0.0)

    # Right wing (top half, positive y_span)
    tip_le_r = _to_px(sweep_offset_mm, half_span_mm)
    tip_te_r = _to_px(sweep_offset_mm + tip_chord_mm, half_span_mm)

    # Left wing (bottom half, negative y_span — mirror)
    tip_le_l = _to_px(sweep_offset_mm, -half_span_mm)
    tip_te_l = _to_px(sweep_offset_mm + tip_chord_mm, -half_span_mm)

    # Fill planform with solid color when solid mode is enabled
    if solid:
        right_outline = [root_le, tip_le_r, tip_te_r, root_te]
        left_outline = [root_le, tip_le_l, tip_te_l, root_te]
        _fill_quad(canvas, right_outline, _CLR_WING_FILL)
        _fill_quad(canvas, left_outline, _CLR_WING_FILL)

    # Draw outlines
    right_outline = [root_le, tip_le_r, tip_te_r, root_te]
    canvas.draw_polyline(right_outline, _CLR_WING, thickness=2, closed=True)

    left_outline = [root_le, tip_le_l, tip_te_l, root_te]
    canvas.draw_polyline(left_outline, _CLR_WING, thickness=2, closed=True)

    # Section lines at intermediate span stations
    sections = max(2, sections)
    if sections > 2:
        for i in range(1, sections):
            frac = i / float(sections)
            span_y = frac * half_span_mm
            sweep_at = frac * sweep_offset_mm
            chord_at = root_chord_mm + frac * (tip_chord_mm - root_chord_mm)
            le = _to_px(sweep_at, span_y)
            te = _to_px(sweep_at + chord_at, span_y)
            canvas.draw_line(int(round(le[0])), int(round(le[1])),
                             int(round(te[0])), int(round(te[1])),
                             _CLR_WING, thickness=1)
            # Mirror side
            le_m = _to_px(sweep_at, -span_y)
            te_m = _to_px(sweep_at + chord_at, -span_y)
            canvas.draw_line(int(round(le_m[0])), int(round(le_m[1])),
                             int(round(te_m[0])), int(round(te_m[1])),
                             _CLR_WING, thickness=1)

    # Control surface span region
    if split_pct > 0 and cs_outboard_pct > cs_inboard_pct:
        split_frac = split_pct / 100.0
        ib = cs_inboard_pct   # fraction of half-span
        ob = cs_outboard_pct  # fraction of half-span

        for sign in (1.0, -1.0):
            ib_span = ib * half_span_mm * sign
            ob_span = ob * half_span_mm * sign

            # Chord and sweep offset at each station
            ib_frac = ib
            ob_frac = ob
            sweep_ib = ib_frac * sweep_offset_mm
            sweep_ob = ob_frac * sweep_offset_mm
            chord_ib = root_chord_mm + ib_frac * (tip_chord_mm - root_chord_mm)
            chord_ob = root_chord_mm + ob_frac * (tip_chord_mm - root_chord_mm)

            # Hinge (split) x position at each station
            hinge_x_ib = sweep_ib + split_frac * chord_ib
            hinge_x_ob = sweep_ob + split_frac * chord_ob

            # Trailing edge x at each station
            te_x_ib = sweep_ib + chord_ib
            te_x_ob = sweep_ob + chord_ob

            # Four corners of the CS trapezoid (hinge-to-TE, ib-to-ob)
            p_hinge_ib = _to_px(hinge_x_ib, ib_span)
            p_te_ib = _to_px(te_x_ib, ib_span)
            p_te_ob = _to_px(te_x_ob, ob_span)
            p_hinge_ob = _to_px(hinge_x_ob, ob_span)

            # Fill the CS region
            _fill_quad(canvas, [p_hinge_ib, p_te_ib, p_te_ob, p_hinge_ob],
                       _CLR_CS_FILL)

            # Hinge line (dashed, from inboard to outboard)
            _draw_dashed_line(canvas, p_hinge_ib, p_hinge_ob, _CLR_SPLIT,
                              dash=4, gap=3)

            # Chord-wise boundary at inboard station
            canvas.draw_line(int(round(p_hinge_ib[0])), int(round(p_hinge_ib[1])),
                             int(round(p_te_ib[0])), int(round(p_te_ib[1])),
                             _CLR_SPLIT, thickness=1)
            # Chord-wise boundary at outboard station
            canvas.draw_line(int(round(p_hinge_ob[0])), int(round(p_hinge_ob[1])),
                             int(round(p_te_ob[0])), int(round(p_te_ob[1])),
                             _CLR_SPLIT, thickness=1)

    # Root chord line (emphasized)
    canvas.draw_line(int(round(root_le[0])), int(round(root_le[1])),
                     int(round(root_te[0])), int(round(root_te[1])),
                     _CLR_AIRFOIL, thickness=2)

    # Dashed center line when mirror mode is active
    if mirror:
        _draw_dashed_line(canvas, root_le, root_te, _CLR_SPLIT,
                          dash=6, gap=4)


def _find_surface_y(coords, x_frac):
    """Find upper and lower Y at a given X fraction. Returns (upper_y, lower_y)."""
    result = Geometry.find_upper_lower_at(coords, x_frac)
    if result is None:
        return (0.0, 0.0)
    _, upper_pt, _, lower_pt, _, _ = result
    return (upper_pt[1], lower_pt[1])


# -- Simple airfoil shape for icons -------------------------------------------

def _simple_airfoil():
    """
    Return a simple NACA-like airfoil shape as normalised coords.
    Approximate NACA 0012 using the standard thickness formula.
    """
    pts = []
    n = 40
    # Upper surface: TE to LE
    for i in range(n, -1, -1):
        x = (i / n) ** 1.5  # cosine-like spacing via power
        # NACA 0012 half-thickness
        yt = 0.12 / 0.2 * (
            0.2969 * math.sqrt(x)
            - 0.1260 * x
            - 0.3516 * x * x
            + 0.2843 * x * x * x
            - 0.1015 * x * x * x * x
        )
        pts.append((x, yt))
    # Lower surface: LE to TE (skip LE duplicate)
    for i in range(1, n + 1):
        x = (i / n) ** 1.5
        yt = 0.12 / 0.2 * (
            0.2969 * math.sqrt(x)
            - 0.1260 * x
            - 0.3516 * x * x
            + 0.2843 * x * x * x
            - 0.1015 * x * x * x * x
        )
        pts.append((x, -yt))
    return pts


def render_icon(icon_type, width=300, height=60):
    """
    Generate a small section icon for group headers.

    Parameters
    ----------
    icon_type : str
        One of "profile", "split", "offset", "spars".

    Returns PNG bytes.
    """
    canvas = Canvas(width, height)
    canvas.fill(*_CLR_BACKGROUND)

    margin = 6
    coords = _simple_airfoil()
    pixels = _coords_to_pixels(coords, width, height, margin)

    if icon_type == "profile":
        canvas.draw_polyline(pixels, _CLR_AIRFOIL, thickness=2, closed=True)

    elif icon_type == "split":
        canvas.draw_polyline(pixels, _CLR_AIRFOIL, thickness=2, closed=True)
        # Vertical split line at 75%
        upper_y, lower_y = _find_surface_y(coords, 0.75)
        px0, py0 = _norm_to_pixel(0.75, upper_y, width, height, margin)
        px1, py1 = _norm_to_pixel(0.75, lower_y, width, height, margin)
        canvas.draw_line(int(round(px0)), int(round(py0)),
                         int(round(px1)), int(round(py1)),
                         _CLR_SPLIT, thickness=1)

    elif icon_type == "offset":
        canvas.draw_polyline(pixels, _CLR_AIRFOIL, thickness=2, closed=True)
        # Simplified inner contour: scale coords inward
        inner = []
        # Center of airfoil (approx)
        cx_avg = sum(c[0] for c in coords) / len(coords)
        cy_avg = sum(c[1] for c in coords) / len(coords)
        shrink = 0.85
        for x, y in coords:
            ix = cx_avg + (x - cx_avg) * shrink
            iy = cy_avg + (y - cy_avg) * shrink
            inner.append((ix, iy))
        inner_px = _coords_to_pixels(inner, width, height, margin)
        canvas.draw_polyline(inner_px, _CLR_OFFSET, thickness=1, closed=True)

    elif icon_type == "spars":
        canvas.draw_polyline(pixels, _CLR_AIRFOIL, thickness=2, closed=True)
        # Two circles at ~25% and ~65% chord
        for spar_x in (0.25, 0.65):
            upper_y, lower_y = _find_surface_y(coords, spar_x)
            cy = (upper_y + lower_y) / 2.0
            thickness = upper_y - lower_y
            px, py = _norm_to_pixel(spar_x, cy, width, height, margin)
            draw_h = height - 2 * margin
            r_px = (thickness / 2.0) * draw_h * _Y_SCALE * 0.6
            canvas.draw_circle(px, py, r_px, _CLR_SPAR, thickness=1)

    elif icon_type == "wing":
        # Simplified planform trapezoid icon
        draw_w = width - 2 * margin
        draw_h = height - 2 * margin
        cx_mid = width / 2.0
        cy_mid = height / 2.0
        # Root chord (horizontal, centered)
        root_half = draw_w * 0.35
        # Tip chord (shorter, offset for sweep)
        tip_half = draw_w * 0.18
        span_half = draw_h * 0.38
        sweep_px = draw_w * 0.06  # slight sweep offset
        # Right half
        r_pts = [
            (cx_mid - root_half, cy_mid),
            (cx_mid - tip_half + sweep_px, cy_mid - span_half),
            (cx_mid + tip_half + sweep_px, cy_mid - span_half),
            (cx_mid + root_half, cy_mid),
        ]
        canvas.draw_polyline(r_pts, _CLR_WING, thickness=2, closed=True)
        # Left half (mirror)
        l_pts = [
            (cx_mid - root_half, cy_mid),
            (cx_mid - tip_half + sweep_px, cy_mid + span_half),
            (cx_mid + tip_half + sweep_px, cy_mid + span_half),
            (cx_mid + root_half, cy_mid),
        ]
        canvas.draw_polyline(l_pts, _CLR_WING, thickness=2, closed=True)

    return canvas.to_png_bytes()


# -- File I/O -----------------------------------------------------------------

def write_file(png_bytes, filepath):
    """Write PNG bytes to disk (binary mode)."""
    dirpath = os.path.dirname(filepath)
    if dirpath and not os.path.isdir(dirpath):
        os.makedirs(dirpath, exist_ok=True)
    with open(filepath, "wb") as f:
        f.write(png_bytes)
