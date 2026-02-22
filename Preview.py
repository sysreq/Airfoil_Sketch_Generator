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


# -- Colors -------------------------------------------------------------------

_CLR_BACKGROUND  = (240, 240, 240)
_CLR_AIRFOIL     = (30, 60, 120)
_CLR_OFFSET      = (100, 140, 200)
_CLR_SPLIT       = (200, 50, 50)
_CLR_SPAR        = (50, 160, 50)


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
        row = pixel * self.width
        for y in range(self.height):
            offset = y * self.width * 4
            self._buf[offset:offset + self.width * 4] = row

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
    compressed = zlib.compress(bytes(raw_rows), 6)
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
    Airfoil Y typically spans about [-0.06 .. 0.12] for a NACA 0012.
    We map Y range [-0.15 .. 0.20] to fill the vertical space.
    """
    draw_w = width - 2 * margin
    draw_h = height - 2 * margin

    px = margin + x_norm * draw_w
    # Center Y=0 at 57% from top (airfoils have more camber on upper surface)
    py = margin + draw_h * 0.57 - y_norm * draw_h * 2.85
    return px, py


def _coords_to_pixels(coords, width, height, margin):
    """Convert a list of normalised (x,y) tuples to pixel coords."""
    return [_norm_to_pixel(x, y, width, height, margin) for x, y in coords]


# -- Main Render Functions ----------------------------------------------------

def render_preview(coords, chord_cm=10.0, enable_split=False, split_pct=75.0,
                   enable_offset=False, offset_mm=2.0, enable_spars=False,
                   spars=None, enable_lightening=False, lightening_holes=None,
                   cs_coords=None, offset_pts=None,
                   width=360, height=180):
    """
    Render a preview PNG of the airfoil with optional features.

    All geometry inputs (coords, offset_pts, spars, lightening_holes,
    cs_coords) are in NORMALIZED [0..1] space.

    Returns PNG bytes.
    """
    canvas = Canvas(width, height)
    canvas.fill(*_CLR_BACKGROUND)

    margin = 12

    # Main airfoil contour
    if coords:
        pixels = _coords_to_pixels(coords, width, height, margin)
        canvas.draw_polyline(pixels, _CLR_AIRFOIL, thickness=2, closed=True)

    # Offset contour
    if enable_offset and offset_pts:
        off_pixels = _coords_to_pixels(offset_pts, width, height, margin)
        canvas.draw_polyline(off_pixels, _CLR_OFFSET, thickness=1, closed=True)

    # Control surface contour
    if cs_coords:
        cs_pixels = _coords_to_pixels(cs_coords, width, height, margin)
        canvas.draw_polyline(cs_pixels, _CLR_AIRFOIL, thickness=2, closed=True)

    # Split line
    if enable_split and coords:
        split_x = split_pct / 100.0
        # Find approximate upper and lower Y at split position by scanning coords
        upper_y, lower_y = _find_surface_y(coords, split_x)
        px0, py0 = _norm_to_pixel(split_x, upper_y, width, height, margin)
        px1, py1 = _norm_to_pixel(split_x, lower_y, width, height, margin)
        canvas.draw_line(int(round(px0)), int(round(py0)),
                         int(round(px1)), int(round(py1)),
                         _CLR_SPLIT, thickness=1)

    # Spar circles
    if enable_spars and spars:
        for cx, cy, thickness in spars:
            px, py = _norm_to_pixel(cx, cy, width, height, margin)
            # Radius in pixels: thickness/2 scaled like Y
            draw_h = height - 2 * margin
            r_px = (thickness / 2.0) * draw_h * 2.85
            canvas.draw_circle(px, py, r_px, _CLR_SPAR, thickness=1)

    # Lightening holes
    if enable_lightening and lightening_holes:
        for cx, cy, diam in lightening_holes:
            px, py = _norm_to_pixel(cx, cy, width, height, margin)
            draw_h = height - 2 * margin
            r_px = (diam / 2.0) * draw_h * 2.85
            canvas.draw_circle(px, py, r_px, _CLR_SPAR, thickness=1)

    return canvas.to_png_bytes()


def _find_surface_y(coords, x_frac):
    """
    Find approximate upper and lower Y at a given X fraction by
    scanning the coordinate list. Returns (upper_y, lower_y).
    """
    best_upper = 0.0
    best_lower = 0.0
    best_dist = float("inf")
    found = False

    for i in range(len(coords) - 1):
        x0, y0 = coords[i]
        x1, y1 = coords[i + 1]
        if x0 == x1:
            continue
        if (x0 - x_frac) * (x1 - x_frac) <= 0:
            t = (x_frac - x0) / (x1 - x0)
            y = y0 + t * (y1 - y0)
            if not found:
                best_upper = y
                best_lower = y
                found = True
            else:
                if y > best_upper:
                    best_upper = y
                if y < best_lower:
                    best_lower = y

    return (best_upper, best_lower)


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
            r_px = (thickness / 2.0) * draw_h * 2.85 * 0.6
            canvas.draw_circle(px, py, r_px, _CLR_SPAR, thickness=1)

    return canvas.to_png_bytes()


# -- File I/O -----------------------------------------------------------------

def write_file(png_bytes, filepath):
    """Write PNG bytes to disk (binary mode)."""
    dirpath = os.path.dirname(filepath)
    if dirpath and not os.path.isdir(dirpath):
        os.makedirs(dirpath, exist_ok=True)
    with open(filepath, "wb") as f:
        f.write(png_bytes)
