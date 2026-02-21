"""
Airfoil Utility Functions
-------------------------
Pure-Python helpers for parsing .dat airfoil files, interpolating
coordinate lists, and locating control-surface split points.

No Fusion 360 dependencies — safe to unit-test standalone.
"""

import os
import math

_EPSILON = 1e-12
_COLLAPSE_FRACTION = 0.1   # fraction of offset used as collapse threshold


# -- .dat File Parsing ---------------------------------------------------------

def parse_dat_file(filepath):
    """
    Parse a Selig or Lednicer format .dat file.

    Selig format  : name on line 1, then x y pairs going TE->upper->LE->lower->TE
    Lednicer format: name on line 1, point counts on line 2, then upper surface
                     (LE->TE) followed by lower surface (LE->TE)

    Returns (name: str, coords: [(x, y), ...])
    """
    with open(filepath, "r") as fh:
        raw_lines = fh.readlines()

    if not raw_lines:
        raise ValueError(f"Empty file: {filepath}")

    name = raw_lines[0].strip()

    # Collect non-empty data lines after the header
    data_lines = [l.strip() for l in raw_lines[1:] if l.strip()]

    if not data_lines:
        raise ValueError(f"No coordinate data in {filepath!r}")

    def try_floats(line):
        try:
            return [float(t) for t in line.split()]
        except ValueError:
            return []

    first_vals = try_floats(data_lines[0])

    # Lednicer detection: first data line has exactly 2 tokens, both are
    # whole numbers >= 2 (they are point counts, e.g. "17.   17.")
    is_lednicer = (
        len(first_vals) == 2
        and all(v == int(v) and v >= 2 for v in first_vals)
    )

    if is_lednicer:
        n_upper = int(first_vals[0])
        n_lower = int(first_vals[1])
        coord_lines = data_lines[1:]

        def parse_pair(line):
            v = try_floats(line)
            if len(v) < 2:
                raise ValueError(f"Bad coordinate line: {line!r}")
            return (v[0], v[1])

        upper = [parse_pair(coord_lines[i]) for i in range(n_upper)]
        lower = [parse_pair(coord_lines[n_upper + i]) for i in range(n_lower)]

        # Lednicer: upper goes LE(0) -> TE(1), lower goes LE(0) -> TE(1)
        # Convert to a single closed loop: TE -> upper -> LE -> lower -> TE
        coords = list(reversed(upper)) + lower[1:]  # skip duplicate LE point

    else:
        coords = []
        for line in data_lines:
            vals = try_floats(line)
            if len(vals) >= 2:
                coords.append((vals[0], vals[1]))

    if not coords:
        raise ValueError(f"Could not parse any coordinates from {filepath!r}")

    return name, coords


def load_data_folder(script_dir):
    """
    Scan <script_dir>/data/ for *.dat files.

    Returns
    -------
    airfoils : dict  { display_name: (filepath, coords) }
    errors   : list  of (filename, error_message) tuples
    """
    data_dir = os.path.join(script_dir, "data")
    airfoils = {}
    errors   = []

    if not os.path.isdir(data_dir):
        return airfoils, errors

    for filename in sorted(os.listdir(data_dir)):
        if not filename.lower().endswith(".dat"):
            continue
        filepath = os.path.join(data_dir, filename)
        try:
            name, coords = parse_dat_file(filepath)
            display = name
            suffix  = 2
            while display in airfoils:
                display = f"{name} ({suffix})"
                suffix += 1
            airfoils[display] = (filepath, coords)
        except Exception as exc:
            errors.append((filename, str(exc)))

    return airfoils, errors


# -- Geometry Helpers ----------------------------------------------------------

def double_points(coords):
    """
    Insert a midpoint between each consecutive pair of vertices,
    approximately doubling the point count.

    Parameters
    ----------
    coords : list of (x, y) tuples

    Returns
    -------
    doubled : list of (x, y) tuples  (~2N - 1 points)
    """
    if len(coords) < 2:
        return list(coords)

    result = []
    for i in range(len(coords) - 1):
        x0, y0 = coords[i]
        x1, y1 = coords[i + 1]
        result.append((x0, y0))
        result.append(((x0 + x1) / 2.0, (y0 + y1) / 2.0))
    result.append(coords[-1])
    return result


def _split_surfaces(coords):
    """
    Split the airfoil contour into upper and lower segments at the LE.

    Returns (le_idx, upper_seg, lower_seg) where:
      upper_seg = coords[0 .. le_idx]   (TE -> LE, X decreasing)
      lower_seg = coords[le_idx .. end]  (LE -> TE, X increasing)
    """
    le_idx = min(range(len(coords)), key=lambda i: coords[i][0])
    return le_idx, coords[:le_idx + 1], coords[le_idx:]


def _find_crossing(segment, x_target):
    """
    Linear-interpolate to find where *segment* crosses *x_target*.

    Returns (index, point) where *index* is the second element of the
    crossing pair, or (None, None) if no crossing is found.
    """
    for j in range(1, len(segment)):
        xa, ya = segment[j - 1]
        xb, yb = segment[j]
        if xa == xb:
            continue
        if (xa - x_target) * (xb - x_target) <= 0:
            t = (x_target - xa) / (xb - xa)
            return j, (x_target, ya + t * (yb - ya))
    return None, None


def _pts_equal(a, b):
    """Check if two points are effectively identical."""
    return abs(a[0] - b[0]) < _EPSILON and abs(a[1] - b[1]) < _EPSILON


def _find_upper_lower_at(coords, x_frac):
    """
    Find upper and lower surface points at normalised X position.

    Returns (upper_idx, upper_pt, lower_idx, lower_pt, upper_seg, lower_seg)
    or None if *x_frac* is outside the airfoil range.
    """
    if not coords or x_frac <= 0.0 or x_frac >= 1.0:
        return None
    _, upper_seg, lower_seg = _split_surfaces(coords)
    j, upper_pt = _find_crossing(upper_seg, x_frac)
    k, lower_pt = _find_crossing(lower_seg, x_frac)
    if upper_pt is None or lower_pt is None:
        return None
    return j, upper_pt, k, lower_pt, upper_seg, lower_seg


# -- Public Surface Query Functions --------------------------------------------

def compute_spar_center(coords, chord_frac):
    """
    Return (center_x, center_y, thickness) at the given chord fraction,
    or None if the position is outside the airfoil.

    All values are in normalised [0..1] space.
    """
    result = _find_upper_lower_at(coords, chord_frac)
    if result is None:
        return None
    _, upper_pt, _, lower_pt, _, _ = result
    y_upper, y_lower = upper_pt[1], lower_pt[1]
    return (chord_frac, (y_upper + y_lower) / 2.0, y_upper - y_lower)


def find_optimal_spar_positions(coords, spar_diameter_norm, clearance_norm,
                                max_x_frac=1.0):
    """
    Find fore and aft spar X positions that maximise chordwise separation
    while maintaining minimum clearance between each spar edge and the
    airfoil surface.

    Scans the airfoil from the LE to *max_x_frac* (typically the control-
    surface split line) and places the fore spar at the leftmost valid X
    and the aft spar at the rightmost valid X.

    Parameters
    ----------
    coords : list of (x, y)
        Normalised airfoil contour.
    spar_diameter_norm : float
        Spar diameter in normalised [0..1] chord space.
    clearance_norm : float
        Minimum clearance between spar edge and surface, normalised.
    max_x_frac : float
        Rightmost allowed X fraction (e.g. split line).

    Returns
    -------
    list of (x, y_center, thickness) with 0, 1, or 2 entries.
    """
    min_thickness = spar_diameter_norm + 2.0 * clearance_norm
    cap = min(max_x_frac, 0.999)

    # Scan at 0.1% chord resolution
    valid = []
    for i in range(1, 1000):
        x = cap * i / 1000.0
        result = _find_upper_lower_at(coords, x)
        if result is None:
            continue
        _, upper_pt, _, lower_pt, _, _ = result
        thickness = upper_pt[1] - lower_pt[1]
        if thickness >= min_thickness:
            y_center = (upper_pt[1] + lower_pt[1]) / 2.0
            valid.append((x, y_center, thickness))

    if not valid:
        return []

    fore = valid[0]
    aft = valid[-1]

    # If the two positions overlap, return a single centered spar
    if aft[0] - fore[0] < spar_diameter_norm:
        return [valid[len(valid) // 2]]

    return [fore, aft]


def find_split_points(coords, split_frac):
    """
    Walk the closed airfoil contour and find the upper- and lower-surface
    intersection points at *x = split_frac* (normalised chord).

    The standard contour order is  TE -> upper surface -> LE -> lower surface -> TE.
    The leading-edge (LE) vertex is the one with the smallest X value.

    Returns
    -------
    (upper_xy, lower_xy) : two (x, y) tuples, or None if the split
                           fraction falls outside the airfoil X range.
    """
    result = _find_upper_lower_at(coords, split_frac)
    if result is None:
        return None
    _, upper_pt, _, lower_pt, _, _ = result
    return upper_pt, lower_pt


# -- Offset Contour ------------------------------------------------------------

def _signed_area(coords):
    """Signed area of a closed polygon (shoelace formula).  Positive = CCW."""
    total = 0.0
    n = len(coords)
    for i in range(n):
        x0, y0 = coords[i]
        x1, y1 = coords[(i + 1) % n]
        total += x0 * y1 - x1 * y0
    return total / 2.0


def _point_in_polygon(x, y, polygon):
    """Ray-casting point-in-polygon test for a closed polygon."""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and \
           (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _trim_to_interior(offset_pts, original_coords):
    """
    Remove offset points that escaped outside the original contour.

    The closed-polyline drawing connects the last surviving point back to
    the first, creating a flat inner wall where the trailing edge is too
    thin for the full offset.
    """
    return [p for p in offset_pts
            if _point_in_polygon(p[0], p[1], original_coords)]


def offset_contour(coords, offset_norm):
    """
    Offset a closed contour inward by *offset_norm* (in normalised space).

    Winding direction is auto-detected via the signed area so this works
    for both the main airfoil (CCW in Y-up) and the control-surface
    contour (CW).

    Each vertex is displaced along the average of the inward normals of its
    two adjacent edges.  Points that escape outside the original contour
    (common at the sharp trailing edge) are trimmed, and remaining
    consecutive points closer than 10% of *offset_norm* are collapsed.

    Returns a new list of (x, y) tuples.
    """
    n = len(coords)
    if n < 3 or offset_norm <= 0:
        return list(coords)

    # Determine inward-normal sign from winding direction.
    # CCW (positive area): inward normal of edge (dx,dy) is (-dy, dx)  -> sign = +1
    # CW  (negative area): inward normal of edge (dx,dy) is ( dy,-dx) -> sign = -1
    sign = 1.0 if _signed_area(coords) >= 0 else -1.0

    # Compute per-edge unit inward normals
    edge_normals = []
    for i in range(n):
        x0, y0 = coords[i]
        x1, y1 = coords[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < _EPSILON:
            edge_normals.append((0.0, 0.0))
        else:
            edge_normals.append((sign * -dy / length, sign * dx / length))

    # Offset each vertex along the averaged normal of its two adjacent edges
    raw = []
    for i in range(n):
        nx_prev, ny_prev = edge_normals[(i - 1) % n]
        nx_curr, ny_curr = edge_normals[i]
        nx = nx_prev + nx_curr
        ny = ny_prev + ny_curr
        length = math.hypot(nx, ny)
        if length < _EPSILON:
            raw.append(coords[i])
        else:
            nx /= length
            ny /= length
            x, y = coords[i]
            raw.append((x + nx * offset_norm, y + ny * offset_norm))

    # Remove points that escaped outside the original contour at the TE
    raw = _trim_to_interior(raw, coords)

    # Collapse clusters of close points (e.g. where TE converges)
    result = _collapse_close_points(raw, offset_norm * _COLLAPSE_FRACTION)
    return result


def _collapse_close_points(coords, min_dist):
    """
    Merge consecutive points that are closer than *min_dist* into a single
    averaged point.  Also handles the wrap-around (last <-> first).
    """
    if len(coords) < 3 or min_dist <= 0:
        return list(coords)

    # Walk forward, clustering consecutive close points
    clusters = []
    cluster = [coords[0]]
    for i in range(1, len(coords)):
        px, py = cluster[-1]
        cx, cy = coords[i]
        if math.hypot(cx - px, cy - py) < min_dist:
            cluster.append(coords[i])
        else:
            clusters.append(cluster)
            cluster = [coords[i]]
    clusters.append(cluster)

    # Merge first and last clusters if they are close (wrap-around at TE)
    if len(clusters) > 1:
        ax, ay = clusters[-1][-1]
        bx, by = clusters[0][0]
        if math.hypot(bx - ax, by - ay) < min_dist:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

    # Average each cluster into a single point
    result = []
    for cl in clusters:
        n = len(cl)
        result.append((
            sum(p[0] for p in cl) / n,
            sum(p[1] for p in cl) / n,
        ))

    return result


# -- Control Surface -----------------------------------------------------------

def extract_control_surface(coords, split_frac):
    """
    Extract the closed contour of the control-surface segment (the
    portion of the airfoil aft of *split_frac*).

    Returns a list of (x, y) tuples tracing:
        upper_split -> upper surface -> TE -> lower surface -> lower_split

    The caller should create a closed spline from these points.
    Returns None if the split cannot be computed.
    """
    result = _find_upper_lower_at(coords, split_frac)
    if result is None:
        return None

    j, upper_pt, k, lower_pt, upper_seg, lower_seg = result

    # upper_seg[:j] goes TE -> near-split (X decreasing).
    # Reverse to get split -> TE direction, prepend the exact split point.
    upper_part = [upper_pt] + list(reversed(upper_seg[:j]))

    # lower_seg[k:] goes near-split -> TE (X increasing).
    # Reverse to get TE -> split direction, append the exact split point
    # only if it isn't already the last vertex (avoids duplicate when
    # the crossing lands exactly on an existing vertex).
    rev_lower = list(reversed(lower_seg[k:]))
    if rev_lower and _pts_equal(rev_lower[-1], lower_pt):
        lower_part = rev_lower
    else:
        lower_part = rev_lower + [lower_pt]

    # Combine; skip duplicate TE between the two halves.
    return upper_part + lower_part[1:]
