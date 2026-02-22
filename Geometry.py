"""
Airfoil Geometry — Core Contour Math
-------------------------------------
Surface splitting, crossing detection, and control surface extraction.

Pure Python — only math. No Fusion 360 dependencies.
"""

import math

_EPSILON = 1e-12


# -- Point / Contour Helpers ---------------------------------------------------

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


def _pts_equal(a, b):
    """Check if two points are effectively identical."""
    return abs(a[0] - b[0]) < _EPSILON and abs(a[1] - b[1]) < _EPSILON


# -- Surface Splitting --------------------------------------------------------

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


def find_upper_lower_at(coords, x_frac):
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
    result = find_upper_lower_at(coords, chord_frac)
    if result is None:
        return None
    _, upper_pt, _, lower_pt, _, _ = result
    y_upper, y_lower = upper_pt[1], lower_pt[1]
    return (chord_frac, (y_upper + y_lower) / 2.0, y_upper - y_lower)


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
    result = find_upper_lower_at(coords, split_frac)
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

    # Combine; skip duplicate TE only when upper and lower share the same point.
    if upper_part and lower_part and _pts_equal(upper_part[-1], lower_part[0]):
        return upper_part + lower_part[1:]
    return upper_part + lower_part


def build_elevator_contour(coords, split_frac, inset_norm):
    """
    Build a control-surface contour with X-direction inset.

    The CS wall is moved from split_frac to split_frac + inset_norm
    (toward the trailing edge), creating a horizontal gap between
    the main body and the control surface for hinge clearance.
    """
    wall_frac = split_frac + inset_norm
    return extract_control_surface(coords, wall_frac)


def _fillet_corner(p_prev, p_corner, p_next, radius, n_arc=8):
    """
    Replace a sharp corner with a circular fillet arc.

    Returns arc points from the tangent on the (p_prev -> p_corner) edge
    to the tangent on the (p_corner -> p_next) edge.  Falls back to
    [p_corner] when the fillet cannot be computed.
    """
    dx1 = p_prev[0] - p_corner[0]
    dy1 = p_prev[1] - p_corner[1]
    dx2 = p_next[0] - p_corner[0]
    dy2 = p_next[1] - p_corner[1]

    len1 = math.hypot(dx1, dy1)
    len2 = math.hypot(dx2, dy2)
    if len1 < _EPSILON or len2 < _EPSILON:
        return [p_corner]

    dx1, dy1 = dx1 / len1, dy1 / len1
    dx2, dy2 = dx2 / len2, dy2 / len2

    dot = max(-1.0, min(1.0, dx1 * dx2 + dy1 * dy2))
    half = math.acos(dot) / 2.0
    if half < 1e-6:
        return [p_corner]

    sin_half = math.sin(half)
    tan_half = math.tan(half)
    if sin_half < _EPSILON or abs(tan_half) < _EPSILON:
        return [p_corner]

    tan_dist = radius / tan_half

    # Shrink radius if tangent distance exceeds available edge length
    if tan_dist > len1 * 0.9 or tan_dist > len2 * 0.9:
        tan_dist = min(len1, len2) * 0.9
        radius = tan_dist * tan_half

    t1 = (p_corner[0] + dx1 * tan_dist, p_corner[1] + dy1 * tan_dist)
    t2 = (p_corner[0] + dx2 * tan_dist, p_corner[1] + dy2 * tan_dist)

    bx = dx1 + dx2
    by = dy1 + dy2
    blen = math.hypot(bx, by)
    if blen < _EPSILON:
        return [p_corner]
    bx, by = bx / blen, by / blen

    cx = p_corner[0] + bx * (radius / sin_half)
    cy = p_corner[1] + by * (radius / sin_half)

    a1 = math.atan2(t1[1] - cy, t1[0] - cx)
    a2 = math.atan2(t2[1] - cy, t2[0] - cx)
    da = a2 - a1
    if da > math.pi:
        da -= 2.0 * math.pi
    elif da < -math.pi:
        da += 2.0 * math.pi

    return [
        (cx + radius * math.cos(a1 + da * i / n_arc),
         cy + radius * math.sin(a1 + da * i / n_arc))
        for i in range(n_arc + 1)
    ]


def round_cs_wall_corners(contour, radius_norm, n_arc=8):
    """
    Round the two wall-side corners of a control-surface contour.

    The closing segment (last point -> first point) is the wall.
    Filleting these corners allows better range of motion when the
    control surface rotates on its hinge.
    """
    if len(contour) < 4 or radius_norm <= 0:
        return list(contour)

    fillet_top = _fillet_corner(
        contour[-1], contour[0], contour[1], radius_norm, n_arc)
    fillet_bot = _fillet_corner(
        contour[-2], contour[-1], contour[0], radius_norm, n_arc)

    return fillet_top + list(contour[1:-1]) + fillet_bot
