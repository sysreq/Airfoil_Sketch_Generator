"""
Offset Contour
--------------
Inward offset of a closed airfoil contour, with optional truncation
at the fore spar.

Pure Python — only math. No Fusion 360 dependencies.
"""

import math

_EPSILON = 1e-12
_COLLAPSE_FRACTION = 0.1   # fraction of offset used as collapse threshold


# -- Circle / Tangent Helpers --------------------------------------------------

def _tangent_to_circle(px, py, cx, cy, r):
    """Return two tangent points on circle (cx,cy,r) from external point (px,py)."""
    dx, dy = px - cx, py - cy
    d = math.hypot(dx, dy)
    if d <= r:
        return None  # point inside circle
    phi = math.atan2(dy, dx)
    alpha = math.acos(r / d)
    return (
        (cx + r * math.cos(phi + alpha), cy + r * math.sin(phi + alpha)),
        (cx + r * math.cos(phi - alpha), cy + r * math.sin(phi - alpha)),
    )


def _arc_points(cx, cy, r, angle_start, angle_end, n=12):
    """Generate n points along a circular arc from angle_start to angle_end (radians)."""
    pts = []
    for i in range(n):
        t = i / max(n - 1, 1)
        a = angle_start + t * (angle_end - angle_start)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


# -- Polygon Helpers -----------------------------------------------------------

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


def _seg_intersect(a1, a2, b1, b2):
    """Find the intersection point of segments a1-a2 and b1-b2, or None."""
    dx_a, dy_a = a2[0] - a1[0], a2[1] - a1[1]
    dx_b, dy_b = b2[0] - b1[0], b2[1] - b1[1]
    denom = dx_a * dy_b - dy_a * dx_b
    if abs(denom) < _EPSILON:
        return None
    dx_ab, dy_ab = b1[0] - a1[0], b1[1] - a1[1]
    t = (dx_ab * dy_b - dy_ab * dx_b) / denom
    u = (dx_ab * dy_a - dy_ab * dx_a) / denom
    if 0.0 < t < 1.0 and 0.0 < u < 1.0:
        return (a1[0] + t * dx_a, a1[1] + t * dy_a)
    return None


def _trim_te_crossing(coords):
    """Trim self-intersection near the trailing edge of an offset contour.

    Works for both the main airfoil (TE at the start/end wrap-around)
    and the control-surface contour (TE in the middle).  Finds the TE
    as the max-X point, checks segments approaching the TE against
    segments leaving it, and keeps the non-TE loop when a crossing
    is found.
    """
    n = len(coords)
    if n < 4:
        return coords

    te_idx = max(range(n), key=lambda k: coords[k][0])
    max_check = min(n // 3, 8)

    for di in range(1, max_check + 1):
        i = (te_idx - di) % n
        i_next = (i + 1) % n
        for dj in range(0, max_check):
            j = (te_idx + dj) % n
            j_next = (j + 1) % n
            if i == j or i_next == j or i == j_next or i_next == j_next:
                continue
            pt = _seg_intersect(coords[i], coords[i_next],
                                coords[j], coords[j_next])
            if pt is not None:
                # Walk from j_next around the non-TE side back to i
                result = [pt]
                idx = j_next
                steps = 0
                while idx != i_next and steps < n:
                    result.append(coords[idx])
                    idx = (idx + 1) % n
                    steps += 1
                return result

    return coords


# -- Collapse Close Points -----------------------------------------------------

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


# -- Offset Contour ------------------------------------------------------------

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

    # Trim where upper and lower offset paths cross near the TE
    result = _trim_te_crossing(result)
    return result


def truncate_offset_at_spar(offset_pts, spar_cx, spar_cy, spar_radius):
    """Truncate offset contour at the fore spar, replacing the LE section
    with tangent lines and an arc along the spar circle.

    Finds the closest offset-contour points (upper and lower) whose X
    coordinate is past the spar's aft edge, draws tangent lines from
    each to the spar circle, and connects them with an arc.  All nose
    geometry forward of the spar is removed.

    Returns modified contour, or original if truncation not possible."""
    if len(offset_pts) < 4:
        return offset_pts

    # Find LE index (minimum X point) in offset contour
    le_idx = min(range(len(offset_pts)), key=lambda i: offset_pts[i][0])
    threshold = spar_cx + spar_radius

    # Upper side: scan from LE toward TE (decreasing index) to find
    # the first point with X past the spar's aft edge.
    upper_idx = None
    for i in range(le_idx, -1, -1):
        if offset_pts[i][0] > threshold:
            upper_idx = i
            break
    if upper_idx is None:
        return offset_pts

    # Lower side: scan from LE toward TE (increasing index).
    lower_idx = None
    for i in range(le_idx, len(offset_pts)):
        if offset_pts[i][0] > threshold:
            lower_idx = i
            break
    if lower_idx is None:
        return offset_pts

    upper_pt = offset_pts[upper_idx]
    lower_pt = offset_pts[lower_idx]

    # Compute tangent from upper trim point to spar circle
    tan_upper = _tangent_to_circle(
        upper_pt[0], upper_pt[1], spar_cx, spar_cy, spar_radius
    )
    if tan_upper is None:
        return offset_pts
    # Pick the tangent point with higher Y (upper side)
    upper_tan = tan_upper[0] if tan_upper[0][1] >= tan_upper[1][1] else tan_upper[1]

    # Compute tangent from lower trim point to spar circle
    tan_lower = _tangent_to_circle(
        lower_pt[0], lower_pt[1], spar_cx, spar_cy, spar_radius
    )
    if tan_lower is None:
        return offset_pts
    # Pick the tangent point with lower Y (lower side)
    lower_tan = tan_lower[0] if tan_lower[0][1] <= tan_lower[1][1] else tan_lower[1]

    # Arc from upper tangent to lower tangent going around the LE side (through pi)
    angle_upper = math.atan2(upper_tan[1] - spar_cy, upper_tan[0] - spar_cx)
    angle_lower = math.atan2(lower_tan[1] - spar_cy, lower_tan[0] - spar_cx)

    # Ensure we go the LE-side way (through pi / the left side of the circle)
    if angle_upper < angle_lower:
        angle_upper += 2.0 * math.pi

    arc = _arc_points(spar_cx, spar_cy, spar_radius,
                      angle_upper, angle_lower, n=16)

    # Assemble: TE → upper side → tangent → arc → tangent → lower side → TE
    # offset_pts[:upper_idx+1] includes the trim point; the polyline then
    # draws straight to upper_tan (the tangent line).
    result = (
        offset_pts[:upper_idx + 1]
        + [upper_tan]
        + arc
        + [lower_tan]
        + offset_pts[lower_idx:]
    )
    return result
