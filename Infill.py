"""
Infill — Spar Placement + Lightening Holes
-------------------------------------------
Finds optimal spar positions and places variable-diameter lightening
holes between them.

Pure Python — only math + Geometry. No Fusion 360 dependencies.
"""

import Geometry


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
    spar_radius = spar_diameter_norm / 2.0
    cap = min(max_x_frac - spar_radius - clearance_norm, 0.999)

    # Scan at 0.1% chord resolution
    valid = []
    for i in range(1, 1000):
        x = cap * i / 1000.0
        result = Geometry.find_upper_lower_at(coords, x)
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


def find_lightening_holes(coords, fore_spar_x, aft_spar_x,
                           spar_diameter_norm, offset_norm, clearance_norm):
    """
    Place variable-diameter lightening holes between two spars.

    Uses a two-pass approach:
      1. Greedy walk to estimate how many holes fit.
      2. Distribute that many centers evenly, then size each hole to
         its local available space (larger in the thick middle, smaller
         near the spars).  Capped so adjacent holes maintain at least
         *clearance_norm* web between edges.

    Parameters
    ----------
    coords : list of (x, y)
        Normalised airfoil contour.
    fore_spar_x, aft_spar_x : float
        Normalised X positions of the two spar centers.
    spar_diameter_norm : float
        Spar diameter, normalised.
    offset_norm : float
        Skin offset distance, normalised.
    clearance_norm : float
        Minimum wall/web thickness, normalised.

    Returns
    -------
    list of (x, y_center, diameter) in normalised space.
    """
    spar_radius = spar_diameter_norm / 2.0
    x_start = fore_spar_x + spar_radius + clearance_norm
    x_end = aft_spar_x - spar_radius - clearance_norm

    if x_end <= x_start:
        return []

    def _max_diam_at(x_frac):
        """Max hole diameter that fits inside the offset shell at *x_frac*."""
        info = Geometry.compute_spar_center(coords, min(x_frac, 0.999))
        if info is None:
            return 0.0
        _, _, thickness = info
        return max(0.0, thickness - 2.0 * offset_norm)

    # Pass 1: greedy walk to estimate hole count
    n_est = 0
    x_left = x_start
    while x_left < x_end:
        diam = _max_diam_at(x_left)
        if diam <= 0:
            x_left += clearance_norm
            continue
        r = diam / 2.0
        diam = min(diam, _max_diam_at(x_left + r))
        if diam <= 0:
            x_left += clearance_norm
            continue
        r = diam / 2.0
        if x_left + 2.0 * r > x_end:
            break
        n_est += 1
        x_left += diam + clearance_norm

    # Pass 2: distribute centers evenly, size each at its position.
    # Try n_est first; if any hole can't fit, reduce count and retry.
    for n in range(n_est, 0, -1):
        band = (x_end - x_start) / n
        if band <= clearance_norm:
            continue
        holes = []
        valid = True
        for i in range(n):
            cx = x_start + band * (i + 0.5)
            if cx <= 0.0 or cx >= 1.0:
                valid = False
                break
            diam = _max_diam_at(cx)
            # Cap to band width minus clearance to prevent overlap
            diam = min(diam, band - clearance_norm)
            if diam <= 0:
                valid = False
                break
            info = Geometry.compute_spar_center(coords, cx)
            if info is None:
                valid = False
                break
            _, cy, _ = info
            holes.append((cx, cy, diam))
        if valid and holes:
            return holes

    return []
