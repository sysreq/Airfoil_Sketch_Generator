"""
Infill — Spar Placement + Lightening Holes
-------------------------------------------
Finds optimal spar positions and places variable-diameter lightening
holes between them.

Pure Python — only math + Geometry. No Fusion 360 dependencies.
"""

import Geometry
import Offset


def compute_2d_geometry(coords, cfg):
    """Compute all 2D geometry features from normalised coords + config.

    Parameters
    ----------
    coords : list of (x, y)
        Normalised [0..1] airfoil contour (already densified).
    cfg : dict
        Config dict with keys: chord_cm, enable_split, split_pct,
        enable_offset, offset_mm, enable_spars, spar_diameter_mm,
        spar_clearance_mm, enable_lightening, enable_hinge.

    Returns
    -------
    dict with keys:
        offset_pts       : list of (x, y) or None
        spars            : list of (x, y_center, thickness)
        spar_trunc       : (cx, cy, r) or None — fore spar truncation info
        lightening_holes : list of (cx, cy, diam)
        cs_coords        : list of (x, y) or None
    """
    chord_cm = cfg.get("chord_cm", 10.0)
    chord_mm = chord_cm * Geometry.MM_PER_CM
    enable_split = cfg.get("enable_split", False)
    split_pct = cfg.get("split_pct", 75.0)
    enable_offset = cfg.get("enable_offset", False)
    offset_mm = cfg.get("offset_mm", 2.0)
    enable_spars = cfg.get("enable_spars", False)
    spar_diameter_mm = cfg.get("spar_diameter_mm", 15.5)
    spar_clearance_mm = cfg.get("spar_clearance_mm", 3.0)
    enable_lightening = cfg.get("enable_lightening", False)
    enable_hinge = cfg.get("enable_hinge", False)

    offset_pts = None
    spars = []
    spar_trunc = None
    lightening_holes = []
    cs_coords = None

    # Offset contour
    if enable_offset and offset_mm > 0 and chord_mm > 0:
        offset_norm = offset_mm / chord_mm
        offset_pts = Offset.offset_contour(coords, offset_norm)

    # Spars
    if enable_spars and chord_mm > 0:
        spar_diam_cm = spar_diameter_mm / Geometry.MM_PER_CM
        max_x = compute_spar_max_x(
            split_pct, enable_split, enable_hinge,
            spar_clearance_mm, chord_mm,
        )
        spars = find_optimal_spar_positions(
            coords,
            spar_diam_cm / chord_cm,
            spar_clearance_mm / chord_mm,
            max_x,
        )

        # Truncate offset at fore spar
        if offset_pts and spars:
            cx, cy, _ = spars[0]
            spar_r = spar_diam_cm / chord_cm / 2.0
            spar_trunc = (cx, cy, spar_r)
            offset_pts = Offset.truncate_offset_at_spar(
                offset_pts, cx, cy, spar_r)

        # Lightening holes
        if (enable_lightening and len(spars) == 2
                and enable_offset and offset_mm > 0):
            lightening_holes = find_lightening_holes(
                coords,
                fore_spar_x=spars[0][0],
                aft_spar_x=spars[1][0],
                spar_diameter_norm=spar_diam_cm / chord_cm,
                offset_norm=offset_mm / chord_mm,
                clearance_norm=spar_clearance_mm / chord_mm,
            )

    # Control surface contour
    if enable_split and 0 < split_pct < 100:
        cs_coords = Geometry.extract_control_surface(coords, split_pct / 100.0)

    return {
        "offset_pts": offset_pts,
        "spars": spars,
        "spar_trunc": spar_trunc,
        "lightening_holes": lightening_holes,
        "cs_coords": cs_coords,
    }


def compute_spar_max_x(split_pct, enable_split, enable_hinge,
                       spar_clearance_mm, chord_mm):
    """Compute rightmost X fraction for spar placement.

    Accounts for CS split and minimum hinge gap when hinge is enabled.
    """
    max_x = 1.0
    if enable_split and 0 < split_pct < 100:
        max_x = split_pct / 100.0
        if enable_hinge and chord_mm > 0:
            min_hinge_gap_mm = 2.0
            shortfall = max(0.0, min_hinge_gap_mm - spar_clearance_mm)
            max_x -= shortfall / chord_mm
    return max_x


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

    # Pre-compute surface split once for the entire scan
    query = Geometry.create_surface_query(coords)

    # Scan at 0.1% chord resolution
    valid = []
    for i in range(1, 1000):
        x = cap * i / 1000.0
        info = query(x)
        if info is None:
            continue
        _, y_center, thickness = info
        if thickness >= min_thickness:
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

    # Pre-compute surface split once for all queries
    query = Geometry.create_surface_query(coords)

    def _max_diam_at(x_frac):
        """Max hole diameter that fits inside the offset shell at *x_frac*."""
        info = query(min(x_frac, 0.999))
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
            info = query(cx)
            if info is None:
                valid = False
                break
            _, cy, _ = info
            holes.append((cx, cy, diam))
        if valid and holes:
            return holes

    return []


def compute_pin_positions(coords, spar_positions, split_frac, pin_diam_norm):
    """
    Compute alignment pin positions at ~15% and ~85% chord.

    Avoids placing pins within 1.5x pin-diameter of a spar center or
    beyond the CS split line. Returns list of (x, y_center) in
    normalised [0..1] space.
    """
    targets = [0.15, 0.85]
    max_x = split_frac if split_frac and 0 < split_frac < 1 else 0.999
    pins = []

    # Build exclusion zones around spar centers
    exclusions = []
    spar_radius_margin = pin_diam_norm * 1.5
    for sx, _, _ in (spar_positions or []):
        exclusions.append((sx - spar_radius_margin, sx + spar_radius_margin))

    for tx in targets:
        if tx >= max_x:
            continue
        # Check exclusion zones
        blocked = any(lo <= tx <= hi for lo, hi in exclusions)
        if blocked:
            continue
        info = Geometry.compute_spar_center(coords, min(tx, 0.999))
        if info is None:
            continue
        _, cy, thickness = info
        # Pin must fit inside the airfoil with some margin
        if thickness < pin_diam_norm * 2.0:
            continue
        pins.append((tx, cy))

    return pins
