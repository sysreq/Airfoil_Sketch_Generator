"""
Wing — Wing Geometry Math
-------------------------
Spanwise section computation, contour transformation (scale + twist +
sweep/dihedral offsets), and wing parameter validation.

Pure Python — only math + Geometry. No Fusion 360 dependencies.
"""

import math
import Geometry


# -- Section Computation -------------------------------------------------------

def compute_section(span_frac, half_span_cm, chord_root_cm, tip_chord_cm,
                    sweep_deg, dihedral_deg, twist_deg):
    """
    Compute the geometry of one spanwise section.

    Parameters
    ----------
    span_frac : float
        Fraction of the half-span [0..1] (0 = root, 1 = tip).
    half_span_cm : float
        Half-span length in cm.
    chord_root_cm : float
        Root chord length in cm.
    tip_chord_cm : float
        Tip chord length in cm.
    sweep_deg : float
        Leading-edge sweep angle in degrees.
    dihedral_deg : float
        Dihedral angle in degrees.
    twist_deg : float
        Total twist from root to tip in degrees.
        Negative = washout (tip nose-down), positive = washin.

    Returns
    -------
    dict with keys:
        z_cm         – spanwise position in cm
        chord_cm     – local chord length in cm
        x_offset_cm  – chordwise offset from sweep in cm
        y_offset_cm  – vertical offset from dihedral in cm
        twist_rad    – local twist angle in radians
    """
    # Linear taper
    chord_cm = chord_root_cm + (tip_chord_cm - chord_root_cm) * span_frac

    # Dihedral: project span onto Y and Z axes
    dihedral_rad = math.radians(dihedral_deg)
    projected_span = half_span_cm * span_frac
    z_cm = projected_span * math.cos(dihedral_rad)
    y_offset_cm = projected_span * math.sin(dihedral_rad)

    # Sweep: chordwise offset at this spanwise station
    sweep_rad = math.radians(sweep_deg)
    x_offset_cm = z_cm * math.tan(sweep_rad)

    # Linear twist distribution
    # Negate so negative twist_deg (washout) produces positive twist_rad
    # (nose-down rotation in the CCW rotation matrix of transform_contour).
    twist_rad = -math.radians(twist_deg) * span_frac

    return {
        "z_cm": z_cm,
        "chord_cm": chord_cm,
        "x_offset_cm": x_offset_cm,
        "y_offset_cm": y_offset_cm,
        "twist_rad": twist_rad,
    }


# -- Contour Transformation ---------------------------------------------------

def transform_contour(normalized_coords, chord_cm, twist_rad,
                      x_offset_cm, y_offset_cm):
    """
    Scale, twist, and translate a normalised [0..1] airfoil contour.

    1. Scale by *chord_cm*.
    2. Rotate about the quarter-chord point (0.25 * chord_cm, 0) by
       *twist_rad*.
    3. Translate by (*x_offset_cm*, *y_offset_cm*).

    Parameters
    ----------
    normalized_coords : list of (x, y)
        Airfoil contour in normalised [0..1] space.
    chord_cm : float
        Local chord length in cm.
    twist_rad : float
        Twist angle in radians.
    x_offset_cm : float
        Chordwise offset (from sweep) in cm.
    y_offset_cm : float
        Vertical offset (from dihedral) in cm.

    Returns
    -------
    list of (x, y) tuples in cm.
    """
    qc = 0.25 * chord_cm          # quarter-chord X
    cos_t = math.cos(twist_rad)
    sin_t = math.sin(twist_rad)

    result = []
    for nx, ny in normalized_coords:
        # Scale to cm
        sx = nx * chord_cm
        sy = ny * chord_cm

        # Shift origin to quarter-chord
        dx = sx - qc
        dy = sy

        # Rotate
        rx = dx * cos_t - dy * sin_t
        ry = dx * sin_t + dy * cos_t

        # Shift back + apply sweep/dihedral offsets
        result.append((rx + qc + x_offset_cm, ry + y_offset_cm))

    return result


# -- Multi-Section Computation -------------------------------------------------

def compute_sections(n_sections, half_span_cm, chord_root_cm, tip_chord_cm,
                     sweep_deg, dihedral_deg, twist_deg):
    """
    Compute geometry for *n_sections* evenly-spaced spanwise stations.

    Parameters
    ----------
    n_sections : int
        Number of sections (>= 2).  Span fractions are
        [0, 1/(n-1), 2/(n-1), ..., 1.0].
    half_span_cm : float
        Half-span length in cm.
    chord_root_cm : float
        Root chord length in cm.
    tip_chord_cm : float
        Tip chord length in cm.
    sweep_deg : float
        Leading-edge sweep angle in degrees.
    dihedral_deg : float
        Dihedral angle in degrees.
    twist_deg : float
        Total twist from root to tip in degrees.

    Returns
    -------
    list of dict
        Each dict is the output of ``compute_section()`` with an added
        ``span_frac`` key.
    """
    if n_sections < 2:
        n_sections = 2

    sections = []
    for i in range(n_sections):
        frac = i / (n_sections - 1)
        sec = compute_section(frac, half_span_cm, chord_root_cm,
                              tip_chord_cm, sweep_deg, dihedral_deg,
                              twist_deg)
        sec["span_frac"] = frac
        sections.append(sec)
    return sections


# -- Control Surface Span Geometry --------------------------------------------

def compute_cs_span(inboard_pct, outboard_pct, half_span_cm, chord_root_cm,
                    tip_chord_cm, sweep_deg, dihedral_deg, split_pct):
    """
    Compute 3D geometry of a control surface spanning from *inboard_pct*
    to *outboard_pct* (as percentages of half-span, 0-100).

    Parameters
    ----------
    inboard_pct : float
        Inboard edge of the CS as % of half-span (0 = root, 100 = tip).
    outboard_pct : float
        Outboard edge of the CS as % of half-span.
    half_span_cm : float
        Half-span length in cm.
    chord_root_cm : float
        Root chord in cm.
    tip_chord_cm : float
        Tip chord in cm.
    sweep_deg : float
        Leading-edge sweep angle in degrees.
    dihedral_deg : float
        Dihedral angle in degrees.
    split_pct : float
        Chordwise split position as % of chord (0-100).

    Returns
    -------
    dict with keys:
        inboard_section  : section dict at the inboard span station
        outboard_section : section dict at the outboard span station
        hinge_line       : list of (x, y, z) tuples defining the hinge
                           line in cm (from inboard to outboard)
        cs_chord_inboard_cm  : CS chord length at inboard station (cm)
        cs_chord_outboard_cm : CS chord length at outboard station (cm)
    """
    inboard_frac = inboard_pct / 100.0
    outboard_frac = outboard_pct / 100.0
    split_frac = split_pct / 100.0

    # twist_deg is 0 for hinge-line calculation — twist doesn't move
    # the hinge in the planform sense, and the sections returned here
    # are purely for positioning, not for contour transformation.
    inboard_sec = compute_section(inboard_frac, half_span_cm, chord_root_cm,
                                  tip_chord_cm, sweep_deg, dihedral_deg, 0.0)
    outboard_sec = compute_section(outboard_frac, half_span_cm, chord_root_cm,
                                   tip_chord_cm, sweep_deg, dihedral_deg, 0.0)

    # CS chord = portion of local chord aft of the split line
    cs_chord_inboard = inboard_sec["chord_cm"] * (1.0 - split_frac)
    cs_chord_outboard = outboard_sec["chord_cm"] * (1.0 - split_frac)

    # Hinge line: the split-frac position along the local chord at each
    # span station, offset by sweep and dihedral.
    def _hinge_point(sec):
        hinge_x = sec["x_offset_cm"] + sec["chord_cm"] * split_frac
        hinge_y = sec["y_offset_cm"]
        hinge_z = sec["z_cm"]
        return (hinge_x, hinge_y, hinge_z)

    hinge_line = [_hinge_point(inboard_sec), _hinge_point(outboard_sec)]

    return {
        "inboard_section": inboard_sec,
        "outboard_section": outboard_sec,
        "hinge_line": hinge_line,
        "cs_chord_inboard_cm": cs_chord_inboard,
        "cs_chord_outboard_cm": cs_chord_outboard,
    }


# -- Validation ----------------------------------------------------------------

_MM_PER_CM = Geometry.MM_PER_CM


def validate_wing(coords, half_span_mm, root_chord_mm, tip_chord_mm,
                   spar_diam_mm, clearance_mm, sweep_deg, dihedral_deg,
                   twist_deg, n_sections=2):
    """
    Run sanity checks on a wing configuration.

    Parameters
    ----------
    coords : list of (x, y)
        Normalised [0..1] airfoil contour.
    half_span_mm : float
        Half-span in mm.
    root_chord_mm : float
        Root chord in mm.
    tip_chord_mm : float
        Tip chord in mm.
    spar_diam_mm : float
        Spar diameter in mm.
    clearance_mm : float
        Minimum spar-to-surface clearance in mm.
    sweep_deg : float
        Leading-edge sweep angle in degrees.
    dihedral_deg : float
        Dihedral angle in degrees.
    twist_deg : float
        Total twist in degrees (negative = washout).
    n_sections : int, optional
        Number of spanwise sections (default 2 = root + tip only).

    Returns
    -------
    list of (severity, message) tuples.
        severity is ``"warning"`` or ``"error"``.
    """
    messages = []

    # -- Section count vs. sweep/twist accuracy --------------------------------
    if n_sections < 3 and (abs(sweep_deg) > 0.1 or abs(twist_deg) > 0.1):
        messages.append(
            ("warning",
             "Only {} sections with non-zero sweep/twist — "
             "add intermediate sections for a smoother loft."
             .format(n_sections)))

    # Convert to cm for internal use
    half_span_cm = half_span_mm / _MM_PER_CM
    root_chord_cm = root_chord_mm / _MM_PER_CM
    tip_chord_cm = tip_chord_mm / _MM_PER_CM

    # -- Aspect ratio checks ---------------------------------------------------
    mean_chord_cm = (root_chord_cm + tip_chord_cm) / 2.0
    if mean_chord_cm > 0:
        full_span_cm = 2.0 * half_span_cm
        ar = full_span_cm / mean_chord_cm
        if ar > 15.0:
            messages.append(
                ("warning",
                 "Aspect ratio {:.1f} is very high — structural risk.".format(ar)))
        elif ar < 3.0:
            messages.append(
                ("warning",
                 "Aspect ratio {:.1f} is very low — poor aerodynamic efficiency."
                 .format(ar)))

    # -- Taper ratio / tip stall risk ------------------------------------------
    if root_chord_cm > 0:
        taper = tip_chord_cm / root_chord_cm
        washout = -twist_deg          # washout is negative twist_deg
        if taper < 0.5 and washout < 2.0:
            messages.append(
                ("warning",
                 "Taper ratio {:.2f} with {:.1f}\u00b0 washout — tip stall risk. "
                 "Consider more washout or a higher taper ratio."
                 .format(taper, washout)))

    # -- Spar clearance at the tip ---------------------------------------------
    if coords and tip_chord_cm > 0:
        spar_diam_norm = spar_diam_mm / (tip_chord_cm * _MM_PER_CM)
        clearance_norm = clearance_mm / (tip_chord_cm * _MM_PER_CM)
        min_thickness_norm = spar_diam_norm + 2.0 * clearance_norm

        # Check at typical spar location (25% chord)
        info = Geometry.compute_spar_center(coords, 0.25)
        if info is not None:
            _, _, thickness_norm = info
            if thickness_norm < min_thickness_norm:
                messages.append(
                    ("error",
                     "Spar does not fit at the tip — airfoil too thin. "
                     "Reduce spar diameter or increase tip chord."))

    return messages
