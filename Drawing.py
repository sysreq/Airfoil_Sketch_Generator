"""
Fusion 360 Drawing Operations
------------------------------
Translates geometry into adsk.* API calls. Each public function
returns an info string for the summary dialog.
"""

import adsk.core
import adsk.fusion

import Geometry
import Offset
import Infill

MM_PER_CM = 10.0


def draw_polyline(sketch, contour, chord_cm, closed=True):
    """Draw line segments connecting *contour* on *sketch*."""
    lines = sketch.sketchCurves.sketchLines
    n = len(contour)
    end = n if closed else n - 1
    sketch.isComputeDeferred = True
    for i in range(end):
        x0, y0 = contour[i]
        x1, y1 = contour[(i + 1) % n]
        lines.addByTwoPoints(
            adsk.core.Point3D.create(x0 * chord_cm, y0 * chord_cm, 0.0),
            adsk.core.Point3D.create(x1 * chord_cm, y1 * chord_cm, 0.0),
        )
    sketch.isComputeDeferred = False


def draw_offset(sketch, contour, chord_cm, offset_mm, spar_truncation=None):
    """Draw an inward-offset closed polyline, optionally truncated at fore spar."""
    offset_norm = offset_mm / (chord_cm * MM_PER_CM)
    off_pts = Offset.offset_contour(contour, offset_norm)
    if spar_truncation is not None:
        cx, cy, r = spar_truncation
        off_pts = Offset.truncate_offset_at_spar(off_pts, cx, cy, r)
    draw_polyline(sketch, off_pts, chord_cm)


def draw_spar_circles(sketch, spars, chord_cm, spar_diam_cm, clearance_mm):
    """Draw spar circles at pre-computed positions. Returns info string."""
    if not spars:
        return (
            f"\n    Spars    : could not fit "
            f"{spar_diam_cm * MM_PER_CM:.1f}mm dia with "
            f"{clearance_mm:.1f}mm clearance"
        )

    chord_mm = chord_cm * MM_PER_CM
    spar_radius_cm = spar_diam_cm / 2.0
    labels = ["fore", "aft"] if len(spars) == 2 else ["mid"]
    placed = []
    for (cx, cy, thickness), label in zip(spars, labels):
        center = adsk.core.Point3D.create(
            cx * chord_cm, cy * chord_cm, 0.0
        )
        sketch.sketchCurves.sketchCircles.addByCenterRadius(
            center, spar_radius_cm
        )
        clr_mm = (thickness * chord_cm - spar_diam_cm) / 2.0 * MM_PER_CM
        placed.append(f"{label} {cx * 100:.1f}% ({clr_mm:.1f}mm clr)")

    info = (
        f"\n    Spars    : {spar_diam_cm * MM_PER_CM:.1f}mm dia, "
        f"{clearance_mm:.1f}mm min clearance"
        f"\n    Placed   : " + ", ".join(placed)
    )
    if len(spars) == 2:
        sep_mm = (spars[1][0] - spars[0][0]) * chord_mm
        info += f" ({sep_mm:.1f}mm separation)"
    return info


def draw_lightening_holes(sketch, coords, chord_cm, spars,
                            spar_diam_cm, offset_mm, clearance_mm):
    """Draw lightening holes between the two spars. Returns info string."""
    chord_mm = chord_cm * MM_PER_CM
    holes = Infill.find_lightening_holes(
        coords,
        fore_spar_x=spars[0][0],
        aft_spar_x=spars[1][0],
        spar_diameter_norm=spar_diam_cm / chord_cm,
        offset_norm=offset_mm / chord_mm,
        clearance_norm=clearance_mm / chord_mm,
    )

    if not holes:
        return "\n    Lighten  : no room for holes between spars"

    # Shrink each hole by 1 mm to keep clear of the sloped offset walls
    shrink_norm = 1.0 / chord_mm
    holes = [(cx, cy, max(0.0, d - shrink_norm)) for cx, cy, d in holes]
    holes = [(cx, cy, d) for cx, cy, d in holes if d > 0]

    if not holes:
        return "\n    Lighten  : no room for holes between spars"

    for cx, cy, diam in holes:
        center = adsk.core.Point3D.create(
            cx * chord_cm, cy * chord_cm, 0.0
        )
        radius_cm = diam * chord_cm / 2.0
        sketch.sketchCurves.sketchCircles.addByCenterRadius(center, radius_cm)

    diams_mm = [h[2] * chord_mm for h in holes]
    d_min, d_max = min(diams_mm), max(diams_mm)
    if abs(d_max - d_min) < 0.1:
        size_str = f"{d_min:.1f}mm dia"
    else:
        size_str = f"{d_min:.1f}-{d_max:.1f}mm dia"
    return (
        f"\n    Lighten  : {len(holes)} holes ({size_str}) "
        f"between spars"
    )


def _draw_hinge_slot(sketch, x_wall, hinge_y, slot_half, slot_depth,
                     chord_cm, direction):
    """
    Draw a U-shaped hinge slot (3 lines) on *sketch*.

    Parameters
    ----------
    x_wall     : normalised X of the split wall
    hinge_y    : normalised Y of hinge centre (midpoint of thickness)
    slot_half  : half-height of the slot in normalised units
    slot_depth : depth of the slot in normalised units
    chord_cm   : chord length in cm (for scaling)
    direction  : +1 = slot extends toward TE (into CS),
                 -1 = slot extends toward LE (into main body)
    """
    lines = sketch.sketchCurves.sketchLines
    x_end = x_wall + direction * slot_depth
    top_y = hinge_y + slot_half
    bot_y = hinge_y - slot_half

    pts = [
        (x_wall, top_y),
        (x_end,  top_y),
        (x_end,  bot_y),
        (x_wall, bot_y),
    ]
    for i in range(len(pts) - 1):
        lines.addByTwoPoints(
            adsk.core.Point3D.create(
                pts[i][0] * chord_cm, pts[i][1] * chord_cm, 0.0),
            adsk.core.Point3D.create(
                pts[i + 1][0] * chord_cm, pts[i + 1][1] * chord_cm, 0.0),
        )


def draw_control_surface(root, sketch, coords, chord_cm, split_pct,
                           enable_offset, offset_mm, selected_name,
                           enable_hinge=False, hinge_slot_height_mm=1.2,
                           hinge_slot_depth_mm=10.0, cs_inset_mm=2.5):
    """Draw control surface sketch + split line. Returns an info string."""
    split_frac = split_pct / 100.0
    chord_mm = chord_cm * MM_PER_CM

    # Original CS contour (used for offset and non-hinge drawing)
    cs_coords = Geometry.extract_control_surface(coords, split_frac)

    if not cs_coords:
        return (
            f"\n    Split    : could not find surfaces at "
            f"{split_pct:.1f}% chord"
        )

    cs_sketch = root.sketches.add(root.xYConstructionPlane)
    cs_sketch.name = (
        f"{selected_name} ctrl {split_pct:.0f}% "
        f"c={chord_mm:.1f}mm"
    )

    if enable_hinge:
        # Build elevator contour with X-direction inset
        inset_norm = cs_inset_mm / chord_mm
        elev_coords = Geometry.build_elevator_contour(
            coords, split_frac, inset_norm)

        if elev_coords is None:
            elev_coords = cs_coords  # fallback

        # Round wall-side corners for hinge rotation clearance
        elev_coords = Geometry.round_cs_wall_corners(
            elev_coords, inset_norm)

        # Draw elevator contour on CS sketch
        draw_polyline(cs_sketch, elev_coords, chord_cm)

        # Hinge geometry in normalised units
        # Center Y from thickness at split_frac (same for both slots)
        y_upper = cs_coords[0][1]
        y_lower = cs_coords[-1][1]
        hinge_y = (y_upper + y_lower) / 2.0
        slot_half = (hinge_slot_height_mm / chord_mm) / 2.0
        # Each side gets half the remaining depth after subtracting the inset
        slot_depth = max(0.0, hinge_slot_depth_mm - cs_inset_mm) / 2.0 / chord_mm

        # CS slot at wall_frac, opens toward TE
        _draw_hinge_slot(cs_sketch, elev_coords[0][0], hinge_y,
                         slot_half, slot_depth, chord_cm, +1)

        # Main pocket at split_frac, opens toward LE
        _draw_hinge_slot(sketch, cs_coords[0][0], hinge_y,
                         slot_half, slot_depth, chord_cm, -1)
    else:
        # Original behavior: closed polyline for CS
        draw_polyline(cs_sketch, cs_coords, chord_cm)

    # Split line on the main airfoil sketch (full height, always)
    sketch.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(
            cs_coords[0][0] * chord_cm,
            cs_coords[0][1] * chord_cm, 0.0,
        ),
        adsk.core.Point3D.create(
            cs_coords[-1][0] * chord_cm,
            cs_coords[-1][1] * chord_cm, 0.0,
        ),
    )

    # Control-surface offset (uses elevator contour when hinge enabled)
    if enable_offset and offset_mm > 0:
        cs_for_offset = elev_coords if enable_hinge else cs_coords
        draw_offset(cs_sketch, cs_for_offset, chord_cm, offset_mm)

    info = (
        f"\n    Split    : {split_pct:.1f}% chord "
        f"(x = {split_frac * chord_mm:.2f} mm)"
        f'\n    CS sketch: "{cs_sketch.name}"'
    )
    if enable_hinge:
        per_side_mm = max(0.0, hinge_slot_depth_mm - cs_inset_mm) / 2.0
        info += (
            f"\n    Hinge    : {hinge_slot_height_mm:.1f} x "
            f"{per_side_mm:.1f}mm slot (x2), "
            f"{cs_inset_mm:.1f}mm inset, "
            f"{hinge_slot_depth_mm:.1f}mm total"
        )
    return info
