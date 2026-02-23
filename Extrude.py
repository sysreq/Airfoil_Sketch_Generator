"""
Extrude — Fusion 360 Loft Operations
--------------------------------------
Creates 3D wing geometry by lofting between N spanwise airfoil section
sketches on offset construction planes.  Supports solid/surface loft,
mirroring, per-section components, and separate control surface bodies.
"""

import adsk.core
import adsk.fusion

import Wing
import Geometry
import Infill

MM_PER_CM = Geometry.MM_PER_CM


def _rotate_to_leading_edge(coords_cm):
    """
    Rotate a closed contour so the leading-edge (min-X) point is first.

    This ensures every section sketch starts at the same topological
    point, preventing Fusion 360's loft from misaligning profiles when
    sweep, taper, or twist shift the trailing edge between sections.
    """
    if not coords_cm:
        return coords_cm
    le_idx = min(range(len(coords_cm)), key=lambda i: coords_cm[i][0])
    return coords_cm[le_idx:] + coords_cm[:le_idx]


def _draw_section_sketch(comp, plane, coords_cm, name):
    """
    Create a sketch on *plane* and draw a closed polyline from
    pre-transformed *coords_cm* (already in cm).  Returns the sketch.
    """
    sketch = comp.sketches.add(plane)
    sketch.name = name
    lines = sketch.sketchCurves.sketchLines
    n = len(coords_cm)
    sketch.isComputeDeferred = True
    for i in range(n):
        x0, y0 = coords_cm[i]
        x1, y1 = coords_cm[(i + 1) % n]
        lines.addByTwoPoints(
            adsk.core.Point3D.create(x0, y0, 0.0),
            adsk.core.Point3D.create(x1, y1, 0.0),
        )
    sketch.isComputeDeferred = False
    return sketch


def _loft_sections(comp, sketches, is_solid):
    """Loft through sketch profiles. Returns the LoftFeature."""
    loft_feats = comp.features.loftFeatures
    loft_input = loft_feats.createInput(
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for sketch in sketches:
        loft_input.loftSections.add(sketch.profiles.item(0))
    loft_input.isSolid = is_solid
    return loft_feats.add(loft_input)


def _boolean_cut(comp, target_body, tool_body):
    """Boolean subtract tool_body from target_body."""
    combine_feats = comp.features.combineFeatures
    tool_bodies = adsk.core.ObjectCollection.create()
    tool_bodies.add(tool_body)
    combine_input = combine_feats.createInput(target_body, tool_bodies)
    combine_input.operation = (
        adsk.fusion.FeatureOperations.CutFeatureOperation)
    combine_input.isKeepToolBodies = False
    combine_feats.add(combine_input)


def _draw_hole_circles(sketch, positions_norm, sec, radius_cm):
    """
    Draw circles on a section sketch at normalised positions,
    transformed to match the section's chord/twist/offset.

    Works for both spar positions (x, y, thickness) and pin
    positions (x, y) — only the first two elements are used.
    """
    if not positions_norm:
        return
    circles = sketch.sketchCurves.sketchCircles
    for pos in positions_norm:
        center_cm = Wing.transform_contour(
            [(pos[0], pos[1])],
            sec["chord_cm"], sec["twist_rad"],
            sec["x_offset_cm"], sec["y_offset_cm"],
        )
        cx, cy = center_cm[0]
        circles.addByCenterRadius(
            adsk.core.Point3D.create(cx, cy, 0.0),
            radius_cm,
        )


def _inflate_contour(contour, factor=1.005):
    """
    Scale a contour slightly outward from its centroid.

    Used to avoid coincident faces when boolean-subtracting a cutter
    body that shares surfaces with the target (e.g. the CS region of
    the wing).  A 0.5 % enlargement is imperceptible but sufficient
    to prevent ASM_INCONS_FACE errors in Fusion 360.
    """
    n = len(contour)
    if n == 0:
        return contour
    cx = sum(x for x, y in contour) / n
    cy = sum(y for x, y in contour) / n
    return [(cx + (x - cx) * factor, cy + (y - cy) * factor)
            for x, y in contour]


def _create_offset_plane(comp, base_plane, offset_cm, name):
    """Create a construction plane offset from *base_plane* by *offset_cm*."""
    planes = comp.constructionPlanes
    plane_input = planes.createInput()
    offset_val = adsk.core.ValueInput.createByReal(offset_cm)
    plane_input.setByOffset(base_plane, offset_val)
    plane = planes.add(plane_input)
    plane.name = name
    return plane


def _get_or_create_plane(comp, base_plane, z_cm, name, plane_cache):
    """
    Return a construction plane at *z_cm*, reusing from *plane_cache*
    if one already exists at the same offset.  Root plane (z_cm == 0)
    returns *base_plane* directly.
    """
    if z_cm == 0.0:
        return base_plane
    key = round(z_cm, 8)
    if key not in plane_cache:
        plane_cache[key] = _create_offset_plane(comp, base_plane, z_cm, name)
    return plane_cache[key]


def mirror_bodies(root_comp, bodies):
    """
    Mirror a list of bodies about the XY construction plane.

    Parameters
    ----------
    root_comp : adsk.fusion.Component
        Root component containing the bodies.
    bodies : list of adsk.fusion.BRepBody
        Bodies to mirror (None entries are skipped).
    """
    valid = [b for b in bodies if b is not None]
    if not valid:
        return
    mirror_feats = root_comp.features.mirrorFeatures
    body_coll = adsk.core.ObjectCollection.create()
    for b in valid:
        body_coll.add(b)
    mirror_input = mirror_feats.createInput(
        body_coll, root_comp.xYConstructionPlane)
    mirror_feat = mirror_feats.add(mirror_input)
    for i in range(mirror_feat.bodies.count):
        if i < len(valid):
            mirror_feat.bodies.item(i).name = f"{valid[i].name} (mirrored)"


def _create_spar_voids(root_comp, planes, sections, spar_positions_norm,
                       hole_radius_cm, wing_body, airfoil_name):
    """
    Loft spar-hole circles through all sections and boolean-subtract
    the resulting void bodies from *wing_body*.

    Each spar gets its own loft (one circle per section plane), which
    follows the wing's twist and taper.  After the cut, the void body
    is consumed and only the hole in the wing remains.
    """
    for spar_idx, (sx, sy, _) in enumerate(spar_positions_norm):
        spar_sketches = []
        for plane, sec in zip(planes, sections):
            center_cm = Wing.transform_contour(
                [(sx, sy)],
                sec["chord_cm"], sec["twist_rad"],
                sec["x_offset_cm"], sec["y_offset_cm"],
            )
            cx, cy = center_cm[0]
            sketch = root_comp.sketches.add(plane)
            sketch.name = f"{airfoil_name} spar {spar_idx} void"
            sketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(cx, cy, 0.0),
                hole_radius_cm,
            )
            spar_sketches.append(sketch)

        void_loft = _loft_sections(root_comp, spar_sketches, True)

        if void_loft.bodies.count > 0:
            _boolean_cut(root_comp, wing_body, void_loft.bodies.item(0))


def create_wing(app, root_comp, coords, config):
    """
    Create a 3D wing by lofting through N spanwise section sketches.

    Parameters
    ----------
    app : adsk.core.Application
        Fusion 360 application object.
    root_comp : adsk.fusion.Component
        Root component to create geometry in.
    coords : list of (x, y)
        Normalised [0..1] airfoil contour (already densified).
    config : dict
        Required keys: chord_cm, wing_half_span_mm, wing_tip_chord_mm,
        wing_sweep_deg, wing_dihedral_deg, wing_twist_deg.
        Optional keys: airfoil, wing_n_sections (int, default 2),
        wing_solid_loft (bool), wing_mirror (bool),
        wing_components (bool),
        enable_split (bool), split_pct (float), hinge_type (str),
        cs_hinge_thickness_mm (float).

    Returns
    -------
    (str, body_or_None) – info string and the wing body (None on failure
    or when the loft produces 0 bodies).  Mirroring is NOT performed
    here; the caller should call ``mirror_bodies()`` after all boolean
    operations are complete.
    """
    try:
        # -- Extract parameters ------------------------------------------------
        root_chord_cm = config["chord_cm"]
        half_span_mm = config["wing_half_span_mm"]
        tip_chord_mm = config["wing_tip_chord_mm"]
        sweep_deg = config["wing_sweep_deg"]
        dihedral_deg = config["wing_dihedral_deg"]
        twist_deg = config["wing_twist_deg"]
        airfoil_name = config.get("airfoil", "Airfoil")
        n_sections = config.get("wing_sections", 2)
        is_solid = config.get("wing_solid_loft", False)
        do_mirror = config.get("wing_mirror", False)
        # wing_components reserved for future use
        # do_components = config.get("wing_components", False)

        # -- Convert mm -> cm -------------------------------------------------
        half_span_cm = half_span_mm / MM_PER_CM
        tip_chord_cm = tip_chord_mm / MM_PER_CM

        # -- Config values used for spar/pin search limits ---------------------
        enable_split = config.get("enable_split", False)
        split_pct = config.get("split_pct", 75.0)

        # -- Compute all sections via Wing.compute_sections --------------------
        sections = Wing.compute_sections(
            n_sections, half_span_cm, root_chord_cm, tip_chord_cm,
            sweep_deg, dihedral_deg, twist_deg,
        )

        # -- Create section sketches on offset planes --------------------------
        sketches = []
        planes = []
        base_plane = root_comp.xYConstructionPlane

        for idx, sec in enumerate(sections):
            # Transform full airfoil contour for this section
            pts = Wing.transform_contour(
                coords, sec["chord_cm"], sec["twist_rad"],
                sec["x_offset_cm"], sec["y_offset_cm"],
            )
            # Rotate so LE is first — consistent loft alignment
            pts = _rotate_to_leading_edge(pts)

            if idx == 0:
                label = "root"
                plane = base_plane
            else:
                label = "tip" if idx == len(sections) - 1 else f"section {idx}"
                plane = _create_offset_plane(
                    root_comp, base_plane, sec["z_cm"],
                    f"{airfoil_name} wing {label} plane",
                )
            planes.append(plane)
            sketch = _draw_section_sketch(
                root_comp, plane, pts,
                f"{airfoil_name} wing {label}",
            )
            sketches.append(sketch)

        # -- Loft through all sections -----------------------------------------
        loft_feat = _loft_sections(root_comp, sketches, is_solid)

        # Capture the resulting body
        body_name = f"{airfoil_name} Wing"
        wing_body = None
        if loft_feat.bodies.count > 0:
            wing_body = loft_feat.bodies.item(0)
            wing_body.name = body_name

        # -- Spar pass-through holes -------------------------------------------
        enable_spars = config.get("enable_spars", False)
        enable_mfg = config.get("enable_manufacturing", False)
        spar_info_3d = ""
        spar_positions = []
        if enable_spars and enable_mfg:
            spar_diam_mm = config.get("spar_diameter_mm", 6.0)
            spar_clr_mm = config.get("spar_clearance_mm", 1.5)
            print_clr_mm = config.get("spar_print_clearance_mm", 0.3)
            chord_mm = root_chord_cm * MM_PER_CM

            # Spar search limit: forward of CS split if enabled,
            # with hinge gap adjustment matching 2D placement
            enable_hinge = config.get("enable_hinge", False)
            max_x = Infill.compute_spar_max_x(
                split_pct, enable_split, enable_hinge,
                spar_clr_mm, chord_mm,
            )

            spar_positions = Infill.find_optimal_spar_positions(
                coords,
                spar_diam_mm / chord_mm,
                spar_clr_mm / chord_mm,
                max_x,
            )

            if spar_positions:
                hole_diam_mm = spar_diam_mm + print_clr_mm
                hole_radius_cm = (hole_diam_mm / MM_PER_CM) / 2.0

                if is_solid and wing_body is not None:
                    _create_spar_voids(
                        root_comp, planes, sections, spar_positions,
                        hole_radius_cm, wing_body, airfoil_name,
                    )
                else:
                    for sketch, sec in zip(sketches, sections):
                        _draw_hole_circles(sketch, spar_positions, sec,
                                           hole_radius_cm)

                n_spars = len(spar_positions)
                spar_info_3d = (
                    f"\n    Spar holes: {n_spars}x "
                    f"{spar_diam_mm:.1f}mm tube, "
                    f"{hole_diam_mm:.1f}mm hole "
                    f"(+{print_clr_mm:.1f}mm clearance)"
                )

        # -- Alignment pin holes -----------------------------------------------
        pin_info_3d = ""
        enable_pins = config.get("enable_alignment_pins", False)
        if enable_pins and enable_mfg:
            pin_diam_mm = config.get("pin_diameter_mm", 3.0)
            print_clr_mm_pin = config.get("spar_print_clearance_mm", 0.3)
            chord_mm = root_chord_cm * MM_PER_CM
            pin_diam_norm = pin_diam_mm / chord_mm
            split_frac_pin = (split_pct / 100.0
                              if enable_split and 0 < split_pct < 100
                              else None)

            pin_positions = Infill.compute_pin_positions(
                coords, spar_positions, split_frac_pin, pin_diam_norm)

            if pin_positions:
                pin_hole_diam_mm = pin_diam_mm + print_clr_mm_pin
                pin_hole_radius_cm = (pin_hole_diam_mm / MM_PER_CM) / 2.0

                for sketch, sec in zip(sketches, sections):
                    _draw_hole_circles(sketch, pin_positions, sec,
                                       pin_hole_radius_cm)

                pin_info_3d = (
                    f"\n    Pins     : {len(pin_positions)}x "
                    f"{pin_diam_mm:.1f}mm pin, "
                    f"{pin_hole_diam_mm:.1f}mm hole "
                    f"(+{print_clr_mm_pin:.1f}mm clearance)"
                )

        # -- Build info string -------------------------------------------------
        root_chord_mm = root_chord_cm * MM_PER_CM
        tip_mm = tip_chord_cm * MM_PER_CM
        taper = tip_chord_cm / root_chord_cm if root_chord_cm > 0 else 0.0
        mean_chord_cm = (root_chord_cm + tip_chord_cm) / 2.0
        full_span_cm = 2.0 * half_span_cm
        wing_area_cm2 = full_span_cm * mean_chord_cm
        ar = full_span_cm / mean_chord_cm if mean_chord_cm > 0 else 0.0
        loft_type = "solid" if is_solid else "surface"
        mirror_str = ", mirrored" if do_mirror else ""

        info = (
            f"\n    Wing     : half-span {half_span_mm:.1f}mm, "
            f"{n_sections} sections ({loft_type}{mirror_str})"
            f"\n    Tip chord: {tip_mm:.1f}mm "
            f"(taper {taper:.2f})"
            f"\n    Sweep    : {sweep_deg:.1f}\u00b0, "
            f"dihedral {dihedral_deg:.1f}\u00b0, "
            f"twist {twist_deg:.1f}\u00b0"
            f"\n    AR       : {ar:.2f}, "
            f"area {wing_area_cm2:.1f}cm\u00b2"
            f"{spar_info_3d}"
            f"{pin_info_3d}"
        )
        return (info, wing_body)

    except Exception as e:
        return (f"\n    Wing     : ERROR - {e}", None)


def create_control_surface(app, root_comp, coords, config, wing_body=None):
    """
    Create a separate 3D control surface body by lofting between
    inboard and outboard CS section sketches.

    Should be called **after** ``create_wing()`` when both
    ``enable_wing`` and ``enable_split`` are True.

    Parameters
    ----------
    app : adsk.core.Application
        Fusion 360 application object.
    root_comp : adsk.fusion.Component
        Root component to create geometry in.
    coords : list of (x, y)
        Normalised [0..1] airfoil contour (already densified).
    config : dict
        Required keys (in addition to wing keys):
            enable_split, split_pct, cs_inboard_pct, cs_outboard_pct.
        Optional keys:
            hinge_type (str, default "CA Hinge"),
            cs_hinge_thickness_mm (float, default 0.4),
            enable_hinge (bool), cs_inset_mm (float),
            wing_solid_loft (bool), wing_mirror (bool).
    wing_body : adsk.fusion.BRepBody or None
        When provided (solid loft), the CS body is boolean-subtracted
        from the wing to create a clean separation.

    Returns
    -------
    (str, body_or_None) -- info string and the CS body.  Mirroring is
    NOT performed here; the caller handles it.
    """
    try:
        # -- Extract parameters ------------------------------------------------
        root_chord_cm = config["chord_cm"]
        half_span_mm = config["wing_half_span_mm"]
        tip_chord_mm = config["wing_tip_chord_mm"]
        sweep_deg = config["wing_sweep_deg"]
        dihedral_deg = config["wing_dihedral_deg"]
        twist_deg = config["wing_twist_deg"]
        split_pct = config["split_pct"]
        cs_inboard_pct = config.get("cs_inboard_pct", 10.0)
        cs_outboard_pct = config.get("cs_outboard_pct", 90.0)
        airfoil_name = config.get("airfoil", "Airfoil")
        is_solid = config.get("wing_solid_loft", False)
        do_mirror = config.get("wing_mirror", False)
        hinge_type = config.get("hinge_type", "CA Hinge")
        hinge_thick_mm = config.get("cs_hinge_thickness_mm", 0.4)
        enable_hinge = config.get("enable_hinge", False)
        cs_inset_mm = config.get("cs_inset_mm", 2.5)

        half_span_cm = half_span_mm / MM_PER_CM
        tip_chord_cm = tip_chord_mm / MM_PER_CM
        split_frac = split_pct / 100.0

        # -- Compute CS span geometry ------------------------------------------
        cs_span = Wing.compute_cs_span(
            cs_inboard_pct, cs_outboard_pct, half_span_cm,
            root_chord_cm, tip_chord_cm, sweep_deg, dihedral_deg,
            split_pct,
        )

        inboard_sec = cs_span["inboard_section"]
        outboard_sec = cs_span["outboard_section"]

        # -- Extract the CS contour in normalised space ------------------------
        if hinge_type == "Living Hinge":
            chord_mm_inboard = inboard_sec["chord_cm"] * MM_PER_CM
            skin_thick_norm = hinge_thick_mm / chord_mm_inboard
            hinge_result = Geometry.extract_living_hinge_contour(
                coords, split_frac, skin_thick_norm)
            if hinge_result is not None:
                _, cs_norm, _ = hinge_result
            else:
                cs_norm = Geometry.extract_control_surface(coords, split_frac)
        elif enable_hinge:
            # CA Hinge / Center Pin: elevator contour with inset + rounding
            chord_mm = root_chord_cm * MM_PER_CM
            inset_norm = cs_inset_mm / chord_mm
            cs_norm = Geometry.build_elevator_contour(
                coords, split_frac, inset_norm)
            if cs_norm is not None:
                cs_norm = Geometry.round_cs_wall_corners(cs_norm, inset_norm)
            else:
                cs_norm = Geometry.extract_control_surface(coords, split_frac)
        else:
            cs_norm = Geometry.extract_control_surface(coords, split_frac)

        if cs_norm is None:
            return (
                f"\n    CS body  : could not extract CS contour "
                f"at {split_pct:.1f}% chord",
                None,
            )

        # -- Compute CS section stations ----------------------------------------
        # Match the wing's intermediate sections so the CS loft follows the
        # same taper/sweep/twist interpolation as the main wing body.
        inboard_frac = cs_inboard_pct / 100.0
        outboard_frac = cs_outboard_pct / 100.0

        # CS body inset: 1% gap on each side for clearance against frames
        cs_gap_pct = 1.0
        body_inb_frac = (cs_inboard_pct + cs_gap_pct) / 100.0
        body_outb_frac = (cs_outboard_pct - cs_gap_pct) / 100.0

        # Wing section fractions from config
        n_sections = config.get("wing_sections", 2)
        wing_fracs = [i / (n_sections - 1) for i in range(n_sections)]

        # CS body: endpoints + wing sections that fall inside the body span
        body_fracs = sorted(set(
            [body_inb_frac, body_outb_frac]
            + [f for f in wing_fracs if body_inb_frac < f < body_outb_frac]
        ))

        # Cutter: frame-edge endpoints + wing sections inside frame span
        cut_fracs = sorted(set(
            [inboard_frac, outboard_frac]
            + [f for f in wing_fracs if inboard_frac < f < outboard_frac]
        ))

        base_plane = root_comp.xYConstructionPlane
        plane_cache = {}

        # -- CS body sketches on offset planes ---------------------------------
        cs_sketches = []
        for i, frac in enumerate(body_fracs):
            sec = Wing.compute_section(
                frac, half_span_cm, root_chord_cm, tip_chord_cm,
                sweep_deg, dihedral_deg, twist_deg)
            pts = Wing.transform_contour(
                cs_norm, sec["chord_cm"], sec["twist_rad"],
                sec["x_offset_cm"], sec["y_offset_cm"])
            pts = _rotate_to_leading_edge(pts)

            if i == 0:
                label = "inboard"
            elif i == len(body_fracs) - 1:
                label = "outboard"
            else:
                label = f"section {i}"
            plane = _get_or_create_plane(
                root_comp, base_plane, sec["z_cm"],
                f"{airfoil_name} CS {label} plane", plane_cache)
            sketch = _draw_section_sketch(
                root_comp, plane, pts,
                f"{airfoil_name} CS {label}")
            cs_sketches.append(sketch)

        # -- Loft the CS body --------------------------------------------------
        cs_loft = _loft_sections(root_comp, cs_sketches, is_solid)

        cs_body_name = f"{airfoil_name} Control Surface"
        cs_body = None
        if cs_loft.bodies.count > 0:
            cs_body = cs_loft.bodies.item(0)
            cs_body.name = cs_body_name

        # -- Boolean subtract CS region from wing body -------------------------
        # Always cut the wing at the basic split line (not the inset wall)
        # so the full CS region is removed.  The CS body keeps its elevator
        # contour shape; the gap between them is the hinge clearance.
        if wing_body is not None and cs_body is not None:
            cut_norm = Geometry.extract_control_surface(coords, split_frac)
            if cut_norm is not None:
                # Inflate slightly to avoid coincident faces with wing
                cut_norm = _inflate_contour(cut_norm)

                cut_sketches = []
                for i, frac in enumerate(cut_fracs):
                    sec = Wing.compute_section(
                        frac, half_span_cm, root_chord_cm, tip_chord_cm,
                        sweep_deg, dihedral_deg, twist_deg)
                    pts = Wing.transform_contour(
                        cut_norm, sec["chord_cm"], sec["twist_rad"],
                        sec["x_offset_cm"], sec["y_offset_cm"])
                    pts = _rotate_to_leading_edge(pts)

                    if i == 0:
                        label = "inboard"
                    elif i == len(cut_fracs) - 1:
                        label = "outboard"
                    else:
                        label = f"section {i}"
                    plane = _get_or_create_plane(
                        root_comp, base_plane, sec["z_cm"],
                        f"{airfoil_name} CS cutter {label}", plane_cache)
                    sketch = _draw_section_sketch(
                        root_comp, plane, pts,
                        f"{airfoil_name} CS cutter {label}")
                    cut_sketches.append(sketch)

                cut_feat = _loft_sections(root_comp, cut_sketches, True)

                if cut_feat.bodies.count > 0:
                    _boolean_cut(root_comp, wing_body,
                                 cut_feat.bodies.item(0))

        # -- Build info string -------------------------------------------------
        cs_chord_inb_mm = cs_span["cs_chord_inboard_cm"] * MM_PER_CM
        cs_chord_outb_mm = cs_span["cs_chord_outboard_cm"] * MM_PER_CM
        span_mm = (outboard_sec["z_cm"] - inboard_sec["z_cm"]) * MM_PER_CM
        loft_type = "solid" if is_solid else "surface"
        mirror_str = ", mirrored" if do_mirror else ""

        info = (
            f"\n    CS body  : {cs_inboard_pct:.0f}-{cs_outboard_pct:.0f}% "
            f"span ({span_mm:.1f}mm, {loft_type}{mirror_str})"
            f"\n    CS chord : {cs_chord_inb_mm:.1f}mm inboard, "
            f"{cs_chord_outb_mm:.1f}mm outboard"
            f"\n    Hinge    : {hinge_type}"
        )
        return (info, cs_body)

    except Exception as e:
        return (f"\n    CS body  : ERROR - {e}", None)
