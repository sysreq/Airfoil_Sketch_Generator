"""
GUI — Dialog Builder with Grouped Inputs
-----------------------------------------
Handles all dialog construction, input management, and live preview
for the Fusion 360 airfoil generator.

Depends on adsk.core at module level. Sibling modules (Geometry, Offset,
Infill, Preview, Drawing) are accessed via the _modules dict set by init().
"""

import math
import os
import tempfile
import traceback

import adsk.core

# -- Module-level state --------------------------------------------------------

_app = None
_modules = {}       # {"Geometry", "Offset", "Infill", "Preview", "Drawing"}
_temp_dir = None
_preview_toggle = False   # alternates A/B filenames to bust Fusion image cache
_icons = {}               # icon type -> file path

# Geometry cache: avoids recomputing 2D geometry when only wing params change
_geom_cache_key = None
_geom_cache_val = None

# Keep in sync with Airfoils.EMPTY_LOCAL (can't import at module level)
_EMPTY_LOCAL = "(no local airfoils - add .dat files to the data/ folder)"


def _log_error(exc):
    """Append exception traceback to a log file in the temp directory."""
    if not _temp_dir:
        return
    try:
        log_path = os.path.join(_temp_dir, "gui_errors.log")
        with open(log_path, "a") as f:
            f.write(traceback.format_exc() + "\n")
    except Exception:
        pass

_DEFAULTS = {
    "chord_cm": 10.0,
    "split_pct": 75.0,
    "offset_mm": 2.0,
    "spar_diameter_mm": 15.5,
    "spar_clearance_mm": 3.0,
    "hinge_type": "CA Hinge",
    "hinge_slot_height_mm": 1.2,
    "hinge_slot_depth_mm": 10.0,
    "cs_hinge_thickness_mm": 0.4,
    "cs_inset_mm": 2.5,
    "cs_inboard_pct": 10.0,
    "cs_outboard_pct": 90.0,
    "enable_split": True,
    "enable_offset": True,
    "enable_spars": True,
    "enable_hinge": True,
    "enable_lightening": True,
    "enable_wing": False,
    "wing_half_span_mm": 500.0,
    "wing_tip_chord_mm": 100.0,
    "wing_sweep_deg": 0.0,
    "wing_dihedral_deg": 3.0,
    "wing_twist_deg": -2.0,
    "wing_sections": 2,
    "wing_solid_loft": False,
    "wing_mirror": False,
    "enable_manufacturing": False,
    "spar_print_clearance_mm": 0.3,
    "enable_alignment_pins": True,
    "pin_diameter_mm": 3.0,
}

_HINGE_TYPES = ["CA Hinge", "Living Hinge", "Center Pin"]

MM_PER_CM = 10.0


# -- Initialisation -----------------------------------------------------------

def init(app, modules_dict):
    """Store references and prepare temp directory + icons."""
    global _app, _modules, _temp_dir, _icons
    _app = app
    _modules = modules_dict
    _temp_dir = tempfile.mkdtemp(prefix="airfoil_gui_")
    _icons = _generate_icons(_temp_dir)


def _generate_icons(temp_dir):
    """Render section icons to temp_dir via Preview module. Return path dict."""
    Preview = _modules.get("Preview")
    if Preview is None:
        return {}
    icons = {}
    for icon_type in ("profile", "split", "offset", "spars", "wing"):
        path = os.path.join(temp_dir, f"icon_{icon_type}.png")
        try:
            png_bytes = Preview.render_icon(icon_type)
            Preview.write_file(png_bytes, path)
            icons[icon_type] = path
        except Exception as exc:
            _log_error(exc)
    return icons


# -- Dialog Construction -------------------------------------------------------

def build_inputs(cmd, inputs, airfoils, config):
    """Build the full dialog with tabbed layout: Profile tab + Wing tab."""
    enable_wing = config.get("enable_wing", _DEFAULTS["enable_wing"])
    if enable_wing:
        cmd.setDialogInitialSize(480, 900)
    else:
        cmd.setDialogInitialSize(480, 800)
    cmd.setDialogMinimumSize(480, 600)

    # Try tab-based layout; fall back to flat groups if tabs unavailable
    try:
        tab_profile = inputs.addTabCommandInput("tab_profile", "2D Profile")
        tab_wing = inputs.addTabCommandInput("tab_wing", "3D Wing")
        profile_inputs = tab_profile.children
        wing_inputs = tab_wing.children
    except Exception:
        # Tabs not supported in this Fusion version — use flat layout
        profile_inputs = inputs
        wing_inputs = inputs

    _build_profile_group(profile_inputs, airfoils, config)
    _build_preview_group(profile_inputs, airfoils, config)
    _build_split_group(profile_inputs, config, enable_wing)
    _build_offset_group(profile_inputs, config, enable_wing)
    _build_spars_group(profile_inputs, config, enable_wing)
    _build_wing_group(wing_inputs, config)


def _build_profile_group(inputs, airfoils, config):
    """Group 1: Profile & Chord — always expanded, no enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_profile", "Profile & Chord")
    grp.isExpanded = True
    grp.isEnabledCheckBoxDisplayed = False
    grp_inputs = grp.children

    icon_path = _icons.get("profile")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_profile", "", icon_path)

    # Airfoil dropdown
    local_dd = grp_inputs.addDropDownCommandInput(
        "local_airfoil", "Profile",
        adsk.core.DropDownStyles.TextListDropDownStyle,
    )
    saved_airfoil = config.get("airfoil", "")
    if airfoils:
        for name in airfoils:
            local_dd.listItems.add(
                name, name == saved_airfoil if saved_airfoil else False,
            )
        if local_dd.selectedItem is None and local_dd.listItems.count > 0:
            local_dd.listItems.item(0).isSelected = True
    else:
        local_dd.listItems.add(_EMPTY_LOCAL, True)
        local_dd.isEnabled = False

    # Chord length
    chord_input = grp_inputs.addValueInput(
        "chord", "Chord Length", "mm",
        adsk.core.ValueInput.createByReal(config.get("chord_cm", _DEFAULTS["chord_cm"])),
    )
    chord_input.tooltip = "Root airfoil chord length"


def _build_preview_group(inputs, airfoils, config):
    """Group 2: Preview — always expanded, no enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_preview", "Preview")
    grp.isExpanded = True
    grp.isEnabledCheckBoxDisplayed = False
    grp_inputs = grp.children

    # Generate initial preview image
    preview_path = _render_initial_preview(airfoils, config)
    if preview_path and os.path.isfile(preview_path):
        grp_inputs.addImageCommandInput("preview_img", "", preview_path)
    else:
        # Fallback: add a placeholder image input with empty path
        grp_inputs.addImageCommandInput("preview_img", "", "")


def _next_preview_path():
    """Return the next preview PNG path, toggling A/B for cache busting."""
    global _preview_toggle
    suffix = "preview_a.png" if not _preview_toggle else "preview_b.png"
    _preview_toggle = not _preview_toggle
    return os.path.join(_temp_dir, suffix)


def _render_initial_preview(airfoils, config):
    """Generate the first preview PNG from config defaults. Returns path or None."""
    if not _temp_dir:
        return None

    Preview = _modules.get("Preview")
    Geometry = _modules.get("Geometry")
    if Preview is None or Geometry is None:
        return None

    # Find the selected airfoil
    saved = config.get("airfoil", "")
    if saved and saved in airfoils:
        _, raw_coords = airfoils[saved]
    elif airfoils:
        first_key = next(iter(airfoils))
        _, raw_coords = airfoils[first_key]
    else:
        return None

    path = _next_preview_path()

    try:
        coords = Geometry.double_points(raw_coords)
        _render_to_path(path, coords, config)
    except Exception as exc:
        _log_error(exc)
        return None
    return path


def _geom_cache_key_from(coords, config):
    """Build a hashable cache key from 2D-relevant config params."""
    n = len(coords)
    # Use coord identity via length + first + last point
    first = coords[0] if n > 0 else (0, 0)
    last = coords[-1] if n > 0 else (0, 0)
    return (
        n, first, last,
        config.get("chord_cm"),
        config.get("enable_split"), config.get("split_pct"),
        config.get("enable_offset"), config.get("offset_mm"),
        config.get("enable_spars"),
        config.get("spar_diameter_mm"), config.get("spar_clearance_mm"),
        config.get("enable_lightening"), config.get("enable_hinge"),
    )


def _render_to_path(path, coords, config):
    """Compute all geometry and call Preview.render_preview(), writing to path."""
    global _geom_cache_key, _geom_cache_val

    Infill = _modules.get("Infill")
    Preview = _modules["Preview"]

    chord_cm = config.get("chord_cm", _DEFAULTS["chord_cm"])
    enable_split = config.get("enable_split", _DEFAULTS["enable_split"])
    split_pct = config.get("split_pct", _DEFAULTS["split_pct"])
    enable_wing = config.get("enable_wing", _DEFAULTS["enable_wing"])

    # Shared geometry pipeline with caching
    key = _geom_cache_key_from(coords, config)
    if key == _geom_cache_key and _geom_cache_val is not None:
        geom = _geom_cache_val
    elif Infill:
        geom = Infill.compute_2d_geometry(coords, config)
        _geom_cache_key = key
        _geom_cache_val = geom
    else:
        geom = {"offset_pts": None, "spars": [], "spar_trunc": None,
                "lightening_holes": [], "cs_coords": None}

    offset_pts = geom["offset_pts"]
    spars = geom["spars"]
    lightening_holes = geom["lightening_holes"]
    cs_coords = geom["cs_coords"]

    wing = None
    if enable_wing:
        wing = {
            "half_span_mm": config.get("wing_half_span_mm", _DEFAULTS["wing_half_span_mm"]),
            "tip_chord_mm": config.get("wing_tip_chord_mm", _DEFAULTS["wing_tip_chord_mm"]),
            "sweep_deg": config.get("wing_sweep_deg", _DEFAULTS["wing_sweep_deg"]),
            "dihedral_deg": config.get("wing_dihedral_deg", _DEFAULTS["wing_dihedral_deg"]),
            "twist_deg": config.get("wing_twist_deg", _DEFAULTS["wing_twist_deg"]),
            "sections": config.get("wing_sections", _DEFAULTS["wing_sections"]),
            "mirror": config.get("wing_mirror", _DEFAULTS["wing_mirror"]),
            "solid": config.get("wing_solid_loft", _DEFAULTS["wing_solid_loft"]),
            "cs_inboard_pct": config.get("cs_inboard_pct", _DEFAULTS["cs_inboard_pct"]) / 100.0,
            "cs_outboard_pct": config.get("cs_outboard_pct", _DEFAULTS["cs_outboard_pct"]) / 100.0,
        }

    png_bytes = Preview.render_preview(
        coords,
        chord_cm=chord_cm,
        split_pct=split_pct if enable_split else 0.0,
        spars=spars or None,
        lightening_holes=lightening_holes or None,
        cs_coords=cs_coords,
        offset_pts=offset_pts,
        wing=wing,
    )
    Preview.write_file(png_bytes, path)


def _apply_hinge_visibility(hinge_type, hinge_h, hinge_d, hinge_thick):
    """Show/hide hinge-specific inputs based on selected hinge type."""
    is_ca = (hinge_type == "CA Hinge")
    is_living = (hinge_type == "Living Hinge")
    hinge_h.isVisible = is_ca
    hinge_d.isVisible = is_ca
    hinge_thick.isVisible = is_living


def _build_split_group(inputs, config, wing_enabled=False):
    """Group 3: Control Surface — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_split", "Control Surface")
    grp.isExpanded = config.get("enable_split", _DEFAULTS["enable_split"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_split", _DEFAULTS["enable_split"])
    grp_inputs = grp.children

    icon_path = _icons.get("split")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_split", "", icon_path)

    # -- 2D Profile Inputs --

    split_input = grp_inputs.addValueInput(
        "split_pct", "Split at Chord %", "",
        adsk.core.ValueInput.createByReal(
            config.get("split_pct", _DEFAULTS["split_pct"])
        ),
    )
    split_input.tooltip = "Percentage of chord where the control surface begins"

    cs_inset = grp_inputs.addValueInput(
        "cs_inset", "CS Inset", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("cs_inset_mm", _DEFAULTS["cs_inset_mm"])
            / MM_PER_CM
        ),
    )
    cs_inset.tooltip = "Gap between main body and control surface at the split line"

    # -- Hinge Type Selector --

    enable_hinge_input = grp_inputs.addBoolValueInput(
        "enable_hinge", "Hinge", True, "",
        config.get("enable_hinge", _DEFAULTS["enable_hinge"]),
    )

    saved_hinge = config.get("hinge_type", _DEFAULTS["hinge_type"])
    hinge_dd = grp_inputs.addDropDownCommandInput(
        "hinge_type", "Hinge Type",
        adsk.core.DropDownStyles.TextListDropDownStyle,
    )
    hinge_dd.tooltip = "Type of hinge connecting control surface to main wing"
    for ht in _HINGE_TYPES:
        hinge_dd.listItems.add(ht, ht == saved_hinge)
    if hinge_dd.selectedItem is None and hinge_dd.listItems.count > 0:
        hinge_dd.listItems.item(0).isSelected = True

    # CA Hinge inputs (visible only when hinge_type == "CA Hinge")
    hinge_h = grp_inputs.addValueInput(
        "hinge_slot_height", "Hinge Height", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("hinge_slot_height_mm", _DEFAULTS["hinge_slot_height_mm"])
            / MM_PER_CM
        ),
    )
    hinge_h.tooltip = "Height of the hinge slot cut into the control surface"

    hinge_d = grp_inputs.addValueInput(
        "hinge_slot_depth", "Hinge Depth", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("hinge_slot_depth_mm", _DEFAULTS["hinge_slot_depth_mm"])
            / MM_PER_CM
        ),
    )
    hinge_d.tooltip = "Depth of the hinge slot extending into the control surface"

    # Living Hinge input (visible only when hinge_type == "Living Hinge")
    hinge_thick = grp_inputs.addValueInput(
        "cs_hinge_thickness", "Living Hinge Thickness", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("cs_hinge_thickness_mm", _DEFAULTS["cs_hinge_thickness_mm"])
            / MM_PER_CM
        ),
    )
    hinge_thick.tooltip = "Thickness of the flexible connecting strip on upper surface"

    # Set initial hinge input visibility based on saved hinge type
    _apply_hinge_visibility(saved_hinge, hinge_h, hinge_d, hinge_thick)

    # -- 3D Span Inputs (only relevant with 3D wing) --

    grp_inputs.addTextBoxCommandInput(
        "cs_span_label", "", "<b>Span Placement</b> (3D wing only)", 1, True,
    )

    cs_inboard = grp_inputs.addValueInput(
        "cs_inboard_pct", "Inboard Station %", "",
        adsk.core.ValueInput.createByReal(
            config.get("cs_inboard_pct", _DEFAULTS["cs_inboard_pct"])
        ),
    )
    cs_inboard.tooltip = "Start of control surface as percentage of half-span from root"

    cs_outboard = grp_inputs.addValueInput(
        "cs_outboard_pct", "Outboard Station %", "",
        adsk.core.ValueInput.createByReal(
            config.get("cs_outboard_pct", _DEFAULTS["cs_outboard_pct"])
        ),
    )
    cs_outboard.tooltip = "End of control surface as percentage of half-span from root"

    # Show/hide span inputs based on wing enabled state
    span_visible = wing_enabled
    grp_inputs.itemById("cs_span_label").isVisible = span_visible
    cs_inboard.isVisible = span_visible
    cs_outboard.isVisible = span_visible


def _build_offset_group(inputs, config, wing_enabled=False):
    """Group 4: Inner Offset — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_offset", "Inner Offset")
    # Auto-collapse when wing is enabled to reduce clutter
    if wing_enabled:
        grp.isExpanded = False
    else:
        grp.isExpanded = config.get("enable_offset", _DEFAULTS["enable_offset"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_offset", _DEFAULTS["enable_offset"])
    grp_inputs = grp.children

    icon_path = _icons.get("offset")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_offset", "", icon_path)

    offset_input = grp_inputs.addValueInput(
        "offset", "Distance", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("offset_mm", _DEFAULTS["offset_mm"])
            / MM_PER_CM
        ),
    )
    offset_input.tooltip = "Inward offset distance for the inner contour (skin thickness)"


def _build_spars_group(inputs, config, wing_enabled=False):
    """Group 5: Spars & Lightening — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_spars", "Spars & Lightening")
    # Auto-collapse when wing is enabled to reduce clutter
    if wing_enabled:
        grp.isExpanded = False
    else:
        grp.isExpanded = config.get("enable_spars", _DEFAULTS["enable_spars"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_spars", _DEFAULTS["enable_spars"])
    grp_inputs = grp.children

    icon_path = _icons.get("spars")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_spars", "", icon_path)

    spar_diam = grp_inputs.addValueInput(
        "spar_diameter", "Diameter", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("spar_diameter_mm", _DEFAULTS["spar_diameter_mm"])
            / MM_PER_CM
        ),
    )
    spar_diam.tooltip = "Carbon fiber tube outer diameter for fore and aft spars"

    spar_clr = grp_inputs.addValueInput(
        "spar_clearance", "Clearance", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("spar_clearance_mm", _DEFAULTS["spar_clearance_mm"])
            / MM_PER_CM
        ),
    )
    spar_clr.tooltip = "Minimum clearance between spar holes and airfoil surfaces"

    grp_inputs.addBoolValueInput(
        "enable_lightening", "Lightening Holes", True, "",
        config.get("enable_lightening", _DEFAULTS["enable_lightening"]),
    )


def _build_wing_group(inputs, config):
    """Group 6: 3D Wing — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_wing", "3D Wing")
    grp.isExpanded = config.get("enable_wing", _DEFAULTS["enable_wing"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_wing", _DEFAULTS["enable_wing"])
    grp_inputs = grp.children

    icon_path = _icons.get("wing")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_wing", "", icon_path)

    # -- Planform sub-group --
    grp_planform = grp_inputs.addGroupCommandInput(
        "grp_wing_planform", "Planform")
    grp_planform.isExpanded = True
    grp_planform.isEnabledCheckBoxDisplayed = False
    pf = grp_planform.children

    pf.addValueInput(
        "wing_half_span", "Half-Span", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("wing_half_span_mm", _DEFAULTS["wing_half_span_mm"])
            / MM_PER_CM
        ),
    )

    tip_chord = pf.addValueInput(
        "wing_tip_chord", "Tip Chord", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("wing_tip_chord_mm", _DEFAULTS["wing_tip_chord_mm"])
            / MM_PER_CM
        ),
    )
    tip_chord.tooltip = "Chord length at the wing tip (root chord is set in Profile & Chord)"

    sweep_input = pf.addValueInput(
        "wing_sweep", "Sweep (LE)", "deg",
        adsk.core.ValueInput.createByReal(
            math.radians(config.get("wing_sweep_deg", _DEFAULTS["wing_sweep_deg"]))
        ),
    )
    sweep_input.tooltip = "Leading-edge sweep angle; positive sweeps the tip aft"

    dihedral_input = pf.addValueInput(
        "wing_dihedral", "Dihedral", "deg",
        adsk.core.ValueInput.createByReal(
            math.radians(config.get("wing_dihedral_deg", _DEFAULTS["wing_dihedral_deg"]))
        ),
    )
    dihedral_input.tooltip = "Upward angle of the wing from root to tip for lateral stability"

    # Washout: UI shows positive value, internal storage is negative twist
    washout_deg = -config.get("wing_twist_deg", _DEFAULTS["wing_twist_deg"])
    washout_input = pf.addValueInput(
        "wing_twist", "Washout", "deg",
        adsk.core.ValueInput.createByReal(math.radians(washout_deg)),
    )
    washout_input.tooltip = (
        "Tip twist angle reducing angle of attack at wingtip to prevent tip stall"
    )

    # -- Loft Options sub-group --
    grp_loft = grp_inputs.addGroupCommandInput(
        "grp_wing_loft", "Loft Options")
    grp_loft.isExpanded = True
    grp_loft.isEnabledCheckBoxDisplayed = False
    lf = grp_loft.children

    sections_input = lf.addIntegerSpinnerCommandInput(
        "wing_sections", "Sections",
        2, 10, 1,
        config.get("wing_sections", _DEFAULTS["wing_sections"]),
    )
    sections_input.tooltip = (
        "Number of spanwise loft cross-sections; more sections "
        "capture taper and twist more accurately"
    )

    solid_input = lf.addBoolValueInput(
        "wing_solid", "Solid Body", True, "",
        config.get("wing_solid_loft", _DEFAULTS["wing_solid_loft"]),
    )
    solid_input.tooltip = "Create a solid body instead of a surface loft"

    mirror_input = lf.addBoolValueInput(
        "wing_mirror", "Mirror (Both Halves)", True, "",
        config.get("wing_mirror", _DEFAULTS["wing_mirror"]),
    )
    mirror_input.tooltip = "Create both left and right wing halves mirrored about the root plane"

    # -- Manufacturing inputs (flat, inside Loft Options to avoid nesting limit) --
    lf.addTextBoxCommandInput(
        "mfg_label", "", "<b>Manufacturing</b>", 1, True,
    )

    lf.addBoolValueInput(
        "enable_manufacturing", "Enable Manufacturing", True, "",
        config.get("enable_manufacturing", _DEFAULTS["enable_manufacturing"]),
    )

    spar_print_clr = lf.addValueInput(
        "spar_print_clearance", "Spar Print Clearance", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("spar_print_clearance_mm",
                        _DEFAULTS["spar_print_clearance_mm"])
            / MM_PER_CM
        ),
    )
    spar_print_clr.tooltip = (
        "Extra diameter added to spar holes for FDM printing tolerance"
    )

    enable_pins = lf.addBoolValueInput(
        "enable_alignment_pins", "Alignment Pins", True, "",
        config.get("enable_alignment_pins",
                    _DEFAULTS["enable_alignment_pins"]),
    )
    enable_pins.tooltip = "Add alignment pin holes at section boundaries"

    pin_diam = lf.addValueInput(
        "pin_diameter", "Pin Diameter", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("pin_diameter_mm", _DEFAULTS["pin_diameter_mm"])
            / MM_PER_CM
        ),
    )
    pin_diam.tooltip = "Diameter of alignment pin holes"

    # Manufacturing sub-inputs visible only when enabled
    mfg_on = config.get("enable_manufacturing", _DEFAULTS["enable_manufacturing"])
    spar_print_clr.isVisible = mfg_on
    enable_pins.isVisible = mfg_on
    pin_diam.isVisible = mfg_on and config.get(
        "enable_alignment_pins", _DEFAULTS["enable_alignment_pins"])


# -- Read Inputs ---------------------------------------------------------------

def read_inputs(inputs):
    """Read all input values and return a config-compatible dict.

    Note: inputs.itemById() searches recursively through groups and tabs.
    """
    local_dd = inputs.itemById("local_airfoil")
    selected = local_dd.selectedItem
    selected_name = selected.name if selected else ""

    chord_cm = inputs.itemById("chord").value

    # Group enable checkboxes
    grp_split = inputs.itemById("grp_split")
    grp_offset = inputs.itemById("grp_offset")
    grp_spars = inputs.itemById("grp_spars")
    grp_wing = inputs.itemById("grp_wing")

    # Hinge type dropdown
    hinge_dd = inputs.itemById("hinge_type")
    hinge_type = hinge_dd.selectedItem.name if hinge_dd.selectedItem else _DEFAULTS["hinge_type"]

    # CS span — clamp inboard < outboard
    cs_inboard_pct = inputs.itemById("cs_inboard_pct").value
    cs_outboard_pct = inputs.itemById("cs_outboard_pct").value
    if cs_inboard_pct >= cs_outboard_pct:
        cs_outboard_pct = min(cs_inboard_pct + 5.0, 100.0)

    return {
        "airfoil": selected_name,
        "chord_cm": chord_cm,
        "enable_split": grp_split.isEnabledCheckBoxChecked,
        "split_pct": inputs.itemById("split_pct").value,
        "enable_hinge": inputs.itemById("enable_hinge").value,
        "hinge_type": hinge_type,
        "hinge_slot_height_mm": inputs.itemById("hinge_slot_height").value * MM_PER_CM,
        "hinge_slot_depth_mm": inputs.itemById("hinge_slot_depth").value * MM_PER_CM,
        "cs_hinge_thickness_mm": inputs.itemById("cs_hinge_thickness").value * MM_PER_CM,
        "cs_inset_mm": inputs.itemById("cs_inset").value * MM_PER_CM,
        "cs_inboard_pct": cs_inboard_pct,
        "cs_outboard_pct": cs_outboard_pct,
        "enable_offset": grp_offset.isEnabledCheckBoxChecked,
        "offset_mm": inputs.itemById("offset").value * MM_PER_CM,
        "enable_spars": grp_spars.isEnabledCheckBoxChecked,
        "spar_diameter_mm": inputs.itemById("spar_diameter").value * MM_PER_CM,
        "spar_clearance_mm": inputs.itemById("spar_clearance").value * MM_PER_CM,
        "enable_lightening": inputs.itemById("enable_lightening").value,
        "enable_wing": grp_wing.isEnabledCheckBoxChecked,
        "wing_half_span_mm": inputs.itemById("wing_half_span").value * MM_PER_CM,
        "wing_tip_chord_mm": inputs.itemById("wing_tip_chord").value * MM_PER_CM,
        "wing_dihedral_deg": math.degrees(inputs.itemById("wing_dihedral").value),
        "wing_twist_deg": -math.degrees(inputs.itemById("wing_twist").value),
        "wing_sweep_deg": math.degrees(inputs.itemById("wing_sweep").value),
        "wing_sections": inputs.itemById("wing_sections").value,
        "wing_solid_loft": inputs.itemById("wing_solid").value,
        "wing_mirror": inputs.itemById("wing_mirror").value,
        "enable_manufacturing": inputs.itemById("enable_manufacturing").value,
        "spar_print_clearance_mm": inputs.itemById("spar_print_clearance").value * MM_PER_CM,
        "enable_alignment_pins": inputs.itemById("enable_alignment_pins").value,
        "pin_diameter_mm": inputs.itemById("pin_diameter").value * MM_PER_CM,
    }


# -- Live Preview Update -------------------------------------------------------

def update_preview(inputs, airfoils):
    """Recompute geometry and update the preview image in the dialog."""
    Preview = _modules.get("Preview")
    Geometry = _modules.get("Geometry")
    if Preview is None or Geometry is None:
        return

    config = read_inputs(inputs)
    selected_name = config["airfoil"]

    if not selected_name or selected_name == _EMPTY_LOCAL:
        return
    if selected_name not in airfoils:
        return

    _, raw_coords = airfoils[selected_name]
    coords = Geometry.double_points(raw_coords)

    path = _next_preview_path()

    try:
        _render_to_path(path, coords, config)
    except Exception as exc:
        _log_error(exc)
        return

    preview_img = inputs.itemById("preview_img")
    if preview_img:
        preview_img.imageFile = path


# -- Input Changed Handler ----------------------------------------------------

class AirfoilInputChangedHandler(adsk.core.InputChangedEventHandler):
    """Fires on meaningful input changes and updates the live preview."""

    def __init__(self, airfoils):
        super().__init__()
        self._airfoils = airfoils

    def notify(self, args):
        try:
            changed = args.input
            all_inputs = args.inputs
            visual_ids = {
                "local_airfoil", "chord",
                "split_pct", "enable_hinge", "hinge_type",
                "hinge_slot_height", "hinge_slot_depth",
                "cs_hinge_thickness", "cs_inset",
                "cs_inboard_pct", "cs_outboard_pct",
                "offset",
                "spar_diameter", "spar_clearance", "enable_lightening",
                "grp_split", "grp_offset", "grp_spars",
                "wing_half_span", "wing_tip_chord",
                "wing_dihedral", "wing_twist", "wing_sweep",
                "wing_sections", "wing_solid", "wing_mirror",
                "grp_wing",
                "enable_manufacturing",
                "spar_print_clearance", "enable_alignment_pins",
                "pin_diameter",
            }

            # Dynamic visibility: hinge type controls which inputs are shown
            if changed.id == "hinge_type":
                hinge_dd = all_inputs.itemById("hinge_type")
                sel = hinge_dd.selectedItem
                if sel:
                    _apply_hinge_visibility(
                        sel.name,
                        all_inputs.itemById("hinge_slot_height"),
                        all_inputs.itemById("hinge_slot_depth"),
                        all_inputs.itemById("cs_hinge_thickness"),
                    )

            # Dynamic visibility: manufacturing toggle
            if changed.id == "enable_manufacturing":
                mfg_on = all_inputs.itemById("enable_manufacturing").value
                for mid in ("spar_print_clearance", "enable_alignment_pins",
                            "pin_diameter"):
                    inp = all_inputs.itemById(mid)
                    if inp:
                        inp.isVisible = mfg_on
                # Pin diameter only if both manufacturing and pins are on
                pin_diam = all_inputs.itemById("pin_diameter")
                pins_inp = all_inputs.itemById("enable_alignment_pins")
                if pin_diam and pins_inp:
                    pin_diam.isVisible = mfg_on and pins_inp.value

            # Dynamic visibility: alignment pins toggle pin diameter
            if changed.id == "enable_alignment_pins":
                mfg_inp = all_inputs.itemById("enable_manufacturing")
                mfg_on = mfg_inp.value if mfg_inp else False
                pins_on = all_inputs.itemById("enable_alignment_pins").value
                pin_diam = all_inputs.itemById("pin_diameter")
                if pin_diam:
                    pin_diam.isVisible = mfg_on and pins_on

            # Dynamic visibility: wing enable toggles CS span inputs
            # and manufacturing inputs
            if changed.id == "grp_wing":
                grp_wing = all_inputs.itemById("grp_wing")
                wing_on = grp_wing.isEnabledCheckBoxChecked
                span_label = all_inputs.itemById("cs_span_label")
                inboard = all_inputs.itemById("cs_inboard_pct")
                outboard = all_inputs.itemById("cs_outboard_pct")
                if span_label:
                    span_label.isVisible = wing_on
                if inboard:
                    inboard.isVisible = wing_on
                if outboard:
                    outboard.isVisible = wing_on

            if changed.id not in visual_ids:
                return
            # Skip if a value input has an invalid expression (user still typing)
            if hasattr(changed, 'isValidExpression') and not changed.isValidExpression:
                return
            update_preview(all_inputs, self._airfoils)
        except Exception as exc:
            _log_error(exc)
