"""
GUI — Dialog Builder with Grouped Inputs
-----------------------------------------
Handles all dialog construction, input management, and live preview
for the Fusion 360 airfoil generator.

Depends on adsk.core at module level. Sibling modules (Geometry, Offset,
Infill, Preview, Drawing) are accessed via the _modules dict set by init().
"""

import os
import tempfile

import adsk.core

# -- Module-level state --------------------------------------------------------

_app = None
_modules = {}       # {"Geometry", "Offset", "Infill", "Preview", "Drawing"}
_temp_dir = None
_preview_toggle = False   # alternates A/B filenames to bust Fusion image cache
_icons = {}               # icon type -> file path

_EMPTY_LOCAL = "(no local airfoils - add .dat files to the data/ folder)"

_DEFAULTS = {
    "chord_cm": 10.0,
    "split_pct": 75.0,
    "offset_mm": 2.0,
    "spar_diameter_mm": 15.5,
    "spar_clearance_mm": 3.0,
    "hinge_slot_height_mm": 1.2,
    "hinge_slot_depth_mm": 10.0,
    "cs_inset_mm": 2.5,
    "enable_split": True,
    "enable_offset": True,
    "enable_spars": True,
    "enable_hinge": True,
    "enable_lightening": True,
}

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
    for icon_type in ("profile", "split", "offset", "spars"):
        path = os.path.join(temp_dir, f"icon_{icon_type}.png")
        try:
            png_bytes = Preview.render_icon(icon_type)
            Preview.write_file(png_bytes, path)
            icons[icon_type] = path
        except Exception:
            pass
    return icons


# -- Dialog Construction -------------------------------------------------------

def build_inputs(cmd, inputs, airfoils, config):
    """Build the full dialog with GroupCommandInput sections."""
    cmd.setDialogInitialSize(420, 700)
    cmd.setDialogMinimumSize(420, 550)

    _build_profile_group(inputs, airfoils, config)
    _build_preview_group(inputs, airfoils, config)
    _build_split_group(inputs, config)
    _build_offset_group(inputs, config)
    _build_spars_group(inputs, config)


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
    grp_inputs.addValueInput(
        "chord", "Chord Length", "mm",
        adsk.core.ValueInput.createByReal(config.get("chord_cm", _DEFAULTS["chord_cm"])),
    )


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


def _render_initial_preview(airfoils, config):
    """Generate the first preview PNG from config defaults. Returns path or None."""
    global _preview_toggle
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

    suffix = "preview_a.png" if not _preview_toggle else "preview_b.png"
    _preview_toggle = not _preview_toggle
    path = os.path.join(_temp_dir, suffix)

    try:
        coords = Geometry.double_points(raw_coords)
        _render_to_path(path, coords, config)
    except Exception:
        return None
    return path


def _render_to_path(path, coords, config):
    """Compute all geometry and call Preview.render_preview(), writing to path."""
    Geometry = _modules["Geometry"]
    Offset = _modules.get("Offset")
    Infill = _modules.get("Infill")
    Preview = _modules["Preview"]

    chord_cm = config.get("chord_cm", _DEFAULTS["chord_cm"])
    chord_mm = chord_cm * MM_PER_CM
    enable_split = config.get("enable_split", _DEFAULTS["enable_split"])
    split_pct = config.get("split_pct", _DEFAULTS["split_pct"])
    enable_offset = config.get("enable_offset", _DEFAULTS["enable_offset"])
    offset_mm = config.get("offset_mm", _DEFAULTS["offset_mm"])
    enable_spars = config.get("enable_spars", _DEFAULTS["enable_spars"])
    spar_diameter_mm = config.get("spar_diameter_mm", _DEFAULTS["spar_diameter_mm"])
    spar_clearance_mm = config.get("spar_clearance_mm", _DEFAULTS["spar_clearance_mm"])
    enable_lightening = config.get("enable_lightening", _DEFAULTS["enable_lightening"])
    enable_hinge = config.get("enable_hinge", _DEFAULTS["enable_hinge"])

    offset_pts = None
    spars = []
    lightening_holes = []
    cs_coords = None

    # Offset contour
    if enable_offset and Offset and offset_mm > 0:
        offset_norm = offset_mm / chord_mm
        offset_pts = Offset.offset_contour(coords, offset_norm)

    # Spars
    if enable_spars and Infill:
        spar_diam_cm = spar_diameter_mm / MM_PER_CM
        max_x = (split_pct / 100.0
                 if enable_split and 0 < split_pct < 100
                 else 1.0)
        if enable_split and enable_hinge and 0 < split_pct < 100:
            min_hinge_gap_mm = 2.0
            shortfall = max(0.0, min_hinge_gap_mm - spar_clearance_mm)
            max_x -= shortfall / chord_mm

        spars = Infill.find_optimal_spar_positions(
            coords,
            spar_diam_cm / chord_cm,
            spar_clearance_mm / chord_mm,
            max_x,
        )

        # Truncate offset at fore spar
        if offset_pts and spars and Offset:
            cx, cy, _ = spars[0]
            spar_r = spar_diam_cm / chord_cm / 2.0
            offset_pts = Offset.truncate_offset_at_spar(
                offset_pts, cx, cy, spar_r)

        # Lightening holes
        if enable_lightening and len(spars) == 2 and enable_offset and offset_mm > 0:
            lightening_holes = Infill.find_lightening_holes(
                coords,
                fore_spar_x=spars[0][0],
                aft_spar_x=spars[1][0],
                spar_diameter_norm=spar_diam_cm / chord_cm,
                offset_norm=offset_mm / chord_mm,
                clearance_norm=spar_clearance_mm / chord_mm,
            )

    # Control surface
    if enable_split and 0 < split_pct < 100:
        cs_coords = Geometry.extract_control_surface(coords, split_pct / 100.0)

    png_bytes = Preview.render_preview(
        coords,
        chord_cm=chord_cm,
        enable_split=enable_split,
        split_pct=split_pct,
        enable_offset=enable_offset,
        offset_mm=offset_mm,
        enable_spars=enable_spars,
        spars=spars if spars else None,
        enable_lightening=enable_lightening,
        lightening_holes=lightening_holes if lightening_holes else None,
        cs_coords=cs_coords,
        offset_pts=offset_pts,
    )
    Preview.write_file(png_bytes, path)


def _build_split_group(inputs, config):
    """Group 3: Control Surface — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_split", "Control Surface")
    grp.isExpanded = config.get("enable_split", _DEFAULTS["enable_split"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_split", _DEFAULTS["enable_split"])
    grp_inputs = grp.children

    icon_path = _icons.get("split")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_split", "", icon_path)

    grp_inputs.addValueInput(
        "split_pct", "Split at Chord %", "",
        adsk.core.ValueInput.createByReal(
            config.get("split_pct", _DEFAULTS["split_pct"])
        ),
    )

    grp_inputs.addBoolValueInput(
        "enable_hinge", "Hinge", True, "",
        config.get("enable_hinge", _DEFAULTS["enable_hinge"]),
    )

    grp_inputs.addValueInput(
        "hinge_slot_height", "Hinge Height", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("hinge_slot_height_mm", _DEFAULTS["hinge_slot_height_mm"])
            / MM_PER_CM
        ),
    )

    grp_inputs.addValueInput(
        "hinge_slot_depth", "Hinge Depth", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("hinge_slot_depth_mm", _DEFAULTS["hinge_slot_depth_mm"])
            / MM_PER_CM
        ),
    )

    grp_inputs.addValueInput(
        "cs_inset", "CS Inset", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("cs_inset_mm", _DEFAULTS["cs_inset_mm"])
            / MM_PER_CM
        ),
    )


def _build_offset_group(inputs, config):
    """Group 4: Inner Offset — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_offset", "Inner Offset")
    grp.isExpanded = config.get("enable_offset", _DEFAULTS["enable_offset"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_offset", _DEFAULTS["enable_offset"])
    grp_inputs = grp.children

    icon_path = _icons.get("offset")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_offset", "", icon_path)

    grp_inputs.addValueInput(
        "offset", "Distance", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("offset_mm", _DEFAULTS["offset_mm"])
            / MM_PER_CM
        ),
    )


def _build_spars_group(inputs, config):
    """Group 5: Spars & Lightening — collapsible with enable checkbox."""
    grp = inputs.addGroupCommandInput("grp_spars", "Spars & Lightening")
    grp.isExpanded = config.get("enable_spars", _DEFAULTS["enable_spars"])
    grp.isEnabledCheckBoxDisplayed = True
    grp.isEnabledCheckBoxChecked = config.get("enable_spars", _DEFAULTS["enable_spars"])
    grp_inputs = grp.children

    icon_path = _icons.get("spars")
    if icon_path and os.path.isfile(icon_path):
        grp_inputs.addImageCommandInput("icon_spars", "", icon_path)

    grp_inputs.addValueInput(
        "spar_diameter", "Diameter", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("spar_diameter_mm", _DEFAULTS["spar_diameter_mm"])
            / MM_PER_CM
        ),
    )

    grp_inputs.addValueInput(
        "spar_clearance", "Clearance", "mm",
        adsk.core.ValueInput.createByReal(
            config.get("spar_clearance_mm", _DEFAULTS["spar_clearance_mm"])
            / MM_PER_CM
        ),
    )

    grp_inputs.addBoolValueInput(
        "enable_lightening", "Lightening Holes", True, "",
        config.get("enable_lightening", _DEFAULTS["enable_lightening"]),
    )


# -- Read Inputs ---------------------------------------------------------------

def read_inputs(inputs):
    """Read all input values and return a config-compatible dict.

    Note: inputs.itemById() searches recursively through groups.
    """
    local_dd = inputs.itemById("local_airfoil")
    selected = local_dd.selectedItem
    selected_name = selected.name if selected else ""

    chord_cm = inputs.itemById("chord").value

    # Group enable checkboxes
    grp_split = inputs.itemById("grp_split")
    grp_offset = inputs.itemById("grp_offset")
    grp_spars = inputs.itemById("grp_spars")

    return {
        "airfoil": selected_name,
        "chord_cm": chord_cm,
        "enable_split": grp_split.isEnabledCheckBoxChecked,
        "split_pct": inputs.itemById("split_pct").value,
        "enable_hinge": inputs.itemById("enable_hinge").value,
        "hinge_slot_height_mm": inputs.itemById("hinge_slot_height").value * MM_PER_CM,
        "hinge_slot_depth_mm": inputs.itemById("hinge_slot_depth").value * MM_PER_CM,
        "cs_inset_mm": inputs.itemById("cs_inset").value * MM_PER_CM,
        "enable_offset": grp_offset.isEnabledCheckBoxChecked,
        "offset_mm": inputs.itemById("offset").value * MM_PER_CM,
        "enable_spars": grp_spars.isEnabledCheckBoxChecked,
        "spar_diameter_mm": inputs.itemById("spar_diameter").value * MM_PER_CM,
        "spar_clearance_mm": inputs.itemById("spar_clearance").value * MM_PER_CM,
        "enable_lightening": inputs.itemById("enable_lightening").value,
    }


# -- Live Preview Update -------------------------------------------------------

def update_preview(inputs, airfoils):
    """Recompute geometry and update the preview image in the dialog."""
    global _preview_toggle

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

    suffix = "preview_a.png" if not _preview_toggle else "preview_b.png"
    _preview_toggle = not _preview_toggle
    path = os.path.join(_temp_dir, suffix)

    try:
        _render_to_path(path, coords, config)
    except Exception:
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
            visual_ids = {
                "local_airfoil", "chord",
                "split_pct", "enable_hinge",
                "hinge_slot_height", "hinge_slot_depth", "cs_inset",
                "offset",
                "spar_diameter", "spar_clearance", "enable_lightening",
                "grp_split", "grp_offset", "grp_spars",
            }
            if changed.id not in visual_ids:
                return
            # Skip if a value input has an invalid expression (user still typing)
            if hasattr(changed, 'isValidExpression') and not changed.isValidExpression:
                return
            update_preview(args.inputs, self._airfoils)
        except Exception:
            pass  # Swallow errors in preview -- non-critical
