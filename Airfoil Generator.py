"""
Fusion 360 Script: Airfoil Sketch Generator
--------------------------------------------
Scans a "data/" folder (next to this script) for .dat files and lists
them in a dropdown. Selecting one and clicking OK creates a closed
spline sketch on the XY plane.

Features
--------
- Supports both Selig and Lednicer .dat formats.
- Automatic point interpolation ensures minimum curve fidelity
  (default: spline points every 2.5 mm along X).
- Optional control-surface split line at a configurable chord percentage
  (default: 75 %).

To add airfoils: drop any .dat file into the data/ folder and re-run.

Run via: Tools > Add-Ins > Scripts and Add-Ins > "+" > select file > Run
"""

import os
import sys
import importlib
import traceback
import adsk.core
import adsk.fusion

_EMPTY_LOCAL = "(no local airfoils - add .dat files to the data/ folder)"

_app      = None
_handlers = []
Utils     = None

# Unit conversion
_MM_PER_CM = 10.0

# Defaults
_DEFAULT_SPLIT_PCT        = 75.0
_DEFAULT_OFFSET_MM        = 2.0
_DEFAULT_SPAR_DIAMETER_MM  = 15.5
_DEFAULT_SPAR_CLEARANCE_MM = 3.0


# -- Drawing Helpers -----------------------------------------------------------

def _draw_polyline(sketch, contour, chord_cm, closed=True):
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


def _draw_offset(sketch, contour, chord_cm, offset_mm, spar_truncation=None):
    """Draw an inward-offset closed polyline, optionally truncated at fore spar."""
    offset_norm = offset_mm / (chord_cm * _MM_PER_CM)
    off_pts = Utils.offset_contour(contour, offset_norm)
    if spar_truncation is not None:
        cx, cy, r = spar_truncation
        off_pts = Utils.truncate_offset_at_spar(off_pts, cx, cy, r)
    _draw_polyline(sketch, off_pts, chord_cm)


def _draw_spar_circles(sketch, spars, chord_cm, spar_diam_cm, clearance_mm):
    """Draw spar circles at pre-computed positions. Returns info string."""
    if not spars:
        return (
            f"\n    Spars    : could not fit "
            f"{spar_diam_cm * _MM_PER_CM:.1f}mm dia with "
            f"{clearance_mm:.1f}mm clearance"
        )

    chord_mm = chord_cm * _MM_PER_CM
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
        clr_mm = (thickness * chord_cm - spar_diam_cm) / 2.0 * _MM_PER_CM
        placed.append(f"{label} {cx * 100:.1f}% ({clr_mm:.1f}mm clr)")

    info = (
        f"\n    Spars    : {spar_diam_cm * _MM_PER_CM:.1f}mm dia, "
        f"{clearance_mm:.1f}mm min clearance"
        f"\n    Placed   : " + ", ".join(placed)
    )
    if len(spars) == 2:
        sep_mm = (spars[1][0] - spars[0][0]) * chord_mm
        info += f" ({sep_mm:.1f}mm separation)"
    return info


def _draw_lightening_holes(sketch, coords, chord_cm, spars,
                            spar_diam_cm, offset_mm, clearance_mm):
    """Draw lightening holes between the two spars. Returns info string."""
    chord_mm = chord_cm * _MM_PER_CM
    holes = Utils.find_lightening_holes(
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


def _draw_control_surface(root, sketch, coords, chord_cm, split_pct,
                           enable_offset, offset_mm, selected_name):
    """Draw control surface sketch + split line. Returns an info string."""
    split_frac = split_pct / 100.0
    cs_coords = Utils.extract_control_surface(coords, split_frac)

    if not cs_coords:
        return (
            f"\n    Split    : could not find surfaces at "
            f"{split_pct:.1f}% chord"
        )

    cs_sketch = root.sketches.add(root.xYConstructionPlane)
    cs_sketch.name = (
        f"{selected_name} ctrl {split_pct:.0f}% "
        f"c={chord_cm * _MM_PER_CM:.1f}mm"
    )

    # Closed polyline for CS (closing segment = split line)
    _draw_polyline(cs_sketch, cs_coords, chord_cm)

    # Split line on the main airfoil sketch
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

    # Control-surface offset
    if enable_offset and offset_mm > 0:
        _draw_offset(cs_sketch, cs_coords, chord_cm, offset_mm)

    return (
        f"\n    Split    : {split_pct:.1f}% chord "
        f"(x = {split_frac * chord_cm * _MM_PER_CM:.2f} mm)"
        f'\n    CS sketch: "{cs_sketch.name}"'
    )


# -- Fusion 360 Dialog ---------------------------------------------------------

class AirfoilCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self, script_dir, airfoils, config):
        super().__init__()
        self._script_dir = script_dir
        self._airfoils   = airfoils
        self._config     = config

    def notify(self, args):
        try:
            cmd    = args.command
            inputs = cmd.commandInputs
            cfg    = self._config

            # --- Airfoil selector ---
            local_dd = inputs.addDropDownCommandInput(
                "local_airfoil", "Profile",
                adsk.core.DropDownStyles.TextListDropDownStyle,
            )
            saved_airfoil = cfg.get("airfoil", "")
            if self._airfoils:
                for name in self._airfoils:
                    local_dd.listItems.add(
                        name, name == saved_airfoil if saved_airfoil else False,
                    )
                if local_dd.selectedItem is None and local_dd.listItems.count > 0:
                    local_dd.listItems.item(0).isSelected = True
            else:
                local_dd.listItems.add(_EMPTY_LOCAL, True)
                local_dd.isEnabled = False

            # --- Chord length ---
            inputs.addValueInput(
                "chord", "Chord Length", "mm",
                adsk.core.ValueInput.createByReal(cfg.get("chord_cm", 10.0)),
            )

            # --- Control surface split ---
            inputs.addBoolValueInput(
                "enable_split", "Control Surface Split", True, "",
                cfg.get("enable_split", True),
            )
            inputs.addValueInput(
                "split_pct", "Split at Chord %", "",
                adsk.core.ValueInput.createByReal(
                    cfg.get("split_pct", _DEFAULT_SPLIT_PCT)
                ),
            )

            # --- Inner offset ---
            inputs.addBoolValueInput(
                "enable_offset", "Inner Offset", True, "",
                cfg.get("enable_offset", True),
            )
            inputs.addValueInput(
                "offset", "Offset Distance", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("offset_mm", _DEFAULT_OFFSET_MM) / _MM_PER_CM
                ),
            )

            # --- Spar holes ---
            inputs.addBoolValueInput(
                "enable_spars", "Spar Holes", True, "",
                cfg.get("enable_spars", True),
            )
            inputs.addValueInput(
                "spar_diameter", "Spar Diameter", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("spar_diameter_mm", _DEFAULT_SPAR_DIAMETER_MM)
                    / _MM_PER_CM
                ),
            )
            inputs.addValueInput(
                "spar_clearance", "Spar Clearance", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("spar_clearance_mm", _DEFAULT_SPAR_CLEARANCE_MM)
                    / _MM_PER_CM
                ),
            )
            inputs.addBoolValueInput(
                "enable_lightening", "Lightening Holes", True, "",
                cfg.get("enable_lightening", True),
            )

            on_execute = AirfoilCommandExecuteHandler(
                self._script_dir, self._airfoils,
            )
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

        except Exception:
            _app.userInterface.messageBox(
                "Error building dialog:\n" + traceback.format_exc()
            )


class AirfoilCommandExecuteHandler(adsk.core.CommandEventHandler):
    """Called when the user clicks OK -- creates the sketch."""

    def __init__(self, script_dir, airfoils):
        super().__init__()
        self._script_dir = script_dir
        self._airfoils   = airfoils

    def notify(self, args):
        try:
            inputs = args.command.commandInputs

            # ---- read inputs ----
            local_dd = inputs.itemById("local_airfoil")
            selected = local_dd.selectedItem

            if not selected or selected.name == _EMPTY_LOCAL:
                _app.userInterface.messageBox(
                    "No local airfoil selected.\n"
                    "Add .dat files to the data/ folder and re-run the script."
                )
                return

            selected_name = selected.name
            chord_cm      = inputs.itemById("chord").value
            enable_split  = inputs.itemById("enable_split").value
            split_pct     = inputs.itemById("split_pct").value
            enable_offset = inputs.itemById("enable_offset").value
            offset_cm     = inputs.itemById("offset").value
            offset_mm     = offset_cm * _MM_PER_CM
            enable_spars  = inputs.itemById("enable_spars").value
            spar_diam_cm  = inputs.itemById("spar_diameter").value
            spar_clr_cm   = inputs.itemById("spar_clearance").value
            spar_clr_mm   = spar_clr_cm * _MM_PER_CM
            enable_light  = inputs.itemById("enable_lightening").value

            if selected_name not in self._airfoils:
                _app.userInterface.messageBox(
                    f'"{selected_name}" not found - this should not happen. '
                    "Try re-running the script."
                )
                return

            _, raw_coords = self._airfoils[selected_name]
            raw_count = len(raw_coords)
            coords = Utils.double_points(raw_coords)

            # ---- create sketch ----
            des    = adsk.fusion.Design.cast(_app.activeProduct)
            root   = des.rootComponent
            sketch = root.sketches.add(root.xYConstructionPlane)
            sketch.name = f"{selected_name} c={chord_cm * _MM_PER_CM:.1f}mm"

            _draw_polyline(sketch, coords, chord_cm)

            # ---- spar positions (computed before offset for truncation) ----
            spar_info = ""
            lightening_info = ""
            spars = []
            if enable_spars:
                chord_mm = chord_cm * _MM_PER_CM
                max_x = split_pct / 100.0 if enable_split and 0 < split_pct < 100 else 1.0
                spars = Utils.find_optimal_spar_positions(
                    coords,
                    spar_diam_cm / chord_cm,
                    spar_clr_mm / chord_mm,
                    max_x,
                )

            # ---- inner offset (truncated at fore spar when available) ----
            spar_trunc = None
            if enable_spars and spars:
                cx, cy, _ = spars[0]  # fore spar
                spar_trunc = (cx, cy, spar_diam_cm / chord_cm / 2.0)

            if enable_offset and offset_mm > 0:
                _draw_offset(sketch, coords, chord_cm, offset_mm, spar_trunc)

            # ---- spar circles ----
            if enable_spars:
                spar_info = _draw_spar_circles(
                    sketch, spars, chord_cm, spar_diam_cm, spar_clr_mm,
                )

                # ---- lightening holes ----
                if enable_light and len(spars) == 2 \
                   and enable_offset and offset_mm > 0:
                    lightening_info = _draw_lightening_holes(
                        sketch, coords, chord_cm, spars,
                        spar_diam_cm, offset_mm, spar_clr_mm,
                    )

            # ---- control surface ----
            split_info = ""
            if enable_split and 0 < split_pct < 100:
                split_info = _draw_control_surface(
                    root, sketch, coords, chord_cm, split_pct,
                    enable_offset, offset_mm, selected_name,
                )

            # ---- summary ----
            _app.userInterface.messageBox(
                f'Sketch created: "{sketch.name}"\n'
                f"    Airfoil : {selected_name}\n"
                f"    Chord   : {chord_cm * _MM_PER_CM:.2f} mm\n"
                f"    Points  : {raw_count} raw -> {len(coords)} doubled"
                f"{spar_info}"
                f"{lightening_info}"
                f"{split_info}"
            )

            # ---- persist settings for next run ----
            Utils.save_config(self._script_dir, {
                "airfoil": selected_name,
                "chord_cm": chord_cm,
                "enable_split": enable_split,
                "split_pct": split_pct,
                "enable_offset": enable_offset,
                "offset_mm": offset_mm,
                "enable_spars": enable_spars,
                "spar_diameter_mm": spar_diam_cm * _MM_PER_CM,
                "spar_clearance_mm": spar_clr_mm,
                "enable_lightening": enable_light,
            })

        except Exception:
            _app.userInterface.messageBox(
                "Error generating sketch:\n" + traceback.format_exc()
            )


# -- Entry Point ---------------------------------------------------------------

def run(context):
    global _app, _handlers, Utils
    _handlers = []

    try:
        _app = adsk.core.Application.get()
        ui   = _app.userInterface

        if not adsk.fusion.Design.cast(_app.activeProduct):
            ui.messageBox("No active Fusion 360 design found.\n"
                          "Please open or create a design first.")
            return

        script_dir = os.path.dirname(os.path.realpath(__file__))
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)

        # Reload Utils on every run so code changes take effect without
        # restarting Fusion 360.
        import Utils
        importlib.reload(Utils)

        data_dir = os.path.join(script_dir, "data")
        os.makedirs(data_dir, exist_ok=True)

        airfoils, errors = Utils.load_data_folder(script_dir)
        config = Utils.load_config(script_dir)

        if errors:
            ui.messageBox(
                "Some .dat files could not be loaded (skipped):\n\n"
                + "\n".join(f"  - {f}: {e}" for f, e in errors)
            )

        CMD_ID   = "airfoilSketchGenerator"
        existing = ui.commandDefinitions.itemById(CMD_ID)
        if existing:
            existing.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, "Airfoil Sketch Generator", ""
        )

        on_created = AirfoilCommandCreatedHandler(script_dir, airfoils, config)
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        cmd_def.execute()
        adsk.autoTerminate(False)

    except Exception:
        if _app:
            _app.userInterface.messageBox(
                "Script startup error:\n" + traceback.format_exc()
            )
