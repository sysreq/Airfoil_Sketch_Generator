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
Airfoils  = None
Geometry  = None
Offset    = None
Infill    = None
Drawing   = None

# Defaults
_DEFAULT_SPLIT_PCT           = 75.0
_DEFAULT_OFFSET_MM           = 2.0
_DEFAULT_SPAR_DIAMETER_MM    = 15.5
_DEFAULT_SPAR_CLEARANCE_MM   = 3.0
_DEFAULT_HINGE_SLOT_HEIGHT_MM = 1.2
_DEFAULT_HINGE_SLOT_DEPTH_MM  = 10.0
_DEFAULT_CS_INSET_MM          = 2.5


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

            # --- Hinge geometry ---
            inputs.addBoolValueInput(
                "enable_hinge", "Elevator/Flap Hinge", True, "",
                cfg.get("enable_hinge", True),
            )
            inputs.addValueInput(
                "hinge_slot_height", "Hinge Slot Height", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("hinge_slot_height_mm",
                            _DEFAULT_HINGE_SLOT_HEIGHT_MM)
                    / Drawing.MM_PER_CM
                ),
            )
            inputs.addValueInput(
                "hinge_slot_depth", "Hinge Slot Depth", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("hinge_slot_depth_mm",
                            _DEFAULT_HINGE_SLOT_DEPTH_MM)
                    / Drawing.MM_PER_CM
                ),
            )
            inputs.addValueInput(
                "cs_inset", "CS Leading Edge Inset", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("cs_inset_mm", _DEFAULT_CS_INSET_MM)
                    / Drawing.MM_PER_CM
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
                    cfg.get("offset_mm", _DEFAULT_OFFSET_MM)
                    / Drawing.MM_PER_CM
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
                    / Drawing.MM_PER_CM
                ),
            )
            inputs.addValueInput(
                "spar_clearance", "Spar Clearance", "mm",
                adsk.core.ValueInput.createByReal(
                    cfg.get("spar_clearance_mm", _DEFAULT_SPAR_CLEARANCE_MM)
                    / Drawing.MM_PER_CM
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
            offset_mm     = offset_cm * Drawing.MM_PER_CM
            enable_hinge  = inputs.itemById("enable_hinge").value
            hinge_h_cm    = inputs.itemById("hinge_slot_height").value
            hinge_h_mm    = hinge_h_cm * Drawing.MM_PER_CM
            hinge_d_cm    = inputs.itemById("hinge_slot_depth").value
            hinge_d_mm    = hinge_d_cm * Drawing.MM_PER_CM
            cs_inset_cm   = inputs.itemById("cs_inset").value
            cs_inset_mm   = cs_inset_cm * Drawing.MM_PER_CM
            enable_spars  = inputs.itemById("enable_spars").value
            spar_diam_cm  = inputs.itemById("spar_diameter").value
            spar_clr_cm   = inputs.itemById("spar_clearance").value
            spar_clr_mm   = spar_clr_cm * Drawing.MM_PER_CM
            enable_light  = inputs.itemById("enable_lightening").value

            if selected_name not in self._airfoils:
                _app.userInterface.messageBox(
                    f'"{selected_name}" not found - this should not happen. '
                    "Try re-running the script."
                )
                return

            _, raw_coords = self._airfoils[selected_name]
            raw_count = len(raw_coords)
            coords = Geometry.double_points(raw_coords)

            # ---- create sketch ----
            des    = adsk.fusion.Design.cast(_app.activeProduct)
            root   = des.rootComponent
            sketch = root.sketches.add(root.xYConstructionPlane)
            sketch.name = (
                f"{selected_name} c={chord_cm * Drawing.MM_PER_CM:.1f}mm"
            )

            Drawing.draw_polyline(sketch, coords, chord_cm)

            # ---- spar positions (computed before offset for truncation) ----
            spar_info = ""
            lightening_info = ""
            spars = []
            if enable_spars:
                chord_mm = chord_cm * Drawing.MM_PER_CM
                max_x = (split_pct / 100.0
                         if enable_split and 0 < split_pct < 100
                         else 1.0)
                spars = Infill.find_optimal_spar_positions(
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
                Drawing.draw_offset(
                    sketch, coords, chord_cm, offset_mm, spar_trunc,
                )

            # ---- spar circles ----
            if enable_spars:
                spar_info = Drawing.draw_spar_circles(
                    sketch, spars, chord_cm, spar_diam_cm, spar_clr_mm,
                )

                # ---- lightening holes ----
                if enable_light and len(spars) == 2 \
                   and enable_offset and offset_mm > 0:
                    lightening_info = Drawing.draw_lightening_holes(
                        sketch, coords, chord_cm, spars,
                        spar_diam_cm, offset_mm, spar_clr_mm,
                    )

            # ---- control surface ----
            split_info = ""
            if enable_split and 0 < split_pct < 100:
                split_info = Drawing.draw_control_surface(
                    root, sketch, coords, chord_cm, split_pct,
                    enable_offset, offset_mm, selected_name,
                    enable_hinge, hinge_h_mm, hinge_d_mm, cs_inset_mm,
                )

            # ---- summary ----
            _app.userInterface.messageBox(
                f'Sketch created: "{sketch.name}"\n'
                f"    Airfoil : {selected_name}\n"
                f"    Chord   : {chord_cm * Drawing.MM_PER_CM:.2f} mm\n"
                f"    Points  : {raw_count} raw -> {len(coords)} doubled"
                f"{spar_info}"
                f"{lightening_info}"
                f"{split_info}"
            )

            # ---- persist settings for next run ----
            Airfoils.save_config(self._script_dir, {
                "airfoil": selected_name,
                "chord_cm": chord_cm,
                "enable_split": enable_split,
                "split_pct": split_pct,
                "enable_hinge": enable_hinge,
                "hinge_slot_height_mm": hinge_h_mm,
                "hinge_slot_depth_mm": hinge_d_mm,
                "cs_inset_mm": cs_inset_mm,
                "enable_offset": enable_offset,
                "offset_mm": offset_mm,
                "enable_spars": enable_spars,
                "spar_diameter_mm": spar_diam_cm * Drawing.MM_PER_CM,
                "spar_clearance_mm": spar_clr_mm,
                "enable_lightening": enable_light,
            })

        except Exception:
            _app.userInterface.messageBox(
                "Error generating sketch:\n" + traceback.format_exc()
            )


# -- Entry Point ---------------------------------------------------------------

def run(context):
    global _app, _handlers, Airfoils, Geometry, Offset, Infill, Drawing
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

        # Reload modules on every run so code changes take effect without
        # restarting Fusion 360.  Order matters: Geometry before Infill
        # (Infill imports Geometry), Offset before Drawing (Drawing imports
        # Offset), and all geometry modules before Drawing.
        import Airfoils
        import Geometry
        import Offset
        import Infill
        import Drawing
        importlib.reload(Airfoils)
        importlib.reload(Geometry)
        importlib.reload(Offset)
        importlib.reload(Infill)
        importlib.reload(Drawing)

        data_dir = os.path.join(script_dir, "data")
        os.makedirs(data_dir, exist_ok=True)

        airfoils, errors = Airfoils.load_data_folder(script_dir)
        config = Airfoils.load_config(script_dir)

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
