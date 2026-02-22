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
Preview   = None
Drawing   = None
GUI       = None


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

            # Delegate dialog building to GUI module
            GUI.build_inputs(cmd, inputs, self._airfoils, self._config)

            # Register input-changed handler for live preview
            on_changed = GUI.AirfoilInputChangedHandler(self._airfoils)
            cmd.inputChanged.add(on_changed)
            _handlers.append(on_changed)

            # Register execute handler
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

            # ---- read inputs via GUI module ----
            cfg = GUI.read_inputs(inputs)

            selected_name = cfg["airfoil"]
            if not selected_name or selected_name == _EMPTY_LOCAL:
                _app.userInterface.messageBox(
                    "No local airfoil selected.\n"
                    "Add .dat files to the data/ folder and re-run the script."
                )
                return

            chord_cm      = cfg["chord_cm"]
            enable_split  = cfg["enable_split"]
            split_pct     = cfg["split_pct"]
            enable_offset = cfg["enable_offset"]
            offset_mm     = cfg["offset_mm"]
            enable_hinge  = cfg["enable_hinge"]
            hinge_h_mm    = cfg["hinge_slot_height_mm"]
            hinge_d_mm    = cfg["hinge_slot_depth_mm"]
            cs_inset_mm   = cfg["cs_inset_mm"]
            enable_spars  = cfg["enable_spars"]
            spar_diam_mm  = cfg["spar_diameter_mm"]
            spar_clr_mm   = cfg["spar_clearance_mm"]
            enable_light  = cfg["enable_lightening"]

            spar_diam_cm  = spar_diam_mm / Drawing.MM_PER_CM

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
                # Enforce minimum 2mm gap between aft spar edge and hinge
                if enable_split and enable_hinge and 0 < split_pct < 100:
                    min_hinge_gap_mm = 2.0
                    shortfall = max(0.0, min_hinge_gap_mm - spar_clr_mm)
                    max_x -= shortfall / chord_mm
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
                "spar_diameter_mm": spar_diam_mm,
                "spar_clearance_mm": spar_clr_mm,
                "enable_lightening": enable_light,
            })

        except Exception:
            _app.userInterface.messageBox(
                "Error generating sketch:\n" + traceback.format_exc()
            )


# -- Entry Point ---------------------------------------------------------------

def run(context):
    global _app, _handlers
    global Airfoils, Geometry, Offset, Infill, Preview, Drawing, GUI
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
        # Offset), Preview before GUI, and all geometry modules before Drawing.
        import Airfoils
        import Geometry
        import Offset
        import Infill
        import Preview
        import Drawing
        import GUI
        importlib.reload(Airfoils)
        importlib.reload(Geometry)
        importlib.reload(Offset)
        importlib.reload(Infill)
        importlib.reload(Preview)
        importlib.reload(Drawing)
        importlib.reload(GUI)

        GUI.init(_app, {
            "Geometry": Geometry,
            "Offset": Offset,
            "Infill": Infill,
            "Preview": Preview,
            "Drawing": Drawing,
        })

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
