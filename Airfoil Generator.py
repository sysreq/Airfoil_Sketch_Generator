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

_EMPTY_LOCAL = None  # set from Airfoils.EMPTY_LOCAL after import

_app      = None
_handlers = []
Airfoils  = None
Geometry  = None
Offset    = None
Infill    = None
Preview   = None
Drawing   = None
GUI       = None
Wing      = None
Extrude   = None


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
            hinge_type    = cfg.get("hinge_type", "CA Hinge")
            hinge_h_mm    = cfg["hinge_slot_height_mm"]
            hinge_d_mm    = cfg["hinge_slot_depth_mm"]
            cs_hinge_thick_mm = cfg.get("cs_hinge_thickness_mm", 0.4)
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

            # ---- shared 2D geometry pipeline ----
            geom = Infill.compute_2d_geometry(coords, cfg)
            spars = geom["spars"]
            spar_trunc = geom["spar_trunc"]

            spar_info = ""
            lightening_info = ""

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

            # ---- 3D wing extrusion ----
            wing_info = ""
            enable_wing = cfg.get("enable_wing", False)
            if enable_wing:
                issues = Wing.validate_wing(
                    coords,
                    half_span_mm=cfg.get("wing_half_span_mm", 500.0),
                    root_chord_mm=chord_cm * Drawing.MM_PER_CM,
                    tip_chord_mm=cfg.get("wing_tip_chord_mm", 100.0),
                    spar_diam_mm=spar_diam_mm if enable_spars else 0,
                    clearance_mm=spar_clr_mm if enable_spars else 0,
                    sweep_deg=cfg.get("wing_sweep_deg", 0.0),
                    dihedral_deg=cfg.get("wing_dihedral_deg", 0.0),
                    twist_deg=cfg.get("wing_twist_deg", 0.0),
                    n_sections=cfg.get("wing_sections", 2),
                )

                errors = [msg for sev, msg in issues if sev == "error"]
                warnings = [msg for sev, msg in issues if sev == "warning"]

                if errors:
                    _app.userInterface.messageBox(
                        "Wing validation errors:\n\n"
                        + "\n".join(f"  \u2022 {e}" for e in errors)
                    )
                else:
                    proceed = True
                    if warnings:
                        result = _app.userInterface.messageBox(
                            "Wing validation warnings:\n\n"
                            + "\n".join(f"  \u2022 {w}" for w in warnings)
                            + "\n\nContinue anyway?",
                            "Wing Warnings",
                            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                        )
                        proceed = (result == adsk.core.DialogResults.DialogYes)

                    if proceed:
                        wing_info, wing_body = Extrude.create_wing(
                            _app, root, coords, cfg,
                        )

                        # ---- 3D control surface (separate body) ----
                        cs_body = None
                        is_solid = cfg.get("wing_solid_loft", False)
                        cs_inb = cfg.get("cs_inboard_pct", 10.0)
                        cs_outb = cfg.get("cs_outboard_pct", 90.0)
                        if (enable_split and 0 < split_pct < 100
                                and cs_outb > cs_inb):
                            cs_info, cs_body = Extrude.create_control_surface(
                                _app, root, coords, cfg,
                                wing_body=(wing_body if is_solid else None),
                            )
                            wing_info += cs_info

                        # ---- Mirror all bodies after boolean ops ----
                        if cfg.get("wing_mirror", False):
                            mirror_list = [b for b in [wing_body, cs_body]
                                           if b is not None]
                            if mirror_list:
                                Extrude.mirror_bodies(root, mirror_list)

            # ---- summary ----
            _app.userInterface.messageBox(
                f'Sketch created: "{sketch.name}"\n'
                f"    Airfoil : {selected_name}\n"
                f"    Chord   : {chord_cm * Drawing.MM_PER_CM:.2f} mm\n"
                f"    Points  : {raw_count} raw -> {len(coords)} doubled"
                f"{spar_info}"
                f"{lightening_info}"
                f"{split_info}"
                f"{wing_info}"
            )

            # ---- persist settings for next run ----
            Airfoils.save_config(self._script_dir, cfg)

        except Exception:
            _app.userInterface.messageBox(
                "Error generating sketch:\n" + traceback.format_exc()
            )


# -- Entry Point ---------------------------------------------------------------

def run(context):
    global _app, _handlers, _EMPTY_LOCAL
    global Airfoils, Geometry, Offset, Infill, Preview, Drawing, GUI, Wing, Extrude
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
        # and Wing (both import Geometry), Offset before Drawing (Drawing
        # imports Offset), Wing and Infill before Extrude (Extrude imports
        # both), Preview before GUI, and all geometry modules before Drawing.
        import Airfoils
        import Geometry
        import Offset
        import Infill
        import Wing
        import Preview
        import Drawing
        import Extrude
        import GUI
        importlib.reload(Airfoils)
        _EMPTY_LOCAL = Airfoils.EMPTY_LOCAL
        importlib.reload(Geometry)
        importlib.reload(Offset)
        importlib.reload(Infill)
        importlib.reload(Wing)
        importlib.reload(Preview)
        importlib.reload(Drawing)
        importlib.reload(Extrude)
        importlib.reload(GUI)

        GUI.init(_app, {
            "Geometry": Geometry,
            "Offset": Offset,
            "Infill": Infill,
            "Preview": Preview,
            "Drawing": Drawing,
            "Wing": Wing,
            "Extrude": Extrude,
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
