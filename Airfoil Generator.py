"""
Fusion 360 Script: Airfoil Sketch Generator
--------------------------------------------
Scans a "data/" folder (next to this script) for .dat files and lists
them in a dropdown. Selecting one and clicking OK creates a closed
spline sketch on the XY plane.

Supports both Selig and Lednicer .dat formats.
To add airfoils: drop any .dat file into the data/ folder and re-run.

Run via: Tools > Add-Ins > Scripts and Add-Ins > "+" > select file > Run
"""

import os
import adsk.core
import adsk.fusion
import traceback

_EMPTY_LOCAL = "(no local airfoils - add .dat files to the data/ folder)"

_app      = None
_handlers = []


# -- .dat File Parsing ---------------------------------------------------------

def parse_dat_file(filepath):
    """
    Parse a Selig or Lednicer format .dat file.

    Selig format  : name on line 1, then x y pairs going TE->upper->LE->lower->TE
    Lednicer format: name on line 1, point counts on line 2, then upper surface
                     (LE->TE) followed by lower surface (LE->TE)

    Returns (name: str, coords: [(x, y), ...])
    """
    with open(filepath, "r") as fh:
        raw_lines = fh.readlines()

    if not raw_lines:
        raise ValueError(f"Empty file: {filepath}")

    name = raw_lines[0].strip()

    # Collect non-empty data lines after the header
    data_lines = [l.strip() for l in raw_lines[1:] if l.strip()]

    if not data_lines:
        raise ValueError(f"No coordinate data in {filepath!r}")

    def try_floats(line):
        try:
            return [float(t) for t in line.split()]
        except ValueError:
            return []

    first_vals = try_floats(data_lines[0])

    # Lednicer detection: first data line has exactly 2 tokens, both are
    # whole numbers >= 2 (they are point counts, e.g. "17.   17.")
    is_lednicer = (
        len(first_vals) == 2
        and all(v == int(v) and v >= 2 for v in first_vals)
    )

    if is_lednicer:
        n_upper = int(first_vals[0])
        n_lower = int(first_vals[1])
        coord_lines = data_lines[1:]

        def parse_pair(line):
            v = try_floats(line)
            if len(v) < 2:
                raise ValueError(f"Bad coordinate line: {line!r}")
            return (v[0], v[1])

        upper = [parse_pair(coord_lines[i]) for i in range(n_upper)]
        lower = [parse_pair(coord_lines[n_upper + i]) for i in range(n_lower)]

        # Lednicer: upper goes LE(0) -> TE(1), lower goes LE(0) -> TE(1)
        # Convert to a single closed loop: TE -> upper -> LE -> lower -> TE
        coords = list(reversed(upper)) + lower[1:]  # skip duplicate LE point

    else:
        coords = []
        for line in data_lines:
            vals = try_floats(line)
            if len(vals) >= 2:
                coords.append((vals[0], vals[1]))

    if not coords:
        raise ValueError(f"Could not parse any coordinates from {filepath!r}")

    return name, coords


def load_data_folder(script_dir):
    """
    Scan <script_dir>/data/ for *.dat files.

    Returns
    -------
    airfoils : dict  { display_name: (filepath, coords) }
    errors   : list  of (filename, error_message) tuples
    """
    data_dir = os.path.join(script_dir, "data")
    airfoils = {}
    errors   = []

    if not os.path.isdir(data_dir):
        return airfoils, errors

    for filename in sorted(os.listdir(data_dir)):
        if not filename.lower().endswith(".dat"):
            continue
        filepath = os.path.join(data_dir, filename)
        try:
            name, coords = parse_dat_file(filepath)
            display = name
            suffix  = 2
            while display in airfoils:
                display = f"{name} ({suffix})"
                suffix += 1
            airfoils[display] = (filepath, coords)
        except Exception as exc:
            errors.append((filename, str(exc)))

    return airfoils, errors


# -- Fusion 360 Dialog ---------------------------------------------------------

class AirfoilCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self, script_dir, airfoils):
        super().__init__()
        self._script_dir = script_dir
        self._airfoils   = airfoils

    def notify(self, args):
        try:
            cmd    = args.command
            inputs = cmd.commandInputs

            local_dd = inputs.addDropDownCommandInput(
                "local_airfoil", "Profile",
                adsk.core.DropDownStyles.TextListDropDownStyle,
            )
            if self._airfoils:
                for i, name in enumerate(self._airfoils):
                    local_dd.listItems.add(name, i == 0)
            else:
                local_dd.listItems.add(_EMPTY_LOCAL, True)
                local_dd.isEnabled = False

            inputs.addValueInput(
                "chord", "Chord Length", "mm",
                adsk.core.ValueInput.createByReal(10.0),  # 10 cm = 100 mm
            )

            on_execute = AirfoilCommandExecuteHandler(self._airfoils)
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

        except Exception:
            _app.userInterface.messageBox(
                "Error building dialog:\n" + traceback.format_exc()
            )


class AirfoilCommandExecuteHandler(adsk.core.CommandEventHandler):
    """Called when the user clicks OK -- creates the sketch."""

    def __init__(self, airfoils):
        super().__init__()
        self._airfoils = airfoils

    def notify(self, args):
        try:
            inputs = args.command.commandInputs

            local_dd = inputs.itemById("local_airfoil")
            selected = local_dd.selectedItem

            if not selected or selected.name == _EMPTY_LOCAL:
                _app.userInterface.messageBox(
                    "No local airfoil selected.\n"
                    "Add .dat files to the data/ folder and re-run the script."
                )
                return

            selected_name = selected.name
            chord_cm      = inputs.itemById("chord").value   # Fusion returns cm

            if selected_name not in self._airfoils:
                _app.userInterface.messageBox(
                    f'"{selected_name}" not found - this should not happen. '
                    "Try re-running the script."
                )
                return

            _, coords = self._airfoils[selected_name]

            des      = adsk.fusion.Design.cast(_app.activeProduct)
            root     = des.rootComponent
            sketch   = root.sketches.add(root.xYConstructionPlane)
            sketch.name = f"{selected_name} c={chord_cm * 10:.1f}mm"

            pts = adsk.core.ObjectCollection.create()
            for xn, yn in coords:
                pts.add(adsk.core.Point3D.create(xn * chord_cm, yn * chord_cm, 0.0))
            # Close the loop
            pts.add(adsk.core.Point3D.create(
                coords[0][0] * chord_cm, coords[0][1] * chord_cm, 0.0
            ))

            spline = sketch.sketchCurves.sketchFittedSplines.add(pts)
            spline.isClosed = True

            _app.userInterface.messageBox(
                f'Sketch created: "{sketch.name}"\n'
                f"    Airfoil : {selected_name}\n"
                f"    Chord   : {chord_cm * 10:.2f} mm\n"
                f"    Points  : {len(coords)}"
            )

        except Exception:
            _app.userInterface.messageBox(
                "Error generating sketch:\n" + traceback.format_exc()
            )


# -- Entry Point ---------------------------------------------------------------

def run(context):
    global _app, _handlers
    _handlers = []

    try:
        _app = adsk.core.Application.get()
        ui   = _app.userInterface

        if not adsk.fusion.Design.cast(_app.activeProduct):
            ui.messageBox("No active Fusion 360 design found.\n"
                          "Please open or create a design first.")
            return

        script_dir = os.path.dirname(os.path.realpath(__file__))
        data_dir   = os.path.join(script_dir, "data")
        os.makedirs(data_dir, exist_ok=True)   # create data/ if it doesn't exist

        airfoils, errors = load_data_folder(script_dir)

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

        on_created = AirfoilCommandCreatedHandler(script_dir, airfoils)
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        cmd_def.execute()
        adsk.autoTerminate(False)

    except Exception:
        if _app:
            _app.userInterface.messageBox(
                "Script startup error:\n" + traceback.format_exc()
            )