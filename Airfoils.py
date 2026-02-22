"""
Airfoil Data I/O
----------------
.dat file parsing, data folder scanning, and config persistence.

No Fusion 360 dependencies — only os and json.
"""

import json
import os


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


# -- Config Persistence --------------------------------------------------------

def load_config(script_dir):
    """Load config.json from script_dir. Returns dict or empty dict on failure."""
    path = os.path.join(script_dir, "config.json")
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_config(script_dir, settings):
    """Save settings dict to config.json in script_dir."""
    path = os.path.join(script_dir, "config.json")
    with open(path, "w") as f:
        json.dump(settings, f, indent=2)
