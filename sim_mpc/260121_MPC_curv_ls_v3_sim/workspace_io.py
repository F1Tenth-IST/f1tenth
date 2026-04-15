# workspace_io.py
import pickle
import sys
import types
from pathlib import Path
import inspect

from pathlib import Path
from datetime import datetime


def _get_casadi_context():
    """
    Return CasADi global pickle context if available, else None
    """
    try:
        import casadi as ca
        return ca.global_pickle_context()
    except Exception:
        return None

def _is_pickleable(obj):
    import pickle
    import types
    if isinstance(obj, types.ModuleType):
        return False
    if callable(obj):
        return False
    # Skip sys structs that may behave like namedtuple
    if isinstance(obj, (type(sys.float_info),)):
        return False
    try:
        pickle.dumps(obj)
        return True
    except Exception:
        return False

def _make_pickleable(obj, visited=None, max_depth=5):
    """
    Recursively convert an object into a pickleable form.
    - Stops at max_depth or cyclic references
    - Returns None if unpickleable
    """
    if visited is None:
        visited = set()

    obj_id = id(obj)
    if obj_id in visited:
        return None  # cyclic reference
    visited.add(obj_id)

    if max_depth < 0:
        return None

    # Already pickleable
    if _is_pickleable(obj):
        return obj

    # Dictionary
    if isinstance(obj, dict):
        safe_dict = {}
        for k, v in list(obj.items()):
            safe_v = _make_pickleable(v, visited, max_depth - 1)
            if safe_v is not None:
                safe_dict[k] = safe_v
        return safe_dict

    # Objects with __dict__
    if hasattr(obj, "__dict__"):
        safe_obj = {}
        for attr, val in list(obj.__dict__.items()):
            safe_val = _make_pickleable(val, visited, max_depth - 1)
            if safe_val is not None:
                safe_obj[attr] = safe_val
        return safe_obj

    # Lists, tuples, sets
    if isinstance(obj, (list, tuple, set)):
        safe_list = [_make_pickleable(v, visited, max_depth - 1) for v in obj]
        safe_list = [v for v in safe_list if v is not None]
        return type(obj)(safe_list)

    # Cannot pickle
    return None

from pathlib import Path
from datetime import datetime

def compose_log_filename(
    filename="",
    subfolder=("sim_mpc", "Log_Data"),
    prefix="workspace",
    suffix=".pkl",
):
    """
    Compose a log filename using the current working directory.
    Logic:
    - Base path = current working directory
    - If subfolder exists -> use it
    - Otherwise -> save in current directory
    - If filename empty -> auto-generate timestamped name
    """
    base = Path.cwd()
    log_dir = base.joinpath(*subfolder)

    # Use subfolder only if it already exists
    if log_dir.is_dir():
        target_dir = log_dir
    else:
        target_dir = base

    # Auto filename
    if not filename:
        timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
        filename = f"{prefix}_{timestamp}{suffix}"

    return target_dir / filename

def save_workspace(filename=None, variables=None):
    """
    Save Python variables to a pickle file, automatically stripping unpickleable fields.
    - If filename is None or empty, generates workspace_YYMMDD_HHMMSS.pkl
    - Adds caller metadata (script filename and full path)
    - Prints warnings for skipped variables
    """
    # Automatic filename if None or empty
    if not filename:
        """
        timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
        filename = f"workspace_{timestamp}.pkl"
        """
        filename = compose_log_filename(filename="")

    filename = Path(filename)

    # Get top-level variables if none provided
    if variables is None:
        frame = sys._getframe(1)
        variables = {k: v for k, v in frame.f_globals.items() if not k.startswith("__")}

    safe_vars = {}
    skipped_vars = []

    # Convert each variable to a pickleable form
    for k, v in variables.items():
        safe_v = _make_pickleable(v)
        if safe_v is not None:
            safe_vars[k] = safe_v
        else:
            skipped_vars.append(k)

    # Add caller metadata
    caller_frame = inspect.stack()[1]
    safe_vars["_workspace_metadata"] = {
        "script_filename": caller_frame.filename,
        "script_path": str(Path(caller_frame.filename).resolve())
    }

    # Save with CasADi context if available
    casadi_ctx = _get_casadi_context()
    with open(filename, "wb") as f:
        if casadi_ctx is not None:
            with casadi_ctx:
                pickle.dump(safe_vars, f, protocol=pickle.HIGHEST_PROTOCOL)
        else:
            pickle.dump(safe_vars, f, protocol=pickle.HIGHEST_PROTOCOL)

    print(f"[workspace_io] Saved {len(safe_vars)} variables → {filename.resolve()}")
    if skipped_vars:
        print(f"[workspace_io WARNING] Some variables or fields could not be saved: {skipped_vars}")

def load_workspace(filename):
    """
    Load variables from a pickle file, CasADi-aware if available.
    Returns a dictionary with all saved variables.
    """
    filename = Path(filename)
    casadi_ctx = _get_casadi_context()
    with open(filename, "rb") as f:
        if casadi_ctx is not None:
            with casadi_ctx:
                variables = pickle.load(f)
        else:
            variables = pickle.load(f)
    print(f"[workspace_io] Loaded {len(variables)} variables ← {filename.resolve()}")
    return variables
