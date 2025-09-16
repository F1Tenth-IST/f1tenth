#!/usr/bin/env python3
"""
Python reimplementation of the MATLAB script `sim_traj_mpc_curv_off.m`,
now wired to load your controller/vehicle parameters from a simple YAML-like
config file (exactly like the block you sent).

It uses your existing acados-based model & OCP setup in:
  - acados_settings.py
  - bicycle_model.py

Key features:
- Loads track CSV: [s, x, y, kappa, (nl), (nr)] and allows a constant width override.
- Builds the OCP with `acados_settings(...)` and runs a closed-loop MPC.
- Supports input/actuation delay via `steps_delay` + `t_delay` (rounded to whole steps).
- Soft constraints with Zl/Zu/zl/zu as in your params.
- CLI flag `--config` to read the params file; if omitted, sensible defaults are used.

Run (example):
  python sim_traj_mpc_curv_off_v2.py \
      --csv ../../traj/centerline_test_map.csv \
      --config params.yaml \
      --sim-time 80

Where `params.yaml` is the text you posted.
"""
from __future__ import annotations
import argparse
import sys
from types import SimpleNamespace
from pathlib import Path
import re
from collections import deque
import numpy as np
import casadi as cs
import time

# acados OCP builder from your project
from acados_settings import acados_settings, get_parameters

# ---------------------------- simple YAML-ish parser ----------------------------

def _coerce_scalar(val: str):
    v = val.strip()
    # drop YAML tags like !!float
    v = re.sub(r"^!![a-zA-Z_]+\s*", "", v)
    # strip quotes
    if (len(v) >= 2 and ((v[0] == v[-1] == '"') or (v[0] == v[-1] == "'"))):
        v = v[1:-1]
    # booleans
    if v.lower() in ("true", "yes", "on"): return True
    if v.lower() in ("false", "no", "off"): return False
    # numbers
    try:
        if re.fullmatch(r"[-+]?\d+", v):
            return int(v)
        # allow 1e3, -2.5, etc.
        if re.fullmatch(r"[-+]?\d*(?:\.\d+)?(?:[eE][-+]?\d+)?", v) and v not in ("", "."):
            return float(v)
    except Exception:
        pass
    return v


def load_params_file(path: Path) -> dict:
    """Parse a flat YAML-like file with lines `key: value`. Ignores sections and comments.
    The *last* occurrence of a key wins (like many YAML parsers).
    """
    out: dict[str, object] = {}
    txt = Path(path).read_text(encoding="utf-8")
    for line in txt.splitlines():
        # remove comments
        if "#" in line:
            line = line.split("#", 1)[0]
        if not line.strip():
            continue
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        out[k.strip()] = _coerce_scalar(v)
    return out


# ---------------------------- I/O ----------------------------

def load_track_csv(path: Path, track_width_override: float | None = None):
    """Load track arrays from CSV. Returns dict with s, x, y, kappa, nl, nr."""
    arr = np.genfromtxt(path, delimiter=",", skip_header=1)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 4:
        raise ValueError(
            f"CSV needs at least 4 cols [s,x,y,kappa], got shape {arr.shape}")

    s = arr[:, 0].astype(float)
    x = arr[:, 1].astype(float)
    y = arr[:, 2].astype(float)
    kappa = arr[:, 3].astype(float)

    if arr.shape[1] >= 6:
        nl = arr[:, 4].astype(float)
        nr = arr[:, 5].astype(float)
    else:
        nl = np.ones_like(kappa)
        nr = np.ones_like(kappa)

    if track_width_override is not None:
        nl[:] = track_width_override
        nr[:] = track_width_override

    return dict(s=s, x=x, y=y, kappa=kappa, nl=nl, nr=nr)


# ---------------------------- CONFIG ----------------------------

DEFAULTS = {
    # Controller settings
    "N": 40,
    "MPC_freq": 20,
    "t_delay": 0.0025,
    "steps_delay": 3,
    "track_safety_margin": 0.25,
    "track_max_width": 1e3,
    "overtake_d": 1.0,
    # Cost
    "qjerk": 1e-2,
    "qddelta": 5e-1,
    "qadv": 0.0,
    "qn": 40.0,
    "qalpha": 0.1,
    "qv": 10.0,
    "Zl": 1000.0,
    "Zu": 1000.0,
    "zl": 1000.0,
    "zu": 1000.0,
    # Constraints (state)
    "delta_min": -0.4,
    "delta_max": 0.4,
    "v_min": -5.0,  # note: last occurrence wins
    "v_max": 10.0,
    "a_min": -3.0,
    "a_max": 3.0,
    # Constraints (inputs)
    "ddelta_min": -3.2,
    "ddelta_max": 3.2,
    "jerk_min": -50.0,
    "jerk_max": 50.0,
    # Nonlinear
    "alat_max": 10.0,
    # Flags
    "vy_minimization": True,
    "adv_maximization": False,
    "combined_constraints": "None",  # ellipse/diamond/None
    "load_transfer": True,
    "correct_v_y_dot": True,
    # Vehicle
    "m": 3.54,
    "Iz": 0.05797,
    "lf": 0.162,
    "lr": 0.145,
    "wheelbase": 0.307,
    "h_cg": 0.014,
    # Aerodynamic/drivetrain-ish (passed through; use in your model if needed)
    "C_0d": 0.48,
    "C_d": -1.1,
    "C_acc": 8.29,
    "C_dec": 5.77,
    "C_R": 2.03,
    "C_0v": 100.0,
    "C_v": 20.0,
    # Actuator limits/dynamics
    "tau_steer": 0.15779476,
    "max_steering_angle": 0.4189,
    "max_steering_velocity": 3.2,
    "racecar_version": "NUC2",
}


def make_configs(overrides: dict | None = None) -> tuple[SimpleNamespace, SimpleNamespace, SimpleNamespace]:
    """Create config objects matching the attributes used by your code.
    Values come from DEFAULTS updated by `overrides`.
    """
    #cfg = DEFAULTS.copy()
    cfg = dict()
    for override in overrides or []:
        cfg.update(override)

    # == STMPC (solver + weights + constraints) ==
    stmpc = SimpleNamespace()
    stmpc.N = int(cfg["N"])                     # horizon steps
    stmpc.MPC_freq = float(cfg["MPC_freq"])     # Hz (1/Ts)
    stmpc.t_delay = float(cfg.get("t_delay", 0.0))
    stmpc.steps_delay = int(cfg.get("steps_delay", 0))

    stmpc.track_safety_margin = float(cfg["track_safety_margin"])  # m
    stmpc.track_max_width = float(cfg["track_max_width"])          # sentinel upper bound
    stmpc.overtake_d = float(cfg.get("overtake_d", 0.0))

    # slack penalties
    stmpc.Zl = float(cfg["Zl"]) ; stmpc.Zu = float(cfg["Zu"]) ; stmpc.zl = float(cfg["zl"]) ; stmpc.zu = float(cfg["zu"])

    # weights
    stmpc.qadv = float(cfg["qadv"])       # progress maximization
    stmpc.qv = float(cfg["qv"])           # speed tracking (linear/quad depending on your model)
    stmpc.qn = float(cfg["qn"])           # lateral error
    stmpc.qalpha = float(cfg["qalpha"])   # heading error
    stmpc.qjerk = float(cfg["qjerk"])     # smooth accel
    stmpc.qddelta = float(cfg["qddelta"]) # smooth steering rate

    # bounds for states & inputs
    stmpc.v_min = float(cfg["v_min"]) ; stmpc.v_max = float(cfg["v_max"]) ; stmpc.a_min = float(cfg["a_min"]) ; stmpc.a_max = float(cfg["a_max"]) ;
    stmpc.delta_min = float(cfg["delta_min"]) ; stmpc.delta_max = float(cfg["delta_max"]) ;
    stmpc.ddelta_min = float(cfg["ddelta_min"]) ; stmpc.ddelta_max = float(cfg["ddelta_max"]) ;
    stmpc.jerk_min = float(cfg["jerk_min"]) ; stmpc.jerk_max = float(cfg["jerk_max"]) ;

    # nonlinear constraint
    stmpc.alat_max = float(cfg["alat_max"])    # lateral acceleration cap

    # flags / switches
    stmpc.vy_minimization = bool(cfg["vy_minimization"])
    stmpc.adv_maximization = bool(cfg["adv_maximization"])
    stmpc.load_transfer = bool(cfg["load_transfer"])
    stmpc.correct_v_y_dot = bool(cfg["correct_v_y_dot"])

    cc = str(cfg.get("combined_constraints", "None"))
    stmpc.combined_constraints = cc if cc.lower() != "none" else "None"

    # == Vehicle params ==
    car = SimpleNamespace(
        lr=float(cfg["lr"]),
        lf=float(cfg["lf"]),
        m=float(cfg["m"]),
        Iz=float(cfg["Iz"]),
        h_cg=float(cfg["h_cg"]),
        wheelbase=float(cfg.get("wheelbase", float(cfg["lf"]) + float(cfg["lr"]))),
        # pass-through extra params in case your model uses them
        C_0d=float(cfg.get("C_0d", 0.0)),
        C_d=float(cfg.get("C_d", 0.0)),
        C_acc=float(cfg.get("C_acc", 0.0)),
        C_dec=float(cfg.get("C_dec", 0.0)),
        C_R=float(cfg.get("C_R", 0.0)),
        C_0v=float(cfg.get("C_0v", 0.0)),
        C_v=float(cfg.get("C_v", 0.0)),
        tau_steer=float(cfg.get("tau_steer", 0.0)),
        max_steering_angle=float(cfg.get("max_steering_angle", 0.0)),
        max_steering_velocity=float(cfg.get("max_steering_velocity", 0.0)),
        racecar_version=str(cfg.get("racecar_version", "")),
    )

    # == Tire params (keep placeholders; adapt if your bicycle_model uses these)
    tire = SimpleNamespace(
        friction_coeff=1.0,
        Bf=7.6671, Cf=1.2628, Df=1.2307, Ef=0.0,
        Br=7.1036, Cr=1.7356, Dr=1.0252, Er=0.0,
    )

    return stmpc, car, tire


# ---------------------------- SIM ----------------------------

def run_sim(track_csv: Path, sim_time: float, cfg_overrides: dict | None,
            track_width: float | None):
    # load track
    tr = load_track_csv(track_csv, track_width_override=track_width)
    s0 = tr["s"].copy()
    kappa = tr["kappa"].copy()
    d_left = tr["nl"].copy()
    d_right = tr["nr"].copy()

    # configs
    stmpc, car, tire = make_configs(cfg_overrides)

    # build OCP & solver
    # returns: constraint, model, acados_solver, params(ns)
    constraint, model, solver, params_ns = acados_settings(
        s0, kappa, d_left, d_right, stmpc, car, tire
    )

    # param vector used online (kept constant here)
    p_vec = get_parameters(stmpc).astype(float)
    # set parameters for every stage (0..N)
    for stage in range(stmpc.N + 1):
        solver.set(stage, "p", p_vec)

    # initial state (near zero)
    x = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0], dtype=float)

    # logging
    dt = 1.0 / float(stmpc.MPC_freq)
    n_steps = int(sim_time * stmpc.MPC_freq)
    hist_x = np.zeros((n_steps + 1, model.n_x))
    hist_u = np.zeros((n_steps, model.n_u))
    status_hist = np.zeros(n_steps, dtype=int)
    solve_ms = np.zeros(n_steps)

    hist_x[0, :] = x

    # effective discrete delay in steps (steps_delay + t_delay rounded)
    extra_steps = int(round(stmpc.t_delay * stmpc.MPC_freq)) if hasattr(stmpc, "t_delay") else 0
    delay_steps = max(0, int(getattr(stmpc, "steps_delay", 0)) + extra_steps)
    u_buffer: deque[np.ndarray] = deque([np.zeros(model.n_u) for _ in range(delay_steps)], maxlen=max(delay_steps, 1))

    # last control (for fallback integration)
    u_prev = np.zeros(model.n_u)

    for k in range(n_steps):
        # provide initial condition
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)

        t0 = time.perf_counter()
        ret = solver.solve()
        solve_ms[k] = float((time.perf_counter() - t0) * 1e3)
        status = ret
        status_hist[k] = status

        if status != 0:
            # Failure: integrate one step with previous applied control
            sys.stderr.write(f"[WARN] Solver failed at step {k} with status {status}. Integrating 1 step.\n")
            u_cmd = u_prev.copy()
            quit()
        else:
            u_cmd = np.array(solver.get(stage=0, field="u")).reshape(-1)

        # enqueue commanded u, decide applied u (with delay)
        if delay_steps > 0:
            u_buffer.append(u_cmd.copy())
            u_applied = u_buffer.popleft()
        else:
            u_applied = u_cmd

        # state update
        if status == 0 and delay_steps == 0:
            # use optimizer's predicted next state for speed
            x_next = np.array(solver.get(stage=1, field="x")).reshape(-1)
        else:
            # integrate one step with (possibly delayed) control
            f = model.f_expl_func(
                x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7],
                float(u_applied[0]), float(u_applied[1]), p_vec
            ).full().reshape(-1)
            x_next = (x + dt * f).astype(float)

        # log
        hist_x[k + 1, :] = x_next
        hist_u[k, :] = u_applied
        u_prev = u_applied
        x = x_next

    return {
        "x_hist": hist_x,
        "u_hist": hist_u,
        "status": status_hist,
        "solve_ms": solve_ms,
        "dt": dt,
        "freq": float(stmpc.MPC_freq),
    }


# ---------------------------- MAIN ----------------------------

def main():
    ap = argparse.ArgumentParser(description="Closed-loop MPC sim (acados)")
    ap.add_argument("--csv", type=Path, default="../../traj/centerline_test_map.csv",
                    help="Path to track CSV [s,x,y,kappa,(nl),(nr)]")
    ap.add_argument("--mpc_config", type=Path, default="./mpc_config.yaml",
                    help="YAML-like MPC params file (use the block you sent)")
    ap.add_argument("--car_config", type=Path, default="./car_model.yaml",
                    help="YAML-like car params file (use the block you sent)")
    ap.add_argument("--sim-time", type=float, default=80.0,
                    help="Total sim time [s]")
    ap.add_argument("--track-width", type=float, default=None,
                    help="Override track half-widths nl,nr with this value [m]")
    ap.add_argument("--no-plot", action="store_true", help="Disable plotting")
    args = ap.parse_args()

    overrides = []
    overrides.append(load_params_file(args.car_config)) if args.car_config.exists() else None
    overrides.append(load_params_file(args.mpc_config)) if args.mpc_config.exists() else None

    res = run_sim(
    track_csv=args.csv,
    sim_time=args.sim_time,
    cfg_overrides=overrides,
    track_width=args.track_width,
    )

    x_hist = res["x_hist"]
    u_hist = res["u_hist"]
    print(f"Done. Steps: {u_hist.shape[0]}, avg solve time: {res['solve_ms'].mean():.2f} ms at {res['freq']:.1f} Hz")

    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
        except Exception as e:
            print("matplotlib not available, skipping plots:", e)
            sys.exit(0)

        t = np.arange(x_hist.shape[0]) * res["dt"]
        fig1, ax = plt.subplots(3, 1, sharex=True)
        ax[0].plot(t, x_hist[:, 0]); ax[0].set_ylabel("s [m]")
        ax[1].plot(t, x_hist[:, 1]); ax[1].set_ylabel("n [m]")
        ax[2].plot(t, x_hist[:, 2]); ax[2].set_ylabel("theta [rad]")
        ax[2].set_xlabel("t [s]")

        fig2, ax2 = plt.subplots(3, 1, sharex=True)
        ax2[0].plot(t, x_hist[:, 3]); ax2[0].set_ylabel("v_x [m/s]")
        ax2[1].plot(t, x_hist[:, 4]); ax2[1].set_ylabel("v_y [m/s]")
        ax2[2].plot(t, x_hist[:, 6]); ax2[2].set_ylabel("yaw_rate [rad/s]")
        ax2[2].set_xlabel("t [s]")

        tu = np.arange(u_hist.shape[0]) * res["dt"]
        fig3, ax3 = plt.subplots(2, 1, sharex=True)
        ax3[0].plot(tu, u_hist[:, 0]); ax3[0].set_ylabel("jerk [m/s^3]")
        ax3[1].plot(tu, u_hist[:, 1]); ax3[1].set_ylabel("ddelta [rad/s]")
        ax3[1].set_xlabel("t [s]")

        plt.show()


if __name__ == "__main__":
    main()
