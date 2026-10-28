"""Step-response and disturbance-rejection comparison of PID tunings.

Runs the Ziegler-Nichols closed-loop tuning from ``pid.py`` on a fixed FOPDT
plant, derives P, PI and PID settings from the ultimate gain/period, adds a
model-based SIMC PI tuning, simulates all four loops and writes:

    results/step_metrics.csv
    results/step_response.png
    results/disturbance_metrics.csv
    results/disturbance_rejection.png

Tuning rules
------------
Ziegler-Nichols ultimate-cycle rules (Ziegler & Nichols, 1942,
"Optimum settings for automatic controllers", Trans. ASME 64, 759-768):
    P   : Kc = 0.50 Ku
    PI  : Kc = 0.45 Ku, Ti = Tu / 1.2
    PID : Kc = 0.60 Ku, Ti = Tu / 2,  Td = Tu / 8

SIMC PI for a FOPDT plant K e^(-theta s) / (tau s + 1)
(Skogestad, 2003, "Simple analytic rules for model reduction and PID
controller tuning", J. Process Control 13, 291-309, Eq. (21)-(22)):
    Kc = tau / (K (tau_c + theta)),  Ti = min(tau, 4 (tau_c + theta))
with the recommended "tight but robust" choice tau_c = theta.

The controller in ``pid.py`` is in parallel form, so ki = Kc / Ti and
kd = Kc * Td.

Usage:  python analysis.py
"""
import csv
import logging
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from pid import FirstOrderSystem, PIDController, ziegler_nichols_tuning  # noqa: E402

# Fixed parameters (same plant as the config.py defaults).
K = 1.0
TAU = 1.0
DEAD_TIME = 0.5
SETPOINT = 1.0
DT = 0.01
DURATION = 20.0
TUNING_MAX_TIME = 50.0
SETTLING_BAND = 0.02
DISTURBANCE = 0.5  # load step added at the plant input, in units of u

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

METRIC_FIELDS = [
    "controller", "kp", "ki", "kd",
    "rise_time_s", "overshoot_pct", "settling_time_s",
    "steady_state_error", "iae",
]
DISTURBANCE_FIELDS = [
    "controller", "peak_deviation", "recovery_time_s",
    "steady_state_error", "iae",
]


# --------------------------------------------------------------------------
# Tuning
# --------------------------------------------------------------------------
def ultimate_from_zn_pid(kp: float, ki: float, kd: float) -> Tuple[float, float]:
    """Recover (Ku, Tu) from the classic Z-N PID gains returned by pid.py."""
    ku = kp / 0.6
    tu = 1.2 * ku / ki
    return ku, tu


def zn_rules(ku: float, tu: float) -> Dict[str, Tuple[float, float, float]]:
    """Ziegler-Nichols ultimate-cycle settings as parallel (kp, ki, kd)."""
    p = (0.5 * ku, 0.0, 0.0)
    kc_pi, ti_pi = 0.45 * ku, tu / 1.2
    pi = (kc_pi, kc_pi / ti_pi, 0.0)
    kc_pid, ti_pid, td_pid = 0.6 * ku, tu / 2.0, tu / 8.0
    pid = (kc_pid, kc_pid / ti_pid, kc_pid * td_pid)
    return {"P (Z-N)": p, "PI (Z-N)": pi, "PID (Z-N)": pid}


def simc_pi(k: float, tau: float, theta: float,
            tau_c: Optional[float] = None) -> Tuple[float, float, float]:
    """SIMC PI tuning (Skogestad 2003) as parallel (kp, ki, kd)."""
    if tau_c is None:
        tau_c = theta
    kc = tau / (k * (tau_c + theta))
    ti = min(tau, 4.0 * (tau_c + theta))
    return kc, kc / ti, 0.0


def compute_tunings() -> Dict[str, Tuple[float, float, float]]:
    """Run the Z-N experiment on the fixed plant and return all tunings."""
    system = FirstOrderSystem(K, TAU, DEAD_TIME)
    kp, ki, kd = ziegler_nichols_tuning(system, setpoint=SETPOINT, dt=DT,
                                        max_time=TUNING_MAX_TIME)
    if kp == 0.0:
        raise RuntimeError("Ziegler-Nichols tuning did not find an ultimate gain")
    ku, tu = ultimate_from_zn_pid(kp, ki, kd)
    tunings = zn_rules(ku, tu)
    tunings["PI (SIMC)"] = simc_pi(K, TAU, DEAD_TIME)
    return tunings


# --------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------
def simulate(gains: Tuple[float, float, float], duration: float = DURATION,
             dt: float = DT, setpoint: float = SETPOINT,
             disturbance: float = 0.0,
             disturbance_time: Optional[float] = None
             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Closed-loop simulation. Returns arrays (t, y, u).

    y[i] is the measurement used at t[i]; u[i] is the controller output
    computed from it. A load disturbance, if given, is added to the plant
    input from ``disturbance_time`` on (it then passes through the dead time).
    """
    system = FirstOrderSystem(K, TAU, DEAD_TIME)
    controller = PIDController(*gains)
    n = int(round(duration / dt)) + 1
    t = np.arange(n) * dt
    y = np.zeros(n)
    u = np.zeros(n)
    for i in range(n):
        y[i] = system.y
        u[i] = controller.update(setpoint, y[i], dt)
        d = disturbance if (disturbance_time is not None
                            and t[i] >= disturbance_time - 1e-12) else 0.0
        system.update(u[i] + d, dt)
    return t, y, u


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------
def _crossing_time(t: np.ndarray, y: np.ndarray, level: float) -> float:
    """First time y crosses ``level`` upward, linearly interpolated."""
    idx = np.nonzero(y >= level)[0]
    if idx.size == 0:
        return float("nan")
    i = int(idx[0])
    if i == 0:
        return float(t[0])
    y0, y1 = y[i - 1], y[i]
    return float(t[i - 1] + (level - y0) / (y1 - y0) * (t[i] - t[i - 1]))


def step_metrics(t: np.ndarray, y: np.ndarray, setpoint: float = SETPOINT,
                 band: float = SETTLING_BAND,
                 final_window: float = 0.05) -> Dict[str, float]:
    """Step-response metrics.

    final value y_inf : mean of the last ``final_window`` fraction of samples
    rise time        : 10 % -> 90 % of y_inf (interpolated)
    overshoot        : (max(y) - y_inf) / y_inf * 100, floored at 0
    settling time    : first time after which |y - y_inf| <= band * |y_inf|
    steady-state err : setpoint - y_inf
    IAE              : trapezoidal integral of |setpoint - y|
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    n_final = max(1, int(round(final_window * len(y))))
    y_inf = float(np.mean(y[-n_final:]))

    rise = _crossing_time(t, y, 0.9 * y_inf) - _crossing_time(t, y, 0.1 * y_inf)
    overshoot = max(0.0, (float(np.max(y)) - y_inf) / y_inf * 100.0)

    outside = np.nonzero(np.abs(y - y_inf) > band * abs(y_inf))[0]
    if outside.size == 0:
        settling = float(t[0])
    elif outside[-1] == len(y) - 1:
        settling = float("nan")
    else:
        settling = float(t[outside[-1] + 1])

    err = np.abs(setpoint - y)
    iae = float(np.sum(0.5 * (err[1:] + err[:-1]) * np.diff(t)))
    return {
        "rise_time_s": rise,
        "overshoot_pct": overshoot,
        "settling_time_s": settling,
        "steady_state_error": setpoint - y_inf,
        "iae": iae,
    }


def disturbance_metrics(t: np.ndarray, y: np.ndarray, t_d: float,
                        setpoint: float = SETPOINT,
                        band: float = SETTLING_BAND,
                        final_window: float = 0.05) -> Dict[str, float]:
    """Metrics for the segment after a load step at ``t_d``.

    peak deviation : max |y - y_before| after t_d (y_before = y at t_d)
    recovery time  : time after t_d until |y - setpoint| stays <= band*|setpoint|
                     (NaN if the loop has a permanent offset larger than that)
    IAE            : integral of |setpoint - y| over the segment
    """
    mask = t >= t_d - 1e-12
    ts, ys = t[mask], y[mask]
    y_before = float(ys[0])
    n_final = max(1, int(round(final_window * len(ys))))
    y_inf = float(np.mean(ys[-n_final:]))
    outside = np.nonzero(np.abs(ys - setpoint) > band * abs(setpoint))[0]
    if outside.size == 0:
        recovery = 0.0
    elif outside[-1] == len(ys) - 1:
        recovery = float("nan")
    else:
        recovery = float(ts[outside[-1] + 1] - t_d)
    err = np.abs(setpoint - ys)
    iae = float(np.sum(0.5 * (err[1:] + err[:-1]) * np.diff(ts)))
    return {
        "peak_deviation": float(np.max(np.abs(ys - y_before))),
        "recovery_time_s": recovery,
        "steady_state_error": setpoint - y_inf,
        "iae": iae,
    }


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------
def _fmt(v):
    """Format floats to 4 decimals; avoid printing '-0.0000'."""
    if not isinstance(v, float):
        return v
    text = f"{v:.4f}"
    return "0.0000" if text == "-0.0000" else text


def _write_csv(path: str, fields: List[str], rows: List[Dict]) -> None:
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: _fmt(v) for k, v in row.items()})


def _plot(runs, title: str, path: str, t_d: Optional[float] = None) -> None:
    fig, (ax_y, ax_u) = plt.subplots(2, 1, figsize=(9, 7), sharex=True,
                                     gridspec_kw={"height_ratios": [3, 2]})
    for name, (t, y, u) in runs.items():
        ax_y.plot(t, y, label=name, linewidth=1.6)
        ax_u.plot(t, u, label=name, linewidth=1.2)
    t0 = next(iter(runs.values()))[0]
    ax_y.plot(t0, np.full_like(t0, SETPOINT), "k--", linewidth=1.0,
              label="setpoint")
    ax_y.axhspan(SETPOINT * (1 - SETTLING_BAND), SETPOINT * (1 + SETTLING_BAND),
                 color="0.85", zorder=0, label="±2 % band")
    if t_d is not None:
        for ax in (ax_y, ax_u):
            ax.axvline(t_d, color="0.4", linestyle=":", linewidth=1.0)
        ax_y.annotate(f"load step d = {DISTURBANCE:g}", (t_d, ax_y.get_ylim()[0]),
                      xytext=(4, 6), textcoords="offset points", fontsize=8,
                      color="0.3")
    ax_y.set_ylabel("process variable y")
    ax_y.set_title(title)
    ax_y.grid(True, alpha=0.3)
    ax_y.legend(loc="lower right", fontsize=8)
    ax_u.set_ylabel("controller output u")
    ax_u.set_xlabel("time [s]")
    ax_u.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def run_analysis(out_dir: str = RESULTS_DIR) -> Dict[str, List[Dict]]:
    """Run tuning, simulations, metrics; write CSVs and figures."""
    os.makedirs(out_dir, exist_ok=True)
    tunings = compute_tunings()
    plant = f"G(s) = {K:g}·e^(-{DEAD_TIME:g}s) / ({TAU:g}s + 1)"

    step_runs, step_rows = {}, []
    for name, gains in tunings.items():
        t, y, u = simulate(gains)
        step_runs[name] = (t, y, u)
        row = {"controller": name, "kp": gains[0], "ki": gains[1], "kd": gains[2]}
        row.update(step_metrics(t, y))
        step_rows.append(row)
    _write_csv(os.path.join(out_dir, "step_metrics.csv"), METRIC_FIELDS, step_rows)
    _plot(step_runs, f"Setpoint step response, {plant}",
          os.path.join(out_dir, "step_response.png"))

    t_d = DURATION / 2.0
    dist_runs, dist_rows = {}, []
    for name, gains in tunings.items():
        t, y, u = simulate(gains, disturbance=DISTURBANCE, disturbance_time=t_d)
        dist_runs[name] = (t, y, u)
        row = {"controller": name}
        row.update(disturbance_metrics(t, y, t_d))
        dist_rows.append(row)
    _write_csv(os.path.join(out_dir, "disturbance_metrics.csv"),
               DISTURBANCE_FIELDS, dist_rows)
    _plot(dist_runs, f"Setpoint step + input load step at t = {t_d:g} s, {plant}",
          os.path.join(out_dir, "disturbance_rejection.png"), t_d=t_d)
    return {"step": step_rows, "disturbance": dist_rows}


def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    rows = run_analysis()
    for key in ("step", "disturbance"):
        print(f"\n[{key}]")
        for row in rows[key]:
            print({k: (round(v, 4) if isinstance(v, float) else v)
                   for k, v in row.items()})
    print(f"\nWrote results to {RESULTS_DIR}")


if __name__ == "__main__":
    main()
