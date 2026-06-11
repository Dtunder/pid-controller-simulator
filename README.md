# pid-controller-simulator

PID simulation environment with Ziegler-Nichols parameter tuning.

This simulator models a First Order Plus Dead Time (FOPDT) system and runs an automated Ziegler-Nichols tuning process to find optimal Proportional, Integral, and Derivative (PID) parameters.

## CLI Instructions

### Run the Simulator
Run the simulator and tuning process using:
```bash
python main.py
```
This script executes the Ziegler-Nichols tuning algorithm to determine the optimal PID parameters for a defined First Order Plus Dead Time (FOPDT) system. Afterward, it runs a step response simulation using the tuned parameters and saves the results to `results.csv`.

### Run the Tests
Run the unit tests to ensure all components are functioning correctly:
```bash
python -m unittest test_pid.py
```

## Configuration Setup

The simulation parameters can be configured directly in `main.py` before running the simulation. Look for the system definition section to adjust the First Order Plus Dead Time (FOPDT) characteristics:

```python
# main.py
K = 1.0         # System gain
tau = 1.0       # System time constant
dead_time = 0.5 # System delay / dead time
```

### Logging Setup

The logging system is configured using `logger_setup.py`, which sets up a JSON formatter. Logs are output directly to the console. To change logging verbosity, you can modify the `configure_logging` function in `logger_setup.py`:

```python
# logger_setup.py
logger.setLevel(logging.INFO) # Change to logging.DEBUG for more verbose output
```

## API Reference Documentation

### Core Classes (`pid.py`)

#### `PIDController`
A standard Proportional-Integral-Derivative (PID) controller implementation with anti-windup and output limits.

- **`__init__(self, kp, ki, kd, output_limits=(None, None))`**
  Initializes the controller with tuning parameters (`kp`, `ki`, `kd`) and an optional output limit tuple (`min_limit`, `max_limit`).
- **`reset(self)`**
  Clears the integral accumulator and past error values.
- **`update(self, setpoint, pv, dt)`**
  Calculates the new control variable output given a target `setpoint`, current process variable `pv`, and time step `dt`.

#### `FirstOrderSystem`
Models a First Order Plus Dead Time (FOPDT) system.

- **`__init__(self, K, tau, dead_time)`**
  Initializes the system with gain `K`, time constant `tau`, and `dead_time`.
- **`reset(self)`**
  Resets the internal state and output values.
- **`update(self, u, dt)`**
  Updates the system state given an input `u` over the time step `dt`. Returns the new system output.

### Core Functions (`pid.py`)

- **`ziegler_nichols_tuning(system, setpoint=1.0, dt=0.01, max_time=50.0)`**
  Performs Ziegler-Nichols tuning on a `FirstOrderSystem` to find optimal PID parameters. Returns `(kp, ki, kd)`.
- **`detect_oscillations(history_y, dt)`**
  Detects if the system output is oscillating at a constant amplitude. Returns `(is_oscillating, period, amplitude_ratio)`.
- **`simulate_and_save(system, controller, filename, setpoint=1.0, dt=0.01, duration=20.0)`**
  Simulates the system with the given controller and saves the `(time, setpoint, pv, cv)` data to a CSV file.

### Utilities (`logger_setup.py`)

- **`configure_logging()`**
  Sets up the root logger to output structured JSON logs to the console using `JSONFormatter`.
