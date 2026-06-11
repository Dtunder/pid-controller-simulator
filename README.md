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

The simulation parameters are configured using a `config.json` file located in the root directory, or via environment variables.

### Using `config.json`
A default `config.json` looks like this:

```json
{
  "system": {
    "K": 1.0,
    "tau": 1.0,
    "dead_time": 0.5
  },
  "tuning": {
    "setpoint": 1.0,
    "dt": 0.01,
    "max_time": 50.0
  },
  "simulation": {
    "setpoint": 1.0,
    "dt": 0.01,
    "duration": 30.0,
    "output_file": "results.csv"
  },
  "logging": {
    "level": "INFO"
  }
}
```

### Using Environment Variables
You can override any configuration value using environment variables with the prefix `APP_` followed by the section and key in uppercase.
For example:
- To change the system gain `K` to 2.0: `export APP_SYSTEM_K=2.0`
- To change the simulation duration to 15.0: `export APP_SIMULATION_DURATION=15.0`
- To change the logging level to DEBUG: `export APP_LOGGING_LEVEL=DEBUG`

### Logging Setup

The logging system is configured to output structured JSON logs to the console. The logging level is controlled by the `logging.level` setting in your configuration (e.g., in `config.json` or via `APP_LOGGING_LEVEL`).

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
