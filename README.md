# pid-controller-simulator

PID simulation environment with Ziegler-Nichols parameter tuning.

This simulator models a First Order Plus Dead Time (FOPDT) system and runs an automated Ziegler-Nichols tuning process to find optimal Proportional, Integral, and Derivative (PID) parameters.

## Usage

1. Run the simulator and tuning process:
```bash
python main.py
```
This will run the Ziegler-Nichols tuning algorithm and then simulate a step response with the tuned parameters. The results of the simulation are saved to `results.csv`.

2. Run tests:
```bash
python -m unittest test_pid.py
```
