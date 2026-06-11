import csv
import math
import logging
from collections import deque

logger = logging.getLogger(__name__)

def _check_type_and_value(val, name, expected_types=(int, float), min_val=None, max_val=None):
    """
    Validates the type and value of a parameter.
    
    Args:
        val: The value to check.
        name (str): The name of the parameter (used in error messages).
        expected_types (tuple, optional): Allowed types for the value. Defaults to (int, float).
        min_val (float, optional): The minimum allowed value (inclusive). Defaults to None.
        max_val (float, optional): The maximum allowed value (inclusive). Defaults to None.
        
    Raises:
        TypeError: If the value is not of the expected types.
        ValueError: If the value is outside the specified bounds.
    """
    if not isinstance(val, expected_types):
        err_msg = f"{name} must be of type {expected_types}, got {type(val).__name__}"
        logger.error(err_msg)
        raise TypeError(err_msg)
    if min_val is not None and val < min_val:
        err_msg = f"{name} must be >= {min_val}, got {val}"
        logger.error(err_msg)
        raise ValueError(err_msg)
    if max_val is not None and val > max_val:
        err_msg = f"{name} must be <= {max_val}, got {val}"
        logger.error(err_msg)
        raise ValueError(err_msg)

class PIDController:
    """
    A standard Proportional-Integral-Derivative (PID) controller implementation 
    with anti-windup and output limits.
    """
    
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        """
        Initializes the PID Controller.
        
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            output_limits (tuple): (min_limit, max_limit) for output bounding.
        """
        _check_type_and_value(kp, "kp", min_val=0.0)
        _check_type_and_value(ki, "ki", min_val=0.0)
        _check_type_and_value(kd, "kd", min_val=0.0)
        
        if not isinstance(output_limits, tuple) or len(output_limits) != 2:
            err_msg = "output_limits must be a tuple of length 2"
            logger.error(err_msg)
            raise ValueError(err_msg)
            
        min_limit, max_limit = output_limits
        if min_limit is not None and not isinstance(min_limit, (int, float)):
            err_msg = "output_limits[0] must be None, int, or float"
            logger.error(err_msg)
            raise TypeError(err_msg)
        if max_limit is not None and not isinstance(max_limit, (int, float)):
            err_msg = "output_limits[1] must be None, int, or float"
            logger.error(err_msg)
            raise TypeError(err_msg)
        if min_limit is not None and max_limit is not None and min_limit > max_limit:
            err_msg = f"min_limit ({min_limit}) cannot be greater than max_limit ({max_limit})"
            logger.error(err_msg)
            raise ValueError(err_msg)

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pv = 0.0
        logger.info("PIDController initialized", extra={"extra_info": {"kp": self.kp, "ki": self.ki, "kd": self.kd, "output_limits": self.output_limits}})

    def reset(self):
        """
        Resets the internal state of the PID controller.
        
        Clears the integral accumulator, previous error, and previous process variable.
        This is useful when restarting a simulation or control sequence.
        """
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pv = 0.0
        logger.debug("PIDController state reset")

    def update(self, setpoint, pv, dt):
        """
        Calculates the new control variable output.
        
        Args:
            setpoint (float): The desired target value.
            pv (float): The current process variable.
            dt (float): The time step since the last update.
            
        Returns:
            float: The calculated control output.
        """
        _check_type_and_value(setpoint, "setpoint")
        _check_type_and_value(pv, "pv")
        _check_type_and_value(dt, "dt", min_val=0.0)
        if dt <= 0:
            err_msg = f"dt must be > 0, got {dt}"
            logger.error(err_msg)
            raise ValueError(err_msg)
            
        error = setpoint - pv
        
        # Proportional
        p = self.kp * error
        
        # Integral
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative (on PV to avoid derivative kick)
        # Note: on first step prev_pv is 0 if not initialized properly, 
        # but reset or setting it outside can fix it.
        # Here we just use standard derivative on error for simplicity
        d = self.kd * (error - self.prev_error) / dt
        
        output = p + i + d
        
        # Anti-windup and limits
        min_limit, max_limit = self.output_limits
        if max_limit is not None and output > max_limit:
            output = max_limit
            self.integral -= error * dt # anti-windup
            logger.debug("Output hit max limit, applying anti-windup", extra={"extra_info": {"max_limit": max_limit, "output": output}})
        elif min_limit is not None and output < min_limit:
            output = min_limit
            self.integral -= error * dt # anti-windup
            logger.debug("Output hit min limit, applying anti-windup", extra={"extra_info": {"min_limit": min_limit, "output": output}})
            
        self.prev_error = error
        self.prev_pv = pv
        
        return output

class FirstOrderSystem:
    """
    Models a First Order Plus Dead Time (FOPDT) system.
    
    The system is represented by the differential equation:
    tau * dy/dt + y(t) = K * u(t - dead_time)
    """
    
    def __init__(self, K, tau, dead_time):
        """
        Initializes the First Order System.
        
        Args:
            K (float): The system gain.
            tau (float): The system time constant. Must be > 0.
            dead_time (float): The system delay/dead time. Must be >= 0.
            
        Raises:
            TypeError: If the parameters are not numeric.
            ValueError: If tau or dead_time are out of valid bounds.
        """
        _check_type_and_value(K, "K")
        _check_type_and_value(tau, "tau", min_val=1e-9) # tau > 0, slightly larger than 0 to avoid div by zero
        _check_type_and_value(dead_time, "dead_time", min_val=0.0)
        
        self.K = float(K)
        self.tau = float(tau)
        self.dead_time = float(dead_time)
        
        self.y = 0.0
        self.history_u = deque()
        self._delay_steps_cache = None
        self._dt_cache = None
        logger.info("FirstOrderSystem initialized", extra={"extra_info": {"K": self.K, "tau": self.tau, "dead_time": self.dead_time}})
        
    def reset(self):
        """
        Resets the system state to initial conditions.
        
        Clears the current output value and the history of inputs.
        """
        self.y = 0.0
        self.history_u.clear()
        logger.debug("FirstOrderSystem state reset")
        
    def update(self, u, dt):
        """
        Updates the system state given a new input and time step.
        
        Args:
            u (float): The control input applied to the system.
            dt (float): The time step since the last update.
            
        Returns:
            float: The new system output (process variable).
            
        Raises:
            TypeError: If u or dt are not numeric.
            ValueError: If dt <= 0.
        """
        _check_type_and_value(u, "u")
        _check_type_and_value(dt, "dt", min_val=0.0)
        if dt <= 0:
            err_msg = f"dt must be > 0, got {dt}"
            logger.error(err_msg)
            raise ValueError(err_msg)
            
        self.history_u.append(u)
        
        # Calculate delay_steps only if dt changes
        if dt != self._dt_cache:
            self._dt_cache = dt
            self._delay_steps_cache = int(self.dead_time / dt)
            
        delay_steps = self._delay_steps_cache
        
        # Keep deque bounded to avoid unbounded memory growth
        max_len = delay_steps + 1
        while len(self.history_u) > max_len:
            self.history_u.popleft()
            
        if len(self.history_u) >= max_len:
            delayed_u = self.history_u[0]
        else:
            delayed_u = 0.0
            
        # Euler integration for dy/dt = (K * u(t-L) - y) / tau
        dy = (self.K * delayed_u - self.y) / self.tau
        self.y += dy * dt
        return self.y

def detect_oscillations(history_y, dt):
    """
    Detects if the system output is oscillating at a constant amplitude.
    
    Args:
        history_y (list): The history of the system output.
        dt (float): The time step.
        
    Returns:
        tuple: (is_oscillating, period, amplitude_ratio)
    """
    if not isinstance(history_y, list):
        err_msg = f"history_y must be a list, got {type(history_y).__name__}"
        logger.error(err_msg)
        raise TypeError(err_msg)
    _check_type_and_value(dt, "dt", min_val=0.0)
    if dt <= 0:
        err_msg = f"dt must be > 0, got {dt}"
        logger.error(err_msg)
        raise ValueError(err_msg)
        
    # Needs some buffer to avoid initial transient
    n = len(history_y)
    if n < 100:
        return False, 0.0, 0.0
        
    # Find peaks and troughs
    peaks = []
    troughs = []
    
    # Check last N points directly without slicing to avoid copying
    start_idx = max(0, n - 1000)
    
    prev_val = history_y[start_idx]
    curr_val = history_y[start_idx + 1]
    
    for i in range(start_idx + 1, n - 1):
        next_val = history_y[i + 1]
        
        if prev_val < curr_val > next_val:
            peaks.append((i - start_idx, curr_val))
        elif prev_val > curr_val < next_val:
            troughs.append((i - start_idx, curr_val))
            
        prev_val = curr_val
        curr_val = next_val
            
    if len(peaks) >= 3 and len(troughs) >= 3:
        # Check if amplitudes are relatively constant
        amplitudes = [peaks[j][1] - troughs[j][1] for j in range(min(len(peaks), len(troughs)))]
        
        if max(amplitudes) < 1e-4:
            return False, 0.0, 0.0
            
        # Check if amplitude is growing or shrinking
        amp_ratio = amplitudes[-1] / amplitudes[-2]
        
        # If amplitude ratio is close to 1, we have constant oscillations
        if 0.95 <= amp_ratio <= 1.05:
            period = (peaks[-1][0] - peaks[-2][0]) * dt
            return True, period, amp_ratio
            
        return False, 0.0, amp_ratio
        
    return False, 0.0, 0.0

def ziegler_nichols_tuning(system, setpoint=1.0, dt=0.01, max_time=50.0):
    """
    Performs Ziegler-Nichols tuning on a given system to find optimal PID parameters.
    
    Args:
        system (FirstOrderSystem): The system to tune.
        setpoint (float): The target setpoint for the tuning process.
        dt (float): The time step for the simulation.
        max_time (float): The maximum time to run each simulation step.
        
    Returns:
        tuple: (kp, ki, kd) The tuned PID parameters.
    """
    if not isinstance(system, FirstOrderSystem):
        err_msg = f"system must be a FirstOrderSystem, got {type(system).__name__}"
        logger.error(err_msg)
        raise TypeError(err_msg)
    _check_type_and_value(setpoint, "setpoint")
    _check_type_and_value(dt, "dt", min_val=0.0)
    if dt <= 0:
        err_msg = f"dt must be > 0, got {dt}"
        logger.error(err_msg)
        raise ValueError(err_msg)
    _check_type_and_value(max_time, "max_time", min_val=0.0)
    if max_time <= 0:
        err_msg = f"max_time must be > 0, got {max_time}"
        logger.error(err_msg)
        raise ValueError(err_msg)
        
    logger.info("Starting Ziegler-Nichols tuning...")
    
    ku = 0.0
    tu = 0.0
    
    # Try increasing Kp
    kp_test = 0.1
    kp_step = 0.1
    max_kp = 100.0
    
    # Use binary search-like approach or gradual increase
    # To find marginal stability
    
    low_kp = 0.0
    high_kp = None
    
    max_steps = int(max_time / dt)
    half_steps = max_steps // 2
    
    for iteration in range(20):
        logger.debug(f"Step {iteration+1}: Testing Kp = {kp_test:.3f}", extra={"extra_info": {"iteration": iteration+1, "kp_test": kp_test}})
        
        controller = PIDController(kp_test, 0, 0)
        system.reset()
        
        history_y = []
        is_oscillating = False
        period = 0.0
        diverged = False
        
        # Run simulation
        for t_step in range(max_steps):
            y = system.y
            u = controller.update(setpoint, y, dt)
            y = system.update(u, dt)
            history_y.append(y)
            
            # Early termination if diverging
            if abs(y) > 100:
                diverged = True
                break
            
            # Start checking for oscillations after some time
            if t_step > half_steps and t_step % 100 == 0:
                is_oscillating, period, amp_ratio = detect_oscillations(history_y, dt)
                if is_oscillating:
                    break
        
        _, _, final_amp_ratio = detect_oscillations(history_y, dt)
        
        if is_oscillating:
            logger.info(f"Stable oscillations detected", extra={"extra_info": {"period_Tu": period}})
            ku = kp_test
            tu = period
            break
        elif final_amp_ratio > 1.05 or diverged:
            logger.debug("Unstable (growing oscillations or diverging). Decreasing Kp.")
            high_kp = kp_test
            kp_test = (low_kp + high_kp) / 2
        else:
            logger.debug("Stable (decaying oscillations or no oscillations). Increasing Kp.")
            low_kp = kp_test
            if high_kp is None:
                kp_test *= 2.0
            else:
                kp_test = (low_kp + high_kp) / 2
                
    if ku == 0.0:
        logger.warning("Failed to find ultimate gain.")
        return 0.0, 0.0, 0.0
        
    logger.info("Found Ultimate Gain and Period", extra={"extra_info": {"Ku": ku, "Tu": tu}})
    
    # Classic Z-N PID rules
    kp = 0.6 * ku
    ki = 1.2 * ku / tu
    kd = 0.075 * ku * tu
    
    logger.info("Ziegler-Nichols PID parameters calculated", extra={"extra_info": {"Kp": kp, "Ki": ki, "Kd": kd}})
    
    return kp, ki, kd

def simulate_and_save(system, controller, filename, setpoint=1.0, dt=0.01, duration=20.0):
    """
    Simulates the system with the given controller and saves results to a CSV file.
    
    Args:
        system (FirstOrderSystem): The system to simulate.
        controller (PIDController): The controller to manage the system.
        filename (str): The path to the CSV file where results will be saved.
        setpoint (float, optional): The target value for the controller. Defaults to 1.0.
        dt (float, optional): The simulation time step. Defaults to 0.01.
        duration (float, optional): The total duration of the simulation. Defaults to 20.0.
        
    Returns:
        list: A list of tuples containing (time, setpoint, process_variable, control_variable).
        
    Raises:
        TypeError: If invalid types are provided for arguments.
        ValueError: If numeric arguments are out of bounds or filename is empty.
        IOError: If saving to the file fails.
    """
    if not isinstance(system, FirstOrderSystem):
        err_msg = f"system must be a FirstOrderSystem, got {type(system).__name__}"
        logger.error(err_msg)
        raise TypeError(err_msg)
    if not isinstance(controller, PIDController):
        err_msg = f"controller must be a PIDController, got {type(controller).__name__}"
        logger.error(err_msg)
        raise TypeError(err_msg)
    if not isinstance(filename, str) or not filename:
        err_msg = "filename must be a non-empty string"
        logger.error(err_msg)
        raise ValueError(err_msg)
        
    _check_type_and_value(setpoint, "setpoint")
    _check_type_and_value(dt, "dt", min_val=0.0)
    if dt <= 0:
        err_msg = f"dt must be > 0, got {dt}"
        logger.error(err_msg)
        raise ValueError(err_msg)
    _check_type_and_value(duration, "duration", min_val=0.0)
    if duration <= 0:
        err_msg = f"duration must be > 0, got {duration}"
        logger.error(err_msg)
        raise ValueError(err_msg)
        
    system.reset()
    controller.reset()
    
    results = []
    
    try:
        num_steps = int(duration / dt)
    except Exception as e:
        err_msg = f"Invalid duration or dt: {e}"
        logger.error(err_msg, exc_info=True)
        raise ValueError(err_msg)
        
    for i in range(num_steps):
        t = i * dt
        pv = system.y
        cv = controller.update(setpoint, pv, dt)
        system.update(cv, dt)
        
        results.append((t, setpoint, pv, cv))
        
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'setpoint', 'process_variable', 'control_variable'])
            writer.writerows(results)
    except IOError as e:
        err_msg = f"Failed to write to file {filename}: {e}"
        logger.error(err_msg, exc_info=True)
        raise IOError(err_msg)
        
    logger.info(f"Simulation saved to {filename}")
    return results
