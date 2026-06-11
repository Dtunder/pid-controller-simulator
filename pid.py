import csv
import math

def _check_type_and_value(val, name, expected_types=(int, float), min_val=None, max_val=None):
    if not isinstance(val, expected_types):
        raise TypeError(f"{name} must be of type {expected_types}, got {type(val).__name__}")
    if min_val is not None and val < min_val:
        raise ValueError(f"{name} must be >= {min_val}, got {val}")
    if max_val is not None and val > max_val:
        raise ValueError(f"{name} must be <= {max_val}, got {val}")

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        _check_type_and_value(kp, "kp", min_val=0.0)
        _check_type_and_value(ki, "ki", min_val=0.0)
        _check_type_and_value(kd, "kd", min_val=0.0)
        
        if not isinstance(output_limits, tuple) or len(output_limits) != 2:
            raise ValueError("output_limits must be a tuple of length 2")
            
        min_limit, max_limit = output_limits
        if min_limit is not None and not isinstance(min_limit, (int, float)):
            raise TypeError("output_limits[0] must be None, int, or float")
        if max_limit is not None and not isinstance(max_limit, (int, float)):
            raise TypeError("output_limits[1] must be None, int, or float")
        if min_limit is not None and max_limit is not None and min_limit > max_limit:
            raise ValueError(f"min_limit ({min_limit}) cannot be greater than max_limit ({max_limit})")

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pv = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pv = 0.0

    def update(self, setpoint, pv, dt):
        _check_type_and_value(setpoint, "setpoint")
        _check_type_and_value(pv, "pv")
        _check_type_and_value(dt, "dt", min_val=0.0)
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
            
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
        elif min_limit is not None and output < min_limit:
            output = min_limit
            self.integral -= error * dt # anti-windup
            
        self.prev_error = error
        self.prev_pv = pv
        
        return output

class FirstOrderSystem:
    def __init__(self, K, tau, dead_time):
        _check_type_and_value(K, "K")
        _check_type_and_value(tau, "tau", min_val=1e-9) # tau > 0, slightly larger than 0 to avoid div by zero
        _check_type_and_value(dead_time, "dead_time", min_val=0.0)
        
        self.K = float(K)
        self.tau = float(tau)
        self.dead_time = float(dead_time)
        
        self.y = 0.0
        self.history_u = []
        
    def reset(self):
        self.y = 0.0
        self.history_u = []
        
    def update(self, u, dt):
        _check_type_and_value(u, "u")
        _check_type_and_value(dt, "dt", min_val=0.0)
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
            
        self.history_u.append(u)
        delay_steps = int(self.dead_time / dt)
        
        if len(self.history_u) > delay_steps:
            delayed_u = self.history_u[-(delay_steps + 1)]
        else:
            delayed_u = 0.0
            
        # Euler integration for dy/dt = (K * u(t-L) - y) / tau
        dy = (self.K * delayed_u - self.y) / self.tau
        self.y += dy * dt
        return self.y

def detect_oscillations(history_y, dt):
    if not isinstance(history_y, list):
        raise TypeError(f"history_y must be a list, got {type(history_y).__name__}")
    _check_type_and_value(dt, "dt", min_val=0.0)
    if dt <= 0:
        raise ValueError(f"dt must be > 0, got {dt}")
        
    # Find peaks and troughs
    peaks = []
    troughs = []
    
    # Needs some buffer to avoid initial transient
    if len(history_y) < 100:
        return False, 0.0, 0.0
        
    # Check last N points
    window = history_y[-min(1000, len(history_y)):]
    
    for i in range(1, len(window) - 1):
        if window[i-1] < window[i] > window[i+1]:
            peaks.append((i, window[i]))
        if window[i-1] > window[i] < window[i+1]:
            troughs.append((i, window[i]))
            
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
    if not isinstance(system, FirstOrderSystem):
        raise TypeError(f"system must be a FirstOrderSystem, got {type(system).__name__}")
    _check_type_and_value(setpoint, "setpoint")
    _check_type_and_value(dt, "dt", min_val=0.0)
    if dt <= 0:
        raise ValueError(f"dt must be > 0, got {dt}")
    _check_type_and_value(max_time, "max_time", min_val=0.0)
    if max_time <= 0:
        raise ValueError(f"max_time must be > 0, got {max_time}")
        
    print("Starting Ziegler-Nichols tuning...")
    
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
    
    for iteration in range(20):
        print(f"Step {iteration+1}: Testing Kp = {kp_test:.3f}")
        
        controller = PIDController(kp_test, 0, 0)
        system.reset()
        
        history_y = []
        is_oscillating = False
        period = 0.0
        
        # Run simulation
        for t_step in range(int(max_time / dt)):
            y = system.y
            u = controller.update(setpoint, y, dt)
            y = system.update(u, dt)
            history_y.append(y)
            
            # Start checking for oscillations after some time
            if t_step > int(max_time / dt) / 2 and t_step % 100 == 0:
                is_oscillating, period, amp_ratio = detect_oscillations(history_y, dt)
                if is_oscillating:
                    break
        
        _, _, final_amp_ratio = detect_oscillations(history_y, dt)
        
        if is_oscillating:
            print(f"  -> Stable oscillations detected! Period Tu = {period:.3f}s")
            ku = kp_test
            tu = period
            break
        elif final_amp_ratio > 1.05 or any(abs(val) > 100 for val in history_y):
            print("  -> Unstable (growing oscillations or diverging). Decreasing Kp.")
            high_kp = kp_test
            kp_test = (low_kp + high_kp) / 2
        else:
            print("  -> Stable (decaying oscillations or no oscillations). Increasing Kp.")
            low_kp = kp_test
            if high_kp is None:
                kp_test *= 2.0
            else:
                kp_test = (low_kp + high_kp) / 2
                
    if ku == 0.0:
        print("Failed to find ultimate gain.")
        return 0.0, 0.0, 0.0
        
    print(f"Found Ultimate Gain (Ku) = {ku:.3f}")
    print(f"Found Ultimate Period (Tu) = {tu:.3f}")
    
    # Classic Z-N PID rules
    kp = 0.6 * ku
    ki = 1.2 * ku / tu
    kd = 0.075 * ku * tu
    
    print(f"Ziegler-Nichols PID parameters:")
    print(f"Kp = {kp:.3f}")
    print(f"Ki = {ki:.3f}")
    print(f"Kd = {kd:.3f}")
    
    return kp, ki, kd

def simulate_and_save(system, controller, filename, setpoint=1.0, dt=0.01, duration=20.0):
    if not isinstance(system, FirstOrderSystem):
        raise TypeError(f"system must be a FirstOrderSystem, got {type(system).__name__}")
    if not isinstance(controller, PIDController):
        raise TypeError(f"controller must be a PIDController, got {type(controller).__name__}")
    if not isinstance(filename, str) or not filename:
        raise ValueError("filename must be a non-empty string")
        
    _check_type_and_value(setpoint, "setpoint")
    _check_type_and_value(dt, "dt", min_val=0.0)
    if dt <= 0:
        raise ValueError(f"dt must be > 0, got {dt}")
    _check_type_and_value(duration, "duration", min_val=0.0)
    if duration <= 0:
        raise ValueError(f"duration must be > 0, got {duration}")
        
    system.reset()
    controller.reset()
    
    results = []
    
    try:
        num_steps = int(duration / dt)
    except Exception as e:
        raise ValueError(f"Invalid duration or dt: {e}")
        
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
        raise IOError(f"Failed to write to file {filename}: {e}")
        
    print(f"Simulation saved to {filename}")
    return results
