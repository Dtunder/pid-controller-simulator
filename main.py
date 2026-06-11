from pid import PIDController, FirstOrderSystem, ziegler_nichols_tuning, simulate_and_save

def main():
    print("Welcome to pid-controller-simulator!")
    
    # Define system parameters
    K = 1.0
    tau = 1.0
    dead_time = 0.5
    
    print(f"\nSystem defined as First Order Plus Dead Time (FOPDT):")
    print(f"K={K}, tau={tau}, dead_time={dead_time}")
    
    system = FirstOrderSystem(K, tau, dead_time)
    
    # Tune PID
    print("\n--- Tuning Phase ---")
    kp, ki, kd = ziegler_nichols_tuning(system, setpoint=1.0, dt=0.01, max_time=50.0)
    
    if kp == 0.0 and ki == 0.0 and kd == 0.0:
        print("Tuning failed, exiting.")
        return
        
    # Setup PID with tuned parameters
    print("\n--- Simulation Phase ---")
    controller = PIDController(kp, ki, kd)
    
    # Run final simulation
    simulate_and_save(system, controller, 'results.csv', setpoint=1.0, dt=0.01, duration=30.0)

if __name__ == "__main__":
    main()
