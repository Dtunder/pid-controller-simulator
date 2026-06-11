import logging
from logger_setup import configure_logging
from pid import PIDController, FirstOrderSystem, ziegler_nichols_tuning, simulate_and_save

logger = logging.getLogger(__name__)


def main() -> None:
    """
    Main entry point for the PID simulation.
    
    This function:
    1. Configures logging.
    2. Defines a First Order Plus Dead Time (FOPDT) system.
    3. Uses Ziegler-Nichols tuning to find optimal PID parameters for the system.
    4. Runs a final simulation with the tuned controller.
    5. Saves the results to 'results.csv'.
    """
    configure_logging()
    logger.info("Welcome to pid-controller-simulator!")
    
    # Define system parameters
    K = 1.0
    tau = 1.0
    dead_time = 0.5
    
    logger.info(
        "System defined as First Order Plus Dead Time (FOPDT)",
        extra={"extra_info": {"K": K, "tau": tau, "dead_time": dead_time}}
    )
    
    system = FirstOrderSystem(K, tau, dead_time)
    
    # Tune PID
    logger.info("Starting Tuning Phase")
    kp, ki, kd = ziegler_nichols_tuning(system, setpoint=1.0, dt=0.01, max_time=50.0)
    
    if kp == 0.0 and ki == 0.0 and kd == 0.0:
        logger.error("Tuning failed, exiting.")
        return
        
    # Setup PID with tuned parameters
    logger.info("Starting Simulation Phase")
    controller = PIDController(kp, ki, kd)
    
    # Run final simulation
    simulate_and_save(system, controller, 'results.csv', setpoint=1.0, dt=0.01, duration=30.0)

if __name__ == "__main__":
    main()
