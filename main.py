import logging
from logger_setup import configure_logging
from pid import PIDController, FirstOrderSystem, ziegler_nichols_tuning, simulate_and_save
from config import load_config

logger = logging.getLogger(__name__)


def main() -> None:
    """
    Main entry point for the PID simulation.
    
    This function:
    1. Loads configuration.
    2. Configures logging.
    3. Defines a First Order Plus Dead Time (FOPDT) system based on config.
    4. Uses Ziegler-Nichols tuning to find optimal PID parameters for the system.
    5. Runs a final simulation with the tuned controller.
    6. Saves the results to the configured output file.
    """
    config = load_config()
    
    configure_logging(level=config["logging"]["level"])
    logger.info("Welcome to pid-controller-simulator!")
    
    # Define system parameters
    system_cfg = config["system"]
    K = system_cfg["K"]
    tau = system_cfg["tau"]
    dead_time = system_cfg["dead_time"]
    
    logger.info(
        "System defined as First Order Plus Dead Time (FOPDT)",
        extra={"extra_info": {"K": K, "tau": tau, "dead_time": dead_time}}
    )
    
    system = FirstOrderSystem(K, tau, dead_time)
    
    # Tune PID
    logger.info("Starting Tuning Phase")
    tuning_cfg = config["tuning"]
    kp, ki, kd = ziegler_nichols_tuning(
        system, 
        setpoint=tuning_cfg["setpoint"], 
        dt=tuning_cfg["dt"], 
        max_time=tuning_cfg["max_time"]
    )
    
    if kp == 0.0 and ki == 0.0 and kd == 0.0:
        logger.error("Tuning failed, exiting.")
        return
        
    # Setup PID with tuned parameters
    logger.info("Starting Simulation Phase")
    controller = PIDController(kp, ki, kd)
    
    # Run final simulation
    sim_cfg = config["simulation"]
    simulate_and_save(
        system, 
        controller, 
        sim_cfg["output_file"], 
        setpoint=sim_cfg["setpoint"], 
        dt=sim_cfg["dt"], 
        duration=sim_cfg["duration"]
    )

if __name__ == "__main__":
    main()
