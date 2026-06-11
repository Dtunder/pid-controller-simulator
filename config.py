import os
import json
import logging
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_CONFIG = {
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

def _deep_merge(dict1: Dict[str, Any], dict2: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge dict2 into dict1."""
    result = dict1.copy()
    for key, value in dict2.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result

def load_config(config_file: str = "config.json") -> Dict[str, Any]:
    """
    Load configuration from a JSON file and environment variables,
    falling back to default values.
    """
    config = DEFAULT_CONFIG.copy()
    
    # Try to load from file
    if os.path.exists(config_file):
        try:
            with open(config_file, "r") as f:
                file_config = json.load(f)
                config = _deep_merge(config, file_config)
        except json.JSONDecodeError as e:
            logger.error(f"Error parsing config file {config_file}: {e}")
        except IOError as e:
            logger.error(f"Error reading config file {config_file}: {e}")

    # Override with environment variables
    # Expected format: APP_SECTION_KEY (e.g., APP_SYSTEM_K)
    env_overrides = {}
    for section, values in config.items():
        if isinstance(values, dict):
            env_overrides[section] = {}
            for key, default_val in values.items():
                env_var_name = f"APP_{section.upper()}_{key.upper()}"
                if env_var_name in os.environ:
                    env_val = os.environ[env_var_name]
                    # Attempt to cast the environment variable to the same type as default
                    try:
                        if isinstance(default_val, bool):
                            env_overrides[section][key] = env_val.lower() in ('true', '1', 't', 'y', 'yes')
                        elif isinstance(default_val, int):
                            env_overrides[section][key] = int(env_val)
                        elif isinstance(default_val, float):
                            env_overrides[section][key] = float(env_val)
                        else:
                            env_overrides[section][key] = env_val
                    except ValueError:
                        logger.warning(f"Could not convert env var {env_var_name} value '{env_val}' to {type(default_val).__name__}")
        else:
            env_var_name = f"APP_{section.upper()}"
            if env_var_name in os.environ:
                env_val = os.environ[env_var_name]
                try:
                    if isinstance(values, bool):
                        env_overrides[section] = env_val.lower() in ('true', '1', 't', 'y', 'yes')
                    elif isinstance(values, int):
                        env_overrides[section] = int(env_val)
                    elif isinstance(values, float):
                        env_overrides[section] = float(env_val)
                    else:
                        env_overrides[section] = env_val
                except ValueError:
                    logger.warning(f"Could not convert env var {env_var_name} value '{env_val}' to {type(values).__name__}")

    config = _deep_merge(config, env_overrides)
    return config

