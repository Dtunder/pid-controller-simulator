import logging
import time
from typing import Callable, Any, Type, Tuple, Optional
from functools import wraps

logger = logging.getLogger(__name__)

class ResilienceError(Exception):
    """Base class for resilience exceptions."""
    pass

class ConnectionFailure(ResilienceError):
    """Raised when a connection to a service or sensor fails."""
    pass

class TimeoutFailure(ResilienceError):
    """Raised when an operation times out."""
    pass

class ConfigurationError(ResilienceError):
    """Raised when there is a bad configuration."""
    pass

def retry_with_fallback(
    max_retries: int = 3,
    delay: float = 1.0,
    exceptions: Tuple[Type[Exception], ...] = (ConnectionFailure, TimeoutFailure),
    fallback: Optional[Callable[..., Any]] = None
) -> Callable[..., Any]:
    """
    A decorator that retries a function if it raises specific exceptions.
    If it fails after max_retries, it calls the fallback function if provided,
    otherwise re-raises the last exception.
    
    Args:
        max_retries (int): Maximum number of retries before giving up.
        delay (float): Delay in seconds between retries.
        exceptions (Tuple[Type[Exception], ...]): Tuple of exception types to catch.
        fallback (Optional[Callable[..., Any]]): Function to call if all retries fail.
            Must accept the same arguments as the decorated function.
            
    Returns:
        Callable: The wrapped function.
    """
    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            last_exception = None
            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    logger.warning(
                        f"Attempt {attempt + 1}/{max_retries + 1} failed for {func.__name__} "
                        f"with {type(e).__name__}: {e}"
                    )
                    if attempt < max_retries:
                        time.sleep(delay)
            
            logger.error(f"All {max_retries + 1} attempts failed for {func.__name__}")
            if fallback is not None:
                logger.info(f"Executing fallback for {func.__name__}")
                return fallback(*args, **kwargs)
            
            if last_exception is not None:
                raise last_exception
                
        return wrapper
    return decorator
