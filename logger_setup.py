import logging
import json
from datetime import datetime
from typing import Any, Dict


class JSONFormatter(logging.Formatter):
    """
    Custom JSON formatter for structured logging.
    
    This formatter overrides the standard logging format to output log records
    as JSON strings, which is useful for structured log collection systems.
    """
    def format(self, record: logging.LogRecord) -> str:
        """
        Formats the specified log record as a JSON string.
        
        Args:
            record (logging.LogRecord): The log record to format.
            
        Returns:
            str: The formatted JSON string representing the log record.
        """
        log_record: Dict[str, Any] = {
            "timestamp": datetime.fromtimestamp(record.created).isoformat(),
            "level": record.levelname,
            "name": record.name,
            "message": record.getMessage(),
        }
        
        # Add extra fields if they exist
        if hasattr(record, "extra_info"):
            log_record.update(record.extra_info)
            
        if record.exc_info:
            log_record["exception"] = self.formatException(record.exc_info)
            
        return json.dumps(log_record)

def configure_logging() -> None:
    """
    Configures the root logger to use the JSON formatter and output to the console.
    
    This function sets the logging level to INFO, clears any existing handlers
    on the root logger, and adds a StreamHandler configured with the JSONFormatter.
    """
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    
    # Remove existing handlers if any
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
        
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(JSONFormatter())
    logger.addHandler(console_handler)
