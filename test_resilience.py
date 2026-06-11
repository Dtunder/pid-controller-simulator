import unittest
import logging
from resilience import (
    retry_with_fallback,
    ConnectionFailure,
    TimeoutFailure,
    ConfigurationError
)

# Disable logging output for tests
logging.disable(logging.CRITICAL)

class TestResilienceLayer(unittest.TestCase):
    def setUp(self):
        self.call_count = 0

    def test_successful_execution(self):
        """Test that a successful function is not retried."""
        @retry_with_fallback(max_retries=3, delay=0.01)
        def success_func():
            self.call_count += 1
            return "success"
            
        result = success_func()
        self.assertEqual(result, "success")
        self.assertEqual(self.call_count, 1)

    def test_retry_eventual_success(self):
        """Test that a function retries and eventually succeeds."""
        @retry_with_fallback(max_retries=3, delay=0.01)
        def eventual_success():
            self.call_count += 1
            if self.call_count < 3:
                raise ConnectionFailure("Temporary failure")
            return "success"
            
        result = eventual_success()
        self.assertEqual(result, "success")
        self.assertEqual(self.call_count, 3)

    def test_retry_failure_no_fallback(self):
        """Test that exception is raised if no fallback and all retries fail."""
        @retry_with_fallback(max_retries=2, delay=0.01)
        def persistent_failure():
            self.call_count += 1
            raise TimeoutFailure("Persistent timeout")
            
        with self.assertRaises(TimeoutFailure):
            persistent_failure()
        self.assertEqual(self.call_count, 3) # initial + 2 retries

    def test_retry_with_fallback(self):
        """Test that fallback is executed after all retries fail."""
        def fallback_func():
            return "fallback_result"
            
        @retry_with_fallback(max_retries=2, delay=0.01, fallback=fallback_func)
        def persistent_failure():
            self.call_count += 1
            raise ConnectionFailure("Persistent connection failure")
            
        result = persistent_failure()
        self.assertEqual(result, "fallback_result")
        self.assertEqual(self.call_count, 3)

    def test_exception_not_in_tuple(self):
        """Test that exceptions not in the tuple are not caught and retried."""
        @retry_with_fallback(max_retries=3, delay=0.01, exceptions=(ConnectionFailure,))
        def config_failure():
            self.call_count += 1
            raise ConfigurationError("Bad config")
            
        with self.assertRaises(ConfigurationError):
            config_failure()
        self.assertEqual(self.call_count, 1) # Should fail immediately

    def test_fallback_args_passing(self):
        """Test that arguments are correctly passed to the fallback."""
        def fallback_func(a, b, kwarg1=None):
            return f"fallback: {a}, {b}, {kwarg1}"
            
        @retry_with_fallback(max_retries=1, delay=0.01, fallback=fallback_func)
        def failing_func(a, b, kwarg1=None):
            raise TimeoutFailure("Timeout")
            
        result = failing_func(1, 2, kwarg1="test")
        self.assertEqual(result, "fallback: 1, 2, test")

if __name__ == '__main__':
    unittest.main()
