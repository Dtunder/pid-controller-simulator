"""Master test runner for the PID simulation environment."""

import unittest
import sys

def run_tests() -> None:
    """Discovers and runs all tests, and prints a final test summary."""
    # Discover all tests
    test_loader = unittest.TestLoader()
    test_suite = test_loader.discover(start_dir='.', pattern='test_*.py')

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)

    # Check if all tests passed
    if result.wasSuccessful():
        print("\n=== TEST SUMMARY ===")
        print(f"Total Tests Run: {result.testsRun}")
        print("Status: SUCCESS")
        sys.exit(0)
    else:
        print("\n=== TEST SUMMARY ===")
        print(f"Total Tests Run: {result.testsRun}")
        print(f"Failures: {len(result.failures)}")
        print(f"Errors: {len(result.errors)}")
        print("Status: FAILED")
        sys.exit(1)

if __name__ == '__main__':
    run_tests()
