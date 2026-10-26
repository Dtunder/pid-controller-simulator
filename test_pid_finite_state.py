"""Regression tests for PID arithmetic overflow and atomic state updates."""
import math
import unittest

from pid import PIDController


class TestPIDFiniteState(unittest.TestCase):
    def assert_rejected_atomically(self, controller, setpoint, pv, dt):
        before = (
            controller.integral,
            controller.prev_error,
            controller.prev_pv,
            controller._has_previous_measurement,
        )
        with self.assertRaises(ValueError):
            controller.update(setpoint, pv, dt)
        self.assertEqual(
            (
                controller.integral,
                controller.prev_error,
                controller.prev_pv,
                controller._has_previous_measurement,
            ),
            before,
        )
        self.assertTrue(math.isfinite(controller.update(0.0, 0.0, 1.0)))

    def test_error_subtraction_overflow_does_not_poison_state(self):
        c = PIDController(1.0, 0.0, 0.0)
        self.assert_rejected_atomically(c, 1e308, -1e308, 1.0)

    def test_integral_delta_overflow_does_not_poison_state(self):
        c = PIDController(0.0, 1.0, 0.0)
        self.assert_rejected_atomically(c, 1e308, 0.0, 1e308)

    def test_proportional_product_overflow_does_not_poison_state(self):
        c = PIDController(1e308, 0.0, 0.0)
        self.assert_rejected_atomically(c, 1e308, 0.0, 1.0)

    def test_derivative_difference_overflow_preserves_history(self):
        c = PIDController(0.0, 0.0, 1.0)
        c.update(1e308, 1e308, 1.0)
        self.assert_rejected_atomically(c, -1e308, -1e308, 1.0)

    def test_finite_terms_with_overflowing_sum_are_rejected(self):
        c = PIDController(1e308, 1e308, 0.0)
        self.assert_rejected_atomically(c, 1.0, 0.0, 1.0)

    def test_output_limits_do_not_hide_nonfinite_intermediate_values(self):
        c = PIDController(1e308, 0.0, 0.0, (-10.0, 10.0))
        self.assert_rejected_atomically(c, 1e308, 0.0, 1.0)


if __name__ == "__main__":
    unittest.main()
