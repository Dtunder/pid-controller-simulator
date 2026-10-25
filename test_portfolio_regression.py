"""Independent regression tests for derivative kick and non-finite inputs."""
import unittest
from pid import PIDController, FirstOrderSystem


class TestPortfolioRegression(unittest.TestCase):
    def test_setpoint_step_does_not_create_derivative_kick(self):
        c = PIDController(0.0, 0.0, 2.0)
        self.assertEqual(c.update(0.0, 0.0, 0.1), 0.0)
        self.assertEqual(c.update(10.0, 0.0, 0.1), 0.0)

    def test_derivative_uses_measurement_not_setpoint(self):
        c = PIDController(0.0, 0.0, 2.0)
        self.assertEqual(c.update(10.0, 5.0, 0.5), 0.0)
        self.assertAlmostEqual(c.update(20.0, 6.0, 0.5), -4.0)

    def test_reset_discards_derivative_history(self):
        c = PIDController(0.0, 0.0, 1.0)
        c.update(0.0, 3.0, 0.1)
        self.assertAlmostEqual(c.update(0.0, 4.0, 0.1), -10.0)
        c.reset()
        self.assertEqual(c.update(0.0, 100.0, 0.1), 0.0)

    def test_nan_measurement_rejected_without_state_corruption(self):
        c = PIDController(1.0, 1.0, 1.0)
        c.update(1.0, 0.0, 0.1)
        before = (c.integral, c.prev_error, c.prev_pv)
        with self.assertRaises(ValueError):
            c.update(1.0, float("nan"), 0.1)
        self.assertEqual((c.integral, c.prev_error, c.prev_pv), before)

    def test_infinite_gain_rejected(self):
        with self.assertRaises(ValueError):
            PIDController(float("inf"), 0.0, 0.0)

    def test_nonfinite_output_limit_rejected(self):
        with self.assertRaises(ValueError):
            PIDController(1.0, 0.0, 0.0, output_limits=(float("nan"), 10.0))

    def test_first_order_rejects_nonfinite_input_without_state_change(self):
        s = FirstOrderSystem(1.0, 1.0, 0.0)
        before = (s.y, len(s.history_u))
        with self.assertRaises(ValueError):
            s.update(float("inf"), 0.1)
        self.assertEqual((s.y, len(s.history_u)), before)


if __name__ == "__main__":
    unittest.main()
