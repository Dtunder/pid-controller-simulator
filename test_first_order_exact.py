import math
import unittest

from pid import FirstOrderSystem


class TestExactFirstOrderIntegration(unittest.TestCase):
    def test_large_step_matches_analytic_response(self):
        system = FirstOrderSystem(K=1.0, tau=1.0, dead_time=0.0)
        first = system.update(1.0, 3.0)
        second = system.update(1.0, 3.0)
        self.assertAlmostEqual(first, 1.0 - math.exp(-3.0), places=12)
        self.assertAlmostEqual(second, 1.0 - math.exp(-6.0), places=12)
        self.assertGreaterEqual(first, 0.0)
        self.assertLessEqual(second, 1.0)

    def test_variable_steps_match_analytic_response(self):
        system = FirstOrderSystem(K=2.0, tau=1.5, dead_time=0.0)
        elapsed = 0.0
        for dt in (0.05, 0.3, 1.2, 3.0, 0.4):
            elapsed += dt
            actual = system.update(1.0, dt)
            self.assertAlmostEqual(
                actual, 2.0 * (1.0 - math.exp(-elapsed / 1.5)), places=12
            )

    def test_very_large_finite_dt_remains_bounded(self):
        system = FirstOrderSystem(K=-2.0, tau=1.0, dead_time=0.0)
        self.assertAlmostEqual(system.update(1.0, 1e308), -2.0)
        self.assertTrue(math.isfinite(system.y))

    def test_overflow_does_not_corrupt_state_or_history(self):
        system = FirstOrderSystem(K=1e308, tau=1.0, dead_time=0.0)
        before = (system.y, system._elapsed_time, list(system.history_u))
        with self.assertRaises(ValueError):
            system.update(1e308, 0.1)
        self.assertEqual(
            (system.y, system._elapsed_time, list(system.history_u)), before
        )

    def test_explicit_legacy_euler_is_still_available(self):
        system = FirstOrderSystem(1.0, 1.0, 0.0, integration_method="euler")
        self.assertEqual(system.update(1.0, 3.0), 3.0)
        self.assertEqual(system.update(1.0, 3.0), -3.0)

    def test_unknown_integration_method_rejected(self):
        with self.assertRaises(ValueError):
            FirstOrderSystem(1.0, 1.0, 0.0, integration_method="magic")


if __name__ == "__main__":
    unittest.main()
