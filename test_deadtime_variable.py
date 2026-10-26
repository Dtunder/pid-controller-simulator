import math
import random
import unittest
from fractions import Fraction

from pid import FirstOrderSystem


class TestDeadTimeVariableStep(unittest.TestCase):
    def test_variable_steps_use_elapsed_time_not_sample_count(self):
        s = FirstOrderSystem(K=1, tau=1, dead_time=0.5)
        # Sample u=1 arrives at t=.1; delayed input becomes available at t=.6.
        sequence = [(1, 0.1), (2, 0.2), (3, 0.25), (4, 0.1), (5, 0.1)]
        actual = [s.update(u, dt) for u, dt in sequence]
        self.assertEqual(actual[:3], [0, 0, 0])
        self.assertAlmostEqual(actual[3], -math.expm1(-0.1), places=10)
        self.assertAlmostEqual(actual[4], -math.expm1(-0.2), places=10)

    def test_fractional_delay_does_not_round_down(self):
        s = FirstOrderSystem(K=1, tau=1, dead_time=0.75)
        actual = [s.update(1, 0.5) for _ in range(3)]
        self.assertEqual(actual[:2], [0, 0])
        self.assertAlmostEqual(actual[2], -math.expm1(-0.5), places=10)

    def test_reset_clears_timing_history(self):
        s = FirstOrderSystem(K=1, tau=1, dead_time=0.5)
        for _ in range(10):
            s.update(1, 0.1)
        s.reset()
        self.assertEqual(s.update(7, 0.1), 0)
        self.assertEqual(s.update(8, 0.1), 0)

    def test_zero_delay_remains_immediate_and_bounded(self):
        s = FirstOrderSystem(K=2, tau=1, dead_time=0)
        self.assertAlmostEqual(s.update(3, 0.1), 6 * -math.expm1(-0.1), places=10)
        for _ in range(100):
            s.update(1, 0.1)
        self.assertLessEqual(len(s.history_u), 1)

    def test_independent_fraction_time_reference_2000_steps(self):
        # Independent oracle: exact rational timestamps + full input history,
        # not the production deque/cache algorithm.
        rng = random.Random(20261026)
        dt_choices = ["0.05", "0.1", "0.2", "0.3"]
        for case in range(20):
            delay = rng.choice(["0.15", "0.5", "0.75", "1.2"])
            s = FirstOrderSystem(K=1.25, tau=1.2, dead_time=float(delay))
            now = Fraction(0)
            history = []
            reference_y = 0.0
            for step in range(100):
                dt_string = rng.choice(dt_choices)
                dt_exact = Fraction(dt_string)
                dt = float(dt_exact)
                u = rng.randint(-3, 3)
                now += dt_exact
                history.append((now, u))
                cutoff = now - Fraction(delay)
                delayed = next((v for timestamp, v in reversed(history)
                                if timestamp <= cutoff), 0.0)
                alpha = -math.expm1(-dt / 1.2)
                reference_y = (1 - alpha) * reference_y + alpha * 1.25 * delayed
                actual = s.update(u, dt)
                self.assertAlmostEqual(
                    actual, reference_y, places=9,
                    msg=f"case={case} step={step} t={now} delay={delay}"
                )


if __name__ == "__main__":
    unittest.main()
