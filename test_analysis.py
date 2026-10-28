"""Tests for analysis.py: metric correctness, determinism, controller ranking."""
import csv
import math
import os
import tempfile
import unittest

import numpy as np

import analysis


class TestStepMetricsAnalytic(unittest.TestCase):
    """Metrics on y(t) = 1 - exp(-t/tau), whose properties are known exactly."""

    def setUp(self):
        self.tau = 2.0
        self.t = np.arange(0.0, 40.0 + 1e-9, 0.001)
        self.y = 1.0 - np.exp(-self.t / self.tau)
        self.m = analysis.step_metrics(self.t, self.y, setpoint=1.0)

    def test_rise_time_is_ln9_tau(self):
        # t90 - t10 = tau * ln(9) = 2.1972 tau
        self.assertAlmostEqual(self.m["rise_time_s"], math.log(9.0) * self.tau,
                               places=3)
        self.assertAlmostEqual(self.m["rise_time_s"] / self.tau, 2.197, places=3)

    def test_no_overshoot(self):
        # y_inf is a tail average, so a tiny positive value is expected.
        self.assertAlmostEqual(self.m["overshoot_pct"], 0.0, places=4)

    def test_settling_time_is_ln50_tau(self):
        # |y - 1| <= 0.02  <=>  t >= tau * ln(50) = 3.912 tau
        self.assertAlmostEqual(self.m["settling_time_s"], math.log(50.0) * self.tau,
                               delta=0.005)

    def test_zero_steady_state_error(self):
        self.assertAlmostEqual(self.m["steady_state_error"], 0.0, places=6)

    def test_iae_equals_tau(self):
        # integral_0^inf exp(-t/tau) dt = tau
        self.assertAlmostEqual(self.m["iae"], self.tau, places=4)

    def test_offset_and_overshoot_are_measured(self):
        # Underdamped second-order-like signal settling at 0.8 with a known peak.
        t = np.arange(0.0, 60.0, 0.001)
        y = 0.8 * (1.0 - np.exp(-0.5 * t) * np.cos(2.0 * t))
        m = analysis.step_metrics(t, y, setpoint=1.0)
        self.assertAlmostEqual(m["steady_state_error"], 0.2, places=6)
        expected_os = (np.max(y) - 0.8) / 0.8 * 100.0
        self.assertAlmostEqual(m["overshoot_pct"], expected_os, places=6)
        self.assertGreater(m["overshoot_pct"], 0.0)


class TestTuningRules(unittest.TestCase):
    def test_simc_textbook_values(self):
        # K=1, tau=1, theta=0.5, tau_c=theta -> Kc = 1/(1*1) = 1, Ti = min(1, 4) = 1
        kp, ki, kd = analysis.simc_pi(1.0, 1.0, 0.5)
        self.assertAlmostEqual(kp, 1.0)
        self.assertAlmostEqual(ki, 1.0)
        self.assertEqual(kd, 0.0)

    def test_zn_rules_roundtrip_with_pid_py(self):
        ku, tu = 4.0, 2.0
        rules = analysis.zn_rules(ku, tu)
        kp, ki, kd = rules["PID (Z-N)"]
        # pid.py uses kp=0.6Ku, ki=1.2Ku/Tu, kd=0.075KuTu
        self.assertAlmostEqual(kp, 0.6 * ku)
        self.assertAlmostEqual(ki, 1.2 * ku / tu)
        self.assertAlmostEqual(kd, 0.075 * ku * tu)
        self.assertEqual(analysis.ultimate_from_zn_pid(kp, ki, kd), (ku, tu))


class TestAnalysisRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.rows = analysis.run_analysis(cls.tmp.name)
        cls.by_name = {r["controller"]: r for r in cls.rows["step"]}

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_outputs_written(self):
        for name in ("step_metrics.csv", "step_response.png",
                     "disturbance_metrics.csv", "disturbance_rejection.png"):
            path = os.path.join(self.tmp.name, name)
            self.assertTrue(os.path.isfile(path), name)
            self.assertGreater(os.path.getsize(path), 0, name)
        with open(os.path.join(self.tmp.name, "step_metrics.csv"), newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual([r["controller"] for r in rows],
                         ["P (Z-N)", "PI (Z-N)", "PID (Z-N)", "PI (SIMC)"])
        self.assertEqual(list(rows[0].keys()), analysis.METRIC_FIELDS)

    def test_deterministic(self):
        with tempfile.TemporaryDirectory() as other:
            analysis.run_analysis(other)
            for name in ("step_metrics.csv", "disturbance_metrics.csv"):
                with open(os.path.join(self.tmp.name, name)) as a, \
                        open(os.path.join(other, name)) as b:
                    self.assertEqual(a.read(), b.read(), name)

    def test_pid_beats_p_on_steady_state_error(self):
        p_err = abs(self.by_name["P (Z-N)"]["steady_state_error"])
        pid_err = abs(self.by_name["PID (Z-N)"]["steady_state_error"])
        # Pure P leaves offset 1/(1 + K*Kp); integral action removes it.
        kp = self.by_name["P (Z-N)"]["kp"]
        self.assertAlmostEqual(p_err, 1.0 / (1.0 + analysis.K * kp), places=3)
        self.assertLess(pid_err, 1e-3)
        self.assertLess(pid_err, p_err)

    def test_simc_is_more_conservative_than_zn_pid(self):
        simc, zn = self.by_name["PI (SIMC)"], self.by_name["PID (Z-N)"]
        self.assertLess(simc["overshoot_pct"], zn["overshoot_pct"])
        self.assertGreater(simc["rise_time_s"], zn["rise_time_s"])

    def test_integral_controllers_reject_load_step(self):
        for r in self.rows["disturbance"]:
            if r["controller"] == "P (Z-N)":
                self.assertGreater(abs(r["steady_state_error"]), 0.05)
            else:
                self.assertLess(abs(r["steady_state_error"]), 0.01, r["controller"])


if __name__ == "__main__":
    unittest.main()
