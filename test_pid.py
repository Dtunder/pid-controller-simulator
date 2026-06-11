import unittest
from pid import PIDController, FirstOrderSystem, ziegler_nichols_tuning

class TestPID(unittest.TestCase):
    def test_pid_proportional(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        u1 = pid.update(setpoint=10.0, pv=0.0, dt=1.0)
        self.assertEqual(u1, 20.0)
        
        u2 = pid.update(setpoint=10.0, pv=5.0, dt=1.0)
        self.assertEqual(u2, 10.0)

    def test_pid_integral(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        u1 = pid.update(setpoint=1.0, pv=0.0, dt=0.5)
        self.assertEqual(u1, 0.5)
        u2 = pid.update(setpoint=1.0, pv=0.0, dt=0.5)
        self.assertEqual(u2, 1.0)

    def test_pid_derivative(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
        pid.update(setpoint=1.0, pv=0.0, dt=1.0) # prev_error initialized
        # error changed from 1.0 to 0.0 (decreased by 1.0)
        u = pid.update(setpoint=1.0, pv=1.0, dt=1.0)
        self.assertEqual(u, -1.0)

        # test derivative when dt <= 0 should raise ValueError
        with self.assertRaises(ValueError):
            pid.update(setpoint=1.0, pv=2.0, dt=0.0)
            
        with self.assertRaises(ValueError):
            pid.update(setpoint=1.0, pv=2.0, dt=-1.0)

    def test_pid_reset(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.update(setpoint=1.0, pv=0.0, dt=1.0)
        self.assertNotEqual(pid.integral, 0.0)
        self.assertNotEqual(pid.prev_error, 0.0)
        
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertEqual(pid.prev_pv, 0.0)

    def test_pid_output_limits_and_antiwindup(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0, output_limits=(-10.0, 10.0))
        
        # Drive the integral up to hit the max limit
        for _ in range(20):
            u = pid.update(setpoint=1.0, pv=0.0, dt=1.0)
        
        self.assertEqual(u, 10.0)
        
        # Test negative limit
        pid.reset()
        for _ in range(20):
            u = pid.update(setpoint=-1.0, pv=0.0, dt=1.0)
            
        self.assertEqual(u, -10.0)

    def test_first_order_system(self):
        sys = FirstOrderSystem(K=2.0, tau=1.0, dead_time=0.0)
        # Without dead time, step response should start immediately
        y = sys.update(u=1.0, dt=0.1)
        self.assertTrue(y > 0.0)
        
        # After large time, should converge to K*u = 2.0
        for _ in range(100):
            y = sys.update(u=1.0, dt=0.1)
        self.assertAlmostEqual(y, 2.0, places=2)
        
    def test_first_order_system_dead_time(self):
        sys = FirstOrderSystem(K=1.0, tau=1.0, dead_time=0.5)
        # With dead time 0.5, y should be 0 for t < 0.5
        for _ in range(5): # t=0.1, 0.2, 0.3, 0.4, 0.5
            y = sys.update(u=1.0, dt=0.1)
            self.assertEqual(y, 0.0)
        
        # Now y should increase
        y = sys.update(u=1.0, dt=0.1)
        self.assertTrue(y > 0.0)

    def test_ziegler_nichols(self):
        sys = FirstOrderSystem(K=1.0, tau=1.0, dead_time=0.5)
        kp, ki, kd = ziegler_nichols_tuning(sys, setpoint=1.0, dt=0.01, max_time=20.0)
        self.assertTrue(kp > 0)
        self.assertTrue(ki > 0)
        self.assertTrue(kd > 0)

    def test_ziegler_nichols_failure(self):
        # A system that won't oscillate properly within the given time or limits
        # High tau and no dead time won't easily oscillate
        sys = FirstOrderSystem(K=0.1, tau=10.0, dead_time=0.0)
        
        # We can also mock `detect_oscillations` to always return False to test this branch safely
        from unittest.mock import patch
        with patch('pid.detect_oscillations', return_value=(False, 0.0, 0.0)):
            kp, ki, kd = ziegler_nichols_tuning(sys, setpoint=1.0, dt=0.01, max_time=5.0)
            self.assertEqual(kp, 0.0)
            self.assertEqual(ki, 0.0)
            self.assertEqual(kd, 0.0)

    def test_simulate_and_save(self):
        import os
        sys = FirstOrderSystem(K=1.0, tau=1.0, dead_time=0.5)
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0)
        filename = "test_results.csv"
        
        # Ensure cleanup happens even if assertions fail
        self.addCleanup(lambda: os.remove(filename) if os.path.exists(filename) else None)
        
        from pid import simulate_and_save
        results = simulate_and_save(sys, pid, filename, duration=1.0)
        
        self.assertTrue(os.path.exists(filename))
        self.assertTrue(len(results) > 0)

    def test_detect_oscillations_early_exit(self):
        from pid import detect_oscillations
        # Not enough samples
        is_osc, period, amp_ratio = detect_oscillations([0.0]*50, 0.1)
        self.assertFalse(is_osc)
        self.assertEqual(period, 0.0)

    def test_invalid_types_and_values(self):
        # PIDController init
        with self.assertRaises(TypeError):
            PIDController(kp="1.0", ki=0.0, kd=0.0)
        with self.assertRaises(ValueError):
            PIDController(kp=-1.0, ki=0.0, kd=0.0)
        with self.assertRaises(ValueError):
            PIDController(kp=1.0, ki=0.0, kd=0.0, output_limits=(10.0, 0.0))
            
        # FirstOrderSystem init
        with self.assertRaises(TypeError):
            FirstOrderSystem(K=1.0, tau="1.0", dead_time=0.0)
        with self.assertRaises(ValueError):
            FirstOrderSystem(K=1.0, tau=0.0, dead_time=0.0)
        with self.assertRaises(ValueError):
            FirstOrderSystem(K=1.0, tau=1.0, dead_time=-1.0)
            
        # Update methods
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        sys = FirstOrderSystem(K=1.0, tau=1.0, dead_time=0.0)
        
        with self.assertRaises(TypeError):
            pid.update(setpoint="1.0", pv=0.0, dt=1.0)
        with self.assertRaises(ValueError):
            pid.update(setpoint=1.0, pv=0.0, dt=-1.0)
            
        with self.assertRaises(TypeError):
            sys.update(u="1.0", dt=1.0)
        with self.assertRaises(ValueError):
            sys.update(u=1.0, dt=-0.1)
            
        # Detect oscillations
        from pid import detect_oscillations
        with self.assertRaises(TypeError):
            detect_oscillations("not a list", dt=1.0)
        with self.assertRaises(ValueError):
            detect_oscillations([1.0, 2.0], dt=0.0)
            
        # Z-N tuning
        with self.assertRaises(TypeError):
            ziegler_nichols_tuning("not a system", setpoint=1.0, dt=0.01)
        with self.assertRaises(ValueError):
            ziegler_nichols_tuning(sys, setpoint=1.0, dt=-0.01)
            
        # Simulate and save
        from pid import simulate_and_save
        with self.assertRaises(TypeError):
            simulate_and_save("not a system", pid, "test.csv")
        with self.assertRaises(ValueError):
            simulate_and_save(sys, pid, "")
        with self.assertRaises(ValueError):
            simulate_and_save(sys, pid, "test.csv", duration=-1.0)

if __name__ == '__main__':
    unittest.main()
