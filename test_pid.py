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

if __name__ == '__main__':
    unittest.main()
