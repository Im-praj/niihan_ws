import unittest, math
from gps_precision import precision_command
class SafetyContracts(unittest.TestCase):
 def test_no_movement_when_arrived(self):self.assertEqual(precision_command(.02,.02,1.,2.),(0.,0.,True))
 def test_invalid_inputs_fail_closed(self):
  for args in [(1.1,0,0,2),(.2,0,0,.7),(.2,0,float('nan'),2),(.2,0,0,float('nan'))]:
   with self.assertRaises(ValueError):precision_command(*args)
 def test_bounded_outputs(self):
  for angle in range(-180,181,5):
   v,w,done=precision_command(.4,.3,math.radians(angle),2.)
   self.assertLessEqual(abs(v),.08);self.assertLessEqual(abs(w),.5)
 def test_reverse_can_approach_target_behind(self):
  v,w,done=precision_command(-.2,0,0,2.)
  self.assertLess(v,0);self.assertAlmostEqual(w,0)
if __name__ == '__main__': unittest.main()
