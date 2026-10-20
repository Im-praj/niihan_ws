"""Bounded final approach after Nav2 succeeds, using simulated GPS and IMU heading.

This simulation controller requires open local clearance. It never bypasses the
command arbiter and stops on stale GPS, odometry or scan. Not a hardware controller.
"""
import math

def precision_command(dx,dy,yaw,clearance):
 distance=math.hypot(dx,dy)
 if not all(math.isfinite(v) for v in (dx,dy,yaw)) or distance>1.0:
  raise ValueError('GPS correction exceeds one metre or is invalid')
 if not math.isfinite(clearance) or clearance<.75:raise ValueError('Insufficient scan clearance for final approach')
 if distance<=.05:return 0.,0.,True
 # Control the measured antenna/base position directly. Close targets may
 # lie inside the turning circle, so a heading-first approach can orbit them.
 forward=math.cos(yaw)*dx+math.sin(yaw)*dy
 lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
 linear=max(-.08,min(.08,.5*forward))
 angular=max(-.5,min(.5,2.0*lateral))
 return linear,angular,False
