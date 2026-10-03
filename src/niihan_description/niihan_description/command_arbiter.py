#!/usr/bin/env python3
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CommandArbiter(Node):
    def __init__(self):
        super().__init__('command_arbiter')
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.declare_parameter('watchdog_timeout', 0.5)
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        self.cmd_pub = self.create_publisher(Twist, '/niihan/cmd_vel', 10)

        self.estop_active = False
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.create_subscription(
            Bool, '/niihan/e_stop', self.estop_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self.create_subscription(Twist, '/niihan/cmd_vel/manual', lambda msg: self.cmd_cb(msg, 3), 10)
        self.create_subscription(Twist, '/niihan/cmd_vel/recovery', lambda msg: self.cmd_cb(msg, 2), 10)
        self.create_subscription(Twist, '/niihan/cmd_vel/nav', lambda msg: self.cmd_cb(msg, 1), 10)

        # state to keep track of latest commands from each source
        self.cmds = {3: None, 2: None, 1: None}
        self.cmd_times = {3: None, 2: None, 1: None}

        self.timer = self.create_timer(0.1, self.timer_cb, clock=self._watchdog_clock)

    def estop_cb(self, msg):
        self.estop_active = msg.data
        # Never resume a command received before a stop/reset.
        self.cmds = {3: None, 2: None, 1: None}
        self.cmd_times = {3: None, 2: None, 1: None}
        if self.estop_active:
            self.cmd_pub.publish(Twist()) # synchronous zero

    def cmd_cb(self, msg, priority):
        if self.estop_active:
            return
        if not all(math.isfinite(value) for value in (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)):
            self.cmds[priority] = Twist()
        else:
            self.cmds[priority] = msg
        self.cmd_times[priority] = self._watchdog_clock.now()
        self.evaluate_commands()

    def evaluate_commands(self):
        if self.estop_active:
            self.cmd_pub.publish(Twist())
            return

        now = self._watchdog_clock.now()
        for p in [3, 2, 1]:
            if self.cmds[p] is not None:
                dt = (now - self.cmd_times[p]).nanoseconds / 1e9
                if 0 <= dt <= self.watchdog_timeout:
                    self.cmd_pub.publish(self.cmds[p])
                    return

        # if nothing is active, send zero
        self.cmd_pub.publish(Twist())

    def timer_cb(self):
        self.evaluate_commands()

def main(args=None):
    rclpy.init(args=args)
    node = CommandArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
