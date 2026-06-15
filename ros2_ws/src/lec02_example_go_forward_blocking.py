#!/usr/bin/env python3

# Author: Alberto Quattrini Li
# Date: 2026-03-30
# Description: Example node to move forward for a fixed duration using a blocking while loop.
# Teaching note: This file shows a simpler first example before introducing timers.

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

FREQUENCY = 10 #Hz.
LINEAR_VELOCITY = 0.125 #m/s
DURATION = 8.0 #s how long the message should be published.
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0 #s. Max wait for simulator/controller startup.


class GoForwardBlocking(Node):
    def __init__(self, linear_velocity=LINEAR_VELOCITY,
                 node_name='go_forward_blocking', context=None):
        """Constructor."""
        super().__init__(node_name, context=context)

        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME,
        )
        self.set_parameters([use_sim_time_param])

        self.linear_velocity = linear_velocity
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)

    def move_forward(self, duration):
        """Move forward for a given duration using a blocking while loop."""
        if duration <= 0.0:
            self.get_logger().warn('Duration must be > 0. Nothing to do.')
            return

        self._wait_for_sim_ready(STARTUP_TIMEOUT)

        self.get_logger().info('Starting forward motion with blocking loop...')

        duration = Duration(seconds=duration)
        start_time = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0 / FREQUENCY)
            if self.get_clock().now() - start_time >= duration:
                break
            self._publish_velocity(self.linear_velocity, 0.0)

        self.stop()
        self.get_logger().info('Motion completed.')

    def _wait_for_sim_ready(self, timeout_sec):
        """Wait until simulation clock and cmd_vel subscriber are ready."""
        self.get_logger().info('Waiting for simulation to be ready...')

        start_time = time.monotonic()
        clock_ready = not USE_SIM_TIME

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = self.get_clock().now()
            if time.monotonic() - start_time >= timeout_sec:
                self.get_logger().warn('Startup wait timeout reached. Continuing anyway.')
                return

            if USE_SIM_TIME and now.nanoseconds > 0:
                clock_ready = True

            cmd_ready = self._cmd_pub.get_subscription_count() > 0
            if clock_ready and cmd_ready:
                self.get_logger().info('Simulation ready. Starting motion.')
                return

    def stop(self):
        """Stop the robot."""
        self._publish_velocity(0.0, 0.0)

    def stop_and_flush(self, repeats=3, timeout_sec=0.05):
        """Publish stop commands and briefly spin to increase delivery reliability."""
        if not rclpy.ok():
            return
        for _ in range(repeats):
            self.stop()
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _publish_velocity(self, linear_x, angular_z):
        """Publish a velocity command."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self._cmd_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    go_forward = GoForwardBlocking()
    try:
        go_forward.move_forward(DURATION)
    except KeyboardInterrupt:
        if rclpy.ok():
            go_forward.get_logger().info('Ctrl+C received. Sending stop command...')
        go_forward.stop_and_flush()
    finally:
        if rclpy.ok():
            go_forward.stop_and_flush()
        go_forward.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()