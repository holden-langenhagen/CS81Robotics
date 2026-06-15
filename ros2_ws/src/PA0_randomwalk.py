#!/usr/bin/env python3
# The line above is important so that this file is interpreted with Python when running it.

# Author: TODO: Holden Langenhagen
# Date: TODO: 6/15/26

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
import time

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# NOTE: there might be some other libraries that can be useful
# as seen in lec02_example_go_forward.py, e.g., Duration
from rclpy.duration import Duration

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (TODO: feel free to tune)
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi / 4 # rad/s

# Random turn angle range in degrees (TODO: feel free to tune)
RANDOM_TURN_MIN_DEG = -180.0 / 180 * math.pi # rad
RANDOM_TURN_MAX_DEG = 180.0 / 180 * math.pi # rad

# Threshold of minimum clearance distance (TODO: feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (TODO: feel free to tune)
MIN_SCAN_ANGLE_RAD = -22.5 / 180.0 * math.pi
MAX_SCAN_ANGLE_RAD = +22.5 / 180.0 * math.pi

USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0 # s. Max wait for simulator/controller startup.


class RandomWalk(Node):
    def __init__(
        self,
        linear_velocity=LINEAR_VELOCITY,
        angular_velocity=ANGULAR_VELOCITY,
        min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        random_turn_min_deg=RANDOM_TURN_MIN_DEG,
        random_turn_max_deg=RANDOM_TURN_MAX_DEG,
        scan_angle=None,
        node_name='random_walk',
        context=None,
    ):
        """Constructor."""
        super().__init__(node_name, context=context)

        if scan_angle is None:
            scan_angle = (MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME,
        )
        self.set_parameters([use_sim_time_param])

        # Setting up publishers/subscribers.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)
        self._laser_sub = self.create_subscription(
            LaserScan,
            DEFAULT_SCAN_TOPIC,
            self._laser_callback,
            1,
        )

        # Parameters.
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.min_threshold_distance = min_threshold_distance
        self.random_turn_min_deg = random_turn_min_deg
        self.random_turn_max_deg = random_turn_max_deg
        self.scan_angle = tuple(scan_angle)
        

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False
        self._control_timer = None

        self._motion_duration = None # from Lec02 go forward with timer
        self._motion_start_time = None
        self._motion_timer = None
        self.motion_done = True
        
        self._signed_angular_vel = None # enables bidirectional rotation of robot
        self._rotate_timer_start = True

        

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
                self.get_logger().info('Simulation ready. Starting random walk.')
                return

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        self.move(0.0, 0.0)

    def stop_and_flush(self, repeats=3, timeout_sec=0.05):
        """Publish stop commands and briefly spin to increase delivery reliability."""
        if not rclpy.ok():
            return
        for _ in range(repeats):
            self.stop()
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: index 0 corresponds to min_angle,
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...

        # Find the minimum range value between min_scan_angle and
        # max_scan_angle
        # If the minimum range value found is closer to min_threshold_distance, change the flag self._close_obstacle
        # Note: You have to find the min index and max index.
        # Please double check the LaserScan message http://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html
        ####### TODO: ANSWER CODE BEGIN #######
        min_range = 9999
        startind = len(msg.ranges)-1 + round(MIN_SCAN_ANGLE_RAD / msg.angle_increment)
        endind = round(MAX_SCAN_ANGLE_RAD / msg.angle_increment)
        for i in range(startind,len(msg.ranges)-1): # left side of view
            dist = msg.ranges[i]
            if dist >= msg.range_min and dist <= msg.range_max and min_range > dist: # only consider acceptable ranged sensor values
                min_range = dist
        for i in range(0,endind): # right side of view
            dist = msg.ranges[i]
            if dist >= msg.range_min and dist <= msg.range_max and min_range > dist: # only consider acceptable ranged sensor values
                min_range = dist
        if min_range <= self.min_threshold_distance:
            self._close_obstacle = True # When threshold is surpassed, makes the obstacle flag true
        else:
            self._close_obstacle = False
            ####### TODO: ANSWER CODE END #######

    def start(self):
        """Wait for startup readiness and begin timer-driven control loop."""
        self._wait_for_sim_ready(STARTUP_TIMEOUT)
        self._control_timer = self.create_timer(1.0 / FREQUENCY, self._control_loop_callback)

    def _control_loop_callback(self):
        """Periodic control callback for random walk behavior."""
        # If the flag self._close_obstacle is False, the robot should move forward.
        # Otherwise, the robot should rotate for a random angle (use random.uniform() to generate a random value)
        # after which the flag is set again to False.
        # Use the function move to publish velocities already implemented,
        # passing the default velocities saved in the corresponding class members.

        ####### TODO: ANSWER CODE BEGIN #######
        if not self._close_obstacle and self.motion_done: # ensures that move will not happen until recovery mode finished
            self.move(LINEAR_VELOCITY,0.0)
        elif self._rotate_timer_start: # only start the rotating timer once
            self.stop()
            rand_angle = random.uniform(RANDOM_TURN_MIN_DEG,RANDOM_TURN_MAX_DEG)
            self._signed_angular_vel =  rand_angle/abs(rand_angle) * ANGULAR_VELOCITY # changes sign of angular velocity to match randomly chosen angle
            self.start_motion_with_timer(rand_angle/self._signed_angular_vel) # start timer with appropriate duration to reach random angle
            self._rotate_timer_start = False
        ####### TODO: ANSWER CODE END #######
    
    def start_motion_with_timer(self, duration): # from lec02_example_go_forward.py
        """Move forward for a given duration using a ROS timer callback."""
        if duration <= 0.0:
            self.get_logger().warn('Duration must be > 0. Nothing to do.')
            return False

        self._wait_for_sim_ready(STARTUP_TIMEOUT)

        self._motion_duration = Duration(seconds=duration)
        self._motion_start_time = self.get_clock().now()
        self.motion_done = False
        self._motion_timer = self.create_timer(1.0 / FREQUENCY, self._motion_timer_callback)

        self.get_logger().info('Starting forward motion with timer callback...')
        return True

    def _motion_timer_callback(self):
        """Publish forward commands until the requested duration has elapsed."""
        if self._motion_start_time is None or self._motion_duration is None:
            return

        if self.get_clock().now() - self._motion_start_time >= self._motion_duration:
            self.stop()
            if self._motion_timer is not None:
                self.destroy_timer(self._motion_timer)
                self._motion_timer = None
            self.motion_done = True
            self.get_logger().info('Motion completed.')
            if self._close_obstacle: # if there is still an obstacle after all of this
                self.get_logger().info('Obstacle detected after completed motion... Redoing motion')
            self._rotate_timer_start = True # allows random motion timer to start again since motion is completed
            return
        
        self.move(0.0, self._signed_angular_vel) # publishes angular velocity to reach goal

        
def main(args=None):
    """Main function."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    random_walk = RandomWalk()
    try:
        random_walk.start()
        rclpy.spin(random_walk)
    except KeyboardInterrupt:
        if rclpy.ok():
            random_walk.get_logger().info('Ctrl+C received. Sending stop command...')
        random_walk.stop_and_flush()
    finally:
        if rclpy.ok():
            random_walk.stop_and_flush()
        random_walk.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    """Run the main function."""
    main()