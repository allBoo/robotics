from typing import Tuple

from . import (
    CONTROLLER_TOPIC,
    CMD_VEL_TOPIC,
    SCAN_TOPIC,
    HEAD_TOPIC,
    LEFT_TOPIC,
    RIGHT_TOPIC,
    ERROR_TOPIC,
    CORRECTION_TOPIC,
    CORRECTION_D_TOPIC,
    CORRECTION_I_TOPIC,
    CORRECTION_P_TOPIC
)

from . import (
    STATE_IDLE,
    STATE_SPIN,
    STATE_FOLLOW,
    STATE_NEARBY,
    STATE_RESET
)

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


class MappingNode(Node):

    INITIAL_POINT = (0.0, -3.5)
    FOLLOW_POINT = (-2.5, 1.5)

    COEFF_P = 2.4
    COEFF_D = 0.0
    COEFF_I = 20.0

    FORWARD_SPEED = 0.1
    ANGLE_SPEED = math.radians(10)

    DISTANCE = 0.5

    def __init__(self):
        super().__init__('mapping_node')

        self.state = STATE_IDLE

        cb = ReentrantCallbackGroup()
        self.controller = self.create_subscription(String, CONTROLLER_TOPIC, self.listener_callback, 10,
                                                   callback_group=cb)
        self.laser_sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.laser_callback, 10,
                                                  callback_group=cb)

        self.cmd_vel_t = self.create_publisher(Twist, CMD_VEL_TOPIC, 10, callback_group=cb)
        self.head_t = self.create_publisher(Float64, HEAD_TOPIC, 10, callback_group=cb)
        self.left_t = self.create_publisher(Float64, LEFT_TOPIC, 10, callback_group=cb)
        self.right_t = self.create_publisher(Float64, RIGHT_TOPIC, 10, callback_group=cb)
        self.error_t = self.create_publisher(Float64, ERROR_TOPIC, 10, callback_group=cb)
        self.correction_t = self.create_publisher(Float64, CORRECTION_TOPIC, 10, callback_group=cb)
        self.correction_p_t = self.create_publisher(Float64, CORRECTION_P_TOPIC, 10, callback_group=cb)
        self.correction_d_t = self.create_publisher(Float64, CORRECTION_D_TOPIC, 10, callback_group=cb)
        self.correction_i_t = self.create_publisher(Float64, CORRECTION_I_TOPIC, 10, callback_group=cb)

        self.actual_distance = 0.0
        self.correction_p = 0.0
        self.correction_i = 0.0
        self.correction_d = 0.0

        self.nav = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.INITIAL_POINT[0]
        initial_pose.pose.position.y = self.INITIAL_POINT[1]

        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.nav.setInitialPose(initial_pose)

        self.nav.waitUntilNav2Active()

    def listener_callback(self, msg: String):
        if msg.data == STATE_IDLE:
            self.get_logger().info('Receive Idle command')
            self.stop()
        elif msg.data == STATE_SPIN:
            self.get_logger().info('Receive Spin command')
            self.spin()
        elif msg.data == STATE_FOLLOW:
            self.get_logger().info('Receive Follow command')
            self.follow(self.FOLLOW_POINT)
        elif msg.data == STATE_NEARBY:
            self.get_logger().info('Receive Nearby command')
            self.state = STATE_NEARBY
        elif msg.data == STATE_RESET:
            self.get_logger().info('Receive Reset command')
            self.follow(self.INITIAL_POINT)

    def laser_callback(self, msg):
        try:
            head = msg.ranges[0]
            right = min(msg.ranges[195:345])
            left = min(msg.ranges[15:165])

            self.head_t.publish(Float64(data=head))
            self.left_t.publish(Float64(data=left))
            self.right_t.publish(Float64(data=right))

            self.nearby(head, right, left)
        except Exception as e:
            print(e)

    def stop(self):
        self.state = STATE_IDLE
        self.cmd_vel_t.publish(Twist())

    def spin(self):
        self.state = STATE_SPIN

    def follow(self, point: tuple[float, float]):
        self.state = STATE_FOLLOW

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(goal_pose)

        i = 0
        while not self.nav.isTaskComplete() and self.state == STATE_FOLLOW:
            # Do something with the feedback
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.nav.cancelTask()

        if self.state != STATE_FOLLOW:
            self.nav.cancelTask()

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

        # nav.lifecycleShutdown()

        if self.state == STATE_FOLLOW:
            self.stop()

    def nearby(self, head: float, right: float, left: float):
        if self.state != STATE_NEARBY:
            return

        forward_speed = self.FORWARD_SPEED

        # check for forehead collision
        if head <= self.DISTANCE:
            forward_speed = 0.0

        dt = 1.0
        error = self.DISTANCE - right

        self.correction_p = self.COEFF_P * error
        self.correction_i = 0.0 # self.correction_i + (self.COEFF_P / self.COEFF_I) * error * dt
        self.correction_d = -self.COEFF_P * self.COEFF_D * (right - self.actual_distance) / dt

        correction = self.correction_p + self.correction_i + self.correction_d

        self.error_t.publish(Float64(data=error))
        self.correction_p_t.publish(Float64(data=self.correction_p))
        self.correction_i_t.publish(Float64(data=self.correction_i))
        self.correction_d_t.publish(Float64(data=self.correction_d))
        self.correction_t.publish(Float64(data=correction))

        self.actual_distance = right

        angle_speed = self.ANGLE_SPEED * correction

        move_cmd = Twist()
        move_cmd.linear.x = forward_speed
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = angle_speed
        self.cmd_vel_t.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    subscriber = MappingNode()
    #rclpy.spin(subscriber)

    executor = MultiThreadedExecutor()
    executor.add_node(subscriber)

    try:
        subscriber.get_logger().info('Beginning Mover, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        subscriber.get_logger().info('Keyboard interrupt, shutting down.\n')
    except Exception as e:
        print('Core Exception', e)
        input()

    subscriber.destroy_node()


if __name__ == '__main__':
    main()
