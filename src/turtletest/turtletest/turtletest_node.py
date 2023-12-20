from . import SPINNER_TOPIC

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty


class TurtleTestSpinnerNode(Node):

    START_X = 1.0
    START_Y = 1.0

    WIDTH = 8   # ticks
    HEIGHT = 5

    TURN_VELOCITY = math.radians(90)

    SPEED = 2.0

    def __init__(self):
        super().__init__('turtletest_spinner')

        self.spin = False

        cb = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10, callback_group=cb)
        self.controller = self.create_subscription(Bool, SPINNER_TOPIC, self.listener_callback, 10, callback_group=cb)

        self.clear = self.create_client(Empty, '/clear')
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        while not self.clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def listener_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Receive Start Spin command')

            self.spin = True
            self.do_spin()
        else:
            self.get_logger().info('Receive Stop Spin command')
            self.spin = False

    def do_spin(self, scale=1.0):

        self.teleport.call(TeleportAbsolute.Request(x=1.0, y=1.0, theta=0.0))
        self.clear.call(Empty.Request())

        r = self.create_rate(self.SPEED)

        tick = 0
        target = self.WIDTH
        while self.spin and rclpy.ok():
            self.get_logger().info('spin')

            self.forward()
            r.sleep()

            tick += 1
            if tick == target:
                tick = 0
                target = self.WIDTH if target == self.HEIGHT else self.HEIGHT
                self.turn_left()
                r.sleep()

    def forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.SPEED
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel.publish(move_cmd)

    def turn_left(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = self.TURN_VELOCITY * self.SPEED
        self.cmd_vel.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    subscriber = TurtleTestSpinnerNode()
    #rclpy.spin(subscriber)

    executor = MultiThreadedExecutor()
    executor.add_node(subscriber)

    try:
        subscriber.get_logger().info('Beginning Spinner, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        subscriber.get_logger().info('Keyboard interrupt, shutting down.\n')

    subscriber.destroy_node()


if __name__ == '__main__':
    main()
