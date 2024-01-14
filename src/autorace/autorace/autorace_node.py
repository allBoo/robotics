from . import (
    CONTROLLER_TOPIC,
    CMD_VEL_TOPIC,
    CAMERA_TOPIC,
)

from . import (
    STATE_IDLE,
    STATE_FOLLOW,
)

from .autorace import AutoRacer

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2


class AutoRaceNode(Node):

    FORWARD_SPEED = 0.1

    SOURCE_IMAGE = 'Source Image'

    def __init__(self):
        super().__init__('autorace_node')

        self.state = STATE_IDLE

        cb = ReentrantCallbackGroup()
        self.controller = self.create_subscription(String, CONTROLLER_TOPIC, self.listener_callback, 10,
                                                   callback_group=cb)
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.camera_callback, 10,
                                                  callback_group=MutuallyExclusiveCallbackGroup())

        self.cmd_vel_t = self.create_publisher(Twist, CMD_VEL_TOPIC, 10, callback_group=cb)

        self.racer = AutoRacer(self.get_logger())

    def listener_callback(self, msg: String):
        if msg.data == STATE_FOLLOW:
            self.get_logger().info('Receive Follow command')
            self.state = STATE_FOLLOW
        else:
            self.get_logger().info('Receive Stop command')
            self.stop()
            self.state = STATE_IDLE

    def camera_callback(self, data):
        if self.state != STATE_FOLLOW:
            return

        compensation = self.racer.get_compensation(data)
        if compensation is None:
            return

        move_cmd = Twist()
        move_cmd.linear.x = self.FORWARD_SPEED
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = compensation
        self.cmd_vel_t.publish(move_cmd)

    def stop(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel_t.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    subscriber = AutoRaceNode()

    # executor = MultiThreadedExecutor()
    # executor.add_node(subscriber)

    try:
        subscriber.get_logger().info('Beginning Autorace, shut down with CTRL-C')
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('Keyboard interrupt, shutting down.\n')
    except Exception as e:
        print('Core Exception', e)
        input()

    subscriber.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

