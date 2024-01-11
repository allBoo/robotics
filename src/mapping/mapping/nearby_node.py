import traceback
from .nearby import NearbyController, Response as NearbyResponse

from . import (
    CONTROLLER_TOPIC,
    CMD_VEL_TOPIC,
    SCAN_TOPIC,
    ANGLE_TOPIC,
    DISTANCE_TOPIC,

    DISTANCE_ERROR_TOPIC,
    DISTANCE_CORRECTION_TOPIC,
    DISTANCE_CORRECTION_P_TOPIC,
    DISTANCE_CORRECTION_D_TOPIC,
    DISTANCE_CORRECTION_I_TOPIC,

    ANGLE_ERROR_TOPIC,
    ANGLE_CORRECTION_TOPIC,
    ANGLE_CORRECTION_P_TOPIC,
    ANGLE_CORRECTION_D_TOPIC,
    ANGLE_CORRECTION_I_TOPIC,
)

from . import (
    STATE_IDLE,
    STATE_NEARBY,
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


class NearbyNode(Node):

    FORWARD_SPEED = 0.5
    ANGLE_SPEED = math.radians(45)

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
        self.angle_t = self.create_publisher(Float64, ANGLE_TOPIC, 10, callback_group=cb)
        self.distance_t = self.create_publisher(Float64, DISTANCE_TOPIC, 10, callback_group=cb)

        self.angle_error_t = self.create_publisher(Float64, ANGLE_ERROR_TOPIC, 10, callback_group=cb)
        self.angle_correction_t = self.create_publisher(Float64, ANGLE_CORRECTION_TOPIC, 10, callback_group=cb)
        self.angle_correction_p_t = self.create_publisher(Float64, ANGLE_CORRECTION_P_TOPIC, 10, callback_group=cb)
        self.angle_correction_d_t = self.create_publisher(Float64, ANGLE_CORRECTION_D_TOPIC, 10, callback_group=cb)
        self.angle_correction_i_t = self.create_publisher(Float64, ANGLE_CORRECTION_I_TOPIC, 10, callback_group=cb)

        self.distance_error_t = self.create_publisher(Float64, DISTANCE_ERROR_TOPIC, 10, callback_group=cb)
        self.distance_correction_t = self.create_publisher(Float64, DISTANCE_CORRECTION_TOPIC, 10, callback_group=cb)
        self.distance_correction_p_t = self.create_publisher(Float64, DISTANCE_CORRECTION_P_TOPIC, 10, callback_group=cb)
        self.distance_correction_d_t = self.create_publisher(Float64, DISTANCE_CORRECTION_D_TOPIC, 10, callback_group=cb)
        self.distance_correction_i_t = self.create_publisher(Float64, DISTANCE_CORRECTION_I_TOPIC, 10, callback_group=cb)

        self.nearby_controller = NearbyController(
            self.DISTANCE,
            self.get_logger(),
            self.nearby_callback
        )

    def listener_callback(self, msg: String):
        if self.state == STATE_NEARBY and msg.data != STATE_NEARBY:
            self.nearby_controller.stop()

        if msg.data == STATE_NEARBY:
            self.get_logger().info('Receive Nearby command')
            if self.state != STATE_NEARBY:
                self.state = STATE_NEARBY
                self.nearby_controller.go()
        else:
            self.get_logger().info('Receive Idle command')
            if self.state == STATE_NEARBY:
                self.nearby_controller.stop()
            self.stop()

    def laser_callback(self, msg):
        try:
            # self.head_t.publish(Float64(data=head))
            # self.left_t.publish(Float64(data=left))
            # self.right_t.publish(Float64(data=right))

            self.nearby(msg.ranges)
        except Exception as e:
            print(e)
            print(traceback.format_exc())

    def stop(self):
        self.state = STATE_IDLE
        self.cmd_vel_t.publish(Twist())

    def nearby(self, ranges: list[float]):
        if self.state != STATE_NEARBY:
            return

        self.nearby_controller.update(ranges)

    def nearby_callback(self, msg: NearbyResponse):
        logger = self.get_logger()

        logger.info('Received callback response')

        angle = msg.angle
        distance = msg.distance

        self.angle_t.publish(Float64(data=angle.value or 0.0))
        self.distance_t.publish(Float64(data=distance.value or 0.0))

        self.angle_error_t.publish(Float64(data=angle.error))
        self.angle_correction_t.publish(Float64(data=angle.c))
        self.angle_correction_p_t.publish(Float64(data=angle.p))
        self.angle_correction_d_t.publish(Float64(data=angle.d))
        self.angle_correction_i_t.publish(Float64(data=angle.i))

        self.distance_error_t.publish(Float64(data=distance.error))
        self.distance_correction_t.publish(Float64(data=distance.c))
        self.distance_correction_p_t.publish(Float64(data=distance.p))
        self.distance_correction_d_t.publish(Float64(data=distance.d))
        self.distance_correction_i_t.publish(Float64(data=distance.i))

        # c = msg.angle
        # logger.info(f'Angle Correction c={c.c}, p={c.p}, i={c.i}, d={c.d}, v={c.value}, e={c.error}, ')
        #
        # c = msg.distance
        # logger.info(f'Distance Correction c={c.c}, p={c.p}, i={c.i}, d={c.d}, v={c.value}, e={c.error}')

        angle_speed = self.ANGLE_SPEED * angle.c
        forward_speed = self.FORWARD_SPEED * (-distance.c)

        move_cmd = Twist()
        move_cmd.linear.x = forward_speed
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = angle_speed
        self.cmd_vel_t.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    subscriber = NearbyNode()
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
