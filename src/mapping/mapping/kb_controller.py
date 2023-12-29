import math

from . import (
    CONTROLLER_TOPIC,
    STATE_IDLE,
    STATE_SPIN,
    STATE_FOLLOW,
    STATE_NEARBY,
    STATE_RESET
)

import rclpy
from rclpy.node import Node
from rclpy import get_global_executor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from pynput.keyboard import Key, Listener, KeyCode


class KeyboardController(Node):

    RATE = 0.5
    SPEED_DELTA = 0.1
    ANGULAR_DELTA = math.radians(5)

    MAX_SPEED = 1.5
    MAX_ANGULAR_SPEED = math.radians(30)

    def __init__(self):
        super().__init__('mapping_controller')

        self.speed = 0.0
        self.angular = 0.0

        cb = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_cntrl = self.create_publisher(String, CONTROLLER_TOPIC, 10)
        self.state = STATE_IDLE

        print("Use arrows to move the robot")
        print("Press 0 to control robot by keys (default state)")
        print("Press 1 to follow the predefined point")
        print("Press 2 to move by radius")
        print("Press 3 to move nearby the wall")
        print("Press Esc to move to the original position")

        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            try:
                listener.join()
            except (KeyboardInterrupt, ExternalShutdownException, ):
                listener.stop()
            except Exception as e:
                print('Exception', e)

    def on_press(self, key: KeyCode | Key):
        try:
            if isinstance(key, KeyCode):
                if key.char in ['Q', 'q']:
                    self.exit()
                elif key.char == '0':
                    self.send_state(STATE_IDLE)
                elif key.char == '1':
                    self.send_state(STATE_FOLLOW)
                elif key.char == '2':
                    self.send_state(STATE_SPIN)
                elif key.char == '3':
                    self.send_state(STATE_NEARBY)
            elif isinstance(key, Key) and key == Key.esc:
                self.send_state(STATE_RESET)
            elif isinstance(key, Key) and self.state == STATE_IDLE:
                if key == Key.left:
                    self.left()
                elif key == Key.right:
                    self.right()
                elif key == Key.up:
                    self.forward()
                elif key == Key.down:
                    self.backward()
                elif key == Key.space:
                    self.stop()
        except Exception as e:
            print('Exception', e)

    def on_release(self, key):
        pass

    def move(self):
        if self.state != STATE_IDLE:
            return

        move_cmd = Twist()
        move_cmd.linear.x = self.speed
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = self.angular
        self.cmd_vel.publish(move_cmd)

    def left(self):
        self.get_logger().info('Turn left...')
        self.angular += self.ANGULAR_DELTA
        if self.angular > self.MAX_ANGULAR_SPEED:
            self.angular = self.MAX_ANGULAR_SPEED

        self.move()
        print('Angular speed', self.angular)

    def right(self):
        self.get_logger().info('Turn right...')

        self.angular -= self.ANGULAR_DELTA

        if self.angular < -self.MAX_ANGULAR_SPEED:
            self.angular = -self.MAX_ANGULAR_SPEED

        self.move()
        print('Angular speed', self.angular)

    def forward(self):
        self.get_logger().info('Move forward...')

        self.speed += self.SPEED_DELTA
        if self.speed > self.MAX_SPEED:
            self.speed = self.MAX_SPEED

        self.move()
        print('Speed', self.speed)

    def backward(self):
        self.get_logger().info('Move backward...')

        self.speed -= self.SPEED_DELTA
        if self.speed < -self.MAX_SPEED:
            self.speed = -self.MAX_SPEED

        self.move()
        print('Speed', self.speed)

    def stop(self):
        self.get_logger().info('Stop...')

        self.angular = 0.0
        self.speed = 0.0

        self.move()

    def send_state(self, state: str):
        if self.state != state:
            self.get_logger().info(state + ' it...')
            self.state = state
        else:
            self.get_logger().info('Stop it...')
            self.state = STATE_IDLE

        self.cmd_cntrl.publish(String(data=self.state))

    def exit(self):
        self.get_logger().info('Shut it down1...')
        self.destroy_node()
        rclpy.shutdown()
        exit()


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = KeyboardController()
        rclpy.spin(controller)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        controller.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print('Global exception', e)
        input()


if __name__ == '__main__':
    main()
