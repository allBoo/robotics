from . import SPINNER_TOPIC

import rclpy
from rclpy.node import Node
from rclpy import get_global_executor
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from pynput.keyboard import Key, Listener, KeyCode


class TurtleTestKeyboardController(Node):

    def __init__(self):
        super().__init__('turtletest_controller')

        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd_spin = self.create_publisher(Bool, SPINNER_TOPIC, 10)
        self.spinner = False

        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            try:
                listener.join()
            except (KeyboardInterrupt, ExternalShutdownException, ):
                listener.stop()

    def on_press(self, key: KeyCode | Key):
        if isinstance(key, KeyCode):
            if key.char in ['Q', 'q']:
                self.stop()
        elif isinstance(key, Key):
            if key == Key.left:
                self.left()
            elif key == Key.right:
                self.right()
            elif key == Key.up:
                self.forward()
            elif key == Key.down:
                self.backward()
            elif key == Key.space:
                self.spin()

    def on_release(self, key):
        if key == Key.esc:
            # Stop listener
            self.stop()

    def left(self):
        self.get_logger().info('Turn left...')

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 1.0
        self.cmd_vel.publish(move_cmd)

    def right(self):
        self.get_logger().info('Turn right...')

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = -1.0
        self.cmd_vel.publish(move_cmd)

    def forward(self):
        self.get_logger().info('Move forward...')

        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel.publish(move_cmd)

    def backward(self):
        self.get_logger().info('Move backward...')

        move_cmd = Twist()
        move_cmd.linear.x = -1.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel.publish(move_cmd)

    def spin(self):
        if not self.spinner:
            self.get_logger().info('Spin it...')
            self.spinner = True
        else:
            self.get_logger().info('Stop Spin...')
            self.spinner = False
        spin_msg = Bool()
        spin_msg.data = self.spinner
        self.cmd_spin.publish(spin_msg)

    def stop(self):
        self.get_logger().info('Shut it down1...')
        self.destroy_node()
        rclpy.shutdown()
        exit()


def main(args=None):
    rclpy.init(args=args)

    controller = TurtleTestKeyboardController()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
