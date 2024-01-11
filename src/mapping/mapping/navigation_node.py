from . import (
    INITIAL_POINT,
    CONTROLLER_TOPIC,
    CMD_VEL_TOPIC,
)

from . import (
    STATE_IDLE,
    STATE_FOLLOW,
    STATE_RESET
)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


class NavigationNode(Node):

    FOLLOW_POINT = (-2.5, 1.5)

    def __init__(self):
        super().__init__('mapping_node')

        self.state = STATE_IDLE

        cb = ReentrantCallbackGroup()
        self.controller = self.create_subscription(String, CONTROLLER_TOPIC, self.listener_callback, 10,
                                                   callback_group=cb)

        self.cmd_vel_t = self.create_publisher(Twist, CMD_VEL_TOPIC, 10, callback_group=cb)

        self.nav = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = INITIAL_POINT[0]
        initial_pose.pose.position.y = INITIAL_POINT[1]
        
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.nav.setInitialPose(initial_pose)

        self.nav.waitUntilNav2Active()

    def listener_callback(self, msg: String):
        if msg.data == STATE_IDLE:
            self.get_logger().info('Receive Idle command')
            self.stop()
        elif msg.data == STATE_FOLLOW:
            self.get_logger().info('Receive Follow command')
            self.follow(self.FOLLOW_POINT)
        elif msg.data == STATE_RESET:
            self.get_logger().info('Receive Reset command')
            self.follow(INITIAL_POINT)

    def stop(self):
        self.state = STATE_IDLE
        self.cmd_vel_t.publish(Twist())

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

        if self.state == STATE_FOLLOW:
            self.stop()


def main(args=None):
    rclpy.init(args=args)

    subscriber = NavigationNode()
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
