#!/usr/bin/env python3

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from kobuki_ros_interfaces.action import AutoDocking

class AutoDockingActionClient(Node):

    def __init__(self):
        super().__init__('auto_docking_action_client')
        self._action_client = ActionClient(self, AutoDocking, '/mobile_base/auto_docking_action')

    def send_goal(self):
        goal_msg = AutoDocking.Goal()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Sent Goal!')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info('Sent Goal! 2')
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.get_logger().info('Sent Goal! 3')

    def get_result_callback(self, future):
        self.get_logger().info('Sent Goal! 4')
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init(args=None)

    action_client = AutoDockingActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)

