#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from kobuki_ros_interfaces.msg import ButtonEvent

from auto_docking_client import AutoDockingActionClient

class activation_button(Node):
    def __init__(self):
        super().__init__('activation_button_node')
        self.subscription = self.create_subscription(
            ButtonEvent,
            '/mobile_base/events/button',
            self.button_callback,
            10
        )
        self.get_logger().info("Auto Docking listen to BUTTON2")  
        
    def button_callback(self, data):
        if (data.button == ButtonEvent.BUTTON2) and (data.state == ButtonEvent.PRESSED):
            self.get_logger().info("Button %s was pressed."%data.button)

            action_client = AutoDockingActionClient()
            action_client.send_goal()
            rclpy.spin(action_client)

def main(args=None):
    rclpy.init(args=args)

    button = activation_button()

    rclpy.spin(button)

    button.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()