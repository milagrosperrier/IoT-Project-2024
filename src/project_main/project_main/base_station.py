import time
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from nav_msgs.msg import Odometry

import math_utils


NUMBER_OF_BALLOONS = int(sys.argv[1])

class BaseStation(Node):

    def __init__(self):
        super().__init__('base_station')

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            'odometry',
            self.store_position,
            10
        )

        for i in range(NUMBER_OF_BALLOONS):
            self.balloon_controller_publisher = self.create_subscription(
                String,
                f'Balloon_{i}/odometry',
                self.balloon_controller_callback,
                10
            )

    def balloon_controller_callback(self, msg: String):
        # This function will be called whenever a message is received from balloon_controller
        # Process the received data here
        self.get_logger().info(f'Received data from balloon_controller: {msg.data}')

    def store_position(self, odometry_msg : Odometry):
        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )


def main():
    rclpy.init()
    base_station = BaseStation()
    rclpy.spin(base_station)
    base_station.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
