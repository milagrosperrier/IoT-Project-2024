import time
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from project_interfaces import SensorData
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from sim_utils import EventScheduler

import math_utils


NUMBER_OF_BALLOONS = int(sys.argv[1])
WORLD_NAME = "iot_project_world"

class BaseStation(Node):

    def __init__(self):
        super().__init__('base_station')
        
        self.position = None

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/base_station/odometry',
            self.store_position,
            10
        )

        self.balloon_subscribers = []
        for i in range(NUMBER_OF_BALLOONS):
            self.balloon_subscribers.append(
                self.create_subscription(
                    SensorData,
                    f'Balloon_{i}/rx_data', 
                    lambda msg, balloon_id=i: self.balloon_callback(balloon_id, msg),
                    10
                )
            )

        # We will use this to know the time in which the message was received (self.event_scheduler.current_time)
        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

    def balloon_callback(self, balloon_id, msg: SensorData):
        # Here we should call a method that calculates all metrics from the received data
        self.get_logger().info(f'Received data from Balloon_{balloon_id}: {msg.data}')


    # Base station stores its own position
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
