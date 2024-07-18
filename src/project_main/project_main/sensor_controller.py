import rclpy
import time
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import SensorData
from rosgraph_msgs.msg import Clock

from sim_utils import EventScheduler


WORLD_NAME = "iot_project_world"

class SensorController(Node):

    def __init__(self):
        super().__init__('sensor_controller')

        self.tx_topic = self.create_publisher(
            SensorData,
            'tx_data',
            10
        )

        self.id = self.declare_parameter('id', -1)

        self.generated_data = 0


        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        self.event_scheduler.schedule_event(1, self.simple_publish)

        #self.create_timer(1, self.simple_publish)


    def simple_publish(self):

        id = self.id.get_parameter_value().integer_value

        msg = SensorData()
        msg.sensorid = str(self.id)
        msg.data = str(self.generate_data())
        msg.timestamp = float(time.time())

        self.tx_topic.publish(msg) 


    def generate_data(self):

        self.generated_data += 1
        return self.generated_data


def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor()

    sensor_controller = SensorController()
    executor.add_node(sensor_controller)

    executor.spin()

    sensor_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
