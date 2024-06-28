import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

from sim_utils import EventScheduler
from project_interfaces import SensorData


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
        self.sensor_position = None


        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        self.event_scheduler.schedule_event(1, self.simple_publish)

        #self.create_timer(1, self.simple_publish)

        # Sensor wants to know its own position because it will be included in the SensorData
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            f'Sensor_{self.id}/odometry',  
            self.store_position,
            10
        )


    def simple_publish(self):

        id = self.id.get_parameter_value().integer_value

        msg = SensorData()
        msg.data = f"{self.generate_data()}!"
        # We inform which sensor sensed the data
        msg.sensorId = id
        # Use time to calculate packet delay
        msg.timestamp = self.event_scheduler.current_time
        # Position of sensor
        msg.position = self.sensor_position

        self.tx_topic.publish(msg) 


    def generate_data(self):

        self.generated_data += 1
        return self.generated_data
    
    def store_position(self, odometry_msg: Odometry):
        self.sensor_position = odometry_msg.pose.pose.position


def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor()

    sensor_controller = SensorController()
    executor.add_node(sensor_controller)

    executor.spin()

    sensor_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
