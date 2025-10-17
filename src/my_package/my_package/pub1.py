import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('Nodo_1')
        self.publisher_ = self.create_publisher(Float64, '/sensor_1', 10)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publicando en /sensor_1')

    def timer_callback(self):
        sensor_value = random.uniform(0.0, 10.0) 
        msg = Float64()
        msg.data = sensor_value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando en /sensor_1: {msg.data:.3f}')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
