import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('Nodo_5')
        self.subscription = self.create_subscription(
            Float64,
            '/filtered_sensor',
            self.listener_callback,
            10)
        
        self.get_logger().info('Escuchando /filtered_sensor...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Promedio: {msg.data:.4f}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()