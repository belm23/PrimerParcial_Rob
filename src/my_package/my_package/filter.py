import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64

class DatosGlobales:
    def __init__(self):
        self.sensor1 = None
        self.sensor2 = None
        self.sensor3 = None

class MinimalSuscriber(Node):
    def __init__(self, datos):
        super().__init__('Nodo_4S')
        self.datos = datos

        self.create_subscription(Float64, '/sensor_1', self.sensor1_callback, 10)
        self.create_subscription(Float64, '/sensor_2', self.sensor2_callback, 10)
        self.create_subscription(Float64, '/sensor_3', self.sensor3_callback, 10)

    def sensor1_callback(self, msg):
        self.datos.sensor1 = msg.data

    def sensor2_callback(self, msg):
        self.datos.sensor2 = msg.data

    def sensor3_callback(self, msg):
        self.datos.sensor3 = msg.data

class MinimalPublisher(Node):
    def __init__(self, datos):
        super().__init__('Nodo_4P')
        self.datos = datos
        self.publisher_ = self.create_publisher(Float64, '/filtered_sensor', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publicando en /filtered_sensor')

    def timer_callback(self):
        s1 = self.datos.sensor1
        s2 = self.datos.sensor2
        s3 = self.datos.sensor3

        if all(d is not None for d in [s1, s2, s3]):
            promedio = (s1 + s2 + s3) / 3.0
            msg_to_publish = Float64()
            msg_to_publish.data = promedio
            self.publisher_.publish(msg_to_publish)
            self.get_logger().info(f'Promedio: {msg_to_publish.data:.2f}')
        else:
            self.get_logger().warn('Esperando...')

def main(args=None):
    rclpy.init(args=args)

    shared_data_container = DatosGlobales()
    minimal_subscriber = MinimalSuscriber(shared_data_container)
    minimal_publisher = MinimalPublisher(shared_data_container)
    executor = SingleThreadedExecutor()

    executor.add_node(minimal_subscriber)
    executor.add_node(minimal_publisher)
    executor.spin()

    executor.shutdown()
    minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()