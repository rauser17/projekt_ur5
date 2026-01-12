import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class MojPublisher(Node):
    def __init__(self):
        super().__init__('moj_publisher')
        # Tworzymy temat "dane_projektu"
        self.publisher_ = self.create_publisher(String, 'dane_projektu', 10)
        timer_period = 2.0  # sekundy
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        wartosc = random.randint(0, 100)
        msg.data = f'Wartosc czujnika: {wartosc}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Wysylam: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MojPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
