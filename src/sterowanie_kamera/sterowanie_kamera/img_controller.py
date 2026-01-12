import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        
        # 1. Subskrypcja kamery (nasłuchujemy obrazu)
        # Temat '/image' jest standardowy dla prostych narzędzi, w razie potrzeby zmienimy
        self.subscription = self.create_subscription(
            Image,
            '/burger/image',  # Nazwa tematu z obrazem
            self.image_callback,
            10)
        
        # 2. Publikacja sterowania (wysyłamy komendy do żółwia)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.current_frame = None
        self.window_name = "Interfejs Sterowania Robotem"

    def image_callback(self, msg):
        # Konwersja obrazu ROS -> OpenCV
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(self.window_name, self.current_frame)
            
            # Ustawienie obsługi myszki
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Błąd konwersji obrazu: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        # Reagujemy tylko na kliknięcie lewym przyciskiem (EVENT_LBUTTONDOWN)
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_frame is not None:
                height, width, _ = self.current_frame.shape
                polowa_wysokosci = height // 2
                
                msg = Twist()
                
                # Logika na ocenę 3.0:
                if y < polowa_wysokosci:
                    # Kliknięcie w górnej połowie -> DO PRZODU
                    msg.linear.x = 1.0
                    self.get_logger().info('Kliknięto GÓRA -> Jadę do przodu')
                else:
                    # Kliknięcie w dolnej połowie -> DO TYŁU
                    msg.linear.x = -1.0
                    self.get_logger().info('Kliknięto DÓŁ -> Jadę do tyłu')

                # Wysłanie wiadomości do robota
                self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
