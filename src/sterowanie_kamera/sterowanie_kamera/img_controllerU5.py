import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import math

class VisualControlNode(Node):
    def __init__(self):
        super().__init__('visual_control_node')
        
        self.subscription = self.create_subscription(
            Image, '/burger/image', self.image_callback, 10)
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.bridge = CvBridge()
        self.current_frame = None
        self.window_name = "Panel Sterowania UR5"
        
        self.shoulder_lift = -1.57  
        
        self.click_pos = None       
        self.click_color = (0, 255, 0) 
        self.visual_counter = 0     
        
        self.timer = self.create_timer(0.05, self.publish_joints)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        msg.position = [0.0, self.shoulder_lift, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)

    def image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            height, width, _ = self.current_frame.shape
            
            cv2.line(self.current_frame, (0, height//2), (width, height//2), (255, 255, 0), 2)
            cv2.putText(self.current_frame, "GORA (Podnoszenie)", (10, height//2 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(self.current_frame, "DOL (Opuszczanie)", (10, height//2 + 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if self.visual_counter > 0:
                if self.click_pos is not None:
                    x, y = self.click_pos
                    top_left = (x - 20, y - 20)
                    bottom_right = (x + 20, y + 20)
                    cv2.rectangle(self.current_frame, top_left, bottom_right, self.click_color, 3)
                
        
                self.visual_counter -= 1

            cv2.imshow(self.window_name, self.current_frame)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Błąd wideo: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_frame is not None:
                height, _, _ = self.current_frame.shape
                self.click_pos = (x, y)
                self.visual_counter = 20      
                step = 0.1
                
                if y < height // 2:
                    self.shoulder_lift -= step
                    self.click_color = (0, 255, 0)
                    self.get_logger().info('GÓRA -> Podnoszę')
                else:
                    self.shoulder_lift += step
                    self.click_color = (0, 0, 255)
                    self.get_logger().info('DÓŁ -> Opuszczam')
                
                self.shoulder_lift = max(-3.14, min(0.0, self.shoulder_lift))

def main(args=None):
    rclpy.init(args=args)
    node = VisualControlNode()
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
