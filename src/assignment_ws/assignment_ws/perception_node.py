import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')
        self.bridge      = CvBridge()
        self._last_state = None

        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.pub_overlay = self.create_publisher(Image,  '/perception/image_overlay', 10)
        self.pub_result  = self.create_publisher(String, '/perception/detection',     10)

        self.get_logger().info('Perception node started')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red detection — two ranges because red wraps in HSV
        mask1 = cv2.inRange(hsv, np.array([0,   80, 80]), np.array([15,  255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        mask  = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        overlay  = frame.copy()
        detected = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 3000:  # only detect when drone is directly above
                continue

            detected   = True
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy     = x + w // 2, y + h // 2

            cv2.rectangle(overlay, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(overlay, f'TARGET area={int(area)}', (x, y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Only log on state change
        state = 'DETECTED' if detected else 'NO_DETECTION'
        if state != self._last_state:
            self._last_state = state
            self.get_logger().warn(f'Perception: {state}')

        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))

        result      = String()
        result.data = state
        self.pub_result.publish(result)

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
