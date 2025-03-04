import rclpy, os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower')

        # Declare and get the model path parameter
        package_dir = get_package_share_directory('vedita_bot_addon')
        self.declare_parameter('model_path', f"{os.path.join(package_dir, 'models', 'yolov11n-face.pt')}")
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # ROS2 Subscribers & Publishers
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/face_detector/image', 10)  # New topic for processed images

        # Utilities
        self.bridge = CvBridge()
        self.model = YOLO(model_path)  # Load model dynamically

        # Distance thresholds
        self.min_distance = 100  # Too close
        self.max_distance = 300  # Too far
        self.image_width = 640  # Adjust if needed

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)

        best_face = None
        best_x, best_w = None, None

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box
                face_center_x = (x1 + x2) // 2
                face_width = x2 - x1

                if best_face is None or face_width > best_w:
                    best_face = (x1, y1, x2, y2)
                    best_x, best_w = face_center_x, face_width

        if best_face:
            cv2.rectangle(frame, (best_face[0], best_face[1]), (best_face[2], best_face[3]), (0, 255, 0), 2)
            self.control_robot(best_x, best_w)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def control_robot(self, face_x, face_w):
        twist = Twist()
        image_center = self.image_width // 2  

        if face_x < image_center - 50:
            twist.angular.z = 0.3  
        elif face_x > image_center + 50:
            twist.angular.z = -0.3  

        if face_w < self.min_distance:
            twist.linear.x = 0.2  
        elif face_w > self.max_distance:
            twist.linear.x = -0.2  

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
