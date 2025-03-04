import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower')

        # ROS2 Subscribers & Publishers
        self.subscription = self.create_subscription(Image, '/camera/image_raw/uncompressed', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/face_detector/image', 10)  # New topic for processed images

        # Utilities
        self.bridge = CvBridge()
        self.model = YOLO('yolov11n-face.pt')  # Ensure model file exists

        # Distance thresholds
        self.min_distance = 100  # Too close
        self.max_distance = 300  # Too far
        self.image_width = 640  # Adjust if needed

    def image_callback(self, msg):
        # Convert ROS2 Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8 face detection
        results = self.model(frame)

        best_face = None
        best_x, best_w = None, None

        # Select the largest detected face (closest person)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box
                face_center_x = (x1 + x2) // 2
                face_width = x2 - x1

                # Choose the largest face (closest person)
                if best_face is None or face_width > best_w:
                    best_face = (x1, y1, x2, y2)
                    best_x, best_w = face_center_x, face_width

        if best_face:
            # Draw bounding box around detected face
            cv2.rectangle(frame, (best_face[0], best_face[1]), (best_face[2], best_face[3]), (0, 255, 0), 2)

            # Control robot movement
            self.control_robot(best_x, best_w)

        # Publish processed image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def control_robot(self, face_x, face_w):
        """ Controls the robot based on face position and size. """
        twist = Twist()
        image_center = self.image_width // 2  # Assuming a 640px wide image
        
        # Adjust rotation
        if face_x < image_center - 50:
            twist.angular.z = 0.3  # Turn left
        elif face_x > image_center + 50:
            twist.angular.z = -0.3  # Turn right

        # Adjust forward/backward movement
        if face_w < self.min_distance:
            twist.linear.x = 0.2  # Move forward
        elif face_w > self.max_distance:
            twist.linear.x = -0.2  # Move backward

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
