import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import time

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower')

        # ROS2 Subscribers & Publishers
        self.subscription = self.create_subscription(Image, '/camera/image_raw/uncompressed', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/camera/person_detector', 10)

        # Utilities
        self.bridge = CvBridge()
        self.declare_parameter("model_path", "yolov8n.pt")
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.model = YOLO(model_path)

        # Distance thresholds
        self.declare_parameter('min_distance', 100.0)
        self.declare_parameter('max_distance', 300.0)
        self.declare_parameter('image_width', 640.0)

        # Timeout handling
        self.last_seen_time = time.time()
        self.timeout_secs = 1.5  # Stop if no person is detected within this time

    def image_callback(self, msg):
        # Convert ROS2 Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8 detection
        results = self.model(frame)

        best_person = None
        best_x, best_w = None, None

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])  # Class ID
                if cls == 0:  # Class 0 is "person" in COCO dataset
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box
                    person_center_x = (x1 + x2) // 2
                    person_width = x2 - x1

                    # Choose the largest person (closest)
                    if best_person is None or person_width > best_w:
                        best_person = (x1, y1, x2, y2)
                        best_x, best_w = person_center_x, person_width

        if best_person:
            # Draw bounding box around detected person
            cv2.rectangle(frame, (best_person[0], best_person[1]), (best_person[2], best_person[3]), (0, 255, 0), 2)

            # Control robot movement
            self.control_robot(best_x, best_w)

            # Update last seen time
            self.last_seen_time = time.time()
        else:
            # Stop if person is lost for too long
            if time.time() - self.last_seen_time > self.timeout_secs:
                self.stop_robot()

        # Publish processed image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def control_robot(self, person_x, person_w):
        """ Controls the robot based on person position and size. """
        twist = Twist()
        
        min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        image_width = self.get_parameter('image_width').get_parameter_value().double_value
        image_center = image_width // 2  


        # Adjust rotation
        if person_x < image_center - 50:
            twist.angular.z = 0.3  # Turn left
        elif person_x > image_center + 50:
            twist.angular.z = -0.3  # Turn right

        # Adjust forward/backward movement
        if person_w < min_distance:
            twist.linear.x = 0.2  # Move forward
        elif person_w > max_distance:
            twist.linear.x = -0.2  # Move backward

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """ Stops the robot when no person is detected. """
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
