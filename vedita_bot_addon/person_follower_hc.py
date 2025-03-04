import cv2.data
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower_hc')
        
        # Declare parameters
        self.declare_parameter('cascade_path', cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_size', [40, 40])
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_gain', 0.005)
        self.declare_parameter('detection_threshold', 150)
        
        # Load Haar cascade for face detection
        cascade_path = self.get_parameter('cascade_path').get_parameter_value().string_value
        self.haar_cascade = cv2.CascadeClassifier(cascade_path)
        
        # ROS2 Subscribers & Publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw/uncompressed', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # OpenCV Bridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Get parameters
        scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value
        min_neighbors = self.get_parameter('min_neighbors').get_parameter_value().integer_value
        min_size = tuple(self.get_parameter('min_size').get_parameter_value().integer_array_value)
        linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        detection_threshold = self.get_parameter('detection_threshold').get_parameter_value().integer_value
        
        # Detect faces
        faces = self.haar_cascade.detectMultiScale(gray, scaleFactor=scale_factor, minNeighbors=min_neighbors, minSize=min_size)
        
        if len(faces) > 0:
            # Select the largest detected face (closest one)
            x, y, w, h = max(faces, key=lambda b: b[2] * b[3])
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Compute movement command
            twist = Twist()
            center_x = x + w // 2
            img_center_x = frame.shape[1] // 2
            
            # Turn robot based on position
            twist.angular.z = -angular_gain * (center_x - img_center_x)
            
            # Move forward if face is far
            twist.linear.x = linear_speed if w < detection_threshold else 0.0
            
            self.cmd_vel_pub.publish(twist)
        else:
            # Stop robot if no face detected
            self.cmd_vel_pub.publish(Twist())
        
        cv2.imshow("Face Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
