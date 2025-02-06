import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribing to LiDAR and Ultrasonic sensors
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.ultrasonic_subs = {
            'fr_mid_us_frame': self.create_subscription(Range, '/fr_mid_range', self.ultrasonic_callback, 10),
            'fr_left_us_frame': self.create_subscription(Range, '/fr_left_range', self.ultrasonic_callback, 10),
            'fr_right_us_frame': self.create_subscription(Range, '/fr_right_range', self.ultrasonic_callback, 10),
            'bc_mid_us_frame': self.create_subscription(Range, '/bc_mid_range', self.ultrasonic_callback, 10),
        }

        self.ultrasonic_data = {
            'fr_mid_us_frame': None,
            'fr_left_us_frame': None,
            'fr_right_us_frame': None,
            'bc_mid_us_frame': None
        }

        self.scan_pub = self.create_publisher(LaserScan, '/scan_fused', 10)

        # Define ultrasonic sensor positions in degrees
        self.ultrasonic_angles = {
            'fr_mid_us_frame': 0,
            'fr_left_us_frame': 15,
            'fr_right_us_frame': -15,
            'bc_mid_us_frame': 180
        }

    def ultrasonic_callback(self, msg):
        frame_id = msg.header.frame_id

        # Cap the range if greater than 5 meters
        if msg.range > 4.0:
            # self.get_logger().info(f"Range exceeded 4.0m. Setting to 4.0m.")
            msg.range = 4.0  # Cap the range to 5.0 meters

        if frame_id in self.ultrasonic_data:
            self.ultrasonic_data[frame_id] = msg.range
            # self.get_logger().info(f"Received {frame_id}: {msg.range} meters")
        # else:
            # self.get_logger().warn(f"Unknown ultrasonic sensor frame ID: {frame_id}")

    def lidar_callback(self, msg):
        # Clone the incoming scan message
        fused_scan = msg
        ranges = list(msg.ranges)

        # Modify ranges based on ultrasonic sensors
        for sensor, distance in self.ultrasonic_data.items():
            if distance is not None:
                # Get the angle and convert to index
                angle = self.ultrasonic_angles[sensor]
                start_angle = angle - 7.5  # 15-degree horizontal spread (7.5 degrees to the left)
                end_angle = angle + 7.5    # 15-degree horizontal spread (7.5 degrees to the right)
                
                # Update the range for each angle in the 15-degree spread
                for angle_deg in np.arange(start_angle, end_angle, 0.5):  # Adjust the step size for resolution
                    index = self.angle_to_index(angle_deg, msg.angle_min, msg.angle_increment, len(ranges))
                    
                    # Replace the LiDAR value if ultrasonic detects a closer object
                    if distance < ranges[index]:
                        ranges[index] = distance

        fused_scan.ranges = ranges
        self.scan_pub.publish(fused_scan)

    def angle_to_index(self, angle, angle_min, angle_increment, num_ranges):
        """
        Convert a given angle (degrees) to an index in the LaserScan ranges array.
        """
        angle_rad = np.deg2rad(angle)
        index = int((angle_rad - angle_min) / angle_increment)
        return max(0, min(index, num_ranges - 1))

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
