import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')

        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)

        # Publishers for each ultrasonic sensor with their respective topics
        self.fr_mid_publisher = self.create_publisher(Range, 'fr_mid_range', 10)
        self.fr_left_publisher = self.create_publisher(Range, 'fr_left_range', 10)
        self.fr_right_publisher = self.create_publisher(Range, 'fr_right_range', 10)
        self.bc_mid_publisher = self.create_publisher(Range, 'bc_mid_range', 10)

        self.frame_ids = {
            'fr_mid': 'fr_mid_us_frame',
            'fr_left': 'fr_left_us_frame',
            'fr_right': 'fr_right_us_frame',
            'bc_mid': 'bc_mid_us_frame',
        }

        self.timer = self.create_timer(0.1, self.read_sensor_data)

    def read_sensor_data(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                sensor_id, distance_str = line.split(':')
                distance = float(distance_str)

                # Check which sensor to publish data for
                if sensor_id == 'fr_mid':
                    publisher = self.fr_mid_publisher
                elif sensor_id == 'fr_left':
                    publisher = self.fr_left_publisher
                elif sensor_id == 'fr_right':
                    publisher = self.fr_right_publisher
                elif sensor_id == 'bc_mid':
                    publisher = self.bc_mid_publisher
                else:
                    return  # Ignore any unrecognized sensor

                # Create and populate the Range message
                msg = Range()
                msg.header.frame_id = self.frame_ids[sensor_id]
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.26  # Adjust based on HC-SR04 spec
                msg.min_range = 0.02  # 2 cm
                msg.max_range = 4.0  # 400 cm
                msg.range = distance if msg.min_range <= distance <= msg.max_range else float('inf')

                # Publish the data
                publisher.publish(msg)
                self.get_logger().info(f'Published {sensor_id}: {distance:.2f} cm')

        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
