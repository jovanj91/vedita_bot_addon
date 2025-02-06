import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class Nav2GoalSender(Node):
    def __init__(self, goals):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = goals
        self.current_goal_index = 0

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals completed!')
            rclpy.shutdown()
            return
        
        x, y, z, qx, qy, qz, qw = self.goals[self.current_goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal {self.current_goal_index + 1}/{len(self.goals)}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} rejected :(')
            self.current_goal_index += 1
            self.send_next_goal()
            return
        
        self.get_logger().info(f'Goal {self.current_goal_index + 1} accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result for goal {self.current_goal_index + 1}: {result}')
        self.current_goal_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    
    # List of goals: (x, y, z, qx, qy, qz, qw)
    goals = [
        (5.2, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        (7.9, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    ]
    
    goal_sender = Nav2GoalSender(goals)
    goal_sender.send_next_goal()
    rclpy.spin(goal_sender)

if __name__ == '__main__':
    main()
