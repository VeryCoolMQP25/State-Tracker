import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32  # Import the message type for current floor

class GoalPoseFilterNode(Node):
    def __init__(self):
        super().__init__('goal_pose_filter_node')

        # Default current floor to 1
        self.current_floor = 1

        #  subscription for the current floor
        self.floor_subscription = self.create_subscription(
            Int32,  # current_floor is of type Int32
            '/current_floor',
            self.current_floor_callback,
            10
        )

        # subscription for the filtered goal pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/filtered_goal_pose',
            self.filtered_goal_callback,
            10
        )
        
        #  publisher for the goal pose
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def filtered_goal_callback(self, msg):
        self.get_logger().info('Received filtered goal pose: %s' % str(msg))

        # If the current floor matches the floor of the goal pose, publish it to /goal_pose
        if self.current_floor == msg.header.frame_id:  # Assuming floor info is in frame_id or any other field
            self.publisher.publish(msg)
        else:
            # If it doesn't match, publish the goal pose for floor 0
            self.get_logger().info('Publishing goal pose for floor 0')
            self.publish_goal_pose_for_floor_0()

    def current_floor_callback(self, msg):
        # Update the current floor when a message is received on /current_floor topic
        self.current_floor = msg.data
        self.get_logger().info(f'Current floor updated to: {self.current_floor}')

    def publish_goal_pose_for_floor_0(self):
        goal_pose_floor_0 = PoseStamped()
        goal_pose_floor_0.header.stamp = self.get_clock().now().to_msg()
        goal_pose_floor_0.header.frame_id = 'map'
        goal_pose_floor_0.pose.position.x = 0.0
        goal_pose_floor_0.pose.position.y = 0.0
        goal_pose_floor_0.pose.position.z = 0.0
        goal_pose_floor_0.pose.orientation.x = 0.0
        goal_pose_floor_0.pose.orientation.y = 0.0
        goal_pose_floor_0.pose.orientation.z = 0.0
        goal_pose_floor_0.pose.orientation.w = 1.0

        self.publisher.publish(goal_pose_floor_0)
        self.get_logger().info('Published goal pose for floor 0')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
