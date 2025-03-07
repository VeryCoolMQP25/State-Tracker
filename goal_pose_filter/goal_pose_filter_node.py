import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32  # Import the message type for current floor

class GoalPoseFilterNode(Node):
    def __init__(self):
        super().__init__('goal_pose_filter_node')

        # Default current & requested floor to 1

        self.requested_floor = 1
        self.current_floor = 1

        #  subscription for the current floor

        self.floor_subscription = self.create_subscription(
            Int32,  # current_floor is of type Int32
            '/current_floor',
            self.current_floor_callback,
            10
        )

        self.floor_subscription = self.create_subscription(
            Int32,  # current_floor is of type Int32
            '/requested_floor',
            self.requested_floor_callback,
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
        if self.current_floor == self.requested_floor:  
            self.publisher.publish(msg)
        else:
            # If it doesn't match, publish the goal pose for floor 0
            self.get_logger().info('Publishing goal pose for this floor elevator')
            self.publish_goal_pose_for_floor(self.current_floor)

    def requested_floor_callback(self, msg):
        # Update the current floor when a message is received on /current_floor topic
        self.requested_floor = msg.data
        self.get_logger().info(f'Requested floor updated to: {self.requested_floor}')

    def current_floor_callback(self, msg):
        # Update the current floor when a message is received on /current_floor topic
        self.current_floor = msg.data
        self.get_logger().info(f'Current floor updated to: {self.current_floor}')



    def publish_goal_pose_for_floor(self, floor):
        """Publish the goal pose for current floor elevator."""
        
        floor_coordinates = {
            1: {'x': 5.2, 'y': 3.61, 'z': 0.0, 'orientationZ': 0.0, 'orientationW': 1.0},
            2: {'x': 5.0, 'y': 26.7, 'z': 0.0, 'orientationZ': 0.0, 'orientationW': 1.0},
            3: {'x': 17.9, 'y': 10.5, 'z': 0.0, 'orientationZ': 0.0, 'orientationW': 1.0},
            4: {'x': 18.9, 'y': .243, 'z': 0.0, 'orientationZ': 0.0, 'orientationW': 1.0},
            5: {'x': 35.4, 'y': -1.9, 'z': 0.0, 'orientationZ': 0.0, 'orientationW': 1.0},
        }

        # Ensure the requested floor is valid (between 1 and 5)
        if floor in floor_coordinates:
            coordinates = floor_coordinates[floor]
            
            # Prepare the goal pose message
            goal_message = PoseStamped()
            goal_message.header.stamp = self.get_clock().now().to_msg()
            goal_message.header.frame_id = "map"
            goal_message.pose.position.x = coordinates['x']
            goal_message.pose.position.y = coordinates['y']
            goal_message.pose.position.z = coordinates['z']
            goal_message.pose.orientation.x = 0.0
            goal_message.pose.orientation.y = 0.0
            goal_message.pose.orientation.z = coordinates['orientationZ']
            goal_message.pose.orientation.w = coordinates['orientationW']
            
            # Publish the goal pose message
            self.publisher.publish(goal_message)
            self.get_logger().info(f'Published goal pose for floor {floor} elevator: {coordinates}')
        else:
            self.get_logger().warning(f"Invalid floor: {floor}. No coordinates found.")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
