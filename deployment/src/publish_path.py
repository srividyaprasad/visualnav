import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  
        )
        
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        
        self.path = Path()
        self.path.header.frame_id = 'map'

    def odom_callback(self, msg: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg() 
        pose_stamped.header.frame_id = msg.header.frame_id
        pose_stamped.pose = msg.pose.pose  
        
        self.path.poses.append(pose_stamped)
        
        self.path_publisher.publish(self.path)
        self.get_logger().info(f'Published path with {len(self.path.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    
    node = PathPublisherNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
