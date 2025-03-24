import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from topic_names import JOY_BUMPER_TOPIC

CONFIG_PATH = "../config/robot.yaml"
JOY_CONFIG_PATH = "../config/joystick.yaml"

class Joy2Locobot(Node):
    def __init__(self):
        super().__init__("joy2locobot")

        # Load robot and joystick configuration
        with open(CONFIG_PATH, "r") as f:
            robot_config = yaml.safe_load(f)
        with open(JOY_CONFIG_PATH, "r") as f:
            joy_config = yaml.safe_load(f)

        self.MAX_V = 0.4
        self.MAX_W = 0.8
        self.VEL_TOPIC = robot_config["vel_teleop_topic"]

        self.DEADMAN_SWITCH = joy_config["deadman_switch"]
        self.LIN_VEL_BUTTON = joy_config["lin_vel_button"]
        self.ANG_VEL_BUTTON = joy_config["ang_vel_button"]
        
        self.vel_pub = self.create_publisher(Twist, self.VEL_TOPIC, 1)
        self.bumper_pub = self.create_publisher(Bool, JOY_BUMPER_TOPIC, 1)
        self.joy_sub = self.create_subscription(Joy, "joy", self.callback_joy, 1)

        self.vel_msg = Twist()
        self.button = None
        self.bumper = False

        # Timer to periodically publish messages
        self.create_timer(1.0 / 9, self.publish_messages)

        self.get_logger().info("Node initialized. Waiting for joystick input...")

    def callback_joy(self, data: Joy):
        """Joystick callback function"""
        self.button = data.buttons[self.DEADMAN_SWITCH]
        bumper_button = data.buttons[self.DEADMAN_SWITCH - 1]

        if self.button is not None:  # Deadman switch must be held to teleop
            self.vel_msg.linear.x = self.MAX_V * data.axes[self.LIN_VEL_BUTTON]
            self.vel_msg.angular.z = self.MAX_W * data.axes[self.ANG_VEL_BUTTON]
        else:
            self.vel_msg = Twist()  # Stop motion

        if bumper_button is not None:
            self.bumper = bool(bumper_button)
        else:
            self.bumper = False

    def publish_messages(self):
        """Periodically publish velocity and bumper state"""
        if self.button:
            self.get_logger().info(f"Teleoperating the robot:\n {self.vel_msg}")
            self.vel_pub.publish(self.vel_msg)

        bumper_msg = Bool()
        bumper_msg.data = self.bumper
        self.bumper_pub.publish(bumper_msg)

        if self.bumper:
            self.get_logger().info("Bumper pressed!")

def main(args=None):
    rclpy.init(args=args)
    node = Joy2Locobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

