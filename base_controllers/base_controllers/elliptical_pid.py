import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
import math
import time

class EllipticalMotionPIDNode(Node):
    def __init__(self):
        super().__init__('elliptical_motion_pid_node')

        # Declare parameters for elliptical trajectory and PID control gains
        self.declare_parameter('semi_major_axis', 5.0)  # Semi-major axis (a)
        self.declare_parameter('semi_minor_axis', 3.0)  # Semi-minor axis (b)
        self.declare_parameter('kp_linear', 0.5)  # Linear proportional gain (for both x and y axes)
        self.declare_parameter('ki_linear', 0.0)  # Linear integral gain
        self.declare_parameter('kd_linear', 0.1)  # Linear derivative gain
        self.declare_parameter('kp_angular', 1.0)  # Angular proportional gain
        self.declare_parameter('ki_angular', 0.0)  # Angular integral gain
        self.declare_parameter('kd_angular', 0.1)  # Angular derivative gain

        # Get parameters
        self.a = self.get_parameter('semi_major_axis').get_parameter_value().double_value
        self.b = self.get_parameter('semi_minor_axis').get_parameter_value().double_value
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.ki_linear = self.get_parameter('ki_linear').get_parameter_value().double_value
        self.kd_linear = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.ki_angular = self.get_parameter('ki_angular').get_parameter_value().double_value
        self.kd_angular = self.get_parameter('kd_angular').get_parameter_value().double_value

        # Initialize the time variable
        self.t = 0.0  # Start from time 0

        # Initialize error terms for PID control
        self.previous_error_x = 0.0
        self.integral_x = 0.0
        self.previous_error_y = 0.0
        self.integral_y = 0.0
        self.previous_error_yaw = 0.0
        self.integral_yaw = 0.0
        
        #Subscribe to ODOM
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'model/free_flyer/odometry',
            self.odom_callback,
            10  # QoS History Depth
        )

        # Publish to ACTUATORS
        self.vel_publisher = self.create_publisher(
            Actuators,
            '/base/command/duty_cycle',  
            10  # QoS History Depth
        )