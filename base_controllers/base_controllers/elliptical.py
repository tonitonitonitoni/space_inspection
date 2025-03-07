import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import tf2_ros
import tf2_geometry_msgs
from actuator_msgs.msg import Actuators

class EllipticalMotionNode(Node):
    def __init__(self):
        super().__init__('elliptical')


        # Declare parameters
        # Proportional control constants
        
        # run node with ros2 run my_robot position_control_node --ros-args -p kp_linear:=0.8 -p kp_angular:=1.2
        #self.declare_parameter('kp_linear', 0.5)  # Default value for kp_linear
        #self.declare_parameter('kp_angular', 1.0)  # Default value for kp_angular
        #self.declare_parameter('semi_major_axis', 5.0)  # Semi-major axis (a)
        #self.declare_parameter('semi_minor_axis', 3.0)  # Semi-minor axis (b)
        
        # Get parameters
        self.kp_linear = 0.5 #self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_angular = 0.5 #self.get_parameter('kp_angular').get_parameter_value().double_value
        self.a = 5.0  #self.get_parameter('semi_major_axis').get_parameter_value().double_value
        self.b = 3.0  #self.get_parameter('semi_minor_axis').get_parameter_value().double_value
        
        # Initialize the time variable
        self.t = 0.0  # Start from time 0

        # Subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'model/free_flyer/odometry',  # Change to the appropriate topic name
            self.odom_callback,
            10  # QoS History Depth
        )

          # Publisher for velocity control
        self.vel_publisher = self.create_publisher(
            Actuators,
            '/base/command/duty_cycle',  # Change to the appropriate topic name
            10  # QoS History Depth
        )

        # Define the target orientation

        #self.target_x = 5.0  # Target X position in meters
        #self.target_y = 5.0  # Target Y position in meters
        self.target_orientation = 0  # Target orientation (yaw) in radians

        self.get_logger().info("Elliptical Motion Node started")

    def odom_callback(self, msg: Odometry):
        # Get current position from odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Get current orientation from odometry (quaternion)
        orientation_q = msg.pose.pose.orientation
        current_yaw = self.quaternion_to_yaw(orientation_q)

        # Get current velocity from odometry
        linear_velocity_x = msg.twist.twist.linear.x  
        linear_velocity_y = msg.twist.twist.linear.y 
        rotational_velocity = msg.twist.twist.angular.z  
        
        # Compute the target position along the ellipse
        target_x = self.a * math.cos(self.t) -self.a # Parametric equation for x(t)
        target_y = self.b * math.sin(self.t)   # Parametric equation for y(t)

        # Log current position and target position
        self.get_logger().info(f"Current Position: ({current_x:.2f}, {current_y:.2f})")
        self.get_logger().info(f"Target Position: ({target_x:.2f}, {target_y:.2f})")

        # Calculate the distance and angle to the target
        delta_x = target_x - current_x
        delta_y = target_y - current_y

        distance_to_target = math.sqrt(delta_x ** 2 + delta_y ** 2)
        angle_to_target = math.atan2(delta_y, delta_x)

        # Calculate the angular difference between current yaw and target orientation
        angle_diff = self.normalize_angle(self.target_orientation - current_yaw)

        # Log current position, target position, and angle
        #self.get_logger().info(f"Current Position: ({current_x:.2f}, {current_y:.2f})")
        #self.get_logger().info(f"Current Orientation (Yaw): {current_yaw:.2f}")
        #self.get_logger().info(f"Target Position: ({self.target_x:.2f}, {self.target_y:.2f})")
        #self.get_logger().info(f"Target Orientation (Yaw): {self.target_orientation:.2f}")
        #self.get_logger().info(f"Distance to Target: {distance_to_target:.2f}, Angle to Target: {angle_to_target:.2f}")
        #self.get_logger().info(f"Angle Difference to Target: {angle_diff:.2f}")

        # Check if the robot is close enough to the target position and orientation
        #if distance_to_target > 0.1 or abs(angle_diff) > 0.1:  # If not at target yet
            # Compute the control velocities
        linear_velocity_x_cmd = self.kp_linear * delta_x
        linear_velocity_y_cmd = self.kp_linear * delta_y
        angular_velocity_cmd = self.kp_angular * angle_diff
    
        # Use odometry information to compute control velocity
        actuator_msg = Actuators()
        
        # Define the number of actuators
        num_actuators = 8
        
        # Initialize actuator values array with zero values
        actuator_values = [0.0] * num_actuators

        # Simplest control logic

        if linear_velocity_x > linear_velocity_x_cmd:
            actuator_values[4] = 1.0
            actuator_values[5] = 1.0
        if linear_velocity_x < linear_velocity_x_cmd:
            actuator_values[6] = 1.0
            actuator_values[7] = 1.0

        if linear_velocity_y > linear_velocity_y_cmd:
            actuator_values[2] = 1.0
            actuator_values[3] = 1.0
        if linear_velocity_y < linear_velocity_y_cmd:
            actuator_values[0] = 1.0
            actuator_values[1] = 1.0

        # Control angular velocity 
        if rotational_velocity > angular_velocity_cmd:
            actuator_values[1] = 1.0
            actuator_values[2] = 1.0
            actuator_values[5] = 1.0
            actuator_values[6] = 1.0
        if rotational_velocity < angular_velocity_cmd:
            actuator_values[0] = 1.0
            actuator_values[3] = 1.0
            actuator_values[4] = 1.0
            actuator_values[7] = 1.0

        # Publish velocity control
        # Set the actuator values in the message
        actuator_msg.normalized = actuator_values

        # Publish the actuator message
        self.vel_publisher.publish(actuator_msg)

        # Update the time parameter (this will make the robot move in the next position along the ellipse)
        self.t += 0.001  # Increment the time parameter to move along the ellipse path

        if self.t > 2 * math.pi:  # Once the robot has completed a full circle, reset time
            self.t = 0.0

    def quaternion_to_yaw(self, quaternion):
        """
        Convert quaternion (x, y, z, w) to yaw (angle in radians).
        """
        # Extract yaw from the quaternion
        siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        """
        Normalize angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)

    node = EllipticalMotionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
