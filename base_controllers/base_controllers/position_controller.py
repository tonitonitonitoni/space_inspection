import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import tf2_ros
import tf2_geometry_msgs
from actuator_msgs.msg import Actuators

class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')

        # Declare parameters -later
        # Proportional control constants
        #self.kp_linear = 0.5  # Linear proportional gain
        #self.kp_angular = 1.0  # Angular proportional gain
        
        # run node with ros2 run my_robot position_control_node --ros-args -p kp_linear:=0.8 -p kp_angular:=1.2
        self.declare_parameter('kp_linear', 0.5)  # Default value for kp_linear
        self.declare_parameter('kp_angular', 1.0)  # Default value for kp_angular

        # Get parameters
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        
        
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

        # Define the target position
        self.target_x = 5.0  # Target X position in meters
        self.target_y = 5.0  # Target Y position in meters
        self.target_orientation = math.pi/2  # Target orientation (yaw) in radians

        #self.command_pos = [self.target_x, self.target_y, self.target_orientation]

        self.get_logger().info("Position Control Node started")

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
        
        # Calculate the distance and angle to the target
        delta_x = self.target_x - current_x
        delta_y = self.target_y - current_y

        distance_to_target = math.sqrt(delta_x ** 2 + delta_y ** 2)
        angle_to_target = math.atan2(delta_y, delta_x)

        # Calculate the angular difference between current yaw and target orientation
        angle_diff = self.normalize_angle(self.target_orientation - current_yaw)

        # Log current position, target position, and angle
        self.get_logger().info(f"Current Position: ({current_x:.2f}, {current_y:.2f})")
        self.get_logger().info(f"Current Orientation (Yaw): {current_yaw:.2f}")
        self.get_logger().info(f"Target Position: ({self.target_x:.2f}, {self.target_y:.2f})")
        self.get_logger().info(f"Target Orientation (Yaw): {self.target_orientation:.2f}")
        self.get_logger().info(f"Distance to Target: {distance_to_target:.2f}, Angle to Target: {angle_to_target:.2f}")
        self.get_logger().info(f"Angle Difference to Target: {angle_diff:.2f}")

        # Check if the robot is close enough to the target position and orientation
        if distance_to_target > 0.1 or abs(angle_diff) > 0.1:  # If not at target yet
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

            # Simple control logic

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
        else:
            # If the robot has reached the target, stop moving
            self.get_logger().info("Target reached! Stopping.")
            self.stop_robot()

    def stop_robot(self):
        # Publish a zero velocity to stop the robot
        stop_msg = Actuators()
        num_actuators = 8
        actuator_values = [0.0] * num_actuators
        stop_msg.normalized = actuator_values
        self.vel_publisher.publish(stop_msg)

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

    node = PositionControlNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
