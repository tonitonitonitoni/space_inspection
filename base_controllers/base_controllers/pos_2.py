import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from actuator_msgs.msg import Actuators
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, threshold=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.threshold = threshold  # The minimum control value required to activate an actuator
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time if current_time - self.prev_time > 0 else 1e-3
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        # PID output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update previous values for next calculation
        self.prev_error = error
        self.prev_time = current_time

        return output

    def should_activate(self, output):
        # If the output exceeds the threshold, the actuator should be activated (set to 1)
        return 1 if abs(output) > self.threshold else 0

class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')

        # PID control constants for linear and angular velocities
        self.pid_linear = PIDController(Kp=0.5, Ki=0.0, Kd=0.02)
        self.pid_angular = PIDController(Kp=0.1, Ki=0.0, Kd=0.01)

        # Subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'model/free_flyer/odometry',  
            self.odom_callback,
            10  # QoS History Depth
        )

          # Publisher for velocity control
        self.vel_publisher = self.create_publisher(
            Actuators,
            '/base/command/duty_cycle',  
            10  # QoS History Depth
        )

        # Define the target position
        self.target_x = 5.0  # Target X position in meters
        self.target_y = 5.0  # Target Y position in meters
        self.target_orientation = math.pi/2  # Target orientation (yaw) in radians

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
            linear_velocity_x_cmd = self.pid_linear.update(delta_x)
            linear_velocity_y_cmd = self.pid_linear.update(delta_y)
            angular_velocity_cmd = self.pid_angular.update(angle_diff)
        
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
