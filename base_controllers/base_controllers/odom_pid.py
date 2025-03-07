import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
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

class OdometryVelocityPIDController(Node):
    def __init__(self):
        super().__init__('odometry_velocity_PID_controller')
        self.command_vel = [0, 0, 0]
        
        # Create PID controllers for linear and angular velocities
        self.pid_linear_x = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)
        self.pid_linear_y = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)
        self.pid_angular_z = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)

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

        # Subscriber for cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',  # Change to the appropriate topic name
            self.cmd_vel_callback,
            10  # QoS History Depth
        )

        self.get_logger().info("Odometry Velocity Controller node started")

    def cmd_vel_callback(self, msg: Twist):
        # Received cmd_vel, here you can implement how to process the velocity commands
        self.command_vel[0] = msg.linear.x  # Linear velocity in x
        self.command_vel[1] = msg.linear.y  # Linear velocity in y
        self.command_vel[2] = msg.angular.z  # Angular velocity (z-axis)

    def odom_callback(self, msg: Odometry):
        # Extracting current velocities from Odometry message
        linear_velocity_x = msg.twist.twist.linear.x
        linear_velocity_y = msg.twist.twist.linear.y
        angular_velocity_z = msg.twist.twist.angular.z

        # Calculate errors for each component
        error_linear_x = self.command_vel[0] - linear_velocity_x
        error_linear_y = self.command_vel[1] - linear_velocity_y
        error_angular_z = self.command_vel[2] - angular_velocity_z

        # Get PID control outputs for each component
        control_linear_x = self.pid_linear_x.update(error_linear_x)
        control_linear_y = self.pid_linear_y.update(error_linear_y)
        control_angular_z = self.pid_angular_z.update(error_angular_z)

        # Use PID output to control actuators
        actuator_msg = Actuators()
        actuator_values = [0.0] * 8  # Assume there are 8 actuators

        # Control linear x direction
        if control_linear_x > 0.01:
            actuator_values[4] = 1.0
            actuator_values[5] = 1.0
        if control_linear_x < -0.01:
            actuator_values[6] = 1.0
            actuator_values[7] = 1.0

        # Control linear y direction
        if control_linear_y > 0.01:
            actuator_values[2] = 1.0
            actuator_values[3] = 1.0
        if control_linear_y < -0.01:
            actuator_values[0] = 1.0
            actuator_values[1] = 1.0

        # Control angular z direction (rotational velocity)
        if control_angular_z > 0.01:
            actuator_values[1] = 1.0
            actuator_values[2] = 1.0
            actuator_values[5] = 1.0
            actuator_values[6] = 1.0
        if control_angular_z < -0.01:
            actuator_values[0] = 1.0
            actuator_values[3] = 1.0
            actuator_values[4] = 1.0
            actuator_values[7] = 1.0

        # Publish actuator commands
        actuator_msg.normalized = actuator_values
        self.vel_publisher.publish(actuator_msg)

        # Log information for debugging
        self.get_logger().info(f'Requested: {self.command_vel} True: x: {linear_velocity_x:.2f} y: {linear_velocity_y:.2f} z: {angular_velocity_z:.2f}')
        self.get_logger().info(f'Published actuators: {actuator_values}')

def main(args=None):
    rclpy.init(args=args)

    node = OdometryVelocityPIDController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
