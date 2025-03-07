import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist

class OdometryVelocityController(Node):
    def __init__(self):
        super().__init__('odometry_velocity_controller')
        self.command_vel=[0, 0, 0]
        
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

        # Subscriber for cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',  
            self.cmd_vel_callback,
            10  # QoS History Depth
        )

        self.get_logger().info("Odometry Velocity Controller node started")

    def cmd_vel_callback(self, msg: Twist):
        
        linear_velocity_x_cmd = msg.linear.x
        linear_velocity_y_cmd = msg.linear.y
        angular_velocity_cmd = msg.angular.z
        self.command_vel[0] = linear_velocity_x_cmd
        self.command_vel[1] = linear_velocity_y_cmd
        self.command_vel[2] = angular_velocity_cmd

    def odom_callback(self, msg: Odometry):
        # Extracting linear velocity from Odometry message
        linear_velocity_x = msg.twist.twist.linear.x  # Change to other components if needed
        linear_velocity_y = msg.twist.twist.linear.y 
        rotational_velocity = msg.twist.twist.angular.z  # Change to other components if needed

        # Use odometry information to compute control velocity
        actuator_msg = Actuators()
        
        # Define the number of actuators
        num_actuators = 8
        
        # Initialize actuator values array with zero values
        actuator_values = [0.0] * num_actuators

        # Simple control logic

        if linear_velocity_x > self.command_vel[0]:
            actuator_values[4] = 1.0
            actuator_values[5] = 1.0
        if linear_velocity_x < self.command_vel[0]:
            actuator_values[6] = 1.0
            actuator_values[7] = 1.0

        if linear_velocity_y > self.command_vel[1]:
            actuator_values[2] = 1.0
            actuator_values[3] = 1.0
        if linear_velocity_y < self.command_vel[1]:
            actuator_values[0] = 1.0
            actuator_values[1] = 1.0

        # Control angular velocity 
        if rotational_velocity > self.command_vel[2]:
            actuator_values[1] = 1.0
            actuator_values[2] = 1.0
            actuator_values[5] = 1.0
            actuator_values[6] = 1.0
        if rotational_velocity < self.command_vel[2]:
            actuator_values[0] = 1.0
            actuator_values[3] = 1.0
            actuator_values[4] = 1.0
            actuator_values[7] = 1.0

        # Publish velocity control
       # Set the actuator values in the message
        actuator_msg.normalized = actuator_values

        # Publish the actuator message
        self.vel_publisher.publish(actuator_msg)
        self.get_logger().info(f'Requested: {self.command_vel} true: x: {linear_velocity_x:.2f} y: {linear_velocity_y:.2f} z: {rotational_velocity:.2f}')

    
def main(args=None):
    rclpy.init(args=args)

    node = OdometryVelocityController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
