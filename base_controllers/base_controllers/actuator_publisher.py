import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from actuator_msgs.msg import Actuators

class CmdVelToActuatorsNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_actuators')

        # Create a subscriber for cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create a publisher for actuator_msgs/Actuators
        self.actuator_publisher = self.create_publisher(
            Actuators,
            '/base/command/duty_cycle',
            10
        )

    def cmd_vel_callback(self, msg):
        # Create the Actuators message from the received cmd_vel
        actuator_msg = Actuators()
        
        # Define the number of actuators
        num_actuators = 8
        
        # Initialize actuator values array with zero values
        actuator_values = [0.0] * num_actuators
        
        # Example of how to map the cmd_vel values to actuator values
        # Normalize the linear and angular velocities into a range [0, 1]
        
        # Map linear velocity (x) to first actuator values
        # Assuming linear velocity is between -1 m/s and 1 m/s for normalization
        linear_x_velocity = msg.linear.x  # Clamp the value to [-1, 1]
        linear_y_velocity = msg.linear.y
        rotational_velocity = msg.angular.z

        if linear_x_velocity < -0.1:
            actuator_values[4] = 1.0
            actuator_values[5] = 1.0
        if linear_x_velocity > 0.1:
            actuator_values[6] = 1.0
            actuator_values[7] = 1.0


        if linear_y_velocity < -0.1:
            actuator_values[2] = 1.0
            actuator_values[3] = 1.0
        if linear_y_velocity > 0.1:
            actuator_values[0] = 1.0
            actuator_values[1] = 1.0
        
        if rotational_velocity > 0.1: #clockwise
            actuator_values[1] = 1.0
            actuator_values[2] = 1.0
            actuator_values[5] = 1.0
            actuator_values[6] = 1.0
        if rotational_velocity < -0.1: #counterclockwise
            actuator_values[0] = 1.0
            actuator_values[3] = 1.0
            actuator_values[4] = 1.0
            actuator_values[7] = 1.0

        # Set the actuator values in the message
        actuator_msg.normalized = actuator_values

        # Publish the actuator message
        self.actuator_publisher.publish(actuator_msg)

        self.get_logger().info(f'Received message: x {msg.linear.x}, y {msg.linear.y}, angular {msg.angular.z}')
        self.get_logger().info(f'Published actuators: {actuator_values}')
def main(args=None):
    rclpy.init(args=args)

    node = CmdVelToActuatorsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()