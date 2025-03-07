import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
import numpy as np
import math
from scipy.linalg import solve_continuous_are

class LQRControlNode(Node):
    def __init__(self):
        super().__init__('lqr_control_node')
        
        # Subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'model/free_flyer/odometry',  
            self.odom_callback,
            10
        )

        # Publisher for actuator control
        self.vel_publisher = self.create_publisher(
            Actuators,
            '/base/command/duty_cycle',  
            10
        )
        
        # Target position and orientation
        self.target_x = 5.0
        self.target_y = 5.0
        self.target_orientation = math.pi / 2

        # LQR Gains and System Matrices
        self.Q = np.diag([1, 1, 10, 1, 1, 10])  # Cost on states
        self.R = np.diag([0.1]*8)  # Cost on control inputs

        # State-Space Matrices 
        self.A = np.zeros((6, 6))  
        self.A[0:3,3:6]=np.eye(3,3)
        self.B = np.zeros((6, 8))
        self.B[3,4:8] = [0.2, 0.2, -0.2, -0.2]
        self.B[4,0:4] = 0.2*np.array([-1, -1, 1, 1])
        self.B[5,:]=0.01*np.array([5, -5, -5, 5, 3, -3, -3, 3])

        # Solve Continuous Algebraic Riccati Equation (CARE) to get optimal gain matrix K
        self.K = self.compute_lqr_gain(self.A, self.B, self.Q, self.R)

        self.get_logger().info("LQR Control Node started")

    def odom_callback(self, msg: Odometry):
        # Get the current state (position, velocity, orientation)
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        current_yaw = self.quaternion_to_yaw(orientation_q)
        velocity_x = msg.twist.twist.linear.x
        velocity_y = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z

        # Compute the state vector
        state = np.array([current_x - self.target_x,  # x error
                          current_y - self.target_y,  # y error
                          self.normalize_angle(current_yaw - self.target_orientation),  # angle error
                          velocity_x,  # x velocity
                          velocity_y,  # y velocity
                          angular_velocity])  # angular velocity

        # Compute control input using LQR
        control_input = -np.dot(self.K, state)

        # Convert control input to actuator values
        actuator_values = self.convert_control_to_actuator(control_input)

        # Publish the control inputs to actuators
        actuator_msg = Actuators()
        actuator_msg.normalized = actuator_values
        self.vel_publisher.publish(actuator_msg)

    def compute_lqr_gain(self, A, B, Q, R):
        """
        Compute the LQR gain matrix K using the continuous-time algebraic Riccati equation (CARE).
        """
        P = solve_continuous_are(A, B, Q, R)
        K = -np.linalg.inv(R).dot(B.T).dot(P)
        return K

    def convert_control_to_actuator(self, control_input):
        """
        Convert the LQR control input to actuator values.
        Since actuators can only be 0 or 1, we need to map the control input.
        """
        # Attempt to threshold the control inputs
        actuator_values = np.clip(control_input, 0, 1)
        return actuator_values.tolist()

    def normalize_angle(self, angle):
        """
        Normalize angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def quaternion_to_yaw(self, quaternion):
        """
        Convert quaternion (x, y, z, w) to yaw (angle in radians).
        """
        siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny, cosy)

def main(args=None):
    rclpy.init(args=args)

    node = LQRControlNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
