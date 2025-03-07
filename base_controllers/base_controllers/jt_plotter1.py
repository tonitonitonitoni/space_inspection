import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        # Create a publisher that will publish JointTrajectory messages on the topic /joint_trajectory
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Timer to periodically send JointTrajectory messages
        self.timer = self.create_timer(1.0, self.publish_trajectory)

        # Create a subscriber to listen to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Define the joint names for the trajectory
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Define the trajectory points to go through
        self.trajectory_points = [
            [0.0, 0.0, 0.0, 0.0],  # Point 1 (initial position)
            [1.0, 0.5, -0.5, 0.3], # Point 2 (move to this position)
            [0.5, 0.7, -0.7, -0.2],# Point 3 (move to this position)
            [0.8, -0.3, 0.5, 0.5], # Point 4 (move to this position)
            [0.0, 0.0, 0.0, 0.0],  # Point 5 (return to initial position)
        ]

        self.current_point = 0  # Index to keep track of the current point to publish

        # Variable to store the most recent joint states
        self.current_joint_state = JointState()

        # Data to plot (joint positions for plotting)
        self.joint_positions = {joint: [] for joint in self.trajectory.joint_names}

        # Start the plot
        self.fig, self.ax = plt.subplots(len(self.trajectory.joint_names), 1, figsize=(10, 6))
        if len(self.trajectory.joint_names) == 1:
            self.ax = [self.ax]
        for i, joint in enumerate(self.trajectory.joint_names):
            self.ax[i].set_title(f'{joint} Position')
            self.ax[i].set_xlabel('Time (s)')
            self.ax[i].set_ylabel('Position')
            self.ax[i].grid(True)

        # Time tracker for the plot
        self.time_data = []

    def joint_state_callback(self, msg: JointState):
        # Store the most recent joint states (positions, velocities, efforts)
        self.current_joint_state = msg
        self.get_logger().info(f"Received joint states: {msg.position}")

        # Append the joint positions to the plot data
        for i, joint in enumerate(self.trajectory.joint_names):
            self.joint_positions[joint].append(msg.position[i])

        # Append time data (in seconds)
        self.time_data.append(self.get_clock().now().seconds_nanoseconds()[0])

        # Update the plot with new data
        self.update_plot()

    def update_plot(self):
        # Clear previous plots and update with new data
        for i, joint in enumerate(self.trajectory.joint_names):
            self.ax[i].cla()  # Clear the axes
            self.ax[i].plot(self.time_data, self.joint_positions[joint], label=f'{joint} Position')
            self.ax[i].set_title(f'{joint} Position')
            self.ax[i].set_xlabel('Time (s)')
            self.ax[i].set_ylabel('Position')
            self.ax[i].grid(True)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def publish_trajectory(self):
        if self.current_point < len(self.trajectory_points):
            point = JointTrajectoryPoint()
            point.positions = self.trajectory_points[self.current_point]
            point.velocities = [0.0, 0.0, 0.0, 0.0]  # Velocities set to zero
            point.accelerations = [0.0, 0.0, 0.0, 0.0]  # Accelerations set to zero
            point.effort = [0.0, 0.0, 0.0, 0.0]  # Effort set to zero
            point.time_from_start.sec = 2  # Time to reach this position (2 seconds)

            # Set the header for the trajectory message
            self.trajectory.header = Header()
            self.trajectory.header.stamp = self.get_clock().now().to_msg()

            # Append the point to the trajectory
            self.trajectory.points = [point]

            # Publish the trajectory message
            self.publisher.publish(self.trajectory)
            self.get_logger().info(f'Publishing JointTrajectory Point {self.current_point + 1}')

            # Move to the next point in the sequence
            self.current_point += 1
        else:
            self.get_logger().info('Completed the trajectory cycle, starting over.')
            self.current_point = 0  # Restart the trajectory loop

    def start_plotting(self):
        # Set up the plot to start real-time updating
        plt.ion()
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    # Create the publisher node
    joint_trajectory_publisher = JointTrajectoryPublisher()

    # Start plotting
    joint_trajectory_publisher.start_plotting()

    # Spin the node to keep it running
    rclpy.spin(joint_trajectory_publisher)

    # Cleanup on shutdown
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
