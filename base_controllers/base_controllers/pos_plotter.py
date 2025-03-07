import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math

class PositionPlotterNode(Node):
    def __init__(self):
        super().__init__('position_plotter_node')

        # Subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'model/free_flyer/odometry',  # Change to the appropriate topic name
            self.odom_callback,
            10  # QoS History Depth
        )

        # Lists to store the x and y positions
        self.x_positions = []
        self.y_positions = []

        # Create the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Position Over Time')

        self.get_logger().info("Position Plotter Node started")

    def odom_callback(self, msg: Odometry):
        # Get current position from odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Append the position to the lists
        self.x_positions.append(current_x)
        self.y_positions.append(current_y)

        # Plot the position in real-time
        self.ax.clear()  # Clear previous plot
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Position Over Time')
        self.ax.plot(self.x_positions, self.y_positions, color='b', marker='o')
        plt.draw()
        plt.pause(0.05)  # Pause to update the plot

    def stop_plotting(self):
        # Stop the plot and close the figure
        plt.close(self.fig)

def main(args=None):
    rclpy.init(args=args)

    node = PositionPlotterNode()

    # Set up matplotlib interactive mode for real-time plotting
    plt.ion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop plotting and close the figure when node is shut down
    node.stop_plotting()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
