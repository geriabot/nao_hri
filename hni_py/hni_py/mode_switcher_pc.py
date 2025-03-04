import rclpy
from rclpy.node import Node
import os

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')

        self.get_logger().info("Initializing ModeSwitcher pc...")

 
        self.launch_base_node()
        self.launch_experiment()

        self.get_logger().info("ModeSwitcher pc started successfully.")

    def launch_base_node(self):
        self.get_logger().info("Launching base node...")
        os.system("gnome-terminal --title='Teleop Keyboard' -- bash -c 'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=target; exec bash'")

    def launch_experiment(self):
        self.get_logger().info("Launching experiment...")
        os.system("gnome-terminal --tab --title='Experiment PC' -- bash -c 'cd ~/Documents/Cuarto/TFG/nao_ws && ros2 launch hni_cpp experiment_pc_launch.py; exec bash'")

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping ModeSwitcher pc.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()