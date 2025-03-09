import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile
import os
import time
import subprocess

from nao_lola_command_msgs.msg import JointStiffnesses, JointPositions
from biped_interfaces.msg import SolePoses

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')

        self.get_logger().info("Initializing ModeSwitcher nao...")

        self.sub_target = self.create_subscription(Twist, '/target', self.target_callback, 10)

        self.pub_walk_status = self.create_publisher(Bool, '/walk_status', 10)
        self.pub_walk_control = self.create_publisher(Bool, '/walk_control', 10)
        self.pub_stiffness = self.create_publisher(JointStiffnesses, '/effectors/joint_stiffnesses', 10)
        self.pub_initial_joints_position = self.create_publisher(JointPositions, '/effectors/joint_positions', 10)
        self.pub_initial_position = self.create_publisher(SolePoses, '/motion/sole_poses', 10)

        self.pub_action_req = self.create_publisher(String, "action_req_legs", 10)

        self.sub_action_status = self.create_subscription(
            String, "/nao_pos_action/status", self.action_status_callback, QoSProfile(depth=10)
        )

        self.is_walking = False
        self.is_standing = False
        self.swing_completed = False
        self.stand_completed = False

        self.launch_base_nodes()
        self.launch_experiments()
        self.set_initial_positions()
        self.set_initial_status()

        self.get_logger().info("ModeSwitcher nao started successfully.")

    def launch_base_nodes(self):
        self.get_logger().info("Launching base nodes...")

        processes = [
            subprocess.Popen(["ros2", "run", "nao_ik", "nao_ik"]),
            subprocess.Popen(["ros2", "run", "nao_phase_provider", "nao_phase_provider"]),
            subprocess.Popen(["ros2", "run", "walk", "walk"])
        ]

        self.get_logger().info("Base nodes launched.")

    def launch_experiments(self):
        self.get_logger().info("Launching experiments...")

        command = "cd ~/nao_ws && ros2 launch hni_cpp experiment_nao_launch.py"
    
        subprocess.Popen(["bash", "-c", command])

    def set_initial_positions(self):
        self.get_logger().info("Waiting for subscribers to connect for initial positions...")
        while self.pub_initial_joints_position.get_subscription_count() == 0 and rclpy.ok():
            time.sleep(0.5)
        if rclpy.ok():
            msg = JointPositions(indexes=[3, 19, 2, 18], positions=[0.2, -0.2, 1.5, 1.5])
            self.pub_initial_joints_position.publish(msg)
            self.get_logger().info("Initial positions published.")

    def set_initial_status(self):
        self.get_logger().info("Waiting for subscribers to connect for walk status...")
        while self.pub_walk_status.get_subscription_count() == 0 and rclpy.ok():
            time.sleep(0.5)
        if rclpy.ok():
            self.pub_walk_status.publish(Bool(data=False))
            self.get_logger().info("Initial walk status published.")

    def target_callback(self, msg):
        self.get_logger().info("Message received on /target. Evaluating movement...")
        if self.is_moving(msg):
            if not self.is_walking:
                self.get_logger().info("The robot is trying to start walking.")
                self.pub_walk_status.publish(Bool(data=True))
                self.ensure_stand_then_walk()
        else:
            if self.is_walking:
                self.get_logger().info("The robot stops.")
                self.is_walking = False
                self.pub_walk_status.publish(Bool(data=False))
                self.pub_walk_control.publish(Bool(data=False))
                self.is_standing = False
                self.ensure_stand()

    def is_moving(self, msg):
        return any([
            msg.linear.x != 0.0, msg.linear.y != 0.0, msg.linear.z != 0.0,
            msg.angular.x != 0.0, msg.angular.y != 0.0, msg.angular.z != 0.0
        ])

    def ensure_stand_then_walk(self):
        if not self.swing_completed:
            self.get_logger().info("Waiting for 'swing' to finish before executing 'stand'...")
            return
            
        if not self.is_standing:
            self.get_logger().info("Executing 'stand'...")
            self.try_stand()
        else:
            self.start_walking()

    def ensure_stand(self):
        if not self.is_standing:
            self.try_stand()

    def try_stand(self):
        self.get_logger().info("Publishing 'stand' on action_req_legs...")
        msg = String()
        msg.data = "stand"
        self.pub_action_req.publish(msg)

    def action_status_callback(self, msg):
        self.get_logger().info(f"Action status received: {msg.data}")

        if "succeeded" in msg.data.lower():
            if "only_legs" in msg.data.lower():
                self.get_logger().info("Swing finished...")
                self.swing_completed = True
            elif "stand" in msg.data.lower():
                self.get_logger().info("Stand completed...")
                self.stand_completed = True
                self.is_standing = True

    def start_walking(self):
        self.get_logger().info("Starting walk...")
        self.set_stiffness()
        self.set_walk_sole_position()
        # Sleep so that the robot has time to get to the initial position
        time.sleep(1.0)
        self.pub_walk_control.publish(Bool(data=True))
        self.is_walking = True
        self.is_standing = False
        self.swing_completed = False
        self.get_logger().info("Walk started successfully.")

    def set_stiffness(self):
        self.get_logger().info("Setting joint stiffness.")
        msg = JointStiffnesses(indexes=list(range(25)), stiffnesses=[1.0] * 25)
        self.pub_stiffness.publish(msg)
        self.get_logger().info("Stiffness set.")

    def set_walk_sole_position(self):
        self.get_logger().info("Setting initial position.")
        msg = SolePoses()
        msg.l_sole.position.y, msg.l_sole.position.z = 0.05, -0.315
        msg.r_sole.position.y, msg.r_sole.position.z = -0.05, -0.315
        self.pub_initial_position.publish(msg)
        self.get_logger().info("Initial position set.")

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping ModeSwitcher nao.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()