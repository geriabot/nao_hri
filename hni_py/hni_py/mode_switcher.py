import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import os
import time

from nao_lola_command_msgs.msg import JointStiffnesses, JointPositions
from biped_interfaces.msg import SolePoses
from nao_pos_interfaces.action import PosPlay

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')

        self.sub_target = self.create_subscription(
            Twist, '/target', self.target_callback, 10
        )

        self.pub_walk_status = self.create_publisher(Bool, '/walk_status', 10)
        self.pub_walk_control = self.create_publisher(Bool, '/walk_control', 10)
        self.pub_stiffness = self.create_publisher(JointStiffnesses, '/effectors/joint_stiffnesses', 10)
        self.pub_initial_joints_position = self.create_publisher(JointPositions, '/effectors/joint_positions', 10)
        self.pub_initial_position = self.create_publisher(SolePoses, '/motion/sole_poses', 10)

        self.action_client = ActionClient(self, PosPlay, '/nao_pos_action_legs')

        self.is_walking = False
        self.is_standing = False
        self.stand_attempts = 0
        self.stand_timer = None

        self.launch_base_nodes()
        self.launch_experiments()
        self.set_initial_positions()

    def launch_base_nodes(self):
        os.system("gnome-terminal --tab --title='Nao IK' -- bash -c 'ros2 run nao_ik nao_ik; exec bash'")
        os.system("gnome-terminal --tab --title='Nao Phase Provider' -- bash -c 'ros2 run nao_phase_provider nao_phase_provider --ros-args -r fsr:=/sensors/fsr; exec bash'")
        os.system("gnome-terminal --tab --title='Walk' -- bash -c 'ros2 run walk walk; exec bash'")
        os.system("gnome-terminal --title='Teleop Keyboard' -- bash -c 'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=target; exec bash'")

    def launch_experiments(self):
        os.system("gnome-terminal --tab --title='Experiment PC' -- bash -c 'cd ~/Documents/Cuarto/TFG/nao_ws && ros2 launch hni_cpp experiment_pc_launch.py; exec bash'")
        os.system("gnome-terminal --tab --title='Experiment NAO' -- bash -c 'cd ~/Documents/Cuarto/TFG/nao_ws && ros2 launch hni_cpp experiment_nao_launch.py; exec bash'")

    def set_initial_positions(self):
        while self.pub_initial_joints_position.get_subscription_count() == 0 and rclpy.ok():
            time.sleep(0.5)

        if rclpy.ok():
            msg = JointPositions(indexes=[3, 19, 2, 18], positions=[0.2, -0.2, 1.5, 1.5])
            self.pub_initial_joints_position.publish(msg)

    def target_callback(self, msg):
        if self.is_moving(msg):
            if not self.is_walking:
                self.is_walking = True
                self.ensure_stand_then_walk()
        else:
            if self.is_walking:
                self.is_walking = False
                self.pub_walk_status.publish(Bool(data=False))
                self.pub_walk_control.publish(Bool(data=False))
                self.ensure_stand()

    def is_moving(self, msg):
        return any([
            msg.linear.x != 0.0, msg.linear.y != 0.0, msg.linear.z != 0.0,
            msg.angular.x != 0.0, msg.angular.y != 0.0, msg.angular.z != 0.0
        ])

    def ensure_stand_then_walk(self):
        if not self.is_standing:
            self.get_logger().info("Esperando a que 'stand' termine antes de caminar...")
            self.try_stand(callback=self.start_walking)
        else:
            self.start_walking()


    def start_walking(self):
        self.set_stiffness()
        self.set_initial_position()
        self.pub_walk_status.publish(Bool(data=True))
        self.pub_walk_control.publish(Bool(data=True))

    def ensure_stand(self):
        if not self.is_standing:
            self.try_stand()

    def try_stand(self, callback=None):
        if self.is_standing:
            if callback:
                callback()
            return

        self.stand_attempts += 1
        goal_msg = PosPlay.Goal()
        goal_msg.action_name = "stand"

        future_goal = self.action_client.send_goal_async(goal_msg)
        future_goal.add_done_callback(lambda future: self.stand_position_result_callback(future, callback))

    def stand_position_result_callback(self, future, callback):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.create_timer(1.0, lambda: self.try_stand(callback))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self.stand_position_completed(future, callback))

    def stand_position_completed(self, future, callback):
        """ Verifica si 'stand' fue exitoso antes de ejecutar el callback. """
        result = future.result().result
        if result and result.success:
            self.get_logger().info("Acción 'stand' completada con éxito.")
            self.is_standing = True  # Ahora sí está en 'stand'
            if callback:
                self.get_logger().info("Ejecutando callback después de 'stand'.")
                callback()  # Solo aquí llamamos a 'start_walking()'
        else:
            self.get_logger().error("Fallo en 'stand'. Reintentando en 1 segundo...")
            self.create_timer(1.0, lambda: self.try_stand(callback))

    def set_stiffness(self):
        msg = JointStiffnesses(indexes=list(range(25)), stiffnesses=[1.0] * 25)
        self.pub_stiffness.publish(msg)

    def set_initial_position(self):
        msg = SolePoses()
        msg.l_sole.position.y, msg.l_sole.position.z = 0.05, -0.315
        msg.r_sole.position.y, msg.r_sole.position.z = -0.05, -0.315
        self.pub_initial_position.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
