#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
from rcl_interfaces.msg import SetParametersResult


class ImpedanceTestNode(Node):

    def __init__(self):
        super().__init__('ImpedanceTestNode')
        
        self.add_on_set_parameters_callback(self.on_param_change)

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('num_joints', 3)
        self.declare_parameter('publish_rate_hz', 500.0)
        self.declare_parameter('input_in_degrees', False)

        self.declare_parameter('q_des', [0.0, 0.0, 0.0])
        self.declare_parameter('Kp', [0.0, 0.0, 0.0])
        self.declare_parameter('Kd', [0.0, 0.0, 0.0])
        self.declare_parameter('torque_limits', [0.0, 0.0, 0.0]) # N-m

        self.declare_parameter('knee_min_deg', 6.0)

        self.num_joints = self.get_parameter('num_joints').value
        rate = self.get_parameter('publish_rate_hz').value
        input_deg = self.get_parameter('input_in_degrees').value

        self.q_des = list(self.get_parameter('q_des').value)
        self.Kp = list(self.get_parameter('Kp').value)
        self.Kd = list(self.get_parameter('Kd').value)
        self.torque_limits = list(self.get_parameter('torque_limits').value)

        self.knee_min = math.radians(
            self.get_parameter('knee_min_deg').value
        )

        if input_deg:
            self.q_des = [math.radians(q) for q in self.q_des]

        # -----------------------------
        # State
        # -----------------------------
        self.q = [0.0] * self.num_joints
        self.qd = [0.0] * self.num_joints
        self.state_ready = False

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_effort_controller/command',
            10
        )

        self.timer = self.create_timer(1.0 / rate, self.control_step)

        self.get_logger().info("ImpedanceTestNode running")
        self.get_logger().info(f"q_des = {self.q_des}")

    # -----------------------------

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) < self.num_joints:
            return

        self.q = list(msg.position[:self.num_joints])
        self.qd = list(msg.velocity[:self.num_joints])
        self.state_ready = True

    # -----------------------------

    def control_step(self):
        if not self.state_ready:
            return

        # Singularity guard
        # if self.q[2] < self.knee_min:
        #     self.get_logger().warn_throttle(
        #         1.0,
        #         f"Knee near singularity: {self.q[2]:.3f} rad"
        #     )

        tau = []

        for i in range(self.num_joints):
            t = (
                self.Kp[i] * (self.q_des[i] - self.q[i])
                - self.Kd[i] * self.qd[i]
            )

            t = max(
                -self.torque_limits[i],
                min(self.torque_limits[i], t)
            )

            tau.append(t)

        msg = Float64MultiArray()
        msg.data = tau
        self.pub.publish(msg)

    from rcl_interfaces.msg import SetParametersResult

    # -----------------------------
    def on_param_change(self, params):
        for p in params:
            if p.name == 'Kp':
                self.Kp = list(p.value)
            elif p.name == 'Kd':
                self.Kd = list(p.value)
            elif p.name == 'torque_limits':
                self.torque_limits = list(p.value)
            elif p.name == 'q_des':
                self.q_des = list(p.value)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
