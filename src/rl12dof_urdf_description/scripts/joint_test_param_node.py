#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from safety import validate_command



class JointTestParamNode(Node):
    def __init__(self):
        super().__init__('joint_test_param_node')

        # Declare parameters with defaults
        self.declare_parameter('num_joints', 3)
        self.declare_parameter('delay_seconds', 1.0)
        self.declare_parameter('joint_sequence_flat', [0.0])

        # Read parameters
        num_joints = self.get_parameter(
            'num_joints').get_parameter_value().integer_value
        delay = self.get_parameter(
            'delay_seconds').get_parameter_value().double_value
        flat = self.get_parameter(
            'joint_sequence_flat').get_parameter_value().double_array_value

        flat = list(flat)

        if num_joints <= 0:
            self.get_logger().error("num_joints must be > 0")
            rclpy.shutdown()
            return

        if len(flat) % num_joints != 0:
            self.get_logger().error(
                f"Length of joint_sequence_flat ({len(flat)}) is not a multiple "
                f"of num_joints ({num_joints})"
            )
            rclpy.shutdown()
            return

        # Reshape flat list into list-of-lists
        self.sequence = []
        for i in range(0, len(flat), num_joints):
            self.sequence.append(flat[i:i + num_joints])

        self.delay = float(delay)

        self.get_logger().info(
            f"Loaded {len(self.sequence)} joint commands with {num_joints} joints each"
        )
        self.get_logger().info(f"Delay between steps: {self.delay} sec")

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.step = 0
        self.timer = self.create_timer(self.delay, self.run_step)

    def run_step(self):
        cmd = self.sequence[self.step]

        # Validate command (joint limits + singularity avoidance)
        valid, reason = validate_command(cmd)
        if not valid:
            self.get_logger().error(f"Command rejected: {reason}")
            return

        # Prepare message
        msg = Float64MultiArray()
        msg.data = cmd

        # Log once
        self.get_logger().info(f"Sending command: {msg.data}")

        # Publish once
        self.pub.publish(msg)

        # Move to next command in sequence
        self.step = (self.step + 1) % len(self.sequence)

def main(args=None):
    rclpy.init(args=args)
    node = JointTestParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
