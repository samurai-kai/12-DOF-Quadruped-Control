#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from safety import validate_command
import math


class FKTestNode(Node):
    def __init__(self):
        super().__init__('FKTestNode')

        # Declare parameters with defaults
        self.declare_parameter('num_joints', 3)
        self.declare_parameter('delay_seconds', 1.0)
        self.declare_parameter('joint_sequence_flat', [0.0])
        self.declare_parameter('input_in_degrees', False)
        self.declare_parameter('step_loop', True)   

        # Read parameters
        num_joints = self.get_parameter('num_joints').value
        delay = self.get_parameter('delay_seconds').value
        flat = list(self.get_parameter('joint_sequence_flat').value)
        input_in_degrees = self.get_parameter('input_in_degrees').value
        self.step_loop = self.get_parameter('step_loop').value   # <--- STORE PARAM

        if input_in_degrees:
            self.get_logger().info("Converting joint commands from DEGREES → RADIANS")
            flat = [math.radians(v) for v in flat]

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

        # Feedback
        self.get_logger().info(
            f"Loaded {len(self.sequence)} joint commands with {num_joints} joints each."
        )
        self.get_logger().info(f"Delay between steps: {self.delay} sec")
        self.get_logger().info(f"Looping enabled: {self.step_loop}")

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.step = 0
        self.timer = self.create_timer(self.delay, self.run_step)

    # ----------------------------------------------------------

    def run_step(self):
        cmd = self.sequence[self.step]

        # Validate command
        valid, reason = validate_command(cmd)
        if not valid:
            self.get_logger().error(f"Command rejected: {reason} | cmd={cmd}")
            return

        # Publish
        msg = Float64MultiArray()
        msg.data = cmd
        self.get_logger().info(f"Sending command: {msg.data}")
        self.pub.publish(msg)

        # Advance to next step
        self.step += 1

        # If looping is enabled → wrap around
        if self.step_loop:
            self.step %= len(self.sequence)
            return

        # If looping is OFF → stop after last command
        if self.step >= len(self.sequence):
            self.get_logger().info("Finished all commands — stopping timer.")
            self.timer.cancel()
            return


# --------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FKTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
