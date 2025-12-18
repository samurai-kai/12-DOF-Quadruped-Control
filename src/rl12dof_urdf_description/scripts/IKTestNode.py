#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from safety import validate_command


class IKTestNode(Node):
    def __init__(self):
        super().__init__('IKTestNode')

        # ---- Trajectory parameters ----
        self.declare_parameter('freq', 0.5)
        self.declare_parameter('amp_z', 0.03)
        self.declare_parameter('x_fixed', 0.05)
        self.declare_parameter('y_fixed', 0.05)
        self.declare_parameter('z_nominal', 0.20)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('verbose', True)

        # ---- Link lengths (you set these in YAML) ----
        self.declare_parameter('L2', 0.035)   # hip offset
        self.declare_parameter('L3', 0.11)    # thigh length
        self.declare_parameter('L4', 0.11)    # shank length

        self.freq      = float(self.get_parameter('freq').value)
        self.amp_z     = float(self.get_parameter('amp_z').value)
        self.x_fixed   = float(self.get_parameter('x_fixed').value)
        self.y_fixed   = float(self.get_parameter('y_fixed').value)
        self.z_nominal = float(self.get_parameter('z_nominal').value)
        self.dt        = float(self.get_parameter('dt').value)
        self.verbose   = bool(self.get_parameter('verbose').value)

        self.L2 = float(self.get_parameter('L2').value)
        self.L3 = float(self.get_parameter('L3').value)
        self.L4 = float(self.get_parameter('L4').value)

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.time = 0.0
        self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f"IKTestNode started with L2={self.L2:.3f}, L3={self.L3:.3f}, L4={self.L4:.3f}"
        )

    # ===================================================
    # Analytic IK (your equations)
    # Inputs: x,y,z in leg frame
    #   +X = backward, +Y = outward (right leg), +Z = downward
    # Output: [hip, thigh, knee] in radians
    # ===================================================
    def analytic_ik(self, x_e: float, y_e: float, z_e: float):
        L2, L3, L4 = self.L2, self.L3, self.L4

        # --- θ3 ---
        # θ3 = π - cos⁻¹( (L3² + L4² + L2² - (x_e² + y_e² + z_e²)) / (2 L3 L4) )
        num = (L3**2 + L4**2 + L2**2) - (x_e**2 + y_e**2 + z_e**2)
        den = 2.0 * L3 * L4
        D = num / den

        # clamp for numerical safety
        if D > 1.0:
            D = 1.0
        elif D < -1.0:
            D = -1.0

        theta3 = math.pi - math.acos(D)

        # --- helper for θ1 & θ2 ---
        # r = sqrt(y_e² + z_e² - L2²)
        r_sq = y_e**2 + z_e**2 - L2**2
        if r_sq < 0.0:
            # clamp to avoid NaN, but log once
            if not hasattr(self, '_warned_rsq'):
              self.get_logger().warn(
                  f"r^2 < 0 in IK (value={r_sq:.6f}); clamping to 0."
              )
              self._warned_rsq = True

            r_sq = 0.0
        r = math.sqrt(r_sq)

        # --- θ1 ---
        # θ1 = tan⁻¹( r / L2 ) - tan⁻¹( -z_e / y_e )
        # use atan2 for robustness
        theta1 = math.atan2(r, L2) - math.atan2(-z_e, y_e)

        # --- θ2 ---
        # θ2 = π/2 - tan⁻¹( L4 sinθ3 / (L3 + L4 cosθ3) ) - tan⁻¹( x_e / r )
        A = math.atan2(
            L4 * math.sin(theta3),
            L3 + L4 * math.cos(theta3)
        )
        B = math.atan2(x_e, r)

        theta2 = (math.pi / 2.0) - A - B

        return [theta1, theta2, theta3]

    # ===================================================
    # Timer callback: generate foot trajectory, run IK
    # ===================================================
    def timer_cb(self):
        # Simple test: hold x,y constant, oscillate z
        # Remember: +X = backward, +Y = outward, +Z = downward
        x_e = self.x_fixed
        y_e = self.y_fixed
        z_e = self.z_nominal + self.amp_z * math.sin(2.0 * math.pi * self.freq * self.time)

        cmd = self.analytic_ik(x_e, y_e, z_e)

        # Safety check (your existing script)
        valid, reason = validate_command(cmd)
        if not valid:
            self.get_logger().warn(f"Rejected IK cmd: {reason} | cmd={cmd}")
            self.time += self.dt
            return

        msg = Float64MultiArray()
        msg.data = cmd
        self.pub.publish(msg)

        if self.verbose:
            self.get_logger().info(
                f"t={self.time:.2f}  xyz=({x_e:.3f},{y_e:.3f},{z_e:.3f})  "
                f"θ=[{cmd[0]:+.3f}, {cmd[1]:+.3f}, {cmd[2]:+.3f}]"
            )

        self.time += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = IKTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
