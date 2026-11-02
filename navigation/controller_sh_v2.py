#!/usr/bin/env python3
# Unified ICR Swerve Controller for both joy_drive and Nav2 /cmd_vel
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

L = 0.7777
W = 0.475

CMD_VEL_TIMEOUT_SEC = 0.50
JOY_CMD_TIMEOUT_SEC = 0.30
PUBLISH_RATE_HZ = 20.0
MAX_WHEEL_SPEED = 1.0
MAX_LINEAR_ACCEL = 2.0
MAX_ANGULAR_ACCEL = 3.5

# Only used when vx ~ 0 and wz is actively spinning in place so tires don't stall
MIN_SPIN_WHEEL_SPEED = 0.32
MAX_PIVOT_ANGLE_DEG = 125.0
MAX_STEER_STEP_PER_TICK_DEG = 35.0

# Steering-lag handling: drive speed is scaled down while the MEASURED wheel
# angle (pivot encoders) is far from the target angle, so wheels don't scrub.
ENC_TIMEOUT_SEC = 0.50              # encoder data older than this is ignored
STEER_ERR_FULL_SPEED_DEG = 10.0     # error <= this -> full speed
STEER_ERR_MIN_SPEED_DEG = 40.0      # error >= this -> STEER_MIN_SPEED_SCALE
STEER_MIN_SPEED_SCALE = 0.35        # keep rolling enough to overcome friction

WHEEL_POSITIONS = {
    "LF": ( L / 2.0,  W / 2.0),
    "LR": (-L / 2.0,  W / 2.0),
    "RF": ( L / 2.0, -W / 2.0),
    "RR": (-L / 2.0, -W / 2.0),
}
WHEEL_ORDER = ["RF", "RR", "LF", "LR"]


def step_angle_linear(current_deg, target_deg, max_step=MAX_STEER_STEP_PER_TICK_DEG):
    target_clamped = max(-MAX_PIVOT_ANGLE_DEG, min(MAX_PIVOT_ANGLE_DEG, target_deg))
    diff = target_clamped - current_deg
    if abs(diff) <= max_step:
        return target_clamped
    return current_deg + math.copysign(max_step, diff)


def angle_diff_deg(a, b):
    """Smallest signed difference a - b in degrees, wrapped to [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def steering_speed_scale(max_err_deg):
    """Continuous drive-speed scale vs. worst-case steering error (deg)."""
    if max_err_deg <= STEER_ERR_FULL_SPEED_DEG:
        return 1.0
    if max_err_deg >= STEER_ERR_MIN_SPEED_DEG:
        return STEER_MIN_SPEED_SCALE
    frac = (max_err_deg - STEER_ERR_FULL_SPEED_DEG) / (
        STEER_ERR_MIN_SPEED_DEG - STEER_ERR_FULL_SPEED_DEG)
    return 1.0 - frac * (1.0 - STEER_MIN_SPEED_SCALE)


def compute_icr_from_cmd_vel(vx, wz, last_angles, last_turn_sign):
    """
    Continuous ICR mapping from (vx, wz) with NO discontinuity between straight
    and turning, NO vx sign-flip oscillation, and proportional wheel speeds.
    """
    # 1. Deadband: completely stopped
    if abs(vx) < 1e-3 and abs(wz) < 1e-3:
        return [last_angles[name] for name in WHEEL_ORDER], [0.0] * 4, last_turn_sign

    # 2. Pure straight line (or tiny wz < 0.01 rad/s)
    if abs(wz) < 0.01:
        drive_dir = -1.0 if vx >= 0.0 else 1.0
        v_mag = min(MAX_WHEEL_SPEED, abs(vx))
        angles = [0.0] * 4
        speeds = []
        for name in WHEEL_ORDER:
            side_sign = -1.0 if name in ('LF', 'LR') else 1.0
            speeds.append((v_mag / MAX_WHEEL_SPEED) * drive_dir * side_sign)
        return angles, speeds, last_turn_sign

    # 3. Turning or spinning in place
    # Always follow the sign of wz. Noise near zero is already handled by the
    # straight-line branch above (|wz| < 0.01), and reusing a stale sign for
    # 0.01 < |wz| <= 0.03 steered small planner corrections the wrong way.
    turn_sign = 1.0 if wz > 0.0 else -1.0

    # Clamp negative vx dithering near zero so wheels NEVER flip 180 deg mid-turn!
    # Only reverse drive_dir if Nav2 is explicitly executing a BackUp behavior (vx < -0.08 and wz ~ 0)
    if vx < -0.08 and abs(wz) < 0.15:
        drive_dir = 1.0
        eff_vx = abs(vx)
    else:
        drive_dir = -1.0
        eff_vx = max(0.0, vx)

    # Turning radius Rc = eff_vx / |wz|
    rc_mag = eff_vx / abs(wz)

    max_dist = 0.0
    wheel_geom = []
    for name in WHEEL_ORDER:
        x_i, y_i = WHEEL_POSITIONS[name]
        dx = rc_mag - (turn_sign * y_i)
        dist = math.hypot(x_i, dx)
        wheel_geom.append((name, x_i, dx, dist))
        if dist > max_dist:
            max_dist = dist

    if max_dist < 1e-6:
        max_dist = 1.0

    # True kinematic outer wheel speed: v_outer = |wz| * max_dist
    # (Notice: when rc_mag is large, |wz| * max_dist == eff_vx, so it matches straight speed seamlessly!)
    v_outer = min(MAX_WHEEL_SPEED, abs(wz) * max_dist)

    # ONLY apply minimum spin floor when actually spinning in place (eff_vx < 0.03)
    # so rolling turns stay 100% proportional to Nav2's commanded velocity!
    if eff_vx < 0.03 and abs(wz) > 0.10 and v_outer < MIN_SPIN_WHEEL_SPEED:
        v_outer = MIN_SPIN_WHEEL_SPEED

    angles = []
    speeds = []
    for name, x_i, dx, dist in wheel_geom:
        angle_deg = turn_sign * math.degrees(math.atan2(x_i, dx))
        side_sign = -1.0 if name in ('LF', 'LR') else 1.0
        wheel_speed = (v_outer / MAX_WHEEL_SPEED) * (dist / max_dist) * drive_dir * side_sign
        angles.append(angle_deg)
        speeds.append(max(-1.0, min(1.0, wheel_speed)))

    return angles, speeds, turn_sign


def ramp_value(current, target, max_delta):
    if max_delta <= 0.0:
        return target
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


class SwerveKinematicsNode(Node):
    def __init__(self):
        super().__init__('swerve_kinematics_node')
        self.declare_parameter('use_encoder_feedback', True)
        self.use_encoder_feedback = bool(
            self.get_parameter('use_encoder_feedback').value)
        self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        self.current_vx, self.current_vy, self.current_wz = 0.0, 0.0, 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.last_angles = {name: 0.0 for name in WHEEL_ORDER}
        self.last_turn_sign = 1.0
        self.enc_angles = [0.0] * 4
        self.enc_valid = False
        self.last_enc_time = self.get_clock().now()

        self.joy_angles, self.joy_speeds = [0.0] * 4, [0.0] * 4
        self.joy_seen = False
        self.joy_seq = None
        self.last_joy_time = self.get_clock().now()
        self.source = 'teleop'
        self._logged_source = self.source
        self._src_key = None
        self._src_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_joy = self.create_subscription(Float32MultiArray, '/joy_drive_cmd', self.joy_callback, 10)
        self.sub_enc = self.create_subscription(Float32MultiArray, '/pivot_encoders', self.enc_callback, sensor_qos)

        self.pub_angles = self.create_publisher(Float32MultiArray, 'target_angles', 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.pub_source = self.create_publisher(String, '/drive_source', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_wheel_commands)

        self.get_logger().info('swerve_kinematics_node started (Proportional ICR + No vx-flip + encoder steering-lag scaling)')

    def enc_callback(self, msg):
        if len(msg.data) >= 4:
            vals = [float(a) for a in msg.data[:4]]
            if all(abs(v) <= 360.0 for v in vals):
                self.enc_angles = vals
                self.enc_valid = True
                self.last_enc_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        self.source = 'teleop'

    def joy_callback(self, msg):
        if len(msg.data) < 9:
            return
        self.joy_angles = [float(a) for a in msg.data[:4]]
        self.joy_speeds = [float(s) for s in msg.data[4:8]]
        self.joy_seen = True
        self.last_joy_time = self.get_clock().now()
        seq = int(round(msg.data[8]))
        if seq != self.joy_seq:
            self.joy_seq = seq
            self.source = 'joy'

    def publish_source(self, now):
        key = f"{self.source} {-1 if self.joy_seq is None else self.joy_seq}"
        if key != self._src_key or (now - self._src_time).nanoseconds / 1e9 > 1.0:
            msg = String()
            msg.data = key
            self.pub_source.publish(msg)
            self._src_key = key
            self._src_time = now

    def publish_wheel_commands(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / PUBLISH_RATE_HZ

        if self.source != self._logged_source:
            self.get_logger().info(f'drive source: {self._logged_source} -> {self.source}')
            self._logged_source = self.source
        self.publish_source(now)

        angles_deg, speeds_norm = [], []

        if self.source == 'joy':
            self.current_vx = self.current_vy = self.current_wz = 0.0
            if self.joy_seen and ((now - self.last_joy_time).nanoseconds / 1e9) <= JOY_CMD_TIMEOUT_SEC:
                for idx, name in enumerate(WHEEL_ORDER):
                    target_ang = float(self.joy_angles[idx])
                    stepped_ang = step_angle_linear(self.last_angles[name], target_ang)
                    spd = max(-1.0, min(1.0, float(self.joy_speeds[idx])))
                    self.last_angles[name] = stepped_ang
                    angles_deg.append(stepped_ang)
                    speeds_norm.append(spd)
            else:
                if self.enc_valid:
                    angles_deg = list(self.enc_angles)
                    for name, ang in zip(WHEEL_ORDER, angles_deg):
                        self.last_angles[name] = ang
                else:
                    angles_deg = [self.last_angles[name] for name in WHEEL_ORDER]
                speeds_norm = [0.0] * 4

        else:
            if ((now - self.last_cmd_time).nanoseconds / 1e9) > CMD_VEL_TIMEOUT_SEC:
                tvx, twz = 0.0, 0.0
            else:
                tvx, twz = self.vx, self.wz

            if abs(tvx) < 1e-4 and abs(twz) < 1e-4:
                self.current_vx, self.current_wz = 0.0, 0.0
            else:
                d_lin, d_ang = MAX_LINEAR_ACCEL * dt, MAX_ANGULAR_ACCEL * dt
                self.current_vx = ramp_value(self.current_vx, tvx, d_lin)
                self.current_wz = ramp_value(self.current_wz, twz, d_ang)

            raw_angles, raw_speeds, self.last_turn_sign = compute_icr_from_cmd_vel(
                self.current_vx, self.current_wz, self.last_angles, self.last_turn_sign
            )

            # Remaining steering error across the 4 wheels. Prefer the MEASURED
            # pivot angle (encoders) so real steering-motor lag slows the drive;
            # fall back to the last commanded angle if encoders are missing/stale.
            enc_fresh = (
                self.use_encoder_feedback and self.enc_valid and
                ((now - self.last_enc_time).nanoseconds / 1e9) <= ENC_TIMEOUT_SEC
            )
            if enc_fresh:
                max_steer_err = max(
                    abs(angle_diff_deg(raw_angles[idx], self.enc_angles[idx]))
                    for idx in range(4)
                )
            else:
                max_steer_err = max(
                    abs(raw_angles[idx] - self.last_angles[name])
                    for idx, name in enumerate(WHEEL_ORDER)
                )

            # Smoothly reduce drive speed while wheels are still swinging
            # (e.g. left turn -> right turn), never below STEER_MIN_SPEED_SCALE
            # so the tires keep rolling enough to overcome friction.
            transition_scale = steering_speed_scale(max_steer_err)

            for idx, name in enumerate(WHEEL_ORDER):
                stepped_ang = step_angle_linear(self.last_angles[name], raw_angles[idx])
                self.last_angles[name] = stepped_ang
                angles_deg.append(stepped_ang)
                speeds_norm.append(raw_speeds[idx] * transition_scale)

        a_msg, w_msg = Float32MultiArray(), Float32MultiArray()
        a_msg.data = [float(a) for a in angles_deg]
        w_msg.data = [float(s) for s in speeds_norm] + [0.0]
        self.pub_angles.publish(a_msg)
        self.pub_wheels.publish(w_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
