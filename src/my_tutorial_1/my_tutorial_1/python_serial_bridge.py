#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import serial
import math
import time


class SensorBridge(Node):
    def __init__(self,
                 port='/dev/ttyUSB0',
                 baud=115200,
                 ticks_per_rev=477.0,  # Measured: actual encoder counts per wheel revolution
                 wheel_radius=0.094,   # Measured: 9.4cm radius
                 wheel_base=0.164,     # Measured: 16.4cm wheelbase
                 track_width=0.22,     # Measured: 22cm track width
                 publish_rate_hz=50.0):
        super().__init__('sensor_bridge')

        # ---------------- Publishers ----------------
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)

        # ---------------- Subscriber ----------------
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        # ---------------- Serial ----------------
        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info(f"Opened serial {port} @ {baud}")

        # ---------------- Encoder/Wheel Parameters ----------------
        # FIXME: Check these parameters with your actual hardware!
        self.ticks_per_rev = float(ticks_per_rev)  # Verify this with your encoder datasheet
        self.r = float(wheel_radius)  # Measure your actual wheel radius
        
        # Add scaling factor for debugging - you can adjust this
        self.encoder_scale_factor = 1.0  # Reduce this if positions are too large
        self.L = float(wheel_base) / 2.0
        self.W = float(track_width) / 2.0

        # ---------------- State ----------------
        self.prev_ticks = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = time.time()
        self.first_reading = True  # Flag for first encoder reading

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # ---------------- TF Broadcaster ----------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

    # ---------------- Callback for /cmd_vel ----------------
    def listener_callback(self, msg):
        vx = msg.linear.y
        vy = -msg.linear.x
        omega = msg.angular.z
        command = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent cmd_vel: {command.strip()}")

    # ---------------- Timer callback: read sensors, compute odom ----------------
    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        parts = line.split(',')
        if len(parts) < 11 or parts[0] != "S":
            self.get_logger().warn("Bad serial line: " + line)
            return

        try:
            gx, gy, gz, ax, ay, az, e1, e2, e3, e4 = map(float, parts[1:])
        except ValueError:
            self.get_logger().warn("Parse error: " + line)
            return

        # DEBUG: Print encoder values to check if they're changing
        self.get_logger().info(f"Encoders: [{e1:.0f}, {e2:.0f}, {e3:.0f}, {e4:.0f}]")

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return
        self.prev_time = now

        # ---------------- Publish IMU ----------------
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # FIXED: Proper covariance matrices
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8  # -1 means no orientation data
        imu_msg.angular_velocity_covariance = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3
        ]
        imu_msg.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        ]

        self.imu_pub.publish(imu_msg)

        # ---------------- Encoder -> Wheel Velocities ----------------
        ticks = [e1, e2, e3, e4]
        
        # Skip first reading to initialize prev_ticks
        if self.first_reading:
            self.prev_ticks = ticks
            self.first_reading = False
            return
            
        delta_ticks = [ticks[i] - self.prev_ticks[i] for i in range(4)]
        self.prev_ticks = ticks

        # DEBUG: Check if encoders are changing
        total_change = sum(abs(delta) for delta in delta_ticks)
        if total_change == 0:
            self.get_logger().warn("ENCODERS NOT CHANGING! Check your encoder connections!")
        else:
            self.get_logger().info(f"Encoder deltas: [{delta_ticks[0]:.1f}, {delta_ticks[1]:.1f}, {delta_ticks[2]:.1f}, {delta_ticks[3]:.1f}]")

        omega_w = [(delta_ticks[i] * self.encoder_scale_factor / self.ticks_per_rev) * (2.0 * math.pi) / dt for i in range(4)]
        v_w = [omega_w[i] * self.r for i in range(4)]
        v_FL, v_FR, v_RL, v_RR = v_w

        # ---------------- Mecanum Inverse Kinematics ----------------
        self.vx = (v_FL + v_FR + v_RL + v_RR) / 4.0
        self.vy = (-v_FL + v_FR + v_RL - v_RR) / 4.0
        self.vth = (-v_FL + v_FR - v_RL + v_RR) / (4.0 * (self.L + self.W))

        # DEBUG: Print calculated velocities
        self.get_logger().info(f"Calculated velocities: vx={self.vx:.3f}, vy={self.vy:.3f}, vth={self.vth:.3f}")

        # ---------------- Integrate Pose ----------------
        dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        d_yaw = self.vth * dt
        self.x += dx
        self.y += dy
        self.yaw += d_yaw

        # ---------------- Publish Odometry ----------------
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"  # Make sure this matches your URDF

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Quaternion from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vth

        # FIXED: Proper covariance matrices (36 elements each)
        odom_msg.pose.covariance = [
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,    # High Z uncertainty
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,    # High roll uncertainty
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,    # High pitch uncertainty
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2    # Yaw uncertainty
        ]

        odom_msg.twist.covariance = [
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2
        ]

        self.odom_pub.publish(odom_msg)

        # ---------------- Broadcast TF ----------------
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
