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
                 ticks_per_rev=490.0,  # Corrected CPR
                 wheel_radius=0.047,
                 wheel_base=0.164,
                 track_width=0.22,
                 publish_rate_hz=50.0):
        super().__init__('sensor_bridge')
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info(f"Opened serial {port} @ {baud}")
        self.ticks_per_rev = float(ticks_per_rev)
        self.r = float(wheel_radius)
        self.L = float(wheel_base) / 2.0
        self.W = float(track_width) / 2.0
        self.prev_ticks = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)
        self.last_valid_sensor_time = time.time()
        self.debug_counter = 0

    def listener_callback(self, msg):
        # Fix coordinate frame mismatch as you discovered
        vx = msg.linear.x      # Swap X and Y
        vy = msg.linear.y     # Negate the new Y
        omega = msg.angular.z
        command = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent cmd_vel (corrected): vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}")

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Debug: Log every 10th iteration what we're receiving
        self.debug_counter += 1
        if self.debug_counter % 10 == 0:
            self.get_logger().info(f"Raw serial line: '{line}'")
        
        if not line:
            if time.time() - self.last_valid_sensor_time > 2.0:
                self.get_logger().warn("No serial data received for 2 seconds!")
            return
            
        # Check if this is an expected sensor data line
        if not line.startswith('S'):
            if line.startswith('[') or line.startswith('DEBUG'):
                self.get_logger().warn(f"Arduino sending debug instead of sensor data: {line}")
            else:
                self.get_logger().warn(f"Unexpected serial format: {line}")
            return
            
        parts = line.split(',')
        if len(parts) != 11:  # S + 10 sensor values
            self.get_logger().warn(f"Wrong number of parts ({len(parts)}): {line}")
            return
            
        try:
            gx, gy, gz, ax, ay, az, e1, e2, e3, e4 = map(float, parts[1:])
            self.last_valid_sensor_time = time.time()
        except ValueError as e:
            self.get_logger().warn(f"Parse error: {line} - {e}")
            return

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return
        self.prev_time = now

        # Debug: Log sensor values occasionally
        if self.debug_counter % 50 == 0:
            self.get_logger().info(f"Sensors - IMU: gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}, ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            self.get_logger().info(f"Encoders: e1={e1}, e2={e2}, e3={e3}, e4={e4}")

        # Publish IMU with corrections for upside-down orientation
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Correct for upside-down IMU (flip Z-axis, keep X,Y)
        imu_msg.angular_velocity.x = gx      # Assuming X is correct
        imu_msg.angular_velocity.y = gy      # Assuming Y is correct  
        imu_msg.angular_velocity.z = -gz     # Flip Z for upside-down IMU
        
        imu_msg.linear_acceleration.x = ax   # Assuming X is correct
        imu_msg.linear_acceleration.y = ay   # Assuming Y is correct
        imu_msg.linear_acceleration.z = -az  # Flip Z - gravity should read +9.81 when upside down
        
        imu_msg.orientation_covariance[0] = -1.0  # No orientation data
        imu_msg.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.imu_pub.publish(imu_msg)

        # Encoder -> Wheel Velocities
        ticks = [e1, e2, e3, e4]
        delta_ticks = [ticks[i] - self.prev_ticks[i] for i in range(4)]
        
        # Debug: Log encoder changes
        if any(abs(dt) > 0.1 for dt in delta_ticks):
            self.get_logger().info(f"Encoder deltas: {delta_ticks}")
        
        self.prev_ticks = ticks

        omega_w = [(delta_ticks[i] / self.ticks_per_rev) * (2.0 * math.pi) / dt for i in range(4)]
        v_w = [omega_w[i] * self.r for i in range(4)]
        v_FL, v_FR, v_RL, v_RR = v_w

        # Debug: Log calculated velocities
        if any(abs(v) > 0.01 for v in v_w):
            self.get_logger().info(f"Wheel velocities: FL={v_FL:.3f}, FR={v_FR:.3f}, RL={v_RL:.3f}, RR={v_RR:.3f}")

        # Mecanum Inverse Kinematics
        old_vx, old_vy, old_vth = self.vx, self.vy, self.vth
        self.vx = (v_FL + v_FR + v_RL + v_RR) / 4.0
        self.vy = (-v_FL + v_FR + v_RL - v_RR) / 4.0
        self.vth = (-v_FL + v_FR - v_RL + v_RR) / (4.0 * (self.L + self.W))

        # Debug: Log body velocities when they change significantly
        if abs(self.vx - old_vx) > 0.01 or abs(self.vy - old_vy) > 0.01 or abs(self.vth - old_vth) > 0.01:
            self.get_logger().info(f"Body velocities: vx={self.vx:.3f}, vy={self.vy:.3f}, vth={self.vth:.3f}")

        # Integrate Pose
        dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        d_yaw = self.vth * dt
        
        old_x, old_y, old_yaw = self.x, self.y, self.yaw
        self.x += dx
        self.y += dy
        self.yaw += d_yaw

        # Debug: Log pose changes
        if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(d_yaw) > 0.001:
            self.get_logger().info(f"Pose change: dx={dx:.4f}, dy={dy:.4f}, dyaw={d_yaw:.4f}")
            self.get_logger().info(f"New pose: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}")

        # Publish Odometry (same as before)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vth
        
        # Lower covariance values for better EKF stability
        odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                   0, 0.01, 0, 0, 0, 0,
                                   0, 0, 1e6, 0, 0, 0,
                                   0, 0, 0, 1e6, 0, 0,
                                   0, 0, 0, 0, 1e6, 0,
                                   0, 0, 0, 0, 0, 0.05]

        odom_msg.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 0.05]

        self.odom_pub.publish(odom_msg)

        # Publish TF
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
