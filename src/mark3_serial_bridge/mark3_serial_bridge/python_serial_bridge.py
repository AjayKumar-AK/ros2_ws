'''#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

import serial



class SerialBridge(Node):

    def __init__(self):

        super().__init__('serial_bridge')

        self.subscription = self.create_subscription(

            Twist,

            '/cmd_vel',

            self.listener_callback,

            10)

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Replace with your port

        self.get_logger().info("Serial Bridge Node Started")



    def listener_callback(self, msg):

        vx = msg.linear.x * 100  # convert to cm/s or scale

        vy = msg.linear.y * 100

        omega = msg.angular.z * 100



        command = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"

        self.ser.write(command.encode('utf-8'))

        self.get_logger().info(f"Sent: {command.strip()}")



def main(args=None):

    rclpy.init(args=args)

    bridge = SerialBridge()

    rclpy.spin(bridge)

    bridge.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()'''


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import serial
import math
import time

def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

class SensorBridge(Node):
    def __init__(self,
                 port='/dev/ttyUSB0',
                 baud=115200,
                 ticks_per_rev=975.0,
                 wheel_radius=0.05,
                 wheel_base=0.25,
                 track_width=0.20,
                 publish_rate_hz=50.0):
        super().__init__('sensor_bridge')
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info(f"Opened serial {port} @ {baud}")

        # encoder params
        self.ticks_per_rev = float(ticks_per_rev)
        self.r = float(wheel_radius)
        # use half distances for L and W in formulas
        self.L = float(wheel_base) / 2.0
        self.W = float(track_width) / 2.0

        # storage for previous encoder readings
        self.prev_ticks = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = time.time()

        # pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

    def listener_callback(self, msg):

        vx = msg.linear.x   # convert to cm/s or scale

        vy = msg.linear.y 
        omega = msg.angular.z 



        command = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"

        self.ser.write(command.encode('utf-8'))

        self.get_logger().info(f"Sent: {command.strip()}")
        
    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        # expect 10 comma-separated values
        parts = line.split(',')
        
        if len(parts) < 11 and parts[0] !="S":
            self.get_logger().warn("Bad serial line (expected 10 values): " + line)
            return

        try:
            gx, gy, gz, ax, ay, az, e1, e2, e3, e4 = map(float, parts[1:])
            #self.get_logger().warn("Parse error here: " + vale)
        except ValueError:
            self.get_logger().warn("Parse error: " + line)
            return
        
        self.get_logger().info(f"values are {gx}, {gy}, {gz}, {ax}, {ay}, {az}, {e1}, {e2}, {e3}, {e4} ")

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return

        # ---------------- IMU ----------------
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Arduino sends gyro in deg/s → convert to rad/s here
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        # accel already in m/s^2 (as you said)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.orientation_covariance[0] = -1.0  # no orientation

        self.imu_pub.publish(imu_msg)

        # ---------------- Encoders -> wheel speeds ----------------
        ticks = [e1, e2, e3, e4]
        # compute delta ticks
        delta = [ticks[i] - self.prev_ticks[i] for i in range(4)]

        # wheel angular velocities (rad/s): (delta_ticks / ticks_per_rev) * 2π / dt
        omega_w = [(delta[i] / self.ticks_per_rev) * (2.0 * math.pi) / dt for i in range(4)]

        # wheel linear velocities (m/s)
        v_w = [omega_w[i] * self.r for i in range(4)]
        # wheel order: assume e1=FL, e2=FR, e3=RL, e4=RR (match your Arduino)
        v_FL, v_FR, v_RL, v_RR = v_w

        # ---------------- Mecanum inverse kinematics ----------------
        vx = (v_FL + v_FR + v_RL + v_RR) / 4.0
        vy = (-v_FL + v_FR + v_RL - v_RR) / 4.0
        omega = (-v_FL + v_FR - v_RL + v_RR) / 4.0 * (self.L + self.W)

        # integrate pose (body -> world)
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        d_yaw = omega * dt

        self.x += dx
        self.y += dy
        self.yaw += d_yaw

    


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge(port='/dev/ttyUSB0', baud=115200,
                        ticks_per_rev=975.0, wheel_radius=0.05,
                        wheel_base=0.25, track_width=0.20,
                        publish_rate_hz=50.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

