import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import Imu, LaserScan
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import csv

class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.R = 0.015  # Bán kính bánh xe

        self.state = np.zeros(4)
        self.yaw = 0.0
        self.yaw_dot = 0.0 
        
        # khởi tạo tọa độ
        self.x = 0.0
        self.y = 0.0
        self.last_a = 0.0       
        self.target_x = 1.0     
        self.target_y = 1.0     
        
        self.robot_state = 'MOVING' 
        self.stop_a = 0.0 
        self.stop_yaw = 0.0
        
        # parameter LQR
        self.K1 = np.array([0.0387, 0.0792, 0.3590, 0.0694])
        self.K_fwd = 0.05 
        self.constant_v = 0.45 
        self.K_omega = 0.5   
        self.K_yaw_rate = 0.1

        # config APF
        self.d0 = 0.6 
        self.K_att = 1.0
        self.K_rep = 0.05
        self.laser_data = None

        # record coordinates
        self.csv_file = open('robot_trajectory_with_obstacle.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y'])

        self.imu_subscriber = self.create_subscription(Imu, '/simple_robot/imu', self.imu_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/simple_robot/joint_states', self.joint_callback, 10)
        self.lidar_subcriber = self.create_subscription(LaserScan, '/simple_robot/laser_scan', self.lidar_callback, 10)

        self.left_force_pub = self.create_publisher(Float64, '/model/simple_robot/joint/left_wheel_joint/cmd_force', 10)
        self.right_force_pub = self.create_publisher(Float64, '/model/simple_robot/joint/right_wheel_joint/cmd_force', 10)
        
        self.timer = self.create_timer(0.01, self.controller)
        self.log_counter = 0

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.x + q.y * q.z)
        cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        phi = math.atan2(sinp, cosp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp) + (math.pi/2.0)
        while self.yaw > math.pi: self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi: self.yaw += 2.0 * math.pi

        self.state[2] = phi
        self.state[3] = msg.angular_velocity.x
        self.yaw_dot = msg.angular_velocity.z

    def joint_callback(self, msg):
        left_idx = msg.name.index('left_wheel_joint')
        right_idx = msg.name.index('right_wheel_joint')

        theta_avg = (msg.position[left_idx] + msg.position[right_idx]) / 2.0
        theta_dot_avg = (msg.velocity[left_idx] + msg.velocity[right_idx]) / 2.0

        self.state[0] = self.R * theta_avg
        self.state[1] = self.R * theta_dot_avg
        
        # odometry
        ds = -(self.state[0] - self.last_a)
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)
        self.last_a = self.state[0]
        self.csv_writer.writerow([self.x, self.y])



    def lidar_callback(self, msg):
        self.laser_data = msg


    def controller(self):
        """Hàm quản lý vòng lặp chính"""
        
        # Tính khoảng cách và góc
        theta_d, e_d = self.calculate_apf_heading()
       
        e_theta = theta_d - self.yaw

        # [-pi, pi]
        while e_theta > math.pi: e_theta -= 2.0 * math.pi
        while e_theta < -math.pi: e_theta += 2.0 * math.pi

        if self.robot_state == 'MOVING':
            if e_d <= 0.07: 
                self.robot_state = 'STOPPED'
                self.stop_a = self.state[0]
                self.stop_yaw = self.yaw
                self.get_logger().info("\nĐÃ ĐẾN ĐÍCH\n")
            else:
                self.go_to_goal(e_d, e_theta)

        elif self.robot_state == 'STOPPED':
            self.stop_control()

# thuat toan apf
    def calculate_apf_heading(self):
        
        # 1. LỰC HÚT TỪ ĐÍCH
        e_x = self.target_x - self.x
        e_y = self.target_y - self.y
        d_goal = math.hypot(e_x, e_y)
        
        F_x = self.K_att * e_x
        F_y = self.K_att * e_y

        # 2. LỰC ĐẨY TỪ BỨC TƯỜNG
        if self.laser_data is not None:
            msg = self.laser_data
            current_angle = msg.angle_min
            step = 5  # cứ cách 5 tia thì lấy 1 tia để tính toán lực
            
            # luuc day duoc tinh bang tổng các lực tương ứng với từng tia chiếu của lidar 
            for i in range(0, len(msg.ranges), step):
                r = msg.ranges[i]
                
                if msg.range_min < r < self.d0 and not math.isnan(r) and not math.isinf(r):
                    
                    # Góc của tia sáng
                    global_angle = self.yaw + current_angle
                    
                    # Tọa độ thực của điểm chạm trên tường
                    # obs_x = self.x + r * math.cos(global_angle)
                    # obs_y = self.y + r * math.sin(global_angle)
                    
                    # Vector đẩy dội ngược từ tường về phía xe
                    dx = -r * math.cos(global_angle)
                    dy = -r * math.sin(global_angle)
                    d_obs = math.hypot(dx, dy)
                    
                    if 0 < d_obs < self.d0:
                        rep_magnitude = self.K_rep * (1.0 / d_obs - 1.0 / self.d0) * (1.0 / d_obs**2)
                        
                        F_x += rep_magnitude * (dx / d_obs)
                        F_y += rep_magnitude * (dy / d_obs)
                
                current_angle += msg.angle_increment * step

        # tạo hướng mới
        theta_d_new = math.atan2(F_y, F_x)
        return theta_d_new, d_goal
                

    def go_to_goal(self, e_d, e_theta):
        v_ref = self.constant_v * max(0, math.cos(e_theta)) 
        
        omega_ref = self.K_omega * e_theta
        u_yaw = self.K_yaw_rate * (omega_ref - self.yaw_dot)

        virtual_state = np.copy(self.state)
        virtual_state[0] = 0.0 
        
        u_bal = np.dot(self.K1, virtual_state)
        u_fwd = self.K_fwd * (v_ref - virtual_state[1])
        
        u_total = u_bal + u_fwd
        u_left = (u_bal + u_fwd) / 2.0 + u_yaw
        u_right = (u_bal + u_fwd) / 2.0 - u_yaw

        self.publish_torque(u_left, u_right, f"MOVING | Đích: {e_d:.2f}m | Lệch: {math.degrees(e_theta):.0f}°")

    def stop_control(self):
        virtual_state = np.copy(self.state)
        virtual_state[0] = self.state[0] - self.stop_a

        u_bal = np.dot(self.K1, virtual_state)

        e_theta = self.stop_yaw - self.yaw
        while e_theta > math.pi: e_theta -= 2.0 * math.pi
        while e_theta < -math.pi: e_theta += 2.0 * math.pi

        omega_ref = self.K_omega * e_theta
        u_yaw = self.K_yaw_rate * (omega_ref - self.yaw_dot)

        u_left = (u_bal / 2.0) + u_yaw
        u_right = (u_bal / 2.0) - u_yaw

        self.publish_torque(u_left, u_right, "STOPPED")

    def publish_torque(self, u_left, u_right, mode):
        # đề phòng trường hợp lực bị vung lên quá lớn hoặc không có giá trị (Nan)
        if math.isnan(u_left) or math.isinf(u_left) or math.isnan(u_right) or math.isinf(u_right):
            return

        max_tau = 0.4
        u_left = max(min(u_left, max_tau), -max_tau)
        u_right = max(min(u_right, max_tau), -max_tau)
        
        msg_left = Float64()
        msg_left.data = u_left
        self.left_force_pub.publish(msg_left)

        msg_right = Float64()
        msg_right.data = u_right
        self.right_force_pub.publish(msg_right)

        log_msg = (f"[{mode}] X={self.x:.2f}, Y={self.y:.2f} | Góc={math.degrees(self.yaw):.0f}°")
        
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(log_msg)
    
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
