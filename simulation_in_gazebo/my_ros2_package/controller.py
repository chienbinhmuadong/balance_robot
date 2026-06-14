import rclpy
from rclpy.node import Node
import math
import numpy as np

# Import các kiểu tin nhắn (Messages) từ ROS 2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.R = 0.015  # Bán kính bánh xe (m)

        self.state = np.zeros(4)
        self.yaw = 0.0
        
        # --- BIẾN QUẢN LÝ TRẠNG THÁI (STATE MACHINE) ---
        self.robot_state = 'MOVING' # Chế độ hiện tại: 'MOVING' hoặc 'STOPPED'
        self.start_a = 0.0          # Mốc vị trí bắt đầu đoạn đường 2m
        self.stop_a = 0.0           # Vị trí chốt để giữ thăng bằng khi dừng
        self.t_start_move = None    # Mốc thời gian bắt đầu di chuyển
        self.t_start_stop = None    # Mốc thời gian bắt đầu dừng
        
        # regulator K
        self.K1 = np.array([0.0387, 0.0792, 0.3590, 0.0694])
        self.K2 = np.array([-0.1483, -0.3034, -0.2363, 0.000])

        self.imu_subscriber = self.create_subscription(Imu, '/simple_robot/imu', self.imu_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/simple_robot/joint_states', self.joint_callback, 10)

        self.left_force_pub = self.create_publisher(Float64, '/model/simple_robot/joint/left_wheel_joint/cmd_force', 10)
        self.right_force_pub = self.create_publisher(Float64, '/model/simple_robot/joint/right_wheel_joint/cmd_force', 10)
        
        self.timer = self.create_timer(0.01, self.controller)

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.x + q.y * q.z)
        cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        phi = math.atan2(sinp, cosp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        phi_dot = msg.angular_velocity.x
        self.state[2] = phi
        self.state[3] = phi_dot

    def joint_callback(self, msg):
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')

            theta_avg = (msg.position[left_idx] + msg.position[right_idx]) / 2.0
            theta_dot_avg = (msg.velocity[left_idx] + msg.velocity[right_idx]) / 2.0

            self.state[0] = self.R * theta_avg
            self.state[1] = self.R * theta_dot_avg
        except ValueError:
            pass

    def controller(self):
        """Hàm quản lý máy trạng thái tổng (Chạy 100Hz)"""
        current_time = self.get_clock().now()

        # Khởi tạo lần chạy đầu tiên
        if self.t_start_move is None:
            self.t_start_move = current_time
            self.start_a = self.state[0]

        if self.robot_state == 'MOVING':
            # Quy đổi sang mm để tính toán khoảng cách nội bộ nhằm đảm bảo độ chính xác không gian tuyệt đối
            distance_mm = (self.state[0] - self.start_a) * 1000.0
            
            if distance_mm >= 1000.0:  # Đã đi đủ 2 mét
                self.robot_state = 'STOPPED'
                self.t_start_stop = current_time
                self.stop_a = self.state[0] # Chốt vị trí hiện tại để giữ thăng bằng
            else:
                self.go_control_loop(current_time)

        elif self.robot_state == 'STOPPED':
            # Tính thời gian đã dừng
            t_stop = (current_time - self.t_start_stop).nanoseconds / 1e9
            
            if t_stop >= 1.0: # Đã dừng đủ 1 giây
                self.robot_state = 'MOVING'
                self.t_start_move = current_time
                self.start_a = self.state[0] # Thiết lập mốc 0 mới cho chặng 2m tiếp theo
            else:
                self.stop_controll()

    def go_control_loop(self, current_time):
        """Điều khiển bám quỹ đạo w trong chặng 2m"""
        # Tính thời gian t nội bộ của riêng đoạn đường này
        t = (current_time - self.t_start_move).nanoseconds / 1e9
        w = np.array([0.00, 0.01*t, 0.01, 0])

        # TẠO TRẠNG THÁI ẢO: Xem vị trí bắt đầu đoạn đường này là 0m
        virtual_state = np.copy(self.state)
        virtual_state[0] = self.state[0] - self.start_a 

        if virtual_state[0]==0 and virtual_state[1]==0 and virtual_state[2]==0 and virtual_state[3]==0:
            u = 0
        else:
            u_x = np.dot(self.K1, virtual_state)
            u_w = np.dot(self.K2, w)
            u = u_x + u_w

        self.publish_torque(u, virtual_state[0], "MOVING")

    def stop_controll(self):
        """Giữ thăng bằng tại chỗ"""
        # TẠO TRẠNG THÁI ẢO: Xem vị trí đang đứng là điểm 0 lý tưởng để LQR không kéo xe giật lùi
        virtual_state = np.copy(self.state)
        virtual_state[0] = self.state[0] - self.stop_a

        # Khi dừng, ta không có quỹ đạo w (u_w = 0)
        u = np.dot(self.K1, virtual_state)
        
        self.publish_torque(u, self.state[0], "STOPPED")

    def publish_torque(self, u, display_pos, mode):
        tau_each_wheel = u / 2.0
        max_tau = 0.4
        tau_each_wheel = max(min(tau_each_wheel, max_tau), -max_tau)
        
        force_msg = Float64()
        force_msg.data = tau_each_wheel
        self.left_force_pub.publish(force_msg)
        self.right_force_pub.publish(force_msg)

        log_msg = (
            f"\n--- MODE: {mode} ---\n"
            f"tọa độ x đạt được: {self.state[0]}"
        )
        self.get_logger().info(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()