#!/usr/bin/env python3
"""
ROS 2 Node for 19-State Error-State Extended Kalman Filter (ES-EKF).
Handles message conversion, sensor synchronization, time tracking,
parameter declaration, and watchdog diagnostics.
"""

import math
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, FluidPressure, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from .algorithm import ESEKF19Algorithm


class ESEKF19Node(Node):
    """
    ROS 2 Node encapsulating ESEKF19Algorithm.
    Responsible exclusively for ROS 2 interfacing (parameters, publishers, subscriptions, diagnostics).
    """

    def __init__(self):
        super().__init__('esekf19_node')

        # =====================================================================
        # 1. KHAI BÁO ROS PARAMETERS VỚI GIÁ TRỊ MẶC ĐỊNH (ĐỌC TỪ params.yaml)
        # =====================================================================
        # Topic Names & Frame IDs
        self.declare_parameter('topics.imu', '/imu/data')
        self.declare_parameter('topics.dvl', '/dvl/velocity')
        self.declare_parameter('topics.depth', '/depth/pressure')
        self.declare_parameter('topics.gnss_odom', '/gnss/odometry')
        self.declare_parameter('topics.output_odom', '/odometry/filtered')
        self.declare_parameter('topics.output_calib', '/esekf/calibration')

        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter('frames.base_frame', 'base_link')

        # Cấu hình vật lý & hình học
        self.declare_parameter('system.gravity', 9.80665)
        self.declare_parameter('system.latitude_deg', 20.8)
        self.declare_parameter('system.water_density', 1025.0)       # kg/m^3 (nước biển chuẩn)
        self.declare_parameter('system.atmospheric_pressure', 101325.0) # Pa
        self.declare_parameter('system.lever_arm', [0.5, -0.1, 0.2])  # m [x, y, z] từ IMU đến DVL

        # Đặc tính nhiễu cảm biến (Specs GEO-FOG 3D & DVL A125)
        self.declare_parameter('noise.acc_vrw', 0.002)               # m/s/sqrt(s)
        self.declare_parameter('noise.gyro_arw', 3.49e-6)           # rad/sqrt(s)
        self.declare_parameter('noise.acc_bias_sigma', 1e-4)        # m/s^2
        self.declare_parameter('noise.acc_bias_tau', 3600.0)        # s
        self.declare_parameter('noise.gyro_bias_rw', 1e-6)          # rad/s/sqrt(s)
        self.declare_parameter('noise.scale_rw', 1e-5)              # 1/sqrt(s)
        self.declare_parameter('noise.boresight_rw', 1e-5)          # rad/sqrt(s)

        self.declare_parameter('noise.dvl_noise_floor', 0.01)       # m/s
        self.declare_parameter('noise.dvl_noise_slope', 0.005)
        self.declare_parameter('noise.depth_sigma', 0.05)           # m
        self.declare_parameter('noise.gnss_pos_sigma', 0.02)        # m
        self.declare_parameter('noise.gnss_vel_sigma', 0.02)        # m/s
        self.declare_parameter('noise.gnss_yaw_sigma', 0.0017)      # rad (~0.1 deg)

        # Trạng thái ban đầu & Timeout
        self.declare_parameter('initial_state.position', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_state.velocity', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_state.rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('operation.lock_calibration_underwater', True)
        self.declare_parameter('operation.gnss_timeout_sec', 5.0)
        self.declare_parameter('operation.watchdog_sensor_timeout_sec', 3.0)
        self.declare_parameter('operation.max_dt_threshold', 0.5)   # Ngưỡng phát hiện đứt quãng thời gian

        # =====================================================================
        # 2. KHỞI TẠO THUẬT TOÁN ĐỘC LẬP TỪ THAM SỐ
        # =====================================================================
        algo_cfg = {
            'gravity': self.get_parameter('system.gravity').value,
            'latitude_deg': self.get_parameter('system.latitude_deg').value,
            'lever_arm': self.get_parameter('system.lever_arm').value,
            'acc_vrw': self.get_parameter('noise.acc_vrw').value,
            'gyro_arw': self.get_parameter('noise.gyro_arw').value,
            'acc_bias_sigma': self.get_parameter('noise.acc_bias_sigma').value,
            'acc_bias_tau': self.get_parameter('noise.acc_bias_tau').value,
            'gyro_bias_rw': self.get_parameter('noise.gyro_bias_rw').value,
            'scale_rw': self.get_parameter('noise.scale_rw').value,
            'boresight_rw': self.get_parameter('noise.boresight_rw').value,
            'dvl_noise_floor': self.get_parameter('noise.dvl_noise_floor').value,
            'dvl_noise_slope': self.get_parameter('noise.dvl_noise_slope').value,
            'depth_sigma': self.get_parameter('noise.depth_sigma').value,
            'gnss_pos_sigma': self.get_parameter('noise.gnss_pos_sigma').value,
            'gnss_vel_sigma': self.get_parameter('noise.gnss_vel_sigma').value,
            'gnss_yaw_sigma': self.get_parameter('noise.gnss_yaw_sigma').value,
            'initial_position': self.get_parameter('initial_state.position').value,
            'initial_velocity': self.get_parameter('initial_state.velocity').value,
            'initial_rpy_deg': self.get_parameter('initial_state.rpy_deg').value,
            'lock_calibration_underwater': self.get_parameter('operation.lock_calibration_underwater').value
        }
        self.filter = ESEKF19Algorithm(algo_cfg)

        # Lấy các thông số node sử dụng
        self.water_density = float(self.get_parameter('system.water_density').value)
        self.p_atm = float(self.get_parameter('system.atmospheric_pressure').value)
        self.gravity = float(self.get_parameter('system.gravity').value)
        self.odom_frame = str(self.get_parameter('frames.odom_frame').value)
        self.base_frame = str(self.get_parameter('frames.base_frame').value)
        self.max_dt = float(self.get_parameter('operation.max_dt_threshold').value)
        self.watchdog_timeout = float(self.get_parameter('operation.watchdog_sensor_timeout_sec').value)
        self.gnss_timeout = float(self.get_parameter('operation.gnss_timeout_sec').value)

        # Biến quản lý thời gian và an toàn dt
        self.last_imu_time: Optional[float] = None
        self.last_gyro_meas = np.zeros(3, dtype=np.float64)

        # Watchdog theo dõi thời gian nhận message cuối cùng của từng cảm biến
        self.last_rx_imu: Optional[float] = None
        self.last_rx_dvl: Optional[float] = None
        self.last_rx_depth: Optional[float] = None
        self.last_rx_gnss: Optional[float] = None

        # =====================================================================
        # 3. THIẾT LẬP PUBLISHER & SUBSCRIPTION (QoS sensor_data)
        # =====================================================================
        imu_topic = str(self.get_parameter('topics.imu').value)
        dvl_topic = str(self.get_parameter('topics.dvl').value)
        depth_topic = str(self.get_parameter('topics.depth').value)
        gnss_topic = str(self.get_parameter('topics.gnss_odom').value)
        odom_pub_topic = str(self.get_parameter('topics.output_odom').value)
        calib_pub_topic = str(self.get_parameter('topics.output_calib').value)

        # Subscriptions
        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.sub_dvl = self.create_subscription(
            TwistWithCovarianceStamped, dvl_topic, self.dvl_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            FluidPressure, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.sub_gnss = self.create_subscription(
            Odometry, gnss_topic, self.gnss_callback, qos_profile_sensor_data)

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, odom_pub_topic, 10)
        self.pub_calib = self.create_publisher(Float64MultiArray, calib_pub_topic, 10)

        # Timer kiểm tra Watchdog tình trạng cảm biến (chạy 1 Hz)
        self.timer_watchdog = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info('Node ES-EKF 19-trạng thái đã khởi động thành công!')
        self.get_logger().info(f'Đang lắng nghe: IMU [{imu_topic}], DVL [{dvl_topic}], Sâu [{depth_topic}], GNSS [{gnss_topic}]')

    # =========================================================================
    # 4. CALLBACKS CẢM BIẾN
    # =========================================================================
    def imu_callback(self, msg: Imu):
        """
        Bước dự báo quán tính INS (Predict) chạy theo chu kỳ phát của IMU.
        Tính dt an toàn từ timestamp header.
        """
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_rx_imu = self.get_clock().now().nanoseconds * 1e-9

        # Xử lý an toàn message đầu tiên
        if self.last_imu_time is None:
            self.last_imu_time = t_now
            self.get_logger().info('Đã nhận message IMU đầu tiên. Bắt đầu bộ lọc.')
            return

        dt = t_now - self.last_imu_time

        # Kiểm tra an toàn: dt <= 0 (thời gian lùi hoặc trùng lặp)
        if dt <= 0.0:
            self.get_logger().warn(f'Bỏ qua message IMU do dt <= 0 ({dt:.6f}s)')
            return

        # Kiểm tra khoảng trống thời gian lớn (drop package)
        if dt > self.max_dt:
            self.get_logger().warn(
                f'Phát hiện đứt quãng thời gian IMU: dt = {dt:.4f}s > {self.max_dt}s. Đặt lại dt an toàn.')
            dt = 0.01  # Giá trị danh định an toàn

        self.last_imu_time = t_now

        # Trích xuất dữ liệu đo thô
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float64)
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float64)

        self.last_gyro_meas = gyro

        # Chạy bước dự báo INS trong thuật toán
        self.filter.predict(gyro, accel, dt)

        # Xuất Odometry ước lượng tức thời sau mỗi bước INS
        self.publish_odometry(msg.header.stamp)

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        """
        Bước cập nhật vận tốc đáy DVL (Update DVL).
        Giả định: Vận tốc DVL đo trong khung thân/DVL (vx, vy, vz).
        """
        self.last_rx_dvl = self.get_clock().now().nanoseconds * 1e-9

        # TODO: Nếu partner dùng message DVL custom (ví dụ acoustic_msgs/Dvl),
        # hãy ánh xạ các trường vận tốc vào vector dvl_meas dưới đây:
        dvl_meas = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ], dtype=np.float64)

        accepted, nis = self.filter.update_dvl(dvl_meas, self.last_gyro_meas)
        if not accepted:
            self.get_logger().debug(f'Cổng lọc Chi-square từ chối ngoại lai DVL (NIS={nis:.2f})')

        # Xuất thông tin hiệu chuẩn Boresight và Scale định kỳ
        self.publish_calibration(msg.header.stamp)

    def depth_callback(self, msg: FluidPressure):
        """
        Bước cập nhật độ sâu (Update Depth).
        Chuyển đổi áp suất thủy tĩnh (Pascals) sang độ sâu z (mét).
        """
        self.last_rx_depth = self.get_clock().now().nanoseconds * 1e-9

        # TODO: Nếu mô phỏng Gazebo phát độ sâu trực tiếp dạng PoseStamped hoặc Float64,
        # bỏ qua phép tính áp suất và gán trực tiếp depth_m = msg.data hoặc msg.pose.position.z.
        pressure_pa = msg.fluid_pressure
        depth_m = max(0.0, (pressure_pa - self.p_atm) / (self.water_density * self.gravity))

        accepted, nis = self.filter.update_depth(depth_m)
        if not accepted:
            self.get_logger().debug(f'Cổng lọc Chi-square từ chối độ sâu (NIS={nis:.2f})')

    def gnss_callback(self, msg: Odometry):
        """
        Bước cập nhật GNSS khi nổi ở mặt nước.
        Giả định: Nhận vị trí NED và vận tốc NED từ node GNSS/RTK.
        """
        self.last_rx_gnss = self.get_clock().now().nanoseconds * 1e-9

        # TODO: Nếu partner gửi dạng sensor_msgs/NavSatFix, cần thêm bộ chuyển WGS84 -> NED.
        pos_ned = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=np.float64)

        vel_ned = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ], dtype=np.float64)

        # Tính yaw từ orientation quaternion của GNSS dual antenna
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_meas = math.atan2(siny_cosp, cosy_cosp)

        accepted, nis = self.filter.update_gnss(pos_ned, vel_ned, yaw_meas)
        if not accepted:
            self.get_logger().debug(f'Cổng lọc Chi-square từ chối GNSS (NIS={nis:.2f})')

    # =========================================================================
    # 5. WATCHDOG CHẨN ĐOÁN VÀ AN TOÀN HỆ THỐNG
    # =========================================================================
    def watchdog_callback(self):
        """Kiểm tra định kỳ 1Hz xem có topic cảm biến nào bị mất tín hiệu."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Kiểm tra IMU (Sống còn)
        if self.last_rx_imu is not None and (now_sec - self.last_rx_imu) > self.watchdog_timeout:
            self.get_logger().error(f'MẤT TÍN HIỆU IMU! Đã {(now_sec - self.last_rx_imu):.1f}s không nhận được dữ liệu.')

        # Kiểm tra DVL
        if self.last_rx_dvl is not None and (now_sec - self.last_rx_dvl) > self.watchdog_timeout:
            self.get_logger().warn(f'Mất tín hiệu DVL (Mất đáy/Drop-out)! Đã {(now_sec - self.last_rx_dvl):.1f}s.')

        # Kiểm tra Depth
        if self.last_rx_depth is not None and (now_sec - self.last_rx_depth) > self.watchdog_timeout:
            self.get_logger().warn(f'Mất tín hiệu cảm biến áp suất độ sâu! Đã {(now_sec - self.last_rx_depth):.1f}s.')

        # Kiểm tra GNSS (Khi lặn ngầm > timeout, tự động bật cờ khóa hiệu chuẩn Boresight)
        is_submerged = (self.last_rx_gnss is None) or ((now_sec - self.last_rx_gnss) > self.gnss_timeout)
        self.filter.set_calibration_lock(is_submerged)

    # =========================================================================
    # 6. ĐÓNG GÓI OUTPUT MESSAGES
    # =========================================================================
    def publish_odometry(self, stamp):
        """Xuất thông điệp nav_msgs/Odometry chứa vị trí, tư thế, vận tốc và ma trận P."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        # Vị trí NED (x: North, y: East, z: Down)
        msg.pose.pose.position.x = float(self.filter.pos[0])
        msg.pose.pose.position.y = float(self.filter.pos[1])
        msg.pose.pose.position.z = float(self.filter.pos[2])

        # Quaternion tư thế
        msg.pose.pose.orientation.w = float(self.filter.quat[0])
        msg.pose.pose.orientation.x = float(self.filter.quat[1])
        msg.pose.pose.orientation.y = float(self.filter.quat[2])
        msg.pose.pose.orientation.z = float(self.filter.quat[3])

        # Vận tốc NED
        msg.twist.twist.linear.x = float(self.filter.vel[0])
        msg.twist.twist.linear.y = float(self.filter.vel[1])
        msg.twist.twist.linear.z = float(self.filter.vel[2])

        # Trích xuất 6x6 hiệp phương sai tư thế + vị trí từ ma trận P (19x19)
        # ROS 2 Pose covariance thứ tự: [x, y, z, roll, pitch, yaw]
        pose_cov_6x6 = np.zeros((6, 6), dtype=np.float64)
        pose_cov_6x6[0:3, 0:3] = self.filter.P[0:3, 0:3]
        pose_cov_6x6[3:6, 3:6] = self.filter.P[6:9, 6:9]
        msg.pose.covariance = list(pose_cov_6x6.flatten())

        twist_cov_6x6 = np.zeros((6, 6), dtype=np.float64)
        twist_cov_6x6[0:3, 0:3] = self.filter.P[3:6, 3:6]
        msg.twist.covariance = list(twist_cov_6x6.flatten())

        self.pub_odom.publish(msg)

    def publish_calibration(self, stamp):
        """
        Xuất các tham số hiệu chuẩn trực tuyến để theo dõi hội tụ:
        [bs_roll_deg, bs_pitch_deg, bs_yaw_deg, scale_percent, ba_x_mg, ba_y_mg, ba_z_mg, bg_x_dph, bg_y_dph, bg_z_dph]
        """
        bs_r, bs_p, bs_y = self.filter.get_boresight_deg()
        scale_pct = self.filter.get_scale_percent()
        ba_mg = (self.filter.accel_bias / self.gravity) * 1000.0
        bg_dph = np.degrees(self.filter.gyro_bias) * 3600.0

        msg = Float64MultiArray()
        msg.data = [
            float(bs_r), float(bs_p), float(bs_y), float(scale_pct),
            float(ba_mg[0]), float(ba_mg[1]), float(ba_mg[2]),
            float(bg_dph[0]), float(bg_dph[1]), float(bg_dph[2])
        ]
        self.pub_calib.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ESEKF19Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Đã nhận lệnh dừng từ người dùng.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
