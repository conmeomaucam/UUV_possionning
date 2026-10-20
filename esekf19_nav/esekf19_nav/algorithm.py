"""
Error-State Extended Kalman Filter (19-State ES-EKF) for AUV/UUV Navigation.
Independent Algorithm Module (NO ROS DEPENDENCIES).

States (20 Nominal):
  - pos:        [p_N, p_E, p_D] in NED frame (m)
  - vel:        [v_N, v_E, v_D] in NED frame (m/s)
  - quat:       [q_w, q_x, q_y, q_z] orientation Body -> NED
  - accel_bias: [b_ax, b_ay, b_az] in Body frame (m/s^2)
  - gyro_bias:  [b_gx, b_gy, b_gz] in Body frame (rad/s)
  - scale:      DVL scale factor error s (dimensionless)
  - boresight:  DVL misalignment Euler angles [alpha, beta, gamma] (rad)

Error States (19 Dimensions):
  delta_x = [delta_p(3), delta_v(3), delta_theta(3), delta_ba(3), delta_bg(3), delta_s(1), delta_boresight(3)]
"""

import math
from typing import Dict, Optional, Tuple
import numpy as np


def skew_mat(v: np.ndarray) -> np.ndarray:
    """Return 3x3 skew-symmetric matrix of a 3D vector."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=np.float64)


def quat_to_cbn(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w, x, y, z] to DCM rotation matrix C_b^n."""
    q_norm = np.linalg.norm(q)
    if q_norm > 1e-12:
        q = q / q_norm
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.array([
        [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x * y - w * z),     2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z),     1.0 - 2.0 * (x**2 + z**2), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y),     2.0 * (y * z + w * x),     1.0 - 2.0 * (x**2 + y**2)]
    ], dtype=np.float64)


def cbn_to_euler(cbn: np.ndarray) -> Tuple[float, float, float]:
    """Convert C_b^n matrix to Euler angles (roll, pitch, yaw) in radians (Z-Y-X sequence)."""
    roll = math.atan2(cbn[2, 1], cbn[2, 2])
    sin_pitch = -np.clip(cbn[2, 0], -1.0, 1.0)
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(cbn[1, 0], cbn[0, 0])
    return roll, pitch, yaw


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles (roll, pitch, yaw) in radians to quaternion [w, x, y, z]."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr
    ], dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product q1 * q2 for [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)


def rodrigues_so3(phi: np.ndarray) -> np.ndarray:
    """Exponential map from so(3) Lie algebra vector to SO(3) rotation matrix."""
    theta = float(np.linalg.norm(phi))
    K = skew_mat(phi)
    if theta > 1e-8:
        return np.eye(3) + (math.sin(theta)/theta)*K + ((1.0 - math.cos(theta))/(theta**2))*(K @ K)
    else:
        return np.eye(3) + K + 0.5 * (K @ K)


def so3_log(R: np.ndarray) -> np.ndarray:
    """Logarithmic map from SO(3) rotation matrix to so(3) Lie algebra vector."""
    cos_theta = float(np.clip(0.5 * (np.trace(R) - 1.0), -1.0, 1.0))
    theta = math.acos(cos_theta)
    if theta > 1e-8:
        factor = theta / (2.0 * math.sin(theta))
        return factor * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]], dtype=np.float64)
    else:
        return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]], dtype=np.float64)


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ESEKF19Algorithm:
    """
    19-State Error-State Extended Kalman Filter for Autonomous Underwater Vehicles.
    Fully standalone implementation with online DVL Boresight and Scale factor estimation.
    """

    def __init__(self, config: Optional[Dict] = None):
        cfg = config or {}

        # Vật lý Trái Đất và môi trường
        self.gravity = float(cfg.get('gravity', 9.80665))
        self.latitude = float(cfg.get('latitude_deg', 20.8)) * math.pi / 180.0
        self.height = float(cfg.get('height_m', 0.0))
        self.radius_n = 6378137.0
        self.radius_m = 6356752.3
        self.tan_lat = math.tan(self.latitude)
        self.w_ie_ned = np.array([0.0, 0.0, 7.292115e-5], dtype=np.float64)

        # Cấu hình cảm biến & hình học cánh tay đòn
        lever_arm_list = cfg.get('lever_arm', [0.5, -0.1, 0.2])
        self.lever_arm = np.array(lever_arm_list, dtype=np.float64).reshape((3,))

        # Độ bất định và nhiễu cảm biến
        self.acc_vrw = float(cfg.get('acc_vrw', 0.002))               # m/s/sqrt(s)
        self.gyro_arw = float(cfg.get('gyro_arw', 3.49e-6))           # rad/sqrt(s)
        self.acc_bias_sigma = float(cfg.get('acc_bias_sigma', 1e-4))   # m/s^2
        self.acc_bias_tau = float(cfg.get('acc_bias_tau', 3600.0))     # s
        self.gyro_bias_rw = float(cfg.get('gyro_bias_rw', 1e-6))       # rad/s/sqrt(s)
        self.scale_rw = float(cfg.get('scale_rw', 1e-5))               # 1/sqrt(s)
        self.boresight_rw = float(cfg.get('boresight_rw', 1e-5))       # rad/sqrt(s)

        # Phương sai đo đạc
        self.dvl_noise_floor = float(cfg.get('dvl_noise_floor', 0.01)) # m/s
        self.dvl_noise_slope = float(cfg.get('dvl_noise_slope', 0.005))
        self.gnss_pos_sigma = float(cfg.get('gnss_pos_sigma', 0.02))   # m (RTK)
        self.gnss_vel_sigma = float(cfg.get('gnss_vel_sigma', 0.02))   # m/s
        self.gnss_yaw_sigma = float(cfg.get('gnss_yaw_sigma', 0.0017)) # rad (~0.1 deg)
        self.depth_sigma = float(cfg.get('depth_sigma', 0.05))         # m

        # Cờ khóa hiệu chuẩn (khi lặn ngầm không có GNSS)
        self.lock_calibration = bool(cfg.get('lock_calibration_underwater', True))

        # Khởi tạo trạng thái danh định 20D
        init_pos = cfg.get('initial_position', [0.0, 0.0, 0.0])
        init_vel = cfg.get('initial_velocity', [0.0, 0.0, 0.0])
        init_rpy = cfg.get('initial_rpy_deg', [0.0, 0.0, 0.0])

        rpy_rad = [math.radians(a) for a in init_rpy]
        self.pos = np.array(init_pos, dtype=np.float64).reshape((3,))
        self.vel = np.array(init_vel, dtype=np.float64).reshape((3,))
        self.quat = euler_to_quat(rpy_rad[0], rpy_rad[1], rpy_rad[2])
        self.accel_bias = np.zeros(3, dtype=np.float64)
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.scale = 0.0
        self.boresight = np.zeros(3, dtype=np.float64)

        # Khởi tạo ma trận P (19x19)
        pos_var = float(cfg.get('init_pos_std', 0.02))**2
        vel_var = float(cfg.get('init_vel_std', 0.05))**2
        att_var = math.radians(float(cfg.get('init_att_std_deg', 0.5)))**2
        ba_var = (0.5e-3 * self.gravity)**2
        bg_var = math.radians(float(cfg.get('init_gyro_bias_std_deg_hr', 0.5)) / 3600.0)**2
        scale_var = float(cfg.get('init_scale_std', 0.02))**2
        bs_var = math.radians(float(cfg.get('init_boresight_std_deg', 1.5)))**2

        p_diag = [
            pos_var, pos_var, pos_var,
            vel_var, vel_var, vel_var,
            att_var, att_var, att_var,
            ba_var, ba_var, ba_var,
            bg_var, bg_var, bg_var,
            scale_var,
            bs_var, bs_var, bs_var
        ]
        self.P = np.diag(p_diag)

        # Ma trận Q tích lũy liên tục (sẽ rời rạc hóa theo dt)
        self.Q_base = np.zeros((19, 19), dtype=np.float64)
        self.Q_base[0:3, 0:3] = (self.acc_vrw**2) * np.eye(3)
        self.Q_base[3:6, 3:6] = (self.acc_vrw**2) * np.eye(3)
        self.Q_base[6:9, 6:9] = (self.gyro_arw**2) * np.eye(3)
        q_ba = (2.0 * (self.acc_bias_sigma**2) / max(self.acc_bias_tau, 1e-3))
        self.Q_base[9:12, 9:12] = q_ba * np.eye(3)
        self.Q_base[12:15, 12:15] = (self.gyro_bias_rw**2) * np.eye(3)
        self.Q_base[15, 15] = self.scale_rw**2
        self.Q_base[16:19, 16:19] = (self.boresight_rw**2) * np.eye(3)

    # -------------------------------------------------------------------------
    # 1. BƯỚC DỰ BÁO ĐỘNG HỌC QUÁN TÍNH INS (IMU Callback)
    # -------------------------------------------------------------------------
    def predict(self, gyro_meas: np.ndarray, accel_meas: np.ndarray, dt: float):
        """
        Dự báo động học quán tính và lan truyền ma trận hiệp phương sai P (19x19).
        gyro_meas: [wx, wy, wz] (rad/s)
        accel_meas: [ax, ay, az] (m/s^2)
        dt: thời gian bước tích phân (s)
        """
        if dt <= 0.0 or dt > 1.0:
            return

        Cbn = quat_to_cbn(self.quat)
        Cnb = Cbn.T

        # Bù bias ước lượng
        accel_cor = accel_meas - self.accel_bias
        gyro_cor = gyro_meas - self.gyro_bias

        # Vận tốc góc chuyển dời khung NED (Transport rate)
        w_en_ned = np.array([
            self.vel[1] / (self.radius_n + self.height),
            -self.vel[0] / (self.radius_m + self.height),
            -self.vel[1] * self.tan_lat / (self.radius_n + self.height)
        ], dtype=np.float64)

        # Vận tốc góc thân tương đối
        w_nb_body = gyro_cor - Cnb @ (self.w_ie_ned + w_en_ned)

        # Gia tốc trong hệ NED: Lực riêng + Trọng lực - Coriolis
        coriolis = np.cross(2.0 * self.w_ie_ned + w_en_ned, self.vel)
        accel_ned = Cbn @ accel_cor + np.array([0.0, 0.0, self.gravity]) - coriolis

        # Tích phân vị trí và vận tốc (Euler)
        self.vel = self.vel + accel_ned * dt
        self.pos = self.pos + self.vel * dt

        # Cập nhật quaternion
        delta_angle = w_nb_body * dt
        angle_norm = float(np.linalg.norm(delta_angle))
        if angle_norm > 1e-12:
            dq = np.array([
                math.cos(angle_norm * 0.5),
                math.sin(angle_norm * 0.5) * delta_angle[0] / angle_norm,
                math.sin(angle_norm * 0.5) * delta_angle[1] / angle_norm,
                math.sin(angle_norm * 0.5) * delta_angle[2] / angle_norm
            ], dtype=np.float64)
        else:
            dq = np.array([1.0, 0.5*delta_angle[0], 0.5*delta_angle[1], 0.5*delta_angle[2]], dtype=np.float64)
        self.quat = quat_multiply(self.quat, dq)
        self.quat = self.quat / np.linalg.norm(self.quat)

        # Suy giảm bias gia tốc kế theo Gauss-Markov
        phi_acc = math.exp(-dt / max(self.acc_bias_tau, 1e-6))
        self.accel_bias = phi_acc * self.accel_bias

        # Ma trận chuyển tiếp sai số trạng thái phi tuyến F (19x19)
        F = np.eye(19, dtype=np.float64)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -Cbn @ skew_mat(accel_cor) * dt
        F[3:6, 9:12] = -Cbn * dt
        F[6:9, 6:9] = np.eye(3) - skew_mat(w_nb_body) * dt
        F[6:9, 12:15] = -np.eye(3) * dt
        F[9:12, 9:12] = phi_acc * np.eye(3)

        # Rời rạc hóa ma trận Q
        Qd = self.Q_base * dt
        if self.lock_calibration:
            Qd[15:19, 15:19] = 0.0

        # Lan truyền covariance
        self.P = F @ self.P @ F.T + Qd
        self.P = 0.5 * (self.P + self.P.T)

    # -------------------------------------------------------------------------
    # 2. CẬP NHẬT GNSS RTK (Mặt nước: Vị trí, Vận tốc, Heading 2 Antennas)
    # -------------------------------------------------------------------------
    def update_gnss(self, pos_ned: np.ndarray, vel_ned: np.ndarray, yaw_meas: float) -> Tuple[bool, float]:
        """
        Cập nhật GNSS 7 chiều với cổng lọc Chi-square NIS 99% (threshold = 18.48).
        """
        Cbn = quat_to_cbn(self.quat)
        _, pitch_est, yaw_est = cbn_to_euler(Cbn)
        yaw_res = wrap_to_pi(yaw_meas - yaw_est)

        res = np.zeros(7, dtype=np.float64)
        res[0:3] = pos_ned - self.pos
        res[3:6] = vel_ned - self.vel
        res[6] = yaw_res

        H = np.zeros((7, 19), dtype=np.float64)
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)
        roll_est = math.atan2(Cbn[2, 1], Cbn[2, 2])
        cosp = max(math.cos(pitch_est), 1e-4)
        H[6, 6:9] = [0.0, math.sin(roll_est) / cosp, math.cos(roll_est) / cosp]

        r_diag = [
            self.gnss_pos_sigma**2, self.gnss_pos_sigma**2, self.gnss_pos_sigma**2,
            self.gnss_vel_sigma**2, self.gnss_vel_sigma**2, self.gnss_vel_sigma**2,
            self.gnss_yaw_sigma**2
        ]
        R = np.diag(r_diag)

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
            nis = float(res.T @ S_inv @ res)
        except np.linalg.LinAlgError:
            return False, 999.0

        if not np.isfinite(nis) or nis > 18.48:  # 99% Chi-square (7-DOF)
            return False, nis

        K = self.P @ H.T @ S_inv
        delta_x = K @ res
        self._inject_error(delta_x, H, R, K)
        return True, nis

    # -------------------------------------------------------------------------
    # 3. CẬP NHẬT CẢM BIẾN ĐỘ SÂU (Depth Sensor / Pressure)
    # -------------------------------------------------------------------------
    def update_depth(self, depth_meas: float) -> Tuple[bool, float]:
        """
        Cập nhật độ sâu (trục Down) với cổng Chi-square 1-DOF (threshold = 6.63).
        """
        res = depth_meas - self.pos[2]
        H = np.zeros((1, 19), dtype=np.float64)
        H[0, 2] = 1.0

        R = np.array([[self.depth_sigma**2]], dtype=np.float64)
        S = H @ self.P @ H.T + R
        s_val = float(S[0, 0])
        if s_val <= 0.0:
            return False, 999.0

        nis = float((res**2) / s_val)
        if not np.isfinite(nis) or nis > 6.63:  # 99% Chi-square (1-DOF)
            return False, nis

        K = (self.P @ H.T) / s_val
        delta_x = (K * res).flatten()
        self._inject_error(delta_x, H, R, K)
        return True, nis

    # -------------------------------------------------------------------------
    # 4. CẬP NHẬT VẬN TỐC ĐÁY DVL (A125 / Boresight + Scale Factor)
    # -------------------------------------------------------------------------
    def update_dvl(self, dvl_meas: np.ndarray, gyro_meas: np.ndarray) -> Tuple[bool, float]:
        """
        Cập nhật vận tốc DVL 3D với mô hình hiệu chuẩn Boresight Lie group và Scale factor.
        dvl_meas: [vx, vy, vz] đo tại khung DVL (m/s).
        """
        Cbn = quat_to_cbn(self.quat)
        Cnb = Cbn.T
        v_body = Cnb @ self.vel

        # Bù vận tốc góc tương đối tại DVL
        gyro_cor = gyro_meas - self.gyro_bias
        w_en_ned = np.array([
            self.vel[1] / (self.radius_n + self.height),
            -self.vel[0] / (self.radius_m + self.height),
            -self.vel[1] * self.tan_lat / (self.radius_n + self.height)
        ], dtype=np.float64)
        w_in_b = Cnb @ (self.w_ie_ned + w_en_ned)
        body_rate_rel = gyro_cor - w_in_b

        # Vận tốc tại vị trí lắp đặt DVL (bù cánh tay đòn)
        v_lever = v_body + np.cross(body_rate_rel, self.lever_arm)

        # Ma trận quay Boresight
        Cbs = rodrigues_so3(-self.boresight)

        # Dự báo đo DVL
        h_dvl = (1.0 + self.scale) * (Cbs @ v_lever)
        res = dvl_meas - h_dvl

        # Xây dựng Jacobian H_dvl (3x19)
        H = np.zeros((3, 19), dtype=np.float64)
        factor = (1.0 + self.scale) * Cbs
        H[:, 3:6] = factor @ Cnb
        H[:, 6:9] = factor @ (skew_mat(v_body) + skew_mat(self.lever_arm) @ skew_mat(w_in_b))
        H[:, 12:15] = factor @ skew_mat(self.lever_arm)

        v_norm = float(np.linalg.norm(v_lever))
        if v_norm > 1e-4:
            H[:, 15] = Cbs @ v_lever
            H[:, 16:19] = factor @ skew_mat(v_lever)

        # Nếu đang ở chế độ lặn ngầm khóa hiệu chuẩn:
        if self.lock_calibration:
            H[:, 15:19] = 0.0
            self.P[15:19, 0:15] = 0.0
            self.P[0:15, 15:19] = 0.0

        # Phương sai thích nghi theo vận tốc + Bù nhiễu gyro qua cánh tay đòn
        sigma_dvl = self.dvl_noise_floor + self.dvl_noise_slope * v_norm
        R = (sigma_dvl**2) * np.eye(3)
        dt_approx = 0.05
        sigma_g = self.gyro_arw / math.sqrt(dt_approx)
        skew_r = skew_mat(self.lever_arm)
        R_gyro_lever = ((1.0 + self.scale)**2) * (Cbs @ skew_r @ ((sigma_g**2) * np.eye(3)) @ skew_r.T @ Cbs.T)
        R = R + R_gyro_lever

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
            nis = float(res.T @ S_inv @ res)
        except np.linalg.LinAlgError:
            return False, 999.0

        if not np.isfinite(nis) or nis > 11.34:  # 99% Chi-square (3-DOF)
            return False, nis

        K = self.P @ H.T @ S_inv
        delta_x = K @ res

        if self.lock_calibration:
            delta_x[15:19] = 0.0

        self._inject_error(delta_x, H, R, K)
        return True, nis

    # -------------------------------------------------------------------------
    # 5. ERROR INJECTION VÀ RESET COVARIANCE TRÊN LIE GROUP SO(3)
    # -------------------------------------------------------------------------
    def _inject_error(self, delta_x: np.ndarray, H: np.ndarray, R: np.ndarray, K: np.ndarray):
        """Bơm sai số vào trạng thái danh định và cập nhật P theo công thức Joseph."""
        # Vị trí và vận tốc tuyến tính
        self.pos += delta_x[0:3]
        self.vel += delta_x[3:6]

        # Quaternion tư thế
        delta_theta = delta_x[6:9]
        dq = np.array([1.0, 0.5*delta_theta[0], 0.5*delta_theta[1], 0.5*delta_theta[2]], dtype=np.float64)
        dq = dq / np.linalg.norm(dq)
        self.quat = quat_multiply(self.quat, dq)
        self.quat = self.quat / np.linalg.norm(self.quat)

        # Bias cảm biến và Scale factor
        self.accel_bias += delta_x[9:12]
        self.gyro_bias += delta_x[12:15]
        self.scale += float(delta_x[15])

        # Góc lệch Boresight trên Lie group SO(3)
        delta_bs = delta_x[16:19]
        Cbs_new = rodrigues_so3(-self.boresight) @ rodrigues_so3(-delta_bs)
        self.boresight = -so3_log(Cbs_new)

        # Cập nhật Joseph cho P: P = (I - KH)P(I - KH)' + KRK'
        I_KH = np.eye(19, dtype=np.float64) - K @ H
        P_upd = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        # Ma trận Jacobian Reset G
        G = np.eye(19, dtype=np.float64)
        G[6:9, 6:9] = np.eye(3) - 0.5 * skew_mat(delta_theta)
        G[16:19, 16:19] = np.eye(3) + 0.5 * skew_mat(delta_bs)

        self.P = G @ P_upd @ G.T
        self.P = 0.5 * (self.P + self.P.T)

    # -------------------------------------------------------------------------
    # GETTERS TIỆN ÍCH CHO ROS NODE
    # -------------------------------------------------------------------------
    def get_euler_deg(self) -> Tuple[float, float, float]:
        r, p, y = cbn_to_euler(quat_to_cbn(self.quat))
        return math.degrees(r), math.degrees(p), math.degrees(y)

    def get_boresight_deg(self) -> Tuple[float, float, float]:
        return math.degrees(self.boresight[0]), math.degrees(self.boresight[1]), math.degrees(self.boresight[2])

    def get_scale_percent(self) -> float:
        return self.scale * 100.0

    def set_calibration_lock(self, locked: bool):
        self.lock_calibration = locked
