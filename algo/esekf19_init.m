function [x, P, Q] = esekf19_init(initial_rpy, gravity, noise_cfg)
%ESEKF19_INIT Khởi tạo trạng thái danh định (20x1), ma trận hiệp phương sai sai số P (19x19)
%   và ma trận hiệp phương sai nhiễu quá trình Q (19x19) cho bộ lọc ES-EKF.
%
%   Cú pháp:
%       [x, P, Q] = esekf19_init(initial_rpy, gravity, noise_cfg)
%
%   Đầu vào:
%       initial_rpy - Vector góc Euler ban đầu [roll; pitch; yaw] (rad)
%       gravity     - Gia tốc trọng trường chuẩn g (m/s^2)
%       noise_cfg   - Vector cấu hình nhiễu IMU và tham số hệ thống (8x1):
%                     (1) acc_vrw: Nhiễu ngẫu nhiên vận tốc gia tốc kế (VRW - m/s/sqrt(s))
%                     (2) gyro_arw: Nhiễu ngẫu nhiên góc con quay FOG (ARW - rad/sqrt(s))
%                     (3) acc_bias_sigma: Độ ổn định bias in-run của gia tốc kế (m/s^2)
%                     (4) acc_bias_tau: Thời gian tương quan Gauss-Markov của gia tốc kế (s)
%                     (5) gyro_bias_rw: Tốc độ trôi ngẫu nhiên bias con quay (rad/s/sqrt(s))
%                     (6) scale_rw: Tốc độ trôi hệ số tỉ lệ DVL s (1/sqrt(s))
%                     (7) boresight_rw: Tốc độ rung động cơ học góc Boresight DVL (rad/sqrt(s))
%                     (8) dt: Chu kỳ trích mẫu của IMU (s), ví dụ 1/400 = 0.0025 s
%
%   Đầu ra:
%       x - Vector trạng thái danh định 20 chiều (Nominal State):
%           x(1:3)   : Vị trí trong hệ tọa độ NED [p_N, p_E, p_D] (m)
%           x(4:6)   : Vận tốc trong hệ tọa độ NED [v_N, v_E, v_D] (m/s)
%           x(7:10)  : Quaternion biểu diễn tư thế UUV [q_w, q_x, q_y, q_z]
%           x(11:13) : Bias gia tốc kế khung thân [b_ax, b_ay, b_az] (m/s^2)
%           x(14:16) : Bias con quay khung thân [b_gx, b_gy, b_gz] (rad/s)
%           x(17)    : Hệ số tỉ lệ co giãn DVL scale factor s (không thứ nguyên)
%           x(18:20) : 3 góc lệch lắp đặt cơ khí Boresight [alpha, beta, gamma] (rad)
%
%       P - Ma trận hiệp phương sai sai số 19x19 (Error-state Covariance):
%           Ứng với vector sai số delta_x = [delta_p(3), delta_v(3), delta_theta(3),
%           delta_ba(3), delta_bg(3), delta_s(1), delta_boresight(3)].
%           Lưu ý: Dùng vector sai số góc quay tối thiểu delta_theta (3 chiều) thay vì
%           4 chiều của quaternion để tránh suy biến số học và giữ ma trận P luôn xác định dương.
%
%       Q - Ma trận hiệp phương sai nhiễu quá trình 19x19 (Process Noise Covariance).

% =========================================================================
% 1. CHUYỂN ĐỔI GÓC EULER BAN ĐẦU SANG QUADTERNION (THEO THỨ TỰ Z-Y-X)
% =========================================================================
dt = noise_cfg(8);
roll = initial_rpy(1);
pitch = initial_rpy(2);
yaw = initial_rpy(3);

% Tính nửa góc lượng giác cho phép quay Euler Z-Y-X
cr = cos(roll / 2);
sr = sin(roll / 2);
cp = cos(pitch / 2);
sp = sin(pitch / 2);
cy = cos(yaw / 2);
sy = sin(yaw / 2);

% =========================================================================
% 2. THIẾT LẬP VECTOR TRẠNG THÁI DANH ĐỊNH BAN ĐẦU x (20x1)
% =========================================================================
x = zeros(20, 1);

% Vị trí ban đầu x(1:3) = [0; 0; 0] m (xuất phát tại gốc tọa độ NED)
% Vận tốc ban đầu x(4:6) = [0; 0; 0] m/s (xuất phát từ trạng thái đứng yên)

% Quaternion ban đầu x(7:10) = [q_w; q_x; q_y; q_z]
x(7:10) = [cy*cp*cr + sy*sp*sr; ...
    cy*cp*sr - sy*sp*cr; ...
    cy*sp*cr + sy*cp*sr; ...
    sy*cp*cr - cy*sp*sr];

% x(11:13): Bias gia tốc kế ban đầu = [0; 0; 0] m/s^2 (chưa biết, bộ lọc tự ước lượng)
% x(14:16): Bias con quay ban đầu = [0; 0; 0] rad/s (chưa biết, bộ lọc tự ước lượng)
% x(17)   : Hệ số tỉ lệ DVL scale s ban đầu = 0 (chưa có thông tin sai lệch âm học)
% x(18:20): 3 góc Boresight ban đầu = [0; 0; 0] rad (chưa hiệu chuẩn cơ khí)

% =========================================================================
% 3. KHỞI TẠO MA TRẬN HIỆP PHƯƠNG SAI SAI SỐ P (19x19)
% =========================================================================
% Giá trị khởi tạo thể hiện độ bất định ban đầu (độ lệch chuẩn 1-sigma) của từng biến:
% - Vị trí   : 0.02 m (tương ứng độ chính xác GNSS RTK tại mặt nước trước khi lặn)
% - Vận tốc  : 0.05 m/s
% - Tư thế   : 0.5° = 0.5 * pi / 180 rad (độ lệch góc nghiêng ban đầu)
% - Bias acc : 0.5 mg = 0.5e-3 * g (m/s^2)
% - Bias gyro: 0.5 °/giờ = 0.5 * pi / 180 / 3600 rad/s (chuẩn con quay FOG)
% - DVL scale: 0.02 (tương ứng độ bất định tỉ lệ co giãn 2%)
% - Boresight: 1.5° = 1.5 * pi / 180 rad (dung sai sai lệch cơ khí khi lắp ráp DVL)
P = diag([(0.02)^2 * ones(1, 3), ...
    (0.05)^2 * ones(1, 3), ...
    (0.5*pi/180)^2 * ones(1, 3), ...
    (0.5e-3*gravity)^2 * ones(1, 3), ...
    (0.5*pi/180/3600)^2 * ones(1, 3), ...
    (0.02)^2, ...
    (1.5*pi/180)^2 * ones(1, 3)]);

% =========================================================================
% 4. THIẾT LẬP MA TRẬN HIỆP PHƯƠNG SAI NHIỄU QUÁ TRÌNH Q (19x19)
% =========================================================================
% Q mô tả mức độ bất định được tích lũy vào hệ thống sau mỗi chu kỳ trích mẫu dt.
% Rời rạc hóa giải tích Van Loan từ mô hình nhiễu trắng liên tục của cảm biến:
Q = zeros(19, 19);

sigma_a = noise_cfg(1); % Nhiễu ngẫu nhiên vận tốc gia tốc kế (VRW - m/s/sqrt(s))
sigma_g = noise_cfg(2); % Nhiễu ngẫu nhiên góc con quay (ARW - rad/sqrt(s))

% Khối Vị trí - Vận tốc: Tích phân kép nhiễu trắng gia tốc (Van Loan rời rạc hóa chuẩn)
% Q_pp = (1/3) * sigma_a^2 * dt^3
% Q_pv = (1/2) * sigma_a^2 * dt^2
% Q_vv = sigma_a^2 * dt
Q(1:3, 1:3) = (1/3) * (sigma_a^2) * (dt^3) * eye(3);
Q(1:3, 4:6) = (1/2) * (sigma_a^2) * (dt^2) * eye(3);
Q(4:6, 1:3) = Q(1:3, 4:6)';
Q(4:6, 4:6) = (sigma_a^2) * dt * eye(3);

% Sai số góc tư thế: Gây ra bởi nhiễu ngẫu nhiên góc (ARW) của con quay
Q(7:9, 7:9) = (sigma_g^2) * dt * eye(3);

% Biến thiên bias gia tốc kế: Mô hình Gauss-Markov bậc 1 với thời gian tương quan tau
% Phương sai trạng thái rời rạc: 2 * sigma_ba^2 / tau_ba * dt
Q(10:12, 10:12) = noise_cfg(3)^2 * (2*dt/noise_cfg(4)) * eye(3);

% Biến thiên bias con quay: Mô hình Random Walk (bước đi ngẫu nhiên theo thời gian)
Q(13:15, 13:15) = noise_cfg(5)^2 * dt * eye(3);

% Biến thiên hệ số tỉ lệ DVL: Mô hình Random Walk do biến đổi nhiệt độ/độ mặn nước
Q(16, 16) = noise_cfg(6)^2 * dt;

% Biến thiên góc Boresight: Mô hình Random Walk do rung động cơ học
Q(17:19, 17:19) = noise_cfg(7)^2 * dt * eye(3);

end