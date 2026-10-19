function [state, covariance, update_status, nis, diag_out] = esekf19_step( ...
    state, covariance, process_noise, gyro_meas, accel_meas, ...
    gnss_meas, depth_meas, dvl_meas, measurement_flags, cfg)
%ESEKF19_STEP Thực hiện một bước dự đoán quán tính INS (400 Hz) và cập nhật đo đạc Kalman (5 Hz)
%   cho bộ lọc sai số ES-EKF 19 trạng thái (Error-State Extended Kalman Filter).
%
%   Cú pháp:
%       [state, covariance, update_status, nis] = esekf19_step( ...
%           state, covariance, process_noise, gyro_meas, accel_meas, ...
%           gnss_meas, depth_meas, dvl_meas, measurement_flags, cfg)
%
%   Đầu vào:
%       state             - Vector trạng thái danh định 20 chiều (Nominal State):
%                           [p(1:3); v(4:6); q(7:10); b_a(11:13); b_g(14:16); s(17); phi_bs(18:20)]
%       covariance        - Ma trận hiệp phương sai sai số P kích thước 19x19
%       process_noise     - Ma trận hiệp phương sai nhiễu quá trình Q kích thước 19x19
%       gyro_meas         - Giá trị đo thô con quay FOG [omega_x; omega_y; omega_z] (rad/s)
%       accel_meas        - Giá trị đo thô gia tốc kế [f_x; f_y; f_z] (m/s^2)
%       gnss_meas         - Vector đo GNSS (7x1): [pos_NED(3); vel_NED(3); yaw(1)]
%       depth_meas        - Giá trị đo độ sâu từ cảm biến áp suất thủy tĩnh (m)
%       dvl_meas          - Vector đo vận tốc đáy từ DVL A125 [v_x; v_y; v_z] (m/s)
%       measurement_flags - Cờ hợp lệ của các cảm biến (3x1): [co_GNSS; co_Depth; co_DVL]
%       cfg               - Vector tham số hệ thống (19x1 hoặc 20x1):
%                           (1) dt, (2) gravity, (3:5) w_ie_ned, (6) RN, (7) RM,
%                           (8) tan_latitude, (9) height, (10:12) lever_arm,
%                           (13) dvl_noise_floor, (14) dvl_noise_slope,
%                           (15) gps_pos_sigma, (16) gps_vel_sigma, (17) gps_yaw_sigma,
%                           (18) depth_sigma, (19) accel_bias_tau,
%                           (20) [Tùy chọn E1] sigma_arw con quay (rad/sqrt(s))
%
%   Đầu ra:
%       state         - Vector trạng thái danh định đã được cập nhật (20x1)
%       covariance    - Ma trận hiệp phương sai P đã cập nhật (19x19)
%       update_status - Trạng thái chấp nhận phép đo [GNSS; Depth; DVL] (1: chấp nhận, 0: từ chối)
%       nis           - Giá trị thống kê kiểm định NIS (Normalized Innovation Squared) (3x1)

% =========================================================================
% 1. GIẢI MÃ THAM SỐ HỆ THỐNG VÀ VECTOR TRẠNG THÁI DANH ĐỊNH
% =========================================================================
dt = cfg(1);                      % Chu kỳ trích mẫu của IMU (s)
gravity = cfg(2);                 % Gia tốc trọng trường chuẩn g (m/s^2)
w_ie_ned = cfg(3:5);              % Vận tốc góc tự quay của Trái Đất trong khung NED (rad/s)
radius_n = cfg(6);                % Bán kính vòng cung kinh tuyến RN (m)
radius_m = cfg(7);                % Bán kính vòng cung vĩ tuyến RM (m)
tan_latitude = cfg(8);            % tan của vĩ độ địa lý
height = cfg(9);                  % Độ cao so với mặt elipsoid tham chiếu (m)
lever_arm = cfg(10:12);           % Cánh tay đòn lệch tâm từ IMU đến DVL trong khung thân (m)
accel_bias_tau = cfg(19);         % Thời gian tương quan của bias gia tốc kế Gauss-Markov (s)

position = state(1:3);            % Vị trí NED [p_N; p_E; p_D] (m)
velocity = state(4:6);            % Vận tốc NED [v_N; v_E; v_D] (m/s)
quaternion = state(7:10);         % Quaternion biểu diễn tư thế UUV [q_w; q_x; q_y; q_z]
accel_bias = state(11:13);        % Bias gia tốc kế trong khung thân (m/s^2)
gyro_bias = state(14:16);         % Bias con quay trong khung thân (rad/s)
scale = state(17);                % Hệ số tỉ lệ co giãn DVL scale factor s
boresight = state(18:20);         % 3 góc lệch Boresight DVL [alpha; beta; gamma] (rad)

% =========================================================================
% 2. TẦNG DỰ ĐOÁN ĐỘNG HỌC QUÁN TÍNH INS (400 Hz)
% =========================================================================
% Chuyển quaternion sang ma trận xoay Cbn (Body sang NED)
Cbn = quat_to_cbn(quaternion);
Cnb = Cbn'; % Ma trận xoay nghịch đảo (NED sang Body)

% Trừ bias ước lượng khỏi số đo thô của IMU (Vòng hồi tiếp bù bias)
accel_corrected = accel_meas - accel_bias;
gyro_corrected = gyro_meas - gyro_bias;

% Tính vận tốc góc chuyển dời khung tham chiếu (Transport rate w_en) do UUV dịch chuyển
w_en_ned = [velocity(2)/(radius_n + height); ...
    -velocity(1)/(radius_m + height); ...
    -velocity(2)*tan_latitude/(radius_n + height)];

% Vận tốc góc thực sự của UUV so với hệ quy chiếu dẫn đường biểu diễn trong khung thân
w_nb_body = gyro_corrected - Cnb*(w_ie_ned + w_en_ned);

% Tính gia tốc trong hệ NED: Chuyển lực riêng + Bù trọng lực + Bù lực quán tính Coriolis
accel_ned = Cbn*accel_corrected + [0; 0; gravity] - cross(2*w_ie_ned + w_en_ned, velocity);

% Tích phân vận tốc và vị trí theo thời gian (Tích phân số học Euler)
velocity = velocity + accel_ned*dt;
position = position + velocity*dt;

% Tích phân hướng (Cập nhật Quaternion) qua vector góc quay vi phân delta_angle
delta_angle = w_nb_body*dt;
angle_norm = norm(delta_angle);
delta_quaternion = [cos(angle_norm/2); ...
    sin(angle_norm/2)*delta_angle/max(angle_norm, 1e-12)];
quaternion = quat_multiply(quaternion, delta_quaternion);
quaternion = quaternion/norm(quaternion); % Chuẩn hóa để chống trôi số học

% Suy giảm bias gia tốc kế theo mô hình quá trình Gauss-Markov bậc 1
phi_acc = exp(-dt / max(accel_bias_tau, eps));
accel_bias = phi_acc * accel_bias;

% =========================================================================
% 3. LAN TRUYỀN MA TRẬN HIỆP PHƯƠNG SAI SAI SỐ P (400 Hz)
% =========================================================================
% Ma trận chuyển đổi trạng thái sai số phi tuyến rời rạc F (19x19)
F = eye(19);

% Sai số vị trí phụ thuộc vào sai số vận tốc
F(1:3, 4:6) = eye(3)*dt;

% Sai số vận tốc phụ thuộc vào sai số góc quay (tilt error) qua ma trận phản đối xứng của lực riêng
F(4:6, 7:9) = -Cbn*skew_mat(accel_corrected)*dt;

% Sai số vận tốc phụ thuộc vào sai số bias gia tốc kế
F(4:6, 10:12) = -Cbn*dt;

% Sai số góc quay phụ thuộc vào vận tốc góc quay khung thân
F(7:9, 7:9) = eye(3) - skew_mat(w_nb_body)*dt;

% Sai số góc quay phụ thuộc vào sai số bias con quay
F(7:9, 13:15) = -eye(3)*dt;

% Bias gia tốc kế suy giảm theo thời gian tương quan Gauss-Markov
F(10:12, 10:12) = phi_acc * eye(3);

% Lan truyền ma trận P: P_k = F * P_(k-1) * F' + Q
covariance = F*covariance*F' + process_noise;

% =========================================================================
% 4. TẦNG CẬP NHẬT ĐO ĐẠC KALMAN (5 Hz)
% =========================================================================
update_status = zeros(3, 1);
nis = zeros(3, 1);

% -------------------------------------------------------------------------
% 4.1. CẬP NHẬT GNSS (VỊ TRÍ, VẬN TỐC, GÓC HƯỚNG TỪ 2 ĂNG-TEN)
% -------------------------------------------------------------------------
if measurement_flags(1) ~= 0
    Cbn = quat_to_cbn(quaternion);

    % Tính góc yaw hiện tại và phần dư residual (có cuộn tròn góc [-pi, pi])
    yaw_estimate = atan2(Cbn(2, 1), Cbn(1, 1));
    yaw_residual = atan2(sin(gnss_meas(7) - yaw_estimate), ...
        cos(gnss_meas(7) - yaw_estimate));

    % Vector phần dư đo đạc 7 chiều
    residual = [gnss_meas(1:3) - position; ...
        gnss_meas(4:6) - velocity; yaw_residual];

    % Ma trận quan sát H_gnss (7x19)
    H = zeros(7, 19);
    H(1:3, 1:3) = eye(3); % Quan sát vị trí trực tiếp
    H(4:6, 4:6) = eye(3); % Quan sát vận tốc trực tiếp

    % Đạo hàm riêng của góc yaw theo sai số góc quay delta_theta
    roll_est = atan2(Cbn(3, 2), Cbn(3, 3));
    pitch_est = -asin(max(min(Cbn(3, 1), 1), -1));
    cosp = max(cos(pitch_est), 1e-6);
    H(7, 7:9) = [0, sin(roll_est)/cosp, cos(roll_est)/cosp];

    % Ma trận hiệp phương sai nhiễu đo R_gnss
    R = diag([cfg(15)^2*ones(1, 3), cfg(16)^2*ones(1, 3), cfg(17)^2]);

    % Thực hiện cập nhật Kalman và kiểm tra cổng lọc NIS (7 bậc tự do, ngưỡng 99% là 18.48)
    [delta_state, covariance, accepted, nis_value] = ...
        update_gnss(covariance, H, residual, R);
    update_status(1) = accepted;
    nis(1) = nis_value;

    % Nếu dữ liệu hợp lệ thì bơm sai số vào trạng thái danh định (Error Injection)
    if accepted ~= 0
        [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
            esekf19_inject_error(delta_state, covariance, position, velocity, quaternion, ...
            accel_bias, gyro_bias, scale, boresight);
    end
end

% -------------------------------------------------------------------------
% 4.2. CẬP NHẬT CẢM BIẾN ÁP SUẤT ĐỘ SÂU (DEPTH SENSOR)
% -------------------------------------------------------------------------
if measurement_flags(2) ~= 0
    % Ma trận quan sát H_depth (1x19) - chỉ đo trực tiếp trục Down: position(3)
    H = zeros(1, 19);
    H(1, 3) = 1;
    residual = depth_meas - position(3);

    % Thực hiện cập nhật Kalman và kiểm tra cổng lọc NIS (1 bậc tự do, ngưỡng 99% là 6.63)
    [delta_state, covariance, accepted, nis_value] = ...
        update_depth(covariance, H, residual, cfg(18)^2);
    update_status(2) = accepted;
    nis(2) = nis_value;

    if accepted ~= 0
        [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
            esekf19_inject_error(delta_state, covariance, position, velocity, quaternion, ...
            accel_bias, gyro_bias, scale, boresight);
    end
end

% -------------------------------------------------------------------------
% 4.3. CẬP NHẬT VẬN TỐC ĐÁY DVL A125 (ƯỚC LƯỢNG BORESIGHT VÀ SCALE FACTOR)
% -------------------------------------------------------------------------
if measurement_flags(3) ~= 0
    % Gọi trực tiếp hàm mô hình DVL dùng chung
    current_state = [position; velocity; quaternion; accel_bias; gyro_bias; scale; boresight];
    [h_dvl, H, R_dvl, velocity_at_dvl, body_rate_relative, Cbs] = ...
        esekf19_dvl_model(current_state, gyro_meas, cfg);

    residual = dvl_meas - h_dvl;

    % Cập nhật Kalman và kiểm tra cổng lọc NIS (3 bậc tự do, ngưỡng 99% là 11.34)
    [delta_state, covariance, accepted, nis_value] = ...
        update_dvl(covariance, H, residual, R_dvl);
    update_status(3) = accepted;
    nis(3) = nis_value;

    if accepted ~= 0
        [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
            esekf19_inject_error(delta_state, covariance, position, velocity, quaternion, ...
            accel_bias, gyro_bias, scale, boresight);
    end
end

% =========================================================================
% 5. ĐÓNG GÓI VECTOR TRẠNG THÁI DANH ĐỊNH ĐÃ CẬP NHẬT
% =========================================================================
state(1:3) = position;
state(4:6) = velocity;
state(7:10) = quaternion;
state(11:13) = accel_bias;
state(14:16) = gyro_bias;
state(17) = scale;
state(18:20) = boresight;

% Đóng gói đầu ra chẩn đoán tùy chọn (Optional Diagnostic Output)
if nargout >= 5
    diag_out = struct();
    if measurement_flags(3) ~= 0
        diag_out.h_dvl = h_dvl;
        diag_out.H_dvl = H;
        diag_out.R_dvl = R_dvl;
        diag_out.residual_dvl = residual;
        diag_out.velocity_at_dvl = velocity_at_dvl;
        diag_out.S_dvl = H*covariance*H' + R_dvl;
    end
end
end

% =========================================================================
% CÁC HÀM BỔ TRỢ TÍNH TOÁN CẬP NHẬT KALMAN (LOCAL FUNCTIONS)
% =========================================================================

function [delta_state, covariance, accepted, nis] = update_gnss(covariance, H, residual, R)
%UPDATE_GNSS Cập nhật Kalman cho phép đo GNSS với cổng lọc NIS (7 bậc tự do).
S = H*covariance*H' + R;
nis = residual'*(S\residual);

% Ngưỡng phân phối Chi-square 7 bậc tự do ở mức độ tin cậy 99% là 18.48 (mức 99.9% là 24.32)
accepted = double(isfinite(nis) && nis <= 18.48);
delta_state = zeros(19, 1);
if accepted == 0
    return;
end

% Ma trận độ lợi Kalman K
K = (covariance*H')/S;
delta_state = K*residual;

% Cập nhật ma trận hiệp phương sai theo công thức ổn định số học Joseph:
% P = (I - K*H)*P*(I - K*H)' + K*R*K'
A = eye(19) - K*H;
covariance = A*covariance*A' + K*R*K';
covariance = 0.5*(covariance + covariance'); % Đảm bảo tính đối xứng tuyệt đối
end

function [delta_state, covariance, accepted, nis] = update_depth(covariance, H, residual, variance)
%UPDATE_DEPTH Cập nhật Kalman cho cảm biến áp suất độ sâu (1 bậc tự do).
S = H*covariance*H' + variance;
nis = residual*(S\residual);

% Ngưỡng phân phối Chi-square 1 bậc tự do ở mức độ tin cậy 99% là 6.63
accepted = double(isfinite(nis) && nis <= 6.63);
delta_state = zeros(19, 1);
if accepted == 0
    return;
end

K = (covariance*H')/S;
delta_state = K*residual;

% Cập nhật ma trận hiệp phương sai dạng Joseph
A = eye(19) - K*H;
covariance = A*covariance*A' + K*variance*K';
covariance = 0.5*(covariance + covariance');
end

function [delta_state, covariance, accepted, nis] = update_dvl(covariance, H, residual, R)
%UPDATE_DVL Cập nhật Kalman cho vận tốc đáy DVL A125 (3 bậc tự do).
S = H*covariance*H' + R;
nis = residual'*(S\residual);

% Ngưỡng phân phối Chi-square 3 bậc tự do ở mức độ tin cậy 99% là 11.34
accepted = double(isfinite(nis) && nis <= 11.34);
delta_state = zeros(19, 1);
if accepted == 0
    return;
end

K = (covariance*H')/S;
delta_state = K*residual;

% Công thức Joseph cập nhật ma trận P đảm bảo ma trận luôn nửa xác định dương
A = eye(19) - K*H;
covariance = A*covariance*A' + K*R*K';
covariance = 0.5*(covariance + covariance');
end

function [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
    inject_error(delta_state, covariance, position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight)
%INJECT_ERROR Bơm vector sai số ước lượng vào trạng thái danh định và đặt lại delta_x về 0.

% 1. Cộng bù trực tiếp cho các đại lượng vector tuyến tính
position = position + delta_state(1:3);
velocity = velocity + delta_state(4:6);

% 2. Nhân quaternion danh định với quaternion quay vi phân delta_theta (tránh phi tuyến hóa góc lớn)
delta_theta = delta_state(7:9);
delta_quaternion = [1; 0.5*delta_theta];
delta_quaternion = delta_quaternion/norm(delta_quaternion);
quaternion = quat_multiply(quaternion, delta_quaternion);
quaternion = quaternion/norm(quaternion);

% 3. Cộng bù bias cảm biến và các tham số hiệu chuẩn
accel_bias = accel_bias + delta_state(10:12);
gyro_bias = gyro_bias + delta_state(13:15);
scale = scale + delta_state(16);
% Cập nhật Boresight chuẩn trên nhóm SO(3): Cbs_new = Cbs_old * exp(-[delta_bs x])
delta_bs  = delta_state(17:19);
Cbs_new   = rodrigues_so3(-boresight) * rodrigues_so3(-delta_bs);
boresight = -so3_log(Cbs_new); % Trích xuất lại vector góc quay 3x1

% Ma trận Jacobian đặt lại hiệp phương sai sai số G (19x19) theo chuẩn Joan Solà
% đang bị sai dấu G(17:19 ) sửa  trừ thành cộng chưa được tunning P và Q 
G = eye(19);
G(7:9, 7:9)     = eye(3) - 0.5*skew_mat(delta_theta); % Đặt lại tư thế UUV
G(17:19, 17:19) = eye(3) + 0.5*skew_mat(delta_bs);    % Đặt lại Boresight DVL trên SO(3)
covariance = G*covariance*G';
covariance = 0.5*(covariance + covariance');
end

function Cbn = quat_to_cbn(quaternion)
%QUAT_TO_CBN Chuyển đổi quaternion [w; x; y; z] sang ma trận xoay Cbn (Body sang NED).
quaternion = quaternion/norm(quaternion);
w = quaternion(1); x = quaternion(2); y = quaternion(3); z = quaternion(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
    2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
    2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function S = skew_mat(vector)
%SKEW_MAT Tạo ma trận phản đối xứng 3x3 từ vector 3x1 cho phép tính tích có hướng (cross product).
S = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function quaternion = quat_multiply(first, second)
%QUAT_MULTIPLY Nhân hai quaternion [w; x; y; z].
quaternion = [first(1)*second(1)-first(2)*second(2)-first(3)*second(3)-first(4)*second(4); ...
    first(1)*second(2)+first(2)*second(1)+first(3)*second(4)-first(4)*second(3); ...
    first(1)*second(3)-first(2)*second(4)+first(3)*second(1)+first(4)*second(2); ...
    first(1)*second(4)+first(2)*second(3)-first(3)*second(2)+first(4)*second(1)];
end
function R = rodrigues_so3(phi)
%RODRIGUES_SO3 Ánh xạ mũ exp([phi x]) tạo ma trận xoay trực giao chuẩn SO(3).
theta = norm(phi);
K = skew_mat(phi);
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
end
end

function phi = so3_log(R)
%SO3_LOG Ánh xạ logarit trích xuất vector góc quay phi (3x1) từ ma trận xoay R.
cos_theta = max(min(0.5*(trace(R) - 1), 1), -1);
theta = acos(cos_theta);
if theta > 1e-8
    phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
else
    phi = 0.5 * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
end
end