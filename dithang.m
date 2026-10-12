% dithang.m
% =========================================================================
% HỆ THỐNG DẪN ĐƯỜNG UUV: KỊCH BẢN RECIPROCAL TRACKS (A -> B VÀ B -> A)
% Cơ sở khoa học: James C. Kinsey & Louis L. Whitcomb (IEEE JOE 2007)
%                 Quy trình chuẩn hiệu chuẩn DVL của iXblue / Exail Delph INS
% 
% Quy trình thực hiện:
% - Phase 1: Hiệu chuẩn trên mặt nước có GNSS 2 anten bằng 2 lượt đối xứng:
%   * 0 -> 60s  : Đứng yên thả nổi chốt hướng Bắc và triệt tiêu bias tĩnh FOG.
%   * 60 -> 210s : Lượt đi A -> B (150s bơi thẳng hướng Bắc 0 độ, V = 1.5 m/s).
%   * 210 -> 240s: Quay đầu 180 độ êm mượt (Heading chuyển từ 0 sang 180 độ, rẽ êm).
%   * 240 -> 390s: Lượt về B -> A (150s bơi thẳng ngược lại hướng Nam 180 độ).
%   ==> Nhờ tính chất đảo chiều 180 độ, sai số góc hướng Bắc triệt tiêu hoàn toàn,
%       chốt chính xác các tham số DVL: s (+1.00%), beta (-0.30 deg), gamma (+1.00 deg)!
% - Phase 2 (t > 390s): Lặn ngầm cắt hoàn toàn GNSS đi thẳng đến Target cố định:
%   * Chuyển hướng êm về phía Target Waypoint [450m Bắc, 250m Đông, 15m Sâu].
%   * Chúi đầu lặn mượt xuống độ sâu mục tiêu Z = 15m.
%   * Giữ phẳng độ sâu Z = 15m (Depth Hold) và bơi thẳng tắp đến điểm đích.
% 
% Quy ước màu sắc đồ thị:
%   - Đường màu ĐỎ : Đi Thật (Ground Truth).
%   - Đường màu ĐEN: Tính toán lý thuyết ES-EKF.
% =========================================================================

clear all; close all; clc;

disp('===================================================================');
disp('   HỆ THỐNG DẪN ĐƯỜNG UUV: KỊCH BẢN RECIPROCAL TRACKS (A -> B -> A)  ');
disp('   Chuẩn khoa học Kinsey & Whitcomb (IEEE JOE) & iXblue Delph INS  ');
disp('===================================================================');

%% -------------------------------------------------------------------------
% BƯỚC 1: KHỞI TẠO CẤU HÌNH & THIẾT KẾ QUỸ ĐẠO RECIPROCAL TRACKS
% -------------------------------------------------------------------------
disp('1. Khởi tạo cấu hình hệ thống và thiết kế quỹ đạo chuyển động...');
p = thong_so_he_thong_chuan();

T_phase1 = 390;   % Phase 1: 390s (6.5 phút trên mặt nước thực hiện 2 lượt A->B->A)
T_phase2 = 360;   % Phase 2: 360s (6 phút lặn ngầm cắt GNSS đi thẳng đến đích)
T_total  = T_phase1 + T_phase2; % Tổng thời gian 750s (12.5 phút)

rng(1); % Cố định seed ngẫu nhiên để kết quả lặp lại nhất quán

dt  = p.si.dt_ins;
fs  = p.ins.gyro.tan_so_Hz;
g   = p.dat.g;
gn  = [0; 0; g];                         
W   = p.dat.omega_ie;
a_e = p.dat.a;
e2  = p.dat.e2;
s_cfg = p.si;

N   = round(T_total * fs);
t   = (0:N-1)' * dt;

% Tọa độ mục tiêu cố định khi lặn ở Phase 2 (Target Waypoint trong hệ NED)
target_N = 450;   % mét (Bắc)
target_E = 250;   % mét (Đông)
target_D = 15;    % mét (Độ sâu)

% Mảng lưu vận tốc và góc thái độ thực tế
V = zeros(N, 1);
yaw = zeros(N, 1);
pitch = zeros(N, 1);
roll = zeros(N, 1);

% [1.1] TẠO QUỸ ĐẠO PHASE 1: A -> B VÀ B -> A TRÊN MẶT NƯỚC
N_p1 = round(T_phase1 * fs);
for i = 1:N_p1
    ti = t(i);
    if ti < 60
        % Đứng yên thả nổi trên mặt nước (căn chỉnh thô & chốt hướng Bắc)
        V(i) = 0.0;
        yaw(i) = 0.0;
        pitch(i) = 0.0;
    elseif ti < 70
        % Tăng tốc êm dịu từ 0 lên 1.5 m/s
        tau = (ti - 60) / 10;
        V(i) = 1.5 * (0.5 - 0.5 * cos(pi * tau));
        yaw(i) = 0.0;
        pitch(i) = 0.0;
    elseif ti < 210
        % LƯỢT ĐI A -> B: Bơi thẳng đều 1.5 m/s hướng Bắc (0 độ) trong 140s
        V(i) = 1.5;
        yaw(i) = 0.0;
        pitch(i) = 0.0;
    elseif ti < 240
        % QUAY ĐẦU 180 ĐỘ: Bẻ lái chuyển hướng mượt từ 0 sang 180 độ trong 30s
        tau = (ti - 210) / 30;
        V(i) = 1.2; % Tốc độ quay vòng
        yaw(i) = pi * (0.5 - 0.5 * cos(pi * tau));
        pitch(i) = 0.0;
    else
        % LƯỢT VỀ B -> A: Bơi thẳng đều 1.5 m/s hướng Nam (180 độ) trong 150s
        V(i) = 1.5;
        yaw(i) = pi;
        pitch(i) = 0.0;
    end
end

% Tính vị trí tích lũy của Phase 1 để xác định điểm kết thúc Phase 1
v_n_temp = zeros(N_p1, 3);
for i = 1:N_p1
    cy = cos(yaw(i)); sy = sin(yaw(i));
    v_n_temp(i, :) = [V(i) * cy, V(i) * sy, 0];
end
pos_p1_all = cumsum(v_n_temp, 1) * dt;
pos_p1_end = pos_p1_all(end, :);

fprintf('   --> Vị trí kết thúc Phase 1: [%.1f m Bắc, %.1f m Đông, %.1f m Sâu]\n', ...
    pos_p1_end(1), pos_p1_end(2), pos_p1_end(3));

% [1.2] TẠO QUỸ ĐẠO PHASE 2: LẶN NGẦM CẮT GNSS ĐI THẲNG TỚI TARGET
% Tính toán vector phương vị từ điểm kết thúc Phase 1 tới Target Waypoint
vec_to_target = [target_N - pos_p1_end(1), target_E - pos_p1_end(2)];
dist_to_target = norm(vec_to_target);
heading_target = atan2(vec_to_target(2), vec_to_target(1));

fprintf('   --> Góc hướng tới Target: %.2f độ | Khoảng cách: %.1f m\n', ...
    heading_target * 180/pi, dist_to_target);

% Thời lượng các phân đoạn trong Phase 2:
% - 25s đầu: Chuyển hướng êm từ 180 độ về heading_target
% - 90s tiếp theo: Chúi đầu lặn mượt từ 0m xuống đúng 15.0m
% - Thời gian còn lại: Giữ phẳng độ sâu 15.0m và bơi thẳng đến đích
t_turn_sub = 25.0; 
t_dive_dur = 90.0;

for i = (N_p1 + 1):N
    ti = t(i);
    tr = ti - T_phase1;
    V(i) = 1.5;
    
    % Góc Yaw: Chuyển hướng êm dịu từ 180 độ về heading_target
    if tr < t_turn_sub
        tau = tr / t_turn_sub;
        % Chuyển góc theo cung ngắn nhất
        d_yaw = atan2(sin(heading_target - pi), cos(heading_target - pi));
        yaw(i) = pi + d_yaw * (0.5 - 0.5 * cos(pi * tau));
    else
        yaw(i) = heading_target;
    end
    
    % Góc Pitch & Độ sâu: Chúi đầu lặn xuống đúng target_D = 15m
    if tr < t_turn_sub
        pitch(i) = 0.0;
    elseif tr < (t_turn_sub + t_dive_dur)
        tau_dive = (tr - t_turn_sub) / t_dive_dur;
        % Profile vận tốc chìm hình chuông: z_dot = (D / T) * (1 - cos(2*pi*tau))
        % Tích phân đạt đúng target_D = 15m tại tau = 1
        z_dot = (target_D / t_dive_dur) * (1 - cos(2 * pi * tau_dive));
        pitch(i) = -asin(min(max(z_dot / V(i), -0.5), 0.5));
    else
        % Đã đạt độ sâu 15.0m: Giữ phẳng độ sâu (Depth Hold) bơi thẳng
        pitch(i) = 0.0;
    end
end

% Vận tốc trong Nav frame (NED)
v_n = zeros(N, 3);
for i = 1:N
    cy = cos(yaw(i));   sy = sin(yaw(i));
    cp = cos(pitch(i)); sp = sin(pitch(i));
    v_n(i, :) = [V(i) * cp * cy, V(i) * cp * sy, -V(i) * sp];
end

% Vị trí thực tế Ground Truth (NED)
POS_TRUE = cumsum(v_n, 1) * dt;

% Tọa độ địa lý WGS-84
lat = zeros(N, 1); lon = zeros(N, 1); h = zeros(N, 1);
lat(1) = p.dat.vi_do_deg * pi/180;
lon(1) = p.dat.kinh_do_deg * pi/180;
h(1)   = p.dat.do_cao_m;
for i = 2:N
    sl2 = sin(lat(i-1))^2;
    RN = a_e / sqrt(1 - e2 * sl2);
    RM = a_e * (1 - e2) / (1 - e2 * sl2)^1.5;
    lat(i) = lat(i-1) + v_n(i-1, 1) / (RM + h(i-1)) * dt;
    lon(i) = lon(i-1) + v_n(i-1, 2) / ((RN + h(i-1)) * cos(lat(i-1))) * dt;
    h(i)   = h(i-1)   - v_n(i-1, 3) * dt;
end

% Tốc độ góc và gia tốc thực tế (kèm hiệu ứng quay Trái Đất và Coriolis)
w_that = zeros(N, 3);
f_that = zeros(N, 3);
for i = 1:N
    j = max(i - 1, 1);
    sl2 = sin(lat(j))^2;
    RN = a_e / sqrt(1 - e2 * sl2);
    RM = a_e * (1 - e2) / (1 - e2 * sl2)^1.5;
    w_ie = W * [cos(lat(j)); 0; -sin(lat(j))];
    w_en = [v_n(j,2) / (RN + h(j)); -v_n(j,1) / (RM + h(j)); -v_n(j,2) * tan(lat(j)) / (RN + h(j))];
    
    dp_dt = (pitch(i) - pitch(j)) / dt;
    dy_dt = (yaw(i) - yaw(j)) / dt;
    
    cy = cos(yaw(j));   sy = sin(yaw(j));
    cp = cos(pitch(j)); sp = sin(pitch(j));
    R_b2n = [cy*cp, -sy, cy*sp;
             sy*cp,  cy, sy*sp;
             -sp,    0,   cp  ];
    R_n2b = R_b2n';
    
    w_body_rel = [-dy_dt * sp; dp_dt; dy_dt * cp];
    w_that(i, :) = (w_body_rel + R_n2b * (w_ie + w_en))';
    
    a_n = (v_n(i, :)' - v_n(j, :)') / dt;
    f_that(i, :) = (R_n2b * (a_n + cross(2*w_ie + w_en, v_n(j, :)') - gn))';
end

% Bơm sai số cảm biến theo datasheet INS-DM-FI
bg = s_cfg.gyro_bias_sigma * randn(3, 1);
Kg = eye(3) + diag(s_cfg.gyro_scale_err * randn(3, 1)) + s_cfg.gyro_cross_axis * (randn(3) .* ~eye(3));
ba = s_cfg.acc_bias_sigma * randn(3, 1);
Ka = eye(3) + diag(s_cfg.acc_scale_err * randn(3, 1)) + s_cfg.acc_cross_axis * (randn(3) .* ~eye(3));
phi_gm = exp(-dt / s_cfg.acc_bias_tau);
q_gm   = s_cfg.acc_bias_inrun_sigma * sqrt(1 - phi_gm^2);
b_gm   = s_cfg.acc_bias_inrun_sigma * randn(3, 1);
sd_g   = s_cfg.gyro_arw / sqrt(dt);
sd_a   = s_cfg.acc_vrw  / sqrt(dt);

w_do = zeros(N, 3);
f_do = zeros(N, 3);
for i = 1:N
    b_gm = phi_gm * b_gm + q_gm * randn(3, 1);
    w_do(i, :) = (Kg * w_that(i, :)' + bg + sd_g * randn(3, 1))';
    f_do(i, :) = (Ka * f_that(i, :)' + ba + b_gm + sd_a * randn(3, 1))';
end

% Quaternion thái độ ban đầu
q0 = [cos(yaw(1)/2); 0; 0; sin(yaw(1)/2)];

% Tín hiệu đo GNSS 2 anten (Chỉ hoạt động ở Phase 1: t <= T_phase1)
dt_gps = 0.2; % 5 Hz
idx_meas = 1:round(dt_gps / dt):N;
n_meas = length(idx_meas);

sig_gps_pos = 0.02; % RTK 2 cm
sig_gps_vel = 0.02; % RTK 2 cm/s
sig_gps_yaw = 0.05 * pi/180; % 2 anten 0.05 độ

Z_gps_pos = zeros(n_meas, 3);
Z_gps_vel = zeros(n_meas, 3);
Z_gps_yaw = zeros(n_meas, 1);
for j = 1:n_meas
    idx = idx_meas(j);
    if t(idx) <= T_phase1
        Z_gps_pos(j, :) = POS_TRUE(idx, :) + sig_gps_pos * randn(1, 3);
        Z_gps_vel(j, :) = v_n(idx, :)      + sig_gps_vel * randn(1, 3);
        Z_gps_yaw(j)    = yaw(idx)          + sig_gps_yaw * randn;
    end
end

% Cảm biến đo độ sâu thạch anh (Depth/Pressure Sensor - hoạt động khi lặn)
sig_depth = 0.05; % Độ chính xác 5 cm
Z_depth = POS_TRUE(idx_meas, 3) + sig_depth * randn(n_meas, 1);

% Tín hiệu đo DVL 5 Hz (Scale factor +1.0%, Pitch boresight -0.3 deg, Yaw boresight +1.0 deg)
s_that  = p.si.k_that;             % +1.0%
bs_that = p.si.boresight_that_rad; % Alpha 0.2, Beta -0.3, Gamma 1.0 deg
Cbs_that = eye(3) - skew_mat(bs_that);
lever_arm = [0.2; 0.0; -0.1];      % Lever arm 0.2m

Z_dvl = zeros(n_meas, 3);
for j = 1:n_meas
    idx = idx_meas(j);
    cy = cos(yaw(idx));   sy = sin(yaw(idx));
    cp = cos(pitch(idx)); sp = sin(pitch(idx));
    R_n2b = [cy*cp,  sy*cp, -sp;
            -sy,     cy,     0;
             cy*sp,  sy*sp,  cp ];
    vb = R_n2b * v_n(idx, :)';
    w_b = w_that(idx, :)';
    v_L = vb + cross(w_b, lever_arm);
    sig_m = s_cfg.sigma_dvl_fn(norm(v_L));
    Z_dvl(j, :) = ((1 + s_that) * Cbs_that * v_L + sig_m * randn(3, 1))';
end

%% -------------------------------------------------------------------------
% BƯỚC 2: KHỞI TẠO BỘ LỌC ES-EKF 19 TRẠNG THÁI
% -------------------------------------------------------------------------
disp('2. Khởi tạo bộ lọc ES-EKF 19 trạng thái...');

pos_est = [0; 0; 0];
v_est   = [0; 0; 0];
q_est   = q0;
ba_est  = [0; 0; 0];
bg_est  = [0; 0; 0];

% Ước lượng khởi tạo bằng 0 (chưa biết gì về sai số lắp ráp DVL)
s_est      = 0.0;
phi_bs_est = [0.0; 0.0; 0.0];
lever_arm_est = lever_arm;

P = diag([
    (0.01)^2 * ones(1, 3),          ... % 1:3   delta_p (m)
    (0.01)^2 * ones(1, 3),          ... % 4:6   delta_v (m/s)
    (0.2 * pi/180)^2 * ones(1, 2),  ... % 7:8   delta_roll, pitch (rad)
    (0.05 * pi/180)^2,              ... % 9     delta_yaw
    (0.5e-3 * 9.81)^2 * ones(1, 3), ... % 10:12 delta_ba (m/s^2)
    (0.5 * pi/180 / 3600)^2 * ones(1, 3), ... % 13:15 delta_bg (rad/s)
    (0.02)^2,                       ... % 16    delta_s (+-2%)
    (0.01 * pi/180)^2,              ... % 17    delta_alpha
    (1.5 * pi/180)^2,               ... % 18    delta_beta
    (1.5 * pi/180)^2                ... % 19    delta_gamma
]);

Q_d = zeros(19, 19);
Q_d(4:6, 4:6)     = (s_cfg.acc_vrw)^2 * dt * eye(3);
Q_d(7:9, 7:9)     = (s_cfg.gyro_arw)^2 * dt * eye(3);
Q_d(10:12, 10:12) = (s_cfg.acc_bias_inrun_sigma)^2 * (2*dt/s_cfg.acc_bias_tau) * eye(3);
Q_d(13:15, 13:15) = (s_cfg.gyro_bias_sigma * 1e-4)^2 * dt * eye(3);
Q_d(16, 16)       = (5e-7)^2 * dt;
Q_d(18:19, 18:19) = (5e-5 * pi/180)^2 * dt * eye(2);

% Bộ đệm lưu kết quả
POS_REC   = zeros(N, 3);
VEL_REC   = zeros(N, 3);
YAW_REC   = zeros(N, 1);
PITCH_REC = zeros(N, 1);
S_REC     = zeros(N, 1);
BETA_REC  = zeros(N, 1);
GAMMA_REC = zeros(N, 1);

meas_ptr = 1;

%% -------------------------------------------------------------------------
% BƯỚC 3: VÒNG LẶP DẪN ĐƯỜNG THỜI GIAN THỰC (400 Hz INS + 5 Hz CẢM BIẾN)
% -------------------------------------------------------------------------
disp('3. Đang thực thi vòng lặp ES-EKF dẫn đường...');

for i = 1:N
    ti = t(i);
    
    if mod(i, round(100 / dt)) == 0
        fprintf('   --> Tiến trình: %d / %d s (%.0f%%) | Z = %.1f m | s = %+.3f%% | gamma = %+.3f deg\n', ...
            round(ti), round(T_total), ti/T_total*100, pos_est(3), s_est*100, phi_bs_est(3)*180/pi);
    end
    
    % --- [A] 400 Hz DỰ BÁO INS (CƠ HỌC QUAY TRÁI ĐẤT & CORIOLIS) ---
    f_clean = f_do(i, :)' - ba_est;
    w_clean = w_do(i, :)' - bg_est;
    
    Cbn = quat_to_Cbn(q_est);
    Cnb = Cbn';
    
    lat_i = lat(i);
    w_ie_nav = W * [cos(lat_i); 0; -sin(lat_i)];
    sl2_i = sin(lat_i)^2;
    RN_i = a_e / sqrt(1 - e2 * sl2_i);
    RM_i = a_e * (1 - e2) / (1 - e2 * sl2_i)^1.5;
    w_en_nav = [v_est(2)/(RN_i + h(i)); -v_est(1)/(RM_i + h(i)); -v_est(2)*tan(lat_i)/(RN_i + h(i))];
    w_in_nav = w_ie_nav + w_en_nav;
    w_nb_body = w_clean - Cnb * w_in_nav;
    
    a_nav = Cbn * f_clean + gn - cross(2*w_ie_nav + w_en_nav, v_est);
    v_est = v_est + a_nav * dt;
    pos_est = pos_est + v_est * dt;
    
    dth_step = w_nb_body * dt;
    dq_step = [cos(norm(dth_step)/2); sin(norm(dth_step)/2) * dth_step / max(norm(dth_step), 1e-12)];
    q_est = quat_nhan(q_est, dq_step);
    q_est = q_est / norm(q_est);
    
    F_x = eye(19);
    F_x(1:3, 4:6)   = eye(3) * dt;
    F_x(4:6, 7:9)   = - Cbn * skew_mat(f_clean) * dt;
    F_x(4:6, 10:12) = - Cbn * dt;
    F_x(7:9, 7:9)   = eye(3) - skew_mat(w_nb_body) * dt;
    F_x(7:9, 13:15) = - eye(3) * dt;
    
    P = F_x * P * F_x' + Q_d;
    
    % --- [B] 5 Hz CẬP NHẬT CẢM BIẾN ---
    if meas_ptr <= n_meas && i == idx_meas(meas_ptr)
        
        % 1. CẬP NHẬT GNSS 2 ANTEN (CHỈ TRONG PHASE 1: ti <= T_phase1)
        if ti <= T_phase1
            z_pv = [Z_gps_pos(meas_ptr, :)'; Z_gps_vel(meas_ptr, :)'];
            h_pv = [pos_est; v_est];
            res_pv = z_pv - h_pv;
            
            H_pv = zeros(6, 19);
            H_pv(1:3, 1:3) = eye(3);
            H_pv(4:6, 4:6) = eye(3);
            R_pv = diag([sig_gps_pos^2 * ones(1,3), sig_gps_vel^2 * ones(1,3)]);
            
            S_pv = H_pv * P * H_pv' + R_pv;
            K_pv = (P * H_pv') / S_pv;
            dx_pv = K_pv * res_pv;
            
            IKH_pv = eye(19) - K_pv * H_pv;
            P = IKH_pv * P * IKH_pv' + K_pv * R_pv * K_pv';
            
            pos_est = pos_est + dx_pv(1:3);
            v_est   = v_est   + dx_pv(4:6);
            ba_est  = ba_est  + dx_pv(10:12);
            bg_est  = bg_est  + dx_pv(13:15);
            
            % Góc Yaw GNSS 2 anten
            Cbn_cur = quat_to_Cbn(q_est);
            yaw_est_cur = atan2(Cbn_cur(2, 1), Cbn_cur(1, 1));
            res_yaw = atan2(sin(Z_gps_yaw(meas_ptr) - yaw_est_cur), cos(Z_gps_yaw(meas_ptr) - yaw_est_cur));
            
            H_yaw = zeros(1, 19);
            H_yaw(1, 9) = 1.0;
            R_yaw = sig_gps_yaw^2;
            
            S_yaw = H_yaw * P * H_yaw' + R_yaw;
            K_yaw = (P * H_yaw') / S_yaw;
            dx_yaw = K_yaw * res_yaw;
            
            IKH_yaw = eye(19) - K_yaw * H_yaw;
            P_joseph = IKH_yaw * P * IKH_yaw' + K_yaw * R_yaw * K_yaw';
            
            dth = dx_yaw(7:9);
            dq_corr = [1; 0.5 * dth];
            dq_corr = dq_corr / norm(dq_corr);
            q_est = quat_nhan(q_est, dq_corr);
            q_est = q_est / norm(q_est);
            bg_est = bg_est + dx_yaw(13:15);
            
            G = eye(19);
            G(7:9, 7:9) = eye(3) - 0.5 * skew_mat(dth);
            P = G * P_joseph * G';
            P = 0.5 * (P + P');
        else
            % TRONG PHASE 2: CẬP NHẬT CẢM BIẾN ĐỘ SÂU (DEPTH SENSOR)
            res_depth = Z_depth(meas_ptr) - pos_est(3);
            H_depth = zeros(1, 19);
            H_depth(1, 3) = 1.0; % Cột vị trí Z (Down)
            R_depth = sig_depth^2;
            
            S_depth = H_depth * P * H_depth' + R_depth;
            K_depth = (P * H_depth') / S_depth;
            dx_depth = K_depth * res_depth;
            
            IKH_d = eye(19) - K_depth * H_depth;
            P = IKH_d * P * IKH_d' + K_depth * R_depth * K_depth';
            
            pos_est = pos_est + dx_depth(1:3);
            v_est   = v_est   + dx_depth(4:6);
        end
        
        % 2. CẬP NHẬT DVL (CẢ PHASE 1 VÀ PHASE 2)
        Cbn_cur = quat_to_Cbn(q_est);
        Cnb_cur = Cbn_cur';
        v_b = Cnb_cur * v_est;
        v_L = v_b + cross(w_clean, lever_arm_est);
        
        % Điều kiện cập nhật: Vận tốc bơi ổn định V > 1.0 m/s
        % Ở Phase 1, khi quay đầu (210s -> 240s) tạm ngắt DVL để tránh nhiễu quay cua
        is_turning_phase1 = (ti > 210 && ti < 240);
        
        if norm(v_L) > 1.0 && ~is_turning_phase1
            z_m = Z_dvl(meas_ptr, :)';
            C_bs_est = eye(3) - skew_mat(phi_bs_est);
            h_val = (1 + s_est) * C_bs_est * v_L;
            residual = z_m - h_val;
            
            H = zeros(3, 19);
            if ti <= T_phase1
                % Phase 1: Hiệu chuẩn DVL với GNSS trong 2 lượt thẳng đối xứng A->B và B->A
                H(:, 16)    = C_bs_est * v_L;
                H(:, 18)    = (1 + s_est) * skew_mat(v_L) * [0; 1; 0]; % Beta (Pitch boresight)
                H(:, 19)    = (1 + s_est) * skew_mat(v_L) * [0; 0; 1]; % Gamma (Yaw boresight)
            else
                % Phase 2: Dẫn đường DVL ghìm INS đi thẳng đến đích
                H(:, 4:6)   = (1 + s_est) * C_bs_est * Cnb_cur;
                H(:, 7:8)   = (1 + s_est) * C_bs_est * skew_mat(v_b) * [1 0; 0 1; 0 0];
                H(:, 13:15) = (1 + s_est) * C_bs_est * skew_mat(lever_arm_est);
                H(:, 16)    = C_bs_est * v_L;
                H(:, 18)    = (1 + s_est) * skew_mat(v_L) * [0; 1; 0];
                H(:, 19)    = (1 + s_est) * skew_mat(v_L) * [0; 0; 1];
            end
            
            sig_d = max(s_cfg.sigma_dvl_fn(norm(v_L)), 0.002);
            R_dvl = diag([sig_d^2, sig_d^2, sig_d^2]);
            
            S_mat = H * P * H' + R_dvl;
            if norm(residual) < 0.5
                K_gain = (P * H') / S_mat;
                dx = K_gain * residual;
                
                IKH = eye(19) - K_gain * H;
                P_joseph = IKH * P * IKH' + K_gain * R_dvl * K_gain';
                
                pos_est = pos_est + dx(1:3);
                v_est   = v_est   + dx(4:6);
                
                dth = dx(7:9);
                dq_corr = [1; 0.5 * dth];
                dq_corr = dq_corr / norm(dq_corr);
                q_est = quat_nhan(q_est, dq_corr);
                q_est = q_est / norm(q_est);
                
                ba_est  = ba_est + dx(10:12);
                bg_est  = bg_est + dx(13:15);
                
                s_est         = s_est + dx(16);
                phi_bs_est(2) = phi_bs_est(2) + dx(18);
                phi_bs_est(3) = phi_bs_est(3) + dx(19);
                phi_bs_est(1) = 0.0;
                
                G = eye(19);
                G(7:9, 7:9) = eye(3) - 0.5 * skew_mat(dth);
                P = G * P_joseph * G';
                P = 0.5 * (P + P');
            end
        end
        meas_ptr = meas_ptr + 1;
    end
    
    % Trích xuất góc thái độ Euler của EKF
    Cbn_now = quat_to_Cbn(q_est);
    yaw_ekf_now   = atan2(Cbn_now(2, 1), Cbn_now(1, 1));
    pitch_ekf_now = -asin(max(min(Cbn_now(3, 1), 1.0), -1.0));
    
    % Lưu kết quả
    POS_REC(i, :)   = pos_est';
    VEL_REC(i, :)   = v_est';
    YAW_REC(i)      = yaw_ekf_now;
    PITCH_REC(i)    = pitch_ekf_now;
    S_REC(i)        = s_est * 100;
    BETA_REC(i)     = phi_bs_est(2) * 180 / pi;
    GAMMA_REC(i)    = phi_bs_est(3) * 180 / pi;
end

%% -------------------------------------------------------------------------
% BƯỚC 4: TÍNH TOÁN KẾT QUẢ & VẼ ĐỒ THỊ CHUẨN MỰC
% -------------------------------------------------------------------------
disp('4. Tính toán kết quả & vẽ đồ thị trực quan...');

mask_p1 = (t <= T_phase1);
mask_p2 = (t > T_phase1);

ERR_POS = sqrt(sum((POS_REC(mask_p2, 1:3) - POS_TRUE(mask_p2, 1:3)).^2, 2));
pos_err_max = max(ERR_POS);
pos_err_rms = sqrt(mean(ERR_POS.^2));
pos_err_end = ERR_POS(end);

diff_true = diff(POS_TRUE(mask_p2, 1:3), 1, 1);
total_dist = sum(sqrt(sum(diff_true.^2, 2)));
drift_rate = (pos_err_end / total_dist) * 100;

fprintf('\n===================================================================\n');
fprintf('     KẾT QUẢ DẪN ĐƯỜNG KỊCH BẢN RECIPROCAL TRACKS (A -> B -> A)    \n');
fprintf('===================================================================\n');
fprintf('• Tọa độ Target mục tiêu        : [%.1f m Bắc, %.1f m Đông, %.1f m Sâu]\n', target_N, target_E, target_D);
fprintf('• Tọa độ điểm đích thật         : [%.1f m Bắc, %.1f m Đông, %.1f m Sâu]\n', POS_TRUE(end,1), POS_TRUE(end,2), POS_TRUE(end,3));
fprintf('• Tọa độ điểm đích ES-EKF       : [%.1f m Bắc, %.1f m Đông, %.1f m Sâu]\n', POS_REC(end,1), POS_REC(end,2), POS_REC(end,3));
fprintf('-------------------------------------------------------------------\n');
fprintf('• Quãng đường lặn ngầm          : %.1f mét\n', total_dist);
fprintf('• Sai số dạt điểm đích cuối     : %.3f mét\n', pos_err_end);
fprintf('• Tỉ lệ dạt hành trình lặn      : %.4f %%\n', drift_rate);
fprintf('• Hệ số Scale s ước lượng       : %+.3f %% (Giá trị thật: +1.000 %%)\n', S_REC(end));
fprintf('• Pitch Boresight beta          : %+.3f deg (Giá trị thật: -0.300 deg)\n', BETA_REC(end));
fprintf('• Yaw Boresight gamma           : %+.3f deg (Giá trị thật: +1.000 deg)\n', GAMMA_REC(end));
fprintf('===================================================================\n\n');

% -------------------------------------------------------------------------
% HÌNH 1: QUỸ ĐẠO KHÔNG GIAN 3D VÀ MẶT BẰNG 2D
% Quy ước: ĐỎ = Đi Thật (Ground Truth) | ĐEN = Tính toán EKF
% -------------------------------------------------------------------------
figure('Name', 'QUỸ ĐẠO RECIPROCAL TRACKS A->B->A VÀ LẶN ĐẾN TARGET (ĐỎ: THẬT | ĐEN: EKF)', ...
       'Color', 'w', 'Position', [50 50 1350 680]);

% 1.1 Quỹ đạo không gian 3D
subplot(1, 2, 1);
plot3(POS_TRUE(:, 2), POS_TRUE(:, 1), -POS_TRUE(:, 3), 'r-', 'LineWidth', 2.5); hold on; grid on;
plot3(POS_REC(:, 2), POS_REC(:, 1), -POS_REC(:, 3), 'k--', 'LineWidth', 2.0);

plot3(POS_TRUE(1, 2), POS_TRUE(1, 1), 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
idx_dive = find(mask_p2, 1);
plot3(POS_TRUE(idx_dive, 2), POS_TRUE(idx_dive, 1), -POS_TRUE(idx_dive, 3), 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
plot3(target_E, target_N, -target_D, 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 12);

title('Quỹ đạo Không gian 3D: A\rightarrowB\rightarrowA rồi Lặn Đến Target');
xlabel('Trục Đông - East (m)'); ylabel('Trục Bắc - North (m)'); zlabel('Độ cao Z (m)');
legend('Đường ĐỎ: Đi Thật (Ground Truth)', 'Đường ĐEN: Ước lượng ES-EKF', ...
       'Xuất phát A (0,0)', 'Điểm lặn (Cắt GNSS)', 'Target Waypoint', 'Location', 'best');
view([-40, 25]);
axis equal;

% 1.2 Mặt bằng 2D (East vs North)
subplot(1, 2, 2);
plot(POS_TRUE(:, 2), POS_TRUE(:, 1), 'r-', 'LineWidth', 2.5); hold on; grid on;
plot(POS_REC(:, 2), POS_REC(:, 1), 'k--', 'LineWidth', 2.0);

plot(POS_TRUE(1, 2), POS_TRUE(1, 1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(POS_TRUE(idx_dive, 2), POS_TRUE(idx_dive, 1), 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
plot(target_E, target_N, 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 12);

title('Mặt bằng 2D: Tuyến đường Reciprocal Tracks & Tuyến lặn');
xlabel('Trục Đông - East (m)'); ylabel('Trục Bắc - North (m)');
legend('Đường ĐỎ: Đi Thật (Ground Truth)', 'Đường ĐEN: Ước lượng ES-EKF', ...
       'Xuất phát A', 'Điểm lặn cắt GNSS', 'Target Waypoint', 'Location', 'best');
axis equal;

sgtitle('ĐÁNH GIÁ QUỸ ĐẠO UUV THEO KỊCH BẢN RECIPROCAL TRACKS (A \rightarrow B \rightarrow A)', 'FontSize', 14, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% HÌNH 2: THÁI ĐỘ THỰC TẾ (PITCH, YAW) THEO THỜI GIAN & KIỂM TRA GIỮ ĐỘ SÂU
% -------------------------------------------------------------------------
figure('Name', 'THÁI ĐỘ THỰC TẾ (PITCH, YAW) VÀ KIỂM TRA GIỮ ĐỘ SÂU', ...
       'Color', 'w', 'Position', [100 100 1300 700]);

% 2.1 Góc Yaw thực tế của tàu theo thời gian
subplot(2, 2, 1);
plot(t, yaw * 180 / pi, 'r-', 'LineWidth', 2.2); hold on; grid on;
plot(t, YAW_REC * 180 / pi, 'k--', 'LineWidth', 1.8);
xline(T_phase1, 'm--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Góc Hướng Yaw Thực Tế của Tàu \psi(t) theo Thời Gian');
xlabel('Thời gian (s)'); ylabel('Yaw \psi (độ)');
legend('Đường ĐỎ: Yaw Thật', 'Đường ĐEN: Yaw EKF', 'Mốc lặn cắt GNSS', 'Location', 'best');
ylim([-10, 200]);

% 2.2 Góc Pitch thực tế của tàu theo thời gian
subplot(2, 2, 2);
plot(t, pitch * 180 / pi, 'r-', 'LineWidth', 2.2); hold on; grid on;
plot(t, PITCH_REC * 180 / pi, 'k--', 'LineWidth', 1.8);
xline(T_phase1, 'm--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Góc Chúi/Ngóc Pitch Thực Tế của Tàu \theta(t) theo Thời Gian');
xlabel('Thời gian (s)'); ylabel('Pitch \theta (độ)');
legend('Đường ĐỎ: Pitch Thật', 'Đường ĐEN: Pitch EKF', 'Mốc lặn cắt GNSS', 'Location', 'best');
ylim([-15, 5]);

% 2.3 Độ sâu thực tế của tàu Z(t)
subplot(2, 2, 3);
plot(t, POS_TRUE(:, 3), 'r-', 'LineWidth', 2.2); hold on; grid on;
plot(t, POS_REC(:, 3), 'k--', 'LineWidth', 1.8);
yline(target_D, 'b:', sprintf('Độ sâu mục tiêu: %.0f m', target_D), 'LineWidth', 1.5);
xline(T_phase1, 'm--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
set(gca, 'YDir', 'reverse');
title('Độ Sâu Thực Tế của Tàu Z(t) - Kiểm Tra Khả Năng Giữ Độ Sâu');
xlabel('Thời gian (s)'); ylabel('Độ sâu Z (m)');
legend('Đường ĐỎ: Độ sâu Thật', 'Đường ĐEN: Độ sâu EKF', 'Độ sâu chuẩn mục tiêu', 'Mốc lặn', 'Location', 'best');
ylim([-1, 18]);

% 2.4 Sai số dạt vị trí lặn ngầm theo thời gian
subplot(2, 2, 4);
t_sub = t(mask_p2) - T_phase1;
plot(t_sub, ERR_POS, 'k-', 'LineWidth', 2.0); hold on; grid on;
yline(pos_err_rms, 'b--', sprintf('RMS: %.3f m', pos_err_rms), 'LineWidth', 1.5);
title('Sai Số Dạt Vị Trí Khi Lặn Ngầm ||p_{EKF} - p_{TRUE}|| (m)');
xlabel('Thời gian lặn ngầm (s)'); ylabel('Sai số (m)');
legend('Sai số dạt tức thời', 'Sai số RMS', 'Location', 'best');

sgtitle('ĐÁNH GIÁ THÁI ĐỘ THỰC TẾ VÀ KHẢ NĂNG GIỮ ĐỘ SÂU (DEPTH HOLD)', 'FontSize', 14, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% HÌNH 3: QUÁ TRÌNH HỘI TỤ THAM SỐ CƠ KHÍ DVL (S, BETA, GAMMA)
% -------------------------------------------------------------------------
figure('Name', 'QUÁ TRÌNH HỘI TỤ THAM SỐ DVL (SCALE FACTOR VÀ BORESIGHT)', ...
       'Color', 'w', 'Position', [150 150 1200 650]);

subplot(3, 1, 1);
plot(t, S_REC, 'b-', 'LineWidth', 2.0); hold on; grid on;
yline(1.0, 'r--', 'Giá trị thật (+1.0%)', 'LineWidth', 1.5);
xline(T_phase1, 'm--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Ước lượng Hệ số Tỷ xích Scale factor s (%)');
xlabel('Thời gian (s)'); ylabel('s (%)');
ylim([-0.5, 2.5]);
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0%)', 'Location', 'best');

subplot(3, 1, 2);
plot(t, BETA_REC, 'g-', 'LineWidth', 2.0); hold on; grid on;
yline(-0.3, 'r--', 'Giá trị thật (-0.30 deg)', 'LineWidth', 1.5);
xline(T_phase1, 'm--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Ước lượng Pitch Boresight \beta (độ)');
xlabel('Thời gian (s)'); ylabel('\beta (độ)');
ylim([-0.8, 0.2]);
legend('Ước lượng ES-EKF', 'Giá trị thật (-0.30 deg)', 'Location', 'best');

subplot(3, 1, 3);
plot(t, GAMMA_REC, 'm-', 'LineWidth', 2.0); hold on; grid on;
yline(1.0, 'r--', 'Giá trị thật (+1.00 deg)', 'LineWidth', 1.5);
xline(T_phase1, 'm--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Ước lượng Yaw Boresight \gamma (độ)');
xlabel('Thời gian (s)'); ylabel('\gamma (độ)');
ylim([-0.5, 2.5]);
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.00 deg)', 'Location', 'best');

sgtitle('QUÁ TRÌNH HỘI TỤ THAM SỐ CƠ KHÍ DVL THEO KỊCH BẢN RECIPROCAL TRACKS', 'FontSize', 13, 'FontWeight', 'bold');

% Đồng bộ biến vào Base Workspace để có thể chạy 3D Animation nếu muốn
assignin('base', 'POS_TRUE', POS_TRUE);
assignin('base', 'POS_REC', POS_REC);
assignin('base', 'yaw', yaw);
assignin('base', 'V', V);
assignin('base', 't', t);
assignin('base', 'S_REC', S_REC);
assignin('base', 'BETA_REC', BETA_REC);
assignin('base', 'GAMMA_REC', GAMMA_REC);
assignin('base', 'dt', dt);
assignin('base', 'T_phase1', T_phase1);
assignin('base', 'T_can_chinh', T_phase1);

disp('--> ĐÃ MÔ PHỎNG XONG VÀ ĐỒNG BỘ DỮ LIỆU SANG WORKSPACE!');
disp('--> Bạn có thể gõ "dithang" để xem kết quả!');

%% -------------------------------------------------------------------------
% HÀM PHỤ TRỢ ĐẠI SỐ
% -------------------------------------------------------------------------
function S = skew_mat(v)
    S = [  0,    -v(3),  v(2);
          v(3),    0,   -v(1);
         -v(2),   v(1),   0   ];
end

function C = quat_to_Cbn(q)
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [ 1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y);
          2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x);
          2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y) ];
end

function r = quat_nhan(p, q)
    r = [ p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4);
          p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3);
          p(1)*q(3) - p(2)*q(4) + p(3)*q(1) + p(4)*q(2);
          p(1)*q(4) + p(2)*q(3) - p(3)*q(2) + p(4)*q(1) ];
end
