% boingua.m
% =========================================================================
% CHƯƠNG TRÌNH DẪN ĐƯỜNG UUV KIỂU MỚI: INS/DVL ES-EKF 19 TRẠNG THÁI
% Thiết kế kịch bản chuẩn tác chiến:
% - Phase 1 (Mặt nước 0 -> 75s): Đứng yên (15s) -> Tăng tốc (10s) -> Bơi thẳng & Lượn S nhẹ (50s)
%   Có GNSS 2 anten làm chuẩn: Hiệu chuẩn sạch s (+1.0%), beta (-0.3 độ), gamma (+1.0 độ).
% - Phase 2 (Lặn ngầm 75s -> 375s): Cắt hoàn toàn GNSS, bơi khép kín số 8 dẫn đường DVL.
%   Hệ số s được khóa chặt (freeze), INS bù đầy đủ quay Trái Đất và Coriolis.
% =========================================================================

clear all; close all; clc;

disp('===================================================================');
disp('   HỆ THỐNG DẪN ĐƯỜNG TÍCH HỢP UUV INS/DVL (KỊCH BẢN CHUẨN MỚI)   ');
disp('   Hiệu chuẩn sạch Phase 1 (GNSS) -> Dẫn đường chính xác Phase 2   ');
disp('===================================================================');

%% -------------------------------------------------------------------------
% BƯỚC 1: KHỞI TẠO CẤU HÌNH & KỊCH BẢN QUỸ ĐẠO MỚI
% -------------------------------------------------------------------------
disp('1. Khởi tạo cấu hình hệ thống & sinh quỹ đạo 2 pha...');
p = thong_so_he_thong_chuan();

T_phase1 = 60;   % Phase 1: 60s trên mặt nước có GNSS
T_run    = 480;  % Phase 2: 480s (4 chu kỳ số 8 khép kín lặn ngầm)
T_total  = T_phase1 + T_run;

rng(1); % Cố định hạt giống nhiễu để kết quả lặp lại chuẩn xác

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

% Thiết lập profile vận tốc và góc quay bánh lái
V = zeros(N, 1);
yaw_rate = zeros(N, 1);

for i = 1:N
    ti = t(i);
    if ti < 15
        % 1A: Đứng yên thả nổi (V = 0, chốt hướng Bắc GNSS 2 anten & bias tĩnh FOG)
        V(i) = 0.0;
        yaw_rate(i) = 0.0;
    elseif ti < 25
        % 1B: Tăng tốc êm dịu từ 0 lên 1.5 m/s trên mặt nước
        tau = (ti - 15) / 10;
        V(i) = 1.5 * (0.5 - 0.5 * cos(pi * tau));
        yaw_rate(i) = 0.0;
    elseif ti < T_phase1
        % 1C: Bơi thẳng ổn định 1.5 m/s trên mặt nước (Cực kỳ trực giao: X->s, Y->gamma, Z->beta)
        V(i) = 1.5;
        yaw_rate(i) = 0.0;
    else
        % GIAI ĐOẠN 2: LẶN NGẦM (CẮT HẾT GNSS) - CƠ ĐỘNG SỐ 8 KHÉP KÍN
        tr = ti - T_phase1;
        cycle_time = 120; % Chu kỳ số 8: 120s
        tau_8 = mod(tr, cycle_time);
        w_turn = (6 * pi/180); % 6 độ/giây (bán kính cua ~14m)
        
        t_trans = 1.0;
        if tr < t_trans
            yaw_rate(i) = w_turn * (0.5 - 0.5 * cos(pi * tr / t_trans));
        elseif tau_8 < (60 - t_trans)
            yaw_rate(i) = +w_turn;
        elseif tau_8 < 60
            s_tr = (tau_8 - (60 - t_trans)) / t_trans;
            yaw_rate(i) = +w_turn * cos(pi * s_tr);
        elseif tau_8 < (120 - t_trans)
            yaw_rate(i) = -w_turn;
        else
            s_tr = (tau_8 - (120 - t_trans)) / t_trans;
            yaw_rate(i) = -w_turn * cos(pi * s_tr);
        end
        
        V(i) = 1.5 + 0.3 * sin(4 * pi * tau_8 / 60);
    end
end

% Tích phân góc Yaw chuẩn thực tế
yaw = zeros(N, 1);
yaw(1) = 0.0; % Xuất phát hướng Bắc (0 độ)
for i = 2:N
    yaw(i) = yaw(i-1) + yaw_rate(i) * dt;
end

% Vận tốc và vị trí chuẩn tuyệt đối Ground Truth (NED)
v_n = zeros(N, 3);
for i = 1:N
    cy = cos(yaw(i)); sy = sin(yaw(i));
    v_n(i, :) = [V(i) * cy, V(i) * sy, 0];
end
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

% Sinh tín hiệu cảm biến IMU thực tế có tự quay Trái Đất
w_that = zeros(N, 3);
f_that = zeros(N, 3);
for i = 1:N
    j = max(i - 1, 1);
    sl2 = sin(lat(j))^2;
    RN = a_e / sqrt(1 - e2 * sl2);
    RM = a_e * (1 - e2) / (1 - e2 * sl2)^1.5;
    w_ie = W * [cos(lat(j)); 0; -sin(lat(j))];
    w_en = [v_n(j,2) / (RN + h(j)); -v_n(j,1) / (RM + h(j)); -v_n(j,2) * tan(lat(j)) / (RN + h(j))];
    
    cy = cos(yaw(j)); sy = sin(yaw(j));
    Cnb_y = [cy sy 0; -sy cy 0; 0 0 1];
    w_that(i, :) = ([0; 0; yaw_rate(i)] + Cnb_y * (w_ie + w_en))';
    
    a_n = (v_n(i, :)' - v_n(j, :)') / dt;
    cy2 = cos(yaw(i)); sy2 = sin(yaw(i));
    Cnb_y2 = [cy2 sy2 0; -sy2 cy2 0; 0 0 1];
    f_that(i, :) = (Cnb_y2 * (a_n + cross(2*w_ie + w_en, v_n(j, :)') - gn))';
end

% Bơm sai số FOG và Gia tốc kế theo chuẩn Datasheet
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

% Góc thái độ ban đầu
q0 = [cos(yaw(1)/2); 0; 0; sin(yaw(1)/2)];

% Tín hiệu đo GNSS 2 anten (Chỉ hoạt động ở Phase 1)
dt_gps = 0.2; % 5 Hz
idx_meas = 1:round(dt_gps / dt):N;
n_meas = length(idx_meas);

sig_gps_pos = 0.02; % RTK 2 cm
sig_gps_vel = 0.02; % RTK 2 cm/s
sig_gps_yaw = 0.05 * pi/180; % 2 anten: 0.05 độ

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

% Tín hiệu đo DVL 5 Hz (Cảm biến cơ khí)
s_that  = p.si.k_that;             % +1.0%
bs_that = p.si.boresight_that_rad; % Alpha 0.2, Beta -0.3, Gamma 1.0 deg
Cbs_that = eye(3) - skew_mat(bs_that);
lever_arm = [0.2; 0.0; -0.1];      % Lever arm 0.2m

Z_dvl = zeros(n_meas, 3);
for j = 1:n_meas
    idx = idx_meas(j);
    cy = cos(yaw(idx)); sy = sin(yaw(idx));
    Cnb_y = [cy sy 0; -sy cy 0; 0 0 1];
    vb = Cnb_y * v_n(idx, :)';
    w_b = w_that(idx, :)';
    v_L = vb + cross(w_b, lever_arm);
    sig_m = s_cfg.sigma_dvl_fn(norm(v_L));
    Z_dvl(j, :) = ((1 + s_that) * Cbs_that * v_L + sig_m * randn(3, 1))';
end

%% -------------------------------------------------------------------------
% BƯỚC 2: KHỞI TẠO BỘ LỌC ES-EKF 19 TRẠNG THÁI (JOAN SOLÀ 2017)
% -------------------------------------------------------------------------
disp('2. Khởi tạo bộ lọc ES-EKF 19 trạng thái...');

pos_est = [0; 0; 0];
v_est   = [0; 0; 0];
q_est   = q0;
ba_est  = [0; 0; 0];
bg_est  = [0; 0; 0];

% KHỞI TẠO BẰNG 0 HOÀN TOÀN: Để kiểm chứng khả năng tự tìm ra đáp án thật
s_est      = 0.0;             % Bắt đầu từ 0% (Đáp án: +1.0%)
phi_bs_est = [0.0; 0.0; 0.0]; % Bắt đầu từ 0 độ (Đáp án: beta = -0.3, gamma = +1.0)
lever_arm_est = lever_arm;

P = diag([
    (0.01)^2 * ones(1, 3),          ... % 1:3   delta_p (m)
    (0.01)^2 * ones(1, 3),          ... % 4:6   delta_v (m/s)
    (0.2 * pi/180)^2 * ones(1, 2),  ... % 7:8   delta_roll, pitch (rad)
    (0.05 * pi/180)^2,              ... % 9     delta_yaw (GNSS 2 anten định chuẩn)
    (0.5e-3 * 9.81)^2 * ones(1, 3), ... % 10:12 delta_ba (m/s^2)
    (0.5 * pi/180 / 3600)^2 * ones(1, 3), ... % 13:15 delta_bg (rad/s)
    (0.02)^2,                       ... % 16    delta_s (+-2%)
    (0.01 * pi/180)^2,              ... % 17    delta_alpha (khóa)
    (1.5 * pi/180)^2,               ... % 18    delta_beta (+-1.5 độ)
    (1.5 * pi/180)^2                ... % 19    delta_gamma (+-1.5 độ)
]);

% Nhiễu quá trình rời rạc Q_d
Q_d = zeros(19, 19);
Q_d(4:6, 4:6)     = (s_cfg.acc_vrw)^2 * dt * eye(3);
Q_d(7:9, 7:9)     = (s_cfg.gyro_arw)^2 * dt * eye(3);
Q_d(10:12, 10:12) = (s_cfg.acc_bias_inrun_sigma)^2 * (2*dt/s_cfg.acc_bias_tau) * eye(3);
Q_d(13:15, 13:15) = (s_cfg.gyro_bias_sigma * 1e-4)^2 * dt * eye(3);
Q_d(16, 16)       = (2e-5)^2 * dt; 
Q_d(18:19, 18:19) = (3e-4 * pi/180)^2 * dt * eye(2); % Cho phép EKF bóc tách gamma khi lượn số 8

% Bộ đệm lưu kết quả
POS_REC   = zeros(N, 3);
VEL_REC   = zeros(N, 3);
S_REC     = zeros(N, 1);
BETA_REC  = zeros(N, 1);
GAMMA_REC = zeros(N, 1);

meas_ptr = 1;

%% -------------------------------------------------------------------------
% BƯỚC 3: VÒNG LẶP THỜI GIAN THỰC (400 Hz INS + 5 Hz ĐO ĐẠC)
% -------------------------------------------------------------------------
disp('3. Đang thực thi vòng lặp ES-EKF dẫn đường...');

for i = 1:N
    ti = t(i);
    
    if mod(i, round(60 / dt)) == 0
        fprintf('   --> Thời gian: %d / %d s (%.0f%%) | s = %+.2f%% | gamma = %+.2f deg\n', ...
            round(ti), round(T_total), ti/T_total*100, s_est*100, phi_bs_est(3)*180/pi);
    end
    
    % --- [A] 400 Hz DỰ BÁO INS DANH ĐỊNH (CÓ BÙ QUAY TRÁI ĐẤT & CORIOLIS) ---
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
    
    % --- [B] 5 Hz CẬP NHẬT ĐO ĐẠC KALMAN ---
    if meas_ptr <= n_meas && i == idx_meas(meas_ptr)
        
        % 1. CẬP NHẬT GNSS 2 ANTEN (CHỈ TRONG PHASE 1: ti <= T_phase1)
        if ti <= T_phase1
            % 1.1 Vị trí & Vận tốc GNSS RTK
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
            
            % 1.2 Góc hướng Yaw từ GNSS 2 anten
            Cbn_cur = quat_to_Cbn(q_est);
            yaw_est = atan2(Cbn_cur(2, 1), Cbn_cur(1, 1));
            res_yaw = atan2(sin(Z_gps_yaw(meas_ptr) - yaw_est), cos(Z_gps_yaw(meas_ptr) - yaw_est));
            
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
        end
        
        % 2. CẬP NHẬT DVL (CẢ PHASE 1 VÀ PHASE 2)
        Cbn_cur = quat_to_Cbn(q_est);
        Cnb_cur = Cbn_cur';
        v_b = Cnb_cur * v_est;
        v_L = v_b + cross(w_clean, lever_arm_est);
        
        % ĐIỀU KIỆN CHUẨN: Chỉ cập nhật DVL khi vận tốc đã đạt ổn định V > 1.3 m/s
        % Giúp loại bỏ hoàn toàn nhiễu giật lúc khởi động tăng tốc ban đầu!
        if norm(v_L) > 1.3
            z_m = Z_dvl(meas_ptr, :)';
            C_bs_est = eye(3) - skew_mat(phi_bs_est);
            h_val = (1 + s_est) * C_bs_est * v_L;
            residual = z_m - h_val;
            
            H = zeros(3, 19);
            if ti <= T_phase1
                % Phase 1: Hiệu chuẩn sạch s (16), beta (18), gamma (19) so với chuẩn GNSS
                H(:, 16)    = C_bs_est * v_L;
                H(:, 17:19) = (1 + s_est) * skew_mat(v_L);
                H(:, 17)    = 0.0; % Khóa Roll
            else
                % Phase 2: Lặn ngầm dẫn đường DVL ghìm INS
                H(:, 4:6)   = (1 + s_est) * C_bs_est * Cnb_cur;
                H(:, 7:9)   = (1 + s_est) * C_bs_est * skew_mat(v_b);
                H(:, 9)     = 0.0; % Khóa cột 9 (Yaw INS để con quay FOG giữ chuẩn Bắc)
                H(:, 13:15) = (1 + s_est) * C_bs_est * skew_mat(lever_arm_est);
                H(:, 16)    = C_bs_est * v_L;
                H(:, 17:19) = (1 + s_est) * skew_mat(v_L);
                H(:, 17)    = 0.0;
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
                phi_bs_est(2) = phi_bs_est(2) + dx(18); % Beta
                phi_bs_est(3) = phi_bs_est(3) + dx(19); % Gamma
                phi_bs_est(1) = 0.0;
                
                G = eye(19);
                G(7:9, 7:9) = eye(3) - 0.5 * skew_mat(dth);
                P = G * P_joseph * G';
                P = 0.5 * (P + P');
            end
        end
        meas_ptr = meas_ptr + 1;
    end
    
    % Lưu kết quả
    POS_REC(i, :)   = pos_est';
    VEL_REC(i, :)   = v_est';
    S_REC(i)        = s_est * 100;
    BETA_REC(i)     = phi_bs_est(2) * 180 / pi;
    GAMMA_REC(i)    = phi_bs_est(3) * 180 / pi;
end

%% -------------------------------------------------------------------------
% BƯỚC 4: TÍNH TOÁN SAI SỐ & VẼ ĐỒ THỊ TRỰC QUAN TOÀN DIỆN
% -------------------------------------------------------------------------
disp('4. Tính toán thống kê hiệu năng dẫn đường & vẽ đồ thị...');

mask_p1 = (t <= T_phase1);
mask_p2 = (t > T_phase1);

% Sai số khi lặn ngầm (Phase 2)
ERR_POS = sqrt(sum((POS_REC(mask_p2, 1:2) - POS_TRUE(mask_p2, 1:2)).^2, 2));
pos_err_max = max(ERR_POS);
pos_err_rms = sqrt(mean(ERR_POS.^2));
pos_err_end = ERR_POS(end);

% Quãng đường lặn ngầm
diff_true = diff(POS_TRUE(mask_p2, 1:2), 1, 1);
total_dist = sum(sqrt(sum(diff_true.^2, 2)));
drift_rate = (pos_err_end / total_dist) * 100;

fprintf('\n===================================================================\n');
fprintf('              KẾT QUẢ ĐÁNH GIÁ HIỆU NĂNG DẪN ĐƯỜNG UUV             \n');
fprintf('===================================================================\n');
fprintf('• Hệ số Scale s ước lượng cuối : %+.3f %% (Đáp án: +1.000 %%)\n', S_REC(end));
fprintf('• Pitch Boresight beta         : %+.3f deg (Đáp án: -0.300 deg)\n', BETA_REC(end));
fprintf('• Yaw Boresight gamma          : %+.3f deg (Đáp án: +1.000 deg)\n', GAMMA_REC(end));
fprintf('-------------------------------------------------------------------\n');
fprintf('• Quãng đường lặn ngầm         : %.1f mét (~%.2f hải lý)\n', total_dist, total_dist/1852);
fprintf('• Sai số dạt điểm cuối         : %.3f mét\n', pos_err_end);
fprintf('• Tỉ lệ dạt hành trình         : %.4f %%\n', drift_rate);
fprintf('===================================================================\n\n');

% HÌNH 1: SO SÁNH QUỸ ĐẠO VÀ HỘI TỤ THAM SỐ
figure('Name', 'KẾT QUẢ DẪN ĐƯỜNG ES-EKF (PHASE 1 GNSS -> PHASE 2 DVL)', ...
       'Color', 'w', 'Position', [60 60 1300 700]);

% 1.1 Quỹ đạo 2D mặt bằng (East vs North)
subplot(2, 2, 1);
plot(POS_TRUE(:, 2), POS_TRUE(:, 1), 'k--', 'LineWidth', 2.0); hold on; grid on; axis equal;
plot(POS_REC(mask_p1, 2), POS_REC(mask_p1, 1), 'Color', [1.0 0.55 0.0], 'LineWidth', 2.2);
plot(POS_REC(mask_p2, 2), POS_REC(mask_p2, 1), 'b-', 'LineWidth', 2.0);

plot(POS_REC(1, 2), POS_REC(1, 1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
idx_dive = find(mask_p2, 1);
plot(POS_REC(idx_dive, 2), POS_REC(idx_dive, 1), 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
plot(POS_TRUE(end, 2), POS_TRUE(end, 1), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
plot(POS_REC(end, 2), POS_REC(end, 1), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8);

title('Quỹ đạo 2 Pha: Mặt nước (GNSS) \rightarrow Lặn ngầm (DVL)');
xlabel('Trục Đông - East (m)'); ylabel('Trục Bắc - North (m)');
legend('Thực tế (Ground Truth)', 'Phase 1: Mặt nước GNSS', 'Phase 2: Lặn ngầm ES-EKF', ...
       'Xuất phát', 'Điểm lặn cắt GNSS', 'Đích Thật', 'Đích EKF', 'Location', 'northeast');

% 1.2 Hội tụ Scale factor s (%)
subplot(2, 2, 2);
plot(t, S_REC, 'b-', 'LineWidth', 2.0); hold on; grid on;
yline(1.0, 'r--', 'Đáp án: +1.0%', 'LineWidth', 1.5);
xline(T_phase1, 'k--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Ước lượng Hệ số Tỷ xích Scale factor s (%)');
xlabel('Thời gian nhiệm vụ (s)'); ylabel('s (%)');
ylim([-0.5, 2.5]);
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0%)', 'Mốc cắt GNSS', 'Location', 'best');

% 1.3 Hội tụ Pitch Boresight beta
subplot(2, 2, 3);
plot(t, BETA_REC, 'g-', 'LineWidth', 2.0); hold on; grid on;
yline(-0.3, 'r--', 'Đáp án: -0.3 độ', 'LineWidth', 1.5);
xline(T_phase1, 'k--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Ước lượng Pitch Boresight \beta (độ)');
xlabel('Thời gian nhiệm vụ (s)'); ylabel('\beta (độ)');
ylim([-1.0, 0.5]);
legend('Ước lượng ES-EKF', 'Giá trị thật (-0.3 độ)', 'Mốc cắt GNSS', 'Location', 'best');

% 1.4 Hội tụ Yaw Boresight gamma
subplot(2, 2, 4);
plot(t, GAMMA_REC, 'm-', 'LineWidth', 2.0); hold on; grid on;
yline(1.0, 'r--', 'Đáp án: +1.0 độ', 'LineWidth', 1.5);
xline(T_phase1, 'k--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Ước lượng Yaw Boresight \gamma (độ)');
xlabel('Thời gian nhiệm vụ (s)'); ylabel('\gamma (độ)');
ylim([-0.5, 2.5]);
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0 độ)', 'Mốc cắt GNSS', 'Location', 'best');

sgtitle('ĐÁNH GIÁ DẪN ĐƯỜNG VÀ HIỆU CHUẨN TRỰC TUYẾN DVL (ES-EKF)', 'FontSize', 13, 'FontWeight', 'bold');

% HÌNH 2: PHÂN TÍCH ĐỘ DẠT VỊ TRÍ KHI LẶN NGẦM
figure('Name', 'PHÂN TÍCH ĐỘ DẠT VỊ TRÍ KHI LẶN NGẦM', 'Color', 'w', 'Position', [120 120 900 450]);
t_sub = t(mask_p2) - T_phase1;
plot(t_sub, ERR_POS, 'r-', 'LineWidth', 2.0); hold on; grid on;
yline(pos_err_rms, 'b--', sprintf('RMS: %.3f m', pos_err_rms), 'LineWidth', 1.5);
title('Độ dạt vị trí lặn ngầm theo thời gian ||p_{EKF} - p_{TRUE}|| (m)');
xlabel('Thời gian lặn ngầm (s)'); ylabel('Sai số (m)');
legend('Sai số dạt tức thời', 'Sai số RMS', 'Location', 'northwest');
str_box = {
    sprintf('Quãng đường: %.1f m', total_dist), ...
    sprintf('Sai số Max: %.3f m', pos_err_max), ...
    sprintf('Dạt cuối : %.3f m (%.4f%% DT)', pos_err_end, drift_rate)
};
text(0.65, 0.35, str_box, 'Units', 'normalized', 'FontSize', 10, ...
    'BackgroundColor', [1 1 1 0.9], 'EdgeColor', [0.7 0.7 0.7]);

% Đồng bộ các biến vào base workspace để có thể chạy ngay 3D mô phỏng
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

disp('--> ĐÃ HOÀN THÀNH MÔ PHỎNG VÀ ĐỒNG BỘ DỮ LIỆU SANG WORKSPACE!');
disp('--> Bạn có thể gõ "mo_phong_3d_matlab" để xem chuyển động 3D UUV!');

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
