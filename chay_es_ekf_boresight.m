% chay_es_ekf_boresight.m
% =========================================================================
% THUẬT TOÁN ES-EKF KHÉP KÍN ƯỚC LƯỢNG ĐỒNG THỜI DẪN ĐƯỜNG VÀ HIỆU CHUẨN DVL
% Ước lượng đồng thời: Vị trí, Vận tốc, Thái độ, Bias IMU, Scale factor s, Boresight
% Kịch bản: Căn chỉnh tĩnh (60s) -> Tăng tốc mượt (10s) -> Hình số 8 (240s)
% Chuẩn lý thuyết: Joan Solà (2017)
% =========================================================================

clear all; close all; clc;

disp('===================================================================');
disp('   HỆ THỐNG DẪN ĐƯỜNG TÍCH HỢP INS/DVL (ES-EKF 19 TRẠNG THÁI)     ');
disp('===================================================================');

%% -------------------------------------------------------------------------
% BƯỚC 0: KIỂM CHỨNG MA TRẬN JACOBIAN H BẰNG ĐẠO HÀM SỐ (FINITE DIFFERENCE)
% -------------------------------------------------------------------------
disp('0. Đang kiểm chứng Jacobian giải tích H bằng đạo hàm số...');

% Tạo điểm thử nghiệm ngẫu nhiên
test_v = [1.5; 0.05; -0.02]; % Vận tốc trong Nav frame (NED)
test_q = [cos(0.1); sin(0.1)*[0.2; -0.1; 0.95]/norm([0.2; -0.1; 0.95])];
test_q = test_q / norm(test_q);
test_w = [0.01; -0.02; 0.14]; % Tốc độ góc thân (rad/s)
test_s = 0.01;
test_bs = [0; -0.005; 0.017]; % Boresight (~1 deg)
test_lever = [0.2; 0; -0.1];  % Lever arm 0.2m

% Ma trận chuyển đổi Body -> Nav và Nav -> Body
Cbn_test = quat_to_Cbn(test_q);
Cnb_test = Cbn_test'; % Nav -> Body

% Vận tốc tại tâm DVL trong Body frame
v_b_test = Cnb_test * test_v;
v_L_test = v_b_test + cross(test_w, test_lever);
Cbs_test = eye(3) - skew_mat(test_bs);
h_ana    = (1 + test_s) * Cbs_test * v_L_test;

% Ma trận Jacobian giải tích H_ana (3x19)
H_ana = zeros(3, 19);
H_ana(:, 4:6)   = (1 + test_s) * Cbs_test * Cnb_test;          % d(h)/d(delta_v)
H_ana(:, 7:9)   = (1 + test_s) * Cbs_test * skew_mat(v_b_test);% d(h)/d(delta_theta) trong Body frame
H_ana(:, 13:15) = (1 + test_s) * Cbs_test * skew_mat(test_lever); % d(h)/d(delta_bg) qua Lever arm
H_ana(:, 16)    = Cbs_test * v_L_test;                         % d(h)/d(delta_s)
H_ana(:, 17:19) = (1 + test_s) * skew_mat(v_L_test);           % d(h)/d(delta_phi_bs)

% Tính đạo hàm số (Finite Difference)
H_num = zeros(3, 19);
eps_step = 1e-7;

% 1. Theo delta_v trong Nav frame (cột 4:6)
for c = 1:3
    dv_pert = zeros(3, 1); dv_pert(c) = eps_step;
    v_p = test_v + dv_pert;
    v_L_p = Cnb_test * v_p + cross(test_w, test_lever);
    H_num(:, 3 + c) = ((1 + test_s) * Cbs_test * v_L_p - h_ana) / eps_step;
end

% 2. Theo delta_theta trong Body frame (cột 7:9)
% Joan Solà: q_p = q * exp(0.5 * dth) -> Cbn_p = Cbn * (I + [dth]x) -> Cnb_p = (I - [dth]x) * Cnb
for c = 1:3
    dth = zeros(3, 1); dth(c) = eps_step;
    dq_p = [1; 0.5 * dth]; dq_p = dq_p / norm(dq_p);
    q_p = quat_nhan(test_q, dq_p);
    Cbn_p = quat_to_Cbn(q_p);
    Cnb_p = Cbn_p';
    v_L_p = Cnb_p * test_v + cross(test_w, test_lever);
    H_num(:, 6 + c) = ((1 + test_s) * Cbs_test * v_L_p - h_ana) / eps_step;
end

% 3. Theo delta_bg (cột 13:15) - w_true = w_m - bg - delta_bg
for c = 1:3
    dbg = zeros(3, 1); dbg(c) = eps_step;
    w_p = test_w - dbg;
    v_L_p = v_b_test + cross(w_p, test_lever);
    H_num(:, 12 + c) = ((1 + test_s) * Cbs_test * v_L_p - h_ana) / (-eps_step);
end

% 4. Theo delta_s (cột 16)
s_p = test_s + eps_step;
H_num(:, 16) = ((1 + s_p) * Cbs_test * v_L_test - h_ana) / eps_step;

% 5. Theo delta_phi_bs (cột 17:19)
for c = 1:3
    dbs = zeros(3, 1); dbs(c) = eps_step;
    bs_p = test_bs + dbs;
    Cbs_p = eye(3) - skew_mat(bs_p);
    H_num(:, 16 + c) = ((1 + test_s) * Cbs_p * v_L_test - h_ana) / eps_step;
end

sai_so_H = max(max(abs(H_ana(:, [4:9, 13:19]) - H_num(:, [4:9, 13:19]))));
if sai_so_H < 1e-4
    fprintf('   --> KẾT QUẢ KIỂM CHỨNG: ĐẠT CHUẨN XUẤT SẮC! (Sai số cực đại = %.2e)\n', sai_so_H);
else
    warning('   --> Cảnh báo: Sai số Jacobian = %.2e. Cần kiểm tra lại!', sai_so_H);
end

%% -------------------------------------------------------------------------
% BƯỚC 1: KHỞI TẠO CẤU HÌNH & MÔ PHỎNG DỮ LIỆU CẢM BIẾN
% -------------------------------------------------------------------------
disp('1. Khởi tạo quỹ đạo UUV mượt mà & mô phỏng cảm biến chuẩn datasheet...');
p = thong_so_he_thong_chuan();

T_can_chinh = 60;   % 60 giây đứng yên căn chỉnh ban đầu
T_tang_toc  = 10;   % 10 giây tăng tốc mượt mà (0 -> 1.5 m/s)
T_run       = 1800; % 30 phút (1800 giây) cơ động hình số 8 liên tục
T_total     = T_can_chinh + T_tang_toc + T_run;

rng(1); % Cố định hạt giống nhiễu

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

% Tạo profile vận tốc và tốc độ quay mượt mà (không có giật gia tốc vô lý)
V = zeros(N, 1);
yaw_rate = zeros(N, 1);

t1 = T_can_chinh;
t2 = T_can_chinh + T_tang_toc;

for i = 1:N
    ti = t(i);
    if ti < t1
        % Pha 1: Đứng yên căn chỉnh
        V(i) = 0.0;
        yaw_rate(i) = 0.0;
    elseif ti >= t1 && ti < t2
        % Pha 2: Tăng tốc mượt từ 0 đến 1.5 m/s bằng hàm sin
        tau = (ti - t1) / T_tang_toc;
        V(i) = 1.5 * (0.5 - 0.5 * cos(pi * tau));
        yaw_rate(i) = 0.0;
    else
        % Pha 3: Cơ động Hình số 8 khép kín 2 vòng tròn đối xứng (Chuẩn thử nghiệm biển)
        % Chu kỳ 1 hình số 8 = 120s (60s vòng phải 360 độ + 60s vòng trái 360 độ)
        tr = ti - t2;
        cycle_time = 120;
        tau_8 = mod(tr, cycle_time);
        w_turn = (6 * pi/180); % 6 độ/s -> 60s quay đúng 360 độ
        
        t_trans = 1.0; % Thời gian chuyển bánh lái 1 giây
        if tau_8 < (60 - t_trans)
            yaw_rate(i) = +w_turn;
        elseif tau_8 < 60
            % Đổi bánh lái mượt từ +6 sang -6 độ/s
            s_tr = (tau_8 - (60 - t_trans)) / t_trans;
            yaw_rate(i) = +w_turn * cos(pi * s_tr);
        elseif tau_8 < (120 - t_trans)
            yaw_rate(i) = -w_turn;
        else
            % Đổi bánh lái mượt từ -6 sang +6 độ/s
            s_tr = (tau_8 - (120 - t_trans)) / t_trans;
            yaw_rate(i) = -w_turn * cos(pi * s_tr);
        end
        
        % Vận tốc Surge biến thiên nhịp nhàng (2 chu kỳ mỗi vòng tròn)
        V(i) = 1.5 + 0.4 * sin(4 * pi * tau_8 / 60);
    end
end

% Tích phân hướng Yaw chuẩn
yaw = zeros(N, 1);
yaw(1) = 0.0; % Hướng ban đầu 0 độ (hướng Bắc)
for i = 2:N
    yaw(i) = yaw(i-1) + yaw_rate(i) * dt;
end

% Vận tốc trong Nav frame (NED)
v_n = zeros(N, 3);
for i = 1:N
    cy = cos(yaw(i)); sy = sin(yaw(i));
    v_n(i, :) = [V(i) * cy, V(i) * sy, 0];
end

% Tọa độ vị trí thực tế chuẩn tuyệt đối (Ground Truth trong hệ NED)
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

% Sinh gia tốc và tốc độ góc lý tưởng
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

% Căn chỉnh hướng ban đầu chuẩn xác bằng góc yaw(1)
cy0 = cos(yaw(1)); sy0 = sin(yaw(1));
Cbn0 = [cy0 -sy0 0; sy0 cy0 0; 0 0 1];
q0 = dcm_to_quat(Cbn0);

% Sinh dữ liệu DVL 5 Hz
buoc    = round(fs / p.dvl.ping_Hz);
idx_dvl = (1:buoc:N)';
ndvl    = numel(idx_dvl);

s_that  = p.si.k_that;             % 1.0%
bs_that = p.si.boresight_that_rad; % [Alpha 0.2; Beta -0.3; Gamma 1.0] deg
Cbs_that = eye(3) - skew_mat(bs_that);
lever_arm = [0.2; 0.0; -0.1];      % Lever arm thực tế 0.2m

Z_dvl = zeros(ndvl, 3);
for j = 1:ndvl
    i  = idx_dvl(j);
    cy = cos(yaw(i)); sy = sin(yaw(i));
    Cnb_y = [cy sy 0; -sy cy 0; 0 0 1];
    vb = Cnb_y * v_n(i, :)';
    w_b = w_that(i, :)';
    v_L = vb + cross(w_b, lever_arm);
    % DVL đo vận tốc với nhiễu
    sig_m = s_cfg.sigma_dvl_fn(norm(v_L));
    Z_dvl(j, :) = ((1 + s_that) * Cbs_that * v_L + sig_m * randn(3, 1))';
end

%% -------------------------------------------------------------------------
% BƯỚC 2: KHỞI TẠO BỘ LỌC ES-EKF 19 TRẠNG THÁI
% -------------------------------------------------------------------------
disp('2. Khởi tạo bộ lọc ES-EKF 19 trạng thái...');

% Trạng thái danh định
pos_est = [0; 0; 0];
v_est   = [0; 0; 0];
q_est   = q0;
ba_est  = [0; 0; 0];
bg_est  = [0; 0; 0];

% ƯỚC LƯỢNG BAN ĐẦU: KHỞI TẠO BẰNG 0 HOÀN TOÀN!
s_est      = 0.0;             % Bắt đầu từ 0% (đáp án thật = +1.0%)
phi_bs_est = [0.0; 0.0; 0.0]; % Bắt đầu từ 0 độ (đáp án: beta = -0.3, gamma = 1.0)

% Lever arm cấu hình trong bộ lọc
lever_arm_est = lever_arm;

% Hiệp phương sai sai số ban đầu P (19x19)
P = diag([
    (0.01)^2 * ones(1, 3),          ... % 1:3   delta_p (m)
    (0.01)^2 * ones(1, 3),          ... % 4:6   delta_v (m/s)
    (0.2 * pi/180)^2 * ones(1, 2),  ... % 7:8   delta_roll, pitch (rad)
    (1.0 * pi/180)^2,               ... % 9     delta_yaw (rad)
    (0.5e-3 * 9.81)^2 * ones(1, 3), ... % 10:12 delta_ba (m/s^2)
    (0.5 * pi/180 / 3600)^2 * ones(1, 3), ... % 13:15 delta_bg (rad/s)
    (0.02)^2,                       ... % 16    delta_s (độ bất định ban đầu +-2%)
    (0.01 * pi/180)^2,              ... % 17    delta_alpha (khóa chặt vì không có kích thích Roll)
    (1.5 * pi/180)^2,               ... % 18    delta_beta (độ bất định ban đầu +-1.5 độ)
    (1.5 * pi/180)^2                ... % 19    delta_gamma (độ bất định ban đầu +-1.5 độ)
]);

% Nhiễu quá trình rời rạc Q_d (19x19)
Q_d = zeros(19, 19);
Q_d(4:6, 4:6)     = (s_cfg.acc_vrw)^2 * dt * eye(3);
Q_d(7:9, 7:9)     = (s_cfg.gyro_arw)^2 * dt * eye(3);
Q_d(10:12, 10:12) = (s_cfg.acc_bias_inrun_sigma)^2 * (2*dt/s_cfg.acc_bias_tau) * eye(3);
Q_d(13:15, 13:15) = (s_cfg.gyro_bias_sigma * 1e-4)^2 * dt * eye(3);
Q_d(16, 16)       = (2e-5)^2 * dt;              % Random walk s: giữ độ thở êm dịu
Q_d(18:19, 18:19) = (0.002 * pi/180)^2 * dt * eye(2); % Random walk Boresight: giữ độ thở êm dịu

% Bộ đệm lưu kết quả
POS_REC   = zeros(N, 3);
VEL_REC   = zeros(N, 3);
S_REC     = zeros(N, 1);
BETA_REC  = zeros(N, 1);
GAMMA_REC = zeros(N, 1);

dvl_ptr = 1;

%% -------------------------------------------------------------------------
% BƯỚC 3: VÒNG LẶP THỜI GIAN THỰC (400 Hz INS + 5 Hz DVL CÓ HỒI TIẾP KHÉP KÍN)
% -------------------------------------------------------------------------
disp('3. Đang chạy vòng lặp ES-EKF khép kín 400 Hz / 5 Hz...');

for i = 1:N
    % In tiến độ mỗi 300 giây mô phỏng (5 phút)
    if mod(i, round(300 / dt)) == 0
        fprintf('   --> Tiến độ mô phỏng: %d / %d giây (%.0f%%)...\n', round(t(i)), round(T_total), t(i)/T_total*100);
    end
    
    % --- [A] CHU KỲ NHANH 400 Hz: DỰ BÁO INS DANH ĐỊNH VÀ COVARIANCE ---
    f_b_meas = f_do(i, :)';
    w_b_meas = w_do(i, :)';
    
    % Khử Bias ước lượng
    f_clean = f_b_meas - ba_est;
    w_clean = w_b_meas - bg_est;
    
    % Ma trận quay Body -> Nav
    Cbn = quat_to_Cbn(q_est);
    
    % Tích phân vận tốc và vị trí danh định
    a_nav = Cbn * f_clean + gn;
    v_est = v_est + a_nav * dt;
    pos_est = pos_est + v_est * dt;
    
    % Tích phân Quaternion thái độ
    dth_step = w_clean * dt;
    dq_step = [cos(norm(dth_step)/2); sin(norm(dth_step)/2) * dth_step / max(norm(dth_step), 1e-12)];
    q_est = quat_nhan(q_est, dq_step);
    q_est = q_est / norm(q_est);
    
    % Lan truyền hiệp phương sai P (19x19)
    F_x = eye(19);
    F_x(1:3, 4:6)   = eye(3) * dt;
    F_x(4:6, 7:9)   = - Cbn * skew_mat(f_clean) * dt;
    F_x(4:6, 10:12) = - Cbn * dt;
    F_x(7:9, 7:9)   = eye(3) - skew_mat(w_clean) * dt;
    F_x(7:9, 13:15) = - eye(3) * dt;
    
    P = F_x * P * F_x' + Q_d;
    
    % --- [B] CHU KỲ CHẬM 5 Hz: CẬP NHẬT ĐO LƯỜNG DVL KHÉP KÍN ---
    if dvl_ptr <= ndvl && idx_dvl(dvl_ptr) == i
        z_m = Z_dvl(dvl_ptr, :)';
        
        % Vận tốc tại tâm DVL trong Body frame
        Cbn_cur = quat_to_Cbn(q_est);
        Cnb_cur = Cbn_cur';
        v_b = Cnb_cur * v_est;
        v_L = v_b + cross(w_clean, lever_arm_est);
        
        % Mô hình đo dự báo h(x)
        C_bs_est = eye(3) - skew_mat(phi_bs_est);
        h_val = (1 + s_est) * C_bs_est * v_L;
        
        % Phần dư (Residual)
        residual = z_m - h_val;
        
        % Ma trận Jacobian H (3x19) CHUẨN XÁC THEO JOAN SOLÀ
        H = zeros(3, 19);
        H(:, 4:6)   = (1 + s_est) * C_bs_est * Cnb_cur;
        H(:, 7:9)   = (1 + s_est) * C_bs_est * skew_mat(v_b);
        H(:, 13:15) = (1 + s_est) * C_bs_est * skew_mat(lever_arm_est);
        H(:, 16)    = C_bs_est * v_L;
        H(:, 17:19) = (1 + s_est) * skew_mat(v_L);
        
        % Khóa cột Roll boresight vì không có kích thích
        H(:, 17) = 0.0;
        
        % Ma trận nhiễu đo R (3x3)
        sig_d = max(s_cfg.sigma_dvl_fn(norm(v_L)), 0.002);
        R_dvl = diag([sig_d^2, sig_d^2, sig_d^2]);
        
        % Ma trận đổi mới S
        S_mat = H * P * H' + R_dvl;
        d2 = residual' * (S_mat \ residual);
        
        % Cổng kiểm định vật lý pha Calibration (Lọc rác âm thanh > 0.5 m/s, không bóp nghẹt bộ lọc)
        if norm(residual) < 0.5
            % 1. Kalman Gain K
            K_gain = (P * H') / S_mat;
            
            % 2. Vector sai số dx (19x1)
            dx = K_gain * residual;
            
            % 3. Cập nhật hiệp phương sai Joseph bền vững số học
            IKH = eye(19) - K_gain * H;
            P_joseph = IKH * P * IKH' + K_gain * R_dvl * K_gain';
            
            % 4. HỒI TIẾP SAI SỐ (INJECTION) VÀO TRẠNG THÁI DANH ĐỊNH
            pos_est = pos_est + dx(1:3);
            v_est   = v_est + dx(4:6);
            
            % Bù thái độ Quaternion (Joan Solà)
            dth = dx(7:9);
            dq_corr = [1; 0.5 * dth];
            dq_corr = dq_corr / norm(dq_corr);
            q_est = quat_nhan(q_est, dq_corr);
            q_est = q_est / norm(q_est);
            
            % Bù Bias cảm biến
            ba_est  = ba_est + dx(10:12);
            bg_est  = bg_est + dx(13:15);
            
            % TÍCH LŨY THAM SỐ DVL (SCALE FACTOR VÀ BORESIGHT)
            s_est         = s_est + dx(16);
            phi_bs_est(2) = phi_bs_est(2) + dx(18); % Pitch beta
            phi_bs_est(3) = phi_bs_est(3) + dx(19); % Yaw gamma
            phi_bs_est(1) = 0.0; % Khóa cứng Roll
            
            % 5. RESET ESKF VÀ ĐỔI HIỆP PHƯƠNG SAI P <- G * P * G^T
            G = eye(19);
            G(7:9, 7:9) = eye(3) - 0.5 * skew_mat(dth);
            P = G * P_joseph * G';
            P = 0.5 * (P + P'); % Ép đối xứng
        end
        dvl_ptr = dvl_ptr + 1;
    end
    
    % Ghi nhận lịch sử ước lượng
    POS_REC(i, :)   = pos_est';
    VEL_REC(i, :)   = v_est';
    S_REC(i)        = s_est * 100;              % %
    BETA_REC(i)     = phi_bs_est(2) * 180 / pi; % deg
    GAMMA_REC(i)    = phi_bs_est(3) * 180 / pi; % deg
end

disp('3. Hoàn thành mô phỏng ES-EKF!');

%% -------------------------------------------------------------------------
% BƯỚC 4: XUẤT BẢNG KẾT QUẢ VÀ SO SÁNH VỚI ĐÁP ÁN THẬT
% -------------------------------------------------------------------------
s_that_pct     = s_that * 100;
beta_that_deg  = bs_that(2) * 180 / pi;
gamma_that_deg = bs_that(3) * 180 / pi;

s_final       = S_REC(end);
beta_final    = BETA_REC(end);
gamma_final   = GAMMA_REC(end);

fprintf('\n================== KẾT QUẢ HIỆU CHUẨN ES-EKF TRỰC TUYẾN ==================\n');
fprintf('Tham số            | Giá trị thật | Ước lượng EKF | Sai số tuyệt đối\n');
fprintf('--------------------------------------------------------------------------\n');
fprintf('Scale Factor s (%%) | %12.4f | %13.4f | %12.4f\n', s_that_pct, s_final, s_final - s_that_pct);
fprintf('Pitch Beta (deg)   | %12.4f | %13.4f | %12.4f\n', beta_that_deg, beta_final, beta_final - beta_that_deg);
fprintf('Yaw Gamma (deg)    | %12.4f | %13.4f | %12.4f\n', gamma_that_deg, gamma_final, gamma_final - gamma_that_deg);
fprintf('==========================================================================\n');

% Đánh giá sai số vị trí thực tế so với Ground Truth (trong giai đoạn cơ động)
t_axis = t - T_can_chinh;
mask = t_axis >= 0;

ERR_POS = sqrt(sum((POS_REC(mask, 1:2) - POS_TRUE(mask, 1:2)).^2, 2));
pos_err_max = max(ERR_POS);
pos_err_rms = rms(ERR_POS);
pos_err_end = ERR_POS(end);
total_dist  = sum(sqrt(sum(v_n(mask, 1:2).^2, 2))) * dt;
drift_rate  = (pos_err_end / max(total_dist, 1e-3)) * 100;

fprintf('\n================ ĐÁNH GIÁ ĐỘ CHÍNH XÁC DẪN ĐƯỜNG (GROUND TRUTH) ================\n');
fprintf('Tổng quãng đường di chuyển: %10.1f mét (30 phút)\n', total_dist);
fprintf('Sai số vị trí cực đại     : %10.3f mét\n', pos_err_max);
fprintf('Sai số vị trí trung phương: %10.3f mét (RMS)\n', pos_err_rms);
fprintf('Sai số vị trí điểm cuối   : %10.3f mét\n', pos_err_end);
fprintf('Tỉ lệ trôi dạt tích lũy   : %10.4f %% quãng đường (Chuẩn hàng hải < 0.2%%)\n', drift_rate);
fprintf('=================================================================================\n');

% Xuất file tham số hiệu chuẩn sẵn sàng nạp cho phần cứng nhúng
calib_output.s_est = s_final / 100;
calib_output.boresight_deg = [0; beta_final; gamma_final];
calib_output.ba_est = ba_est;
calib_output.bg_est = bg_est;
calib_output.pos_err_rms = pos_err_rms;
save('dvl_calib_result.mat', 'calib_output');
fprintf('   --> Đã lưu file cấu hình hiệu chuẩn: dvl_calib_result.mat\n');

%% -------------------------------------------------------------------------
% BƯỚC 5: VẼ ĐỒ THỊ KIỂM CHỨNG HỘI TỤ
% -------------------------------------------------------------------------
t_axis = t - T_can_chinh;
mask = t_axis >= 0;

figure('Name', 'QUÁ TRÌNH HỘI TỤ THAM SỐ DVL (ES-EKF KHÉP KÍN)', 'Color', 'w', 'Position', [100 100 1200 650]);

% 1. Đồ thị Quỹ đạo UUV: Đường Thật vs Ước lượng EKF
subplot(2, 2, 1);
plot(POS_TRUE(mask, 2), POS_TRUE(mask, 1), 'k--', 'LineWidth', 1.6); hold on; grid on; axis equal;
plot(POS_REC(mask, 2), POS_REC(mask, 1), 'b-', 'LineWidth', 1.8);
plot(POS_REC(find(mask,1), 2), POS_REC(find(mask,1), 1), 'r*', 'MarkerSize', 10, 'LineWidth', 2);

% Đánh dấu điểm kết thúc của đường Thật và Ước lượng EKF
idx_end = find(mask, 1, 'last');
plot(POS_TRUE(idx_end, 2), POS_TRUE(idx_end, 1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
plot(POS_REC(idx_end, 2), POS_REC(idx_end, 1), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 7);

% Nối vector độ dạt điểm cuối (Màu đỏ nét gạch)
plot([POS_TRUE(idx_end, 2), POS_REC(idx_end, 2)], ...
     [POS_TRUE(idx_end, 1), POS_REC(idx_end, 1)], 'r--', 'LineWidth', 2);

% Hộp text hiển thị ĐỊNH LƯỢNG ĐỘ DẠT trực tiếp trên hình
str_drift = {
    '\bfĐỘ DẠT HẢI TRÌNH (30 PHÚT):\rm', ...
    sprintf('• Quãng đường: %.0f m', total_dist), ...
    sprintf('• Sai số cực đại: %.2f m', pos_err_max), ...
    sprintf('• Sai số RMS: %.2f m', pos_err_rms), ...
    sprintf('• Dạt điểm cuối: %.2f m', pos_err_end), ...
    sprintf('• Tỉ lệ dạt: %.3f%% (< 0.2%%)', drift_rate)
};
text(0.03, 0.22, str_drift, 'Units', 'normalized', 'FontSize', 8.5, ...
    'BackgroundColor', [1 1 1 0.90], 'EdgeColor', [0.6 0.6 0.6], 'Margin', 5);

title('Quỹ đạo: Thật (Ground Truth) vs ES-EKF');
xlabel('Đông (m)'); ylabel('Bắc (m)');
legend('Thực tế (Ground Truth)', 'Ước lượng ES-EKF', 'Bắt đầu', ...
       'Đích Thật', 'Đích EKF', 'Vector dạt cuối', 'Location', 'northeast');

% 2. Hội tụ Scale factor s (%)
subplot(2, 2, 2);
plot(t_axis(mask), S_REC(mask), 'b-', 'LineWidth', 1.8); hold on; grid on;
yline(s_that_pct, 'r--', 'LineWidth', 1.5);
title('Quá trình Hội tụ Hệ số Tỷ lệ s (%)');
xlabel('Thời gian cơ động (s)'); ylabel('s (%)');
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0%)', 'Location', 'best');

% 3. Hội tụ Pitch Boresight beta (độ)
subplot(2, 2, 3);
plot(t_axis(mask), BETA_REC(mask), 'g-', 'LineWidth', 1.8); hold on; grid on;
yline(beta_that_deg, 'r--', 'LineWidth', 1.5);
title('Quá trình Hội tụ Pitch Boresight \beta (độ)');
xlabel('Thời gian cơ động (s)'); ylabel('\beta (độ)');
legend('Ước lượng ES-EKF', 'Giá trị thật (-0.3 độ)', 'Location', 'best');

% 4. Hội tụ Yaw Boresight gamma (độ)
subplot(2, 2, 4);
plot(t_axis(mask), GAMMA_REC(mask), 'm-', 'LineWidth', 1.8); hold on; grid on;
yline(gamma_that_deg, 'r--', 'LineWidth', 1.5);
title('Quá trình Hội tụ Yaw Boresight \gamma (độ)');
xlabel('Thời gian cơ động (s)'); ylabel('\gamma (độ)');
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0 độ)', 'Location', 'best');

sgtitle('KẾT QUẢ ƯỚC LƯỢNG ĐỒNG THỜI DẪN ĐƯỜNG VÀ HIỆU CHUẨN DVL (ES-EKF)', 'FontSize', 14, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% HÌNH 2: PHÂN TÍCH CHI TIẾT ĐỘ DẠT VỊ TRÍ THEO THỜI GIAN
% -------------------------------------------------------------------------
figure('Name', 'PHÂN TÍCH CHI TIẾT ĐỘ DẠT QUỸ ĐẠO THEO THỜI GIAN', 'Color', 'w', 'Position', [150 150 1100 550]);

subplot(2, 1, 1);
plot(t_axis(mask), ERR_POS, 'r-', 'LineWidth', 1.6); hold on; grid on;
yline(pos_err_rms, 'b--', sprintf('RMS: %.2f m', pos_err_rms), 'LineWidth', 1.5);
yline(pos_err_max, 'k:', sprintf('Cực đại: %.2f m', pos_err_max), 'LineWidth', 1.5);
title('Độ dạt vị trí tức thời ||p_{EKF} - p_{TRUE}|| theo thời gian (m)');
xlabel('Thời gian cơ động (s)'); ylabel('Độ dạt (m)');
legend('Sai số tức thời', 'Đường chuẩn RMS', 'Đường cực đại Max', 'Location', 'best');

subplot(2, 1, 2);
err_N = POS_REC(mask, 1) - POS_TRUE(mask, 1);
err_E = POS_REC(mask, 2) - POS_TRUE(mask, 2);
plot(t_axis(mask), err_N, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_axis(mask), err_E, 'm-', 'LineWidth', 1.5);
yline(0, 'k--');
title('Phân tích Độ dạt từng trục: Trục Bắc (\Delta North) và Trục Đông (\Delta East)');
xlabel('Thời gian cơ động (s)'); ylabel('Độ dạt (m)');
legend('Độ dạt trục Bắc (\Delta N)', 'Độ dạt trục Đông (\Delta E)', 'Location', 'best');

sgtitle(sprintf('BIỂU ĐỒ ĐỘ DẠT DẪN ĐƯỜNG (Tổng quãng đường: %.0f m | Dạt cuối: %.2f m = %.3f%%)', ...
    total_dist, pos_err_end, drift_rate), 'FontSize', 13, 'FontWeight', 'bold');

%% -------------------------------------------------------------------------
% CÁC HÀM PHỤ TRỢ ĐẠI SỐ QUATERNION VÀ MA TRẬN
% -------------------------------------------------------------------------
function S = skew_mat(v)
    S = [  0,    -v(3),  v(2);
          v(3),    0,   -v(1);
         -v(2),   v(1),   0   ];
end

function C = quat_to_Cbn(q)
    % Chuyển Quaternion thành ma trận quay C_b^n (Body -> Nav)
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [ 1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y);
          2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x);
          2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y) ];
end

function r = quat_nhan(p, q)
    % Phép nhân Quaternion chuẩn Hamilton: p * q
    r = [ p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4);
          p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3);
          p(1)*q(3) - p(2)*q(4) + p(3)*q(1) + p(4)*q(2);
          p(1)*q(4) + p(2)*q(3) - p(3)*q(2) + p(4)*q(1) ];
end

function q = dcm_to_quat(R)
    % Chuyển ma trận quay R thành Quaternion chuẩn
    tr = trace(R);
    if tr > 0
        S = 2 * sqrt(tr + 1);
        q = [0.25*S; (R(3,2)-R(2,3))/S; (R(1,3)-R(3,1))/S; (R(2,1)-R(1,2))/S];
    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        S = 2 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
        q = [(R(3,2)-R(2,3))/S; 0.25*S; (R(1,2)+R(2,1))/S; (R(1,3)+R(3,1))/S];
    elseif R(2,2) > R(3,3)
        S = 2 * sqrt(1 + R(2,2) - R(1,1) - R(3,3));
        q = [(R(1,3)-R(3,1))/S; (R(1,2)+R(2,1))/S; 0.25*S; (R(2,3)+R(3,2))/S];
    else
        S = 2 * sqrt(1 + R(3,3) - R(1,1) - R(2,2));
        q = [(R(2,1)-R(1,2))/S; (R(1,3)+R(3,1))/S; (R(2,3)+R(3,2))/S; 0.25*S];
    end
    q = q / norm(q);
    if q(1) < 0, q = -q; end
end
