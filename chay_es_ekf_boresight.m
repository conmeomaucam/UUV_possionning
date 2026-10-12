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

T_phase1 = 60;   % Phase 1: 60s trên mặt nước có GNSS 2 anten chốt chuẩn Bắc
T_run    = 600;  % Phase 2: 10 phút (600s = 5 chu kỳ số 8 khép kín lặn ngầm)
T_total  = T_phase1 + T_run;

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

% Tạo profile vận tốc và tốc độ quay cho 2 Pha
V = zeros(N, 1);
yaw_rate = zeros(N, 1);

for i = 1:N
    ti = t(i);
    if ti < 15
        % Giai đoạn 1A: Đứng yên thả nổi trên mặt nước (V = 0, chốt GNSS 2 anten chuẩn Bắc tuyệt đối)
        V(i) = 0.0;
        yaw_rate(i) = 0.0;
    elseif ti < 25
        % Giai đoạn 1B: Tăng tốc êm dịu từ 0 lên 1.5 m/s trên mặt nước
        tau = (ti - 15) / 10;
        V(i) = 1.5 * (0.5 - 0.5 * cos(pi * tau));
        yaw_rate(i) = 0.0;
    elseif ti < T_phase1
        % Giai đoạn 1C: Bơi thẳng ổn định 1.5 m/s trên mặt nước, DVL cập nhật cùng GNSS
        V(i) = 1.5;
        yaw_rate(i) = 0.0;
    else
        % Giai đoạn 2: LẶN NGẦM (CẮT GNSS) - CƠ ĐỘNG SỐ 8 KHÉP KÍN DẪN ĐƯỜNG DVL
        tr = ti - T_phase1;
        cycle_time = 120;
        tau_8 = mod(tr, cycle_time);
        w_turn = (6 * pi/180); % 6 độ/s -> 60s quay đúng 360 độ
        
        t_trans = 1.0; % Thời gian chuyển bánh lái mượt 1 giây
        if tr < t_trans
            % 1 giây đầu khi vừa bắt đầu lặn: bẻ lái mượt từ 0 đến +w_turn
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
        
        % Vận tốc Surge biến thiên nhịp nhàng quanh 1.5 m/s
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

% Sinh dữ liệu đo lường 5 Hz (Cùng chu kỳ buoc = round(fs / 5))
buoc     = round(fs / 5); % 5 Hz (mỗi 0.2s)
idx_meas = (1:buoc:N)';
n_meas   = numel(idx_meas);

% 1. Dữ liệu GNSS 2 anten (Chỉ hoạt động trong Phase 1: t <= T_phase1)
sig_gps_pos = 0.03;          % Sai số vị trí RTK 3 cm
sig_gps_vel = 0.02;          % Sai số vận tốc 2 cm/s
sig_gps_yaw = 0.05 * pi/180; % Sai số góc hướng 2 anten 0.05 độ (chuẩn INS-DM-FI)

Z_gps_pos = zeros(n_meas, 3);
Z_gps_vel = zeros(n_meas, 3);
Z_gps_yaw = zeros(n_meas, 1);

for j = 1:n_meas
    idx = idx_meas(j);
    if t(idx) <= T_phase1
        Z_gps_pos(j, :) = POS_TRUE(idx, :) + sig_gps_pos * randn(1, 3);
        Z_gps_vel(j, :) = v_n(idx, :)      + sig_gps_vel * randn(1, 3);
        Z_gps_yaw(j)    = yaw(idx)         + sig_gps_yaw * randn;
    end
end

% 2. Dữ liệu DVL 5 Hz (Hoạt động khi lặn ngầm trong Phase 2: t > T_phase1)
s_that  = p.si.k_that;             % 1.0%
bs_that = p.si.boresight_that_rad; % [Alpha 0.2; Beta -0.3; Gamma 1.0] deg
Cbs_that = eye(3) - skew_mat(bs_that);
lever_arm = [0.2; 0.0; -0.1];      % Lever arm thực tế 0.2m

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
    (0.05 * pi/180)^2,              ... % 9     delta_yaw (được định chuẩn bởi GNSS 2 anten Phase 1)
    (0.5e-3 * 9.81)^2 * ones(1, 3), ... % 10:12 delta_ba (m/s^2)
    (0.5 * pi/180 / 3600)^2 * ones(1, 3), ... % 13:15 delta_bg (rad/s)
    (0.02)^2,                       ... % 16    delta_s (độ bất định ban đầu +-2%)
    (0.01 * pi/180)^2,              ... % 17    delta_alpha (khóa chặt)
    (1.5 * pi/180)^2,               ... % 18    delta_beta (độ bất định ban đầu +-1.5 độ)
    (1.5 * pi/180)^2                ... % 19    delta_gamma (độ bất định ban đầu +-1.5 độ)
]);

% Nhiễu quá trình rời rạc Q_d (19x19)
Q_d = zeros(19, 19);
Q_d(4:6, 4:6)     = (s_cfg.acc_vrw)^2 * dt * eye(3);
Q_d(7:9, 7:9)     = (s_cfg.gyro_arw)^2 * dt * eye(3);
Q_d(10:12, 10:12) = (s_cfg.acc_bias_inrun_sigma)^2 * (2*dt/s_cfg.acc_bias_tau) * eye(3);
Q_d(13:15, 13:15) = (s_cfg.gyro_bias_sigma * 1e-4)^2 * dt * eye(3);
% Nhiễu quá trình tham số cơ khí cố định s, beta, gamma (giữ mức làm mịn phẳng lì)
Q_d(16, 16)       = (5e-7)^2 * dt;              
Q_d(18:19, 18:19) = (5e-5 * pi/180)^2 * dt * eye(2);

% Bộ đệm lưu kết quả
POS_REC   = zeros(N, 3);
VEL_REC   = zeros(N, 3);
S_REC     = zeros(N, 1);
BETA_REC  = zeros(N, 1);
GAMMA_REC = zeros(N, 1);

meas_ptr = 1;

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
    Cnb = Cbn';
    
    % BƯỚC 1: BÙ TỰ QUAY TRÁI ĐẤT (w_ie) VÀ CHUYỂN DỊCH ĐỊA LÝ (w_en)
    lat_i = lat(i);
    w_ie_nav = W * [cos(lat_i); 0; -sin(lat_i)];
    sl2_i = sin(lat_i)^2;
    RN_i = a_e / sqrt(1 - e2 * sl2_i);
    RM_i = a_e * (1 - e2) / (1 - e2 * sl2_i)^1.5;
    w_en_nav = [v_est(2)/(RN_i + h(i)); -v_est(1)/(RM_i + h(i)); -v_est(2)*tan(lat_i)/(RN_i + h(i))];
    w_in_nav = w_ie_nav + w_en_nav;
    w_nb_body = w_clean - Cnb * w_in_nav; % Tốc độ góc thân tàu tương đối so với khung Nav
    
    % Tích phân vận tốc và vị trí danh định (có bù lực Coriolis)
    a_nav = Cbn * f_clean + gn - cross(2*w_ie_nav + w_en_nav, v_est);
    v_est = v_est + a_nav * dt;
    pos_est = pos_est + v_est * dt;
    
    % Tích phân Quaternion thái độ tương đối khung Nav
    dth_step = w_nb_body * dt;
    dq_step = [cos(norm(dth_step)/2); sin(norm(dth_step)/2) * dth_step / max(norm(dth_step), 1e-12)];
    q_est = quat_nhan(q_est, dq_step);
    q_est = q_est / norm(q_est);
    
    % Lan truyền hiệp phương sai P (19x19)
    F_x = eye(19);
    F_x(1:3, 4:6)   = eye(3) * dt;
    F_x(4:6, 7:9)   = - Cbn * skew_mat(f_clean) * dt;
    F_x(4:6, 10:12) = - Cbn * dt;
    F_x(7:9, 7:9)   = eye(3) - skew_mat(w_nb_body) * dt;
    F_x(7:9, 13:15) = - eye(3) * dt;
    
    P = F_x * P * F_x' + Q_d;
    
    % --- [B] CHU KỲ CHẬM 5 Hz: CẬP NHẬT ĐO LƯỜNG ĐA CẢM BIẾN ---
    if meas_ptr <= n_meas && idx_meas(meas_ptr) == i
        ti = t(i);
        
        % 1. CẬP NHẬT GNSS 2 ANTEN (HOẠT ĐỘNG TRONG PHASE 1: ti <= T_phase1)
        if ti <= T_phase1
            % 1.1. Đo vị trí và vận tốc GNSS (6x1)
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
            
            % 1.2. Đo hướng GNSS 2 anten (Dual-Antenna Heading - 1x1)
            Cbn_cur = quat_to_Cbn(q_est);
            yaw_est = atan2(Cbn_cur(2, 1), Cbn_cur(1, 1));
            res_yaw = atan2(sin(Z_gps_yaw(meas_ptr) - yaw_est), cos(Z_gps_yaw(meas_ptr) - yaw_est));
            
            H_yaw = zeros(1, 19);
            H_yaw(1, 9) = 1.0; % Cột 9: delta_theta_z
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
        
        % 2. CẬP NHẬT DVL KHÉP KÍN (CHẠY KHI TÀU CHUYỂN ĐỘNG: CẢ Ở PHASE 1 VÀ PHASE 2!)
        Cbn_cur = quat_to_Cbn(q_est);
        Cnb_cur = Cbn_cur';
        v_b = Cnb_cur * v_est;
        v_L = v_b + cross(w_clean, lever_arm_est);
        
        % Chỉ cập nhật DVL khi có vận tốc chuyển động (tránh nhiễu khi đứng yên)
        if norm(v_L) > 0.1
            z_m = Z_dvl(meas_ptr, :)';
            
            C_bs_est = eye(3) - skew_mat(phi_bs_est);
            h_val = (1 + s_est) * C_bs_est * v_L;
            residual = z_m - h_val;
            
            H = zeros(3, 19);
            if ti <= T_phase1
                % GIAI ĐOẠN 1 (MẶT NƯỚC CÓ GNSS 2 ANTEN): CHẾ ĐỘ HIỆU CHUẨN TRỰC TUYẾN DVL
                % GNSS đã chốt chuẩn Bắc 0.05 deg và vận tốc RTK 2 cm/s cực chuẩn.
                % DVL chỉ tập trung hiệu chuẩn: Scale s (16), Pitch beta (18), Yaw gamma (19).
                % Khóa toàn bộ H(:, 1:15) = 0 để tách biệt toán học, triệt tiêu nhiễu chéo vào GNSS!
                H(:, 16)    = C_bs_est * v_L;
                H(:, 17:19) = (1 + s_est) * skew_mat(v_L);
                H(:, 17)    = 0.0; % Khóa Roll boresight
            else
                % GIAI ĐOẠN 2 (LẶN NGẦM CẮT GNSS): CHẾ ĐỘ DẪN ĐƯỜNG KẾT HỢP DVL + INS
                % DVL hỗ trợ vận tốc (4:6), góc nghiêng pitch/roll (7:8), lever arm (13:15)
                % và tiếp tục ổn định các tham số (16, 18:19).
                % KHÓA CỘT 9 (YAW INS) ĐỂ DVL KHÔNG VẶN SAI HƯỚNG BẮC CỦA GYRO FOG!
                H(:, 4:6)   = (1 + s_est) * C_bs_est * Cnb_cur;
                H(:, 7:9)   = (1 + s_est) * C_bs_est * skew_mat(v_b);
                H(:, 9)     = 0.0; % Luôn khóa cột Yaw INS
                H(:, 13:15) = (1 + s_est) * C_bs_est * skew_mat(lever_arm_est);
                H(:, 16)    = C_bs_est * v_L;
                H(:, 17:19) = (1 + s_est) * skew_mat(v_L);
                H(:, 17)    = 0.0; % Khóa Roll boresight
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
                phi_bs_est(2) = phi_bs_est(2) + dx(18); % Pitch beta
                phi_bs_est(3) = phi_bs_est(3) + dx(19); % Yaw gamma
                phi_bs_est(1) = 0.0;
                
                G = eye(19);
                G(7:9, 7:9) = eye(3) - 0.5 * skew_mat(dth);
                P = G * P_joseph * G';
                P = 0.5 * (P + P');
            end
        end
        meas_ptr = meas_ptr + 1;
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

% Đánh giá sai số vị trí thực tế so với Ground Truth trong Phase 2 (Lặn ngầm không GNSS)
t_axis  = t - T_phase1;
mask_p1 = t <= T_phase1;
mask_p2 = t > T_phase1;
mask    = mask_p2;

ERR_POS = sqrt(sum((POS_REC(mask_p2, 1:2) - POS_TRUE(mask_p2, 1:2)).^2, 2));
pos_err_max = max(ERR_POS);
pos_err_rms = rms(ERR_POS);
pos_err_end = ERR_POS(end);
total_dist  = sum(sqrt(sum(v_n(mask_p2, 1:2).^2, 2))) * dt;
drift_rate  = (pos_err_end / max(total_dist, 1e-3)) * 100;

fprintf('\n================ ĐÁNH GIÁ ĐỘ CHÍNH XÁC DẪN ĐƯỜNG LẶN NGẦM (PHASE 2) ================\n');
fprintf('Thời gian lặn ngầm không GPS : %10.1f giây (10 phút cơ động số 8)\n', T_run);
fprintf('Tổng quãng đường lặn ngầm    : %10.1f mét\n', total_dist);
fprintf('Sai số vị trí cực đại        : %10.3f mét\n', pos_err_max);
fprintf('Sai số vị trí trung phương   : %10.3f mét (RMS)\n', pos_err_rms);
fprintf('Sai số vị trí điểm cuối      : %10.3f mét\n', pos_err_end);
fprintf('Tỉ lệ trôi dạt tích lũy      : %10.4f %% quãng đường (Chuẩn hàng hải < 0.2%%)\n', drift_rate);
fprintf('====================================================================================\n');

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
figure('Name', 'QUÁ TRÌNH HỘI TỤ THAM SỐ DVL (ES-EKF 2 PHA)', 'Color', 'w', 'Position', [100 100 1200 650]);

% 1. Đồ thị Quỹ đạo UUV: Đường Thật vs Ước lượng EKF (2 Pha)
subplot(2, 2, 1);
plot(POS_TRUE(:, 2), POS_TRUE(:, 1), 'k--', 'LineWidth', 1.6); hold on; grid on; axis equal;
plot(POS_REC(mask_p1, 2), POS_REC(mask_p1, 1), 'Color', [1.0 0.55 0.0], 'LineWidth', 2.2); % Phase 1: Mặt nước GNSS
plot(POS_REC(mask_p2, 2), POS_REC(mask_p2, 1), 'b-', 'LineWidth', 1.8);                      % Phase 2: Lặn ngầm EKF

% Đánh dấu các mốc quan trọng
plot(POS_REC(1, 2), POS_REC(1, 1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7); % Xuất phát
idx_sub = find(mask_p2, 1);
plot(POS_REC(idx_sub, 2), POS_REC(idx_sub, 1), 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 8); % Điểm lặn cắt GPS

idx_end = N;
plot(POS_TRUE(idx_end, 2), POS_TRUE(idx_end, 1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
plot(POS_REC(idx_end, 2), POS_REC(idx_end, 1), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 7);

plot([POS_TRUE(idx_end, 2), POS_REC(idx_end, 2)], ...
     [POS_TRUE(idx_end, 1), POS_REC(idx_end, 1)], 'r--', 'LineWidth', 2);

str_drift = {
    '\bfĐỘ DẠT LẶN NGẦM (PHASE 2):\rm', ...
    sprintf('• Quãng đường lặn: %.0f m', total_dist), ...
    sprintf('• Sai số cực đại: %.2f m', pos_err_max), ...
    sprintf('• Sai số RMS: %.2f m', pos_err_rms), ...
    sprintf('• Dạt điểm cuối: %.2f m', pos_err_end), ...
    sprintf('• Tỉ lệ dạt: %.3f%% (< 0.2%%)', drift_rate)
};
text(0.03, 0.22, str_drift, 'Units', 'normalized', 'FontSize', 8.5, ...
    'BackgroundColor', [1 1 1 0.90], 'EdgeColor', [0.6 0.6 0.6], 'Margin', 5);

title('Quỹ đạo 2 Pha: Mặt nước (GNSS) \rightarrow Lặn ngầm (DVL)');
xlabel('Đông (m)'); ylabel('Bắc (m)');
legend('Thực tế (Ground Truth)', 'Phase 1: Mặt nước GNSS', 'Phase 2: Lặn ngầm EKF', ...
       'Xuất phát', 'Điểm lặn (Cắt GPS)', 'Đích Thật', 'Đích EKF', 'Vector dạt cuối', 'Location', 'northeast');

% 2. Hội tụ Scale factor s (%)
subplot(2, 2, 2);
plot(t, S_REC, 'b-', 'LineWidth', 1.8); hold on; grid on;
yline(s_that_pct, 'r--', 'LineWidth', 1.5);
xline(T_phase1, 'k--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Quá trình Hội tụ Hệ số Tỷ lệ s (%) Toàn Nhiệm vụ');
xlabel('Thời gian nhiệm vụ (s)'); ylabel('s (%)');
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0%)', 'Mốc cắt GNSS lặn', 'Location', 'best');

% 3. Hội tụ Pitch Boresight beta (độ)
subplot(2, 2, 3);
plot(t, BETA_REC, 'g-', 'LineWidth', 1.8); hold on; grid on;
yline(beta_that_deg, 'r--', 'LineWidth', 1.5);
xline(T_phase1, 'k--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Quá trình Hội tụ Pitch Boresight \beta (độ) Toàn Nhiệm vụ');
xlabel('Thời gian nhiệm vụ (s)'); ylabel('\beta (độ)');
legend('Ước lượng ES-EKF', 'Giá trị thật (-0.3 độ)', 'Mốc cắt GNSS lặn', 'Location', 'best');

% 4. Hội tụ Yaw Boresight gamma (độ)
subplot(2, 2, 4);
plot(t, GAMMA_REC, 'm-', 'LineWidth', 1.8); hold on; grid on;
yline(gamma_that_deg, 'r--', 'LineWidth', 1.5);
xline(T_phase1, 'k--', 'Lặn ngầm (Cắt GNSS)', 'LineWidth', 1.3, 'LabelOrientation', 'aligned');
title('Quá trình Hội tụ Yaw Boresight \gamma (độ) Toàn Nhiệm vụ');
xlabel('Thời gian nhiệm vụ (s)'); ylabel('\gamma (độ)');
legend('Ước lượng ES-EKF', 'Giá trị thật (+1.0 độ)', 'Mốc cắt GNSS lặn', 'Location', 'best');

sgtitle('KẾT QUẢ HIỆU CHUẨN DVL VÀ DẪN ĐƯỜNG 2 PHA (PHASE 1 GNSS \rightarrow PHASE 2 DVL)', 'FontSize', 14, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% HÌNH 2: PHÂN TÍCH CHI TIẾT ĐỘ DẠT VỊ TRÍ THEO THỜI GIAN LẶN NGẦM
% -------------------------------------------------------------------------
figure('Name', 'PHÂN TÍCH CHI TIẾT ĐỘ DẠT QUỸ ĐẠO LẶN NGẦM THEO THỜI GIAN', 'Color', 'w', 'Position', [150 150 1100 550]);

subplot(2, 1, 1);
plot(t_axis(mask_p2), ERR_POS, 'r-', 'LineWidth', 1.6); hold on; grid on;
yline(pos_err_rms, 'b--', sprintf('RMS: %.2f m', pos_err_rms), 'LineWidth', 1.5);
yline(pos_err_max, 'k:', sprintf('Cực đại: %.2f m', pos_err_max), 'LineWidth', 1.5);
title('Độ dạt vị trí tức thời ||p_{EKF} - p_{TRUE}|| trong giai đoạn lặn ngầm (m)');
xlabel('Thời gian lặn ngầm (s)'); ylabel('Độ dạt (m)');
legend('Sai số tức thời', 'Đường chuẩn RMS', 'Đường cực đại Max', 'Location', 'best');

subplot(2, 1, 2);
err_N = POS_REC(mask_p2, 1) - POS_TRUE(mask_p2, 1);
err_E = POS_REC(mask_p2, 2) - POS_TRUE(mask_p2, 2);
plot(t_axis(mask_p2), err_N, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_axis(mask_p2), err_E, 'm-', 'LineWidth', 1.5);
yline(0, 'k--');
title('Phân tích Độ dạt từng trục khi lặn: Trục Bắc (\Delta North) và Trục Đông (\Delta East)');
xlabel('Thời gian lặn ngầm (s)'); ylabel('Độ dạt (m)');
legend('Độ dạt trục Bắc (\Delta N)', 'Độ dạt trục Đông (\Delta E)', 'Location', 'best');

sgtitle(sprintf('BIỂU ĐỒ ĐỘ DẠT LẶN NGẦM (Quãng đường: %.0f m | Dạt cuối: %.2f m = %.3f%%)', ...
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
