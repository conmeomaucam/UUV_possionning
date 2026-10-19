function test2()
%% SCRIPT KIỂM ĐỊNH MONTE CARLO ES-EKF 19 TRẠNG THÁI (ĐÃ SỬA LỖI ĐỒNG BỘ)
clc; close all;

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'dieukiendauvao'));
addpath(fullfile(root_dir, 'Sensor'));
addpath(fullfile(root_dir, 'quydaochuyendong'));

%% ========================================================================
% 1. THIẾT LẬP THAM SỐ MÔ PHỎNG MONTE CARLO VÀ CHẾ ĐỘ KIỂM ĐỊNH
% =========================================================================
% LỰA CHỌN CHẾ ĐỘ KIỂM ĐỊNH:
%   'CALIBRATION' - THÍ NGHIỆM B: Kiểm định khả năng tự hiệu chuẩn từ 0 (Cold Start)
%                   Scale_est = 0, Boresight_est = [0;0;0], P0 phản ánh độ bất định trước nghiệm
%   'CONSISTENCY' - THÍ NGHIỆM A: Kiểm định tính nhất quán Kalman
%                   Khởi tạo x0 ~ N(x_true, P0) để kiểm tra ANEES và độ bao phủ ma trận P
test_mode = 'CALIBRATION';      % Tùy chọn: 'CALIBRATION' hoặc 'CONSISTENCY'

M_runs = 300;                 % Số lần chạy Monte Carlo (x  kịch bản ngẫu nhiên)
dt = 0.0025;                  % Chu kỳ trích mẫu IMU chuẩn phần cứng FOG (400 Hz)
T_total = 30.0;               % Tổng thời gian mô phỏng (30 giây)
T_gnss_cut = 15.0;            % Thời điểm cắt GNSS (bắt đầu lặn ngầm từ giây 15)
N_steps = round(T_total / dt);
N_points = N_steps + 1;
time_vec = (0:N_steps)' * dt;      % Mốc thời gian trạng thái [t0, t1, ..., tN] (12.001 mốc)
time_eval = time_vec(2:end);       % Mốc thời gian đánh giá sau mỗi bước lọc [t1, ..., tN] (12.000 mốc)

% Cấu hình vật lý và tham số bộ lọc ES-EKF 19 trạng thái
gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0;
radius_m = 6356752.3;
lat_rad = deg2rad(20.8);
tan_lat = tan(lat_rad);
height = 0;

lever_arm = [0.5; -0.1; 0.2]; % Cánh tay đòn lệch tâm IMU-DVL thực tế (m)

% Đặc tính nhiễu chuẩn của cảm biến
sigma_vrw = 0.02;             % VRW gia tốc kế (m/s/sqrt(s))
sigma_arw = 0.001;            % ARW con quay (rad/sqrt(s))
acc_bias_sigma = 0.005;       % Độ ổn định bias in-run gia tốc kế (m/s^2)
acc_bias_tau = 300;           % Thời gian tương quan Gauss-Markov (s)
gyro_bias_rw = 1e-5;          % Random walk bias con quay (rad/s/sqrt(s))
scale_rw = 1e-5;              % Random walk scale factor DVL (1/sqrt(s))
boresight_rw = 1e-5;          % Random walk góc Boresight (rad/sqrt(s))

% Nhiễu đo đạc cảm biến ngoài
sigma_gps_pos = 0.02;         % Độ chính xác vị trí GNSS RTK (m)
sigma_gps_vel = 0.02;         % Độ chính xác vận tốc GNSS (m/s)
sigma_gps_yaw = deg2rad(0.05);% Độ chính xác góc hướng 2 ăng-ten GNSS (rad)
sigma_depth = 0.05;           % Độ chính xác cảm biến áp suất độ sâu (m)
dvl_noise_floor = 0.01;       % Nhiễu sàn DVL A125 (m/s)
dvl_noise_slope = 0.005;      % Nhiễu theo tỉ lệ vận tốc DVL

% Vector cấu hình cfg (20x1) truyền cho esekf19_step
cfg_mc = zeros(20, 1);
cfg_mc(1) = dt;
cfg_mc(2) = gravity;
cfg_mc(3:5) = w_ie_ned;
cfg_mc(6) = radius_n;
cfg_mc(7) = radius_m;
cfg_mc(8) = tan_lat;
cfg_mc(9) = height;
cfg_mc(10:12) = lever_arm;
cfg_mc(13) = dvl_noise_floor;
cfg_mc(14) = dvl_noise_slope;
cfg_mc(15) = sigma_gps_pos;
cfg_mc(16) = sigma_gps_vel;
cfg_mc(17) = sigma_gps_yaw;
cfg_mc(18) = sigma_depth;
cfg_mc(19) = acc_bias_tau;
cfg_mc(20) = sigma_arw; % Thí nghiệm E1: Bù nhiễu trắng con quay truyền qua cánh tay đòn

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
    gyro_bias_rw; scale_rw; boresight_rw; dt];

%% ========================================================================
% 2. SINH QUỸ ĐẠO DANH ĐỊNH CHUẨN 3D (GROUND TRUTH) CÓ KÍCH THÍCH 3 TRỤC
%    Đồng bộ chuẩn N+1 trạng thái (t0 đến tN) và N khoảng đo IMU
% =========================================================================
true_scale = 0.015;                          % Scale factor thực tế = 1.5%
true_boresight = deg2rad([1.5; -2.0; 3.0]);  % Boresight thực [Roll; Pitch; Yaw]
Cbs_true = rodrigues_so3(-true_boresight);

pos_true = zeros(N_points, 3);
vel_true_ned = zeros(N_points, 3);
vel_true_body = zeros(N_points, 3);
quat_true = zeros(N_points, 4);
accel_true_body = zeros(N_steps, 3);
gyro_true_body = zeros(N_steps, 3);

% Trạng thái ban đầu tại t0 = 0 (Chỉ số k = 1)
p_curr = [0; 0; 0];
q_curr = eul2quat_custom([0; 0; deg2rad(30)]);
Cbn_0 = quat_to_cbn(q_curr);
u_0 = 1.6 + 0.3*sin(0);
v_0 = 0.3*sin(0);
w_0 = 0.15*cos(0);
v_b_0 = [u_0; v_0; w_0];
v_ned_0 = Cbn_0 * v_b_0;

pos_true(1, :)      = p_curr';
vel_true_ned(1, :)  = v_ned_0';
vel_true_body(1, :) = v_b_0';
quat_true(1, :)     = q_curr';

for k = 1:N_steps
    t_start = time_vec(k);
    t_end   = time_vec(k+1);
    t_mid   = 0.5 * (t_start + t_end);

    % 1. Vận tốc góc khung thân tại điểm giữa khoảng tích phân t_mid
    p_rate_mid = deg2rad(6)*sin(0.5*t_mid);
    q_rate_mid = deg2rad(4)*cos(0.4*t_mid);
    r_rate_mid = deg2rad(3)*sin(0.15*t_mid);
    w_nb_b_mid = [p_rate_mid; q_rate_mid; r_rate_mid];

    % Cập nhật tư thế từ t_start đến t_end
    delta_theta_true = w_nb_b_mid * dt;
    ang_norm = norm(delta_theta_true);
    dq = [cos(ang_norm/2); sin(ang_norm/2) * delta_theta_true / max(ang_norm, 1e-12)];
    q_next = quat_multiply(q_curr, dq);
    q_next = q_next / norm(q_next);

    % Tư thế tại thời điểm t_end (Chỉ số k+1)
    Cbn_next = quat_to_cbn(q_next);

    % Vận tốc khung thân tại thời điểm t_end
    u_end = 1.6 + 0.3*sin(0.2*t_end);
    v_end = 0.3*sin(0.35*t_end);
    w_end = 0.15*cos(0.25*t_end);
    v_b_next = [u_end; v_end; w_end];
    v_ned_next = Cbn_next * v_b_next;

    % Tích phân vị trí hình thang từ t_start đến t_end (Đạt độ chính xác bậc 2)
    v_ned_prev = vel_true_ned(k, :)';
    p_next = p_curr + 0.5 * (v_ned_prev + v_ned_next) * dt;

    % 2. Đại lượng đo IMU đại diện cho khoảng tích phân [t_k, t_{k+1}] (Tính tại t_mid)
    dq_half = [cos(ang_norm/4); sin(ang_norm/4) * delta_theta_true / max(ang_norm, 1e-12)];
    q_mid = quat_multiply(q_curr, dq_half);
    q_mid = q_mid / norm(q_mid);
    Cbn_mid = quat_to_cbn(q_mid);
    Cnb_mid = Cbn_mid';

    u_mid = 1.6 + 0.3*sin(0.2*t_mid);
    v_mid = 0.3*sin(0.35*t_mid);
    w_mid = 0.15*cos(0.25*t_mid);
    v_b_mid = [u_mid; v_mid; w_mid];
    v_ned_mid = Cbn_mid * v_b_mid;

    % Tính Transport rate w_en_ned tại t_mid
    w_en_mid = [v_ned_mid(2)/(radius_n + height); ...
        -v_ned_mid(1)/(radius_m + height); ...
        -v_ned_mid(2)*tan_lat/(radius_n + height)];

    % Vận tốc góc tuyệt đối con quay đo trong khoảng [t_k, t_{k+1}]
    w_ib_b_mid = w_nb_b_mid + Cnb_mid * (w_ie_ned + w_en_mid);

    % Lực riêng đo bởi gia tốc kế trong khoảng [t_k, t_{k+1}]
    acc_lin_b_mid = [0.3*0.2*cos(0.2*t_mid); 0.3*0.35*cos(0.35*t_mid); -0.15*0.25*sin(0.25*t_mid)];
    coriolis_ned_mid = cross(2*w_ie_ned + w_en_mid, v_ned_mid);
    f_b_mid = acc_lin_b_mid + cross(w_nb_b_mid, v_b_mid) + Cnb_mid * (coriolis_ned_mid - [0; 0; gravity]);

    % Lưu trạng thái chuẩn tại mốc t_{k+1} (Chỉ số k+1)
    pos_true(k+1, :)      = p_next';
    vel_true_ned(k+1, :)  = v_ned_next';
    vel_true_body(k+1, :) = v_b_next';
    quat_true(k+1, :)     = q_next';

    % Lưu số đo IMU đại diện cho khoảng tích phân k
    accel_true_body(k, :) = f_b_mid';
    gyro_true_body(k, :)  = w_ib_b_mid';

    % Chuyển tiếp trạng thái cho bước lặp sau
    p_curr = p_next;
    q_curr = q_next;
end

%% ========================================================================
% 3. THIẾT LẬP MA TRẬN HIỆP PHƯƠNG SAI BAN ĐẦU P0 VÀ Q
% =========================================================================
[~, P0, Q] = esekf19_init([0; 0; deg2rad(30)], gravity, noise_cfg);
P0(1:3, 1:3)     = (0.05)^2 * eye(3);
P0(4:6, 4:6)     = (0.05)^2 * eye(3);
P0(7:9, 7:9)     = deg2rad(1.0)^2 * eye(3);
P0(10:12, 10:12) = (0.01)^2 * eye(3);
P0(13:15, 13:15) = deg2rad(0.1/3600)^2 * eye(3);

if strcmp(test_mode, 'CALIBRATION')
    % THÍ NGHIỆM B: P0 phản ánh độ bất định cơ khí ban đầu khi hoàn toàn chưa biết thông số
    P0(16, 16)       = (0.03)^2;                % Bất định scale ban đầu 3%
    P0(17:19, 17:19) = deg2rad(4.0)^2 * eye(3); % Bất định góc Boresight ban đầu 4.0 deg
else
    % THÍ NGHIỆM A: P0 phản ánh đúng hiệp phương sai của phân bố delta_x0
    P0(16, 16)       = (0.02)^2;
    P0(17:19, 17:19) = deg2rad(2.0)^2 * eye(3);
end

sqrt_P0 = chol(P0, 'lower');

%% ========================================================================
% 4. THỰC THI VÒNG LẶP MONTE CARLO
% =========================================================================
% Bộ tích lũy thống kê trực tuyến (tiết kiệm RAM tối đa khi chạy 2.000 chu kỳ)
sum_sq_err    = zeros(N_steps, 19);
sum_nees      = zeros(N_steps, 1);
sum_nees_pos  = zeros(N_steps, 1);
sum_nees_vel  = zeros(N_steps, 1);
sum_nees_att  = zeros(N_steps, 1);
sum_nees_bias = zeros(N_steps, 1);
sum_nees_cal  = zeros(N_steps, 1);
sum_cov_diag  = zeros(N_steps, 19);
sum_scale_est = zeros(N_steps, 1);
sum_bs_est    = zeros(N_steps, 3);

% Bộ đếm kiểm định chất lượng cảm biến (Cổng Chi-square & NIS)
total_meas_count    = zeros(3, 1); % [GNSS; Depth; DVL]
accepted_meas_count = zeros(3, 1);
sum_nis             = zeros(3, 1);

chk_idx = round([1 5 10 15 16 20 25 30]/dt);
ep_chk = zeros(M_runs,8); ev_chk = ep_chk; ea_chk = ep_chk; sp_chk = ep_chk;
rej_dvl = zeros(M_runs,1);

rng(1, 'twister'); % Đảm bảo tính lặp lại và so sánh công bằng giữa các phép thử
fprintf('Đang thực thi %d kịch bản Monte Carlo [Chế độ: %s]...\n', M_runs, test_mode);
t_start = tic;

for m = 1:M_runs
    if mod(m, 50) == 0 || m == 1 || m == 10
        elapsed_s = toc(t_start);
        eta_s = (elapsed_s / m) * (M_runs - m);
        fprintf('  -> Tiến độ Monte Carlo: %5d / %d (%5.1f%%) | Đã chạy: %4.1fs | Còn lại: ~%4.1fs\n', ...
            m, M_runs, m/M_runs*100, elapsed_s, eta_s);
    end

    delta_x0 = sqrt_P0 * randn(19, 1);

    % Khởi tạo bias thực tế
    ba_true_k = [0.008; -0.012; 0.006];
    bg_true_k = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];

    % Khởi tạo trạng thái ước lượng dẫn đường (+ delta_x0)
    pos_est = pos_true(1, :)' + delta_x0(1:3);
    vel_est = vel_true_ned(1, :)' + delta_x0(4:6);

    dq0 = [1; 0.5 * delta_x0(7:9)];
    dq0 = dq0 / norm(dq0);
    quat_est = quat_multiply(quat_true(1, :)', dq0);
    quat_est = quat_est / norm(quat_est);

    ba_est    = ba_true_k + delta_x0(10:12);
    bg_est    = bg_true_k + delta_x0(13:15);

    if strcmp(test_mode, 'CALIBRATION')
        % THÍ NGHIỆM B: Khởi tạo từ trạng thái hoàn toàn chưa biết (Scale = 0, Boresight = [0;0;0])
        scale_est = 0;
        Cbs_est   = eye(3);
        bs_est    = [0; 0; 0];
    else
        % THÍ NGHIỆM A: Khởi tạo quanh giá trị thật (Kiểm định nhất quán Kalman)
        scale_est = true_scale + delta_x0(16);
        Cbs_est   = Cbs_true * rodrigues_so3(-delta_x0(17:19));
        bs_est    = -so3_log(Cbs_est);
    end

    state_est = zeros(20, 1);
    state_est(1:3)   = pos_est;
    state_est(4:6)   = vel_est;
    state_est(7:10)  = quat_est;
    state_est(11:13) = ba_est;
    state_est(14:16) = bg_est;
    state_est(17)    = scale_est;
    state_est(18:20) = bs_est;

    P_est = P0;

    phi_acc = exp(-dt / acc_bias_tau);
    q_acc_rw = acc_bias_sigma * sqrt(2 * dt / acc_bias_tau);
    q_gyro_rw = gyro_bias_rw * sqrt(dt);

    for k = 1:N_steps
        idx_target = k + 1;
        t_target = time_vec(idx_target);

        ba_true_k = phi_acc * ba_true_k + q_acc_rw * randn(3, 1);
        bg_true_k = bg_true_k + q_gyro_rw * randn(3, 1);

        f_meas = accel_true_body(k, :)' + ba_true_k + (sigma_vrw / sqrt(dt)) * randn(3, 1);
        w_meas = gyro_true_body(k, :)' + bg_true_k + (sigma_arw / sqrt(dt)) * randn(3, 1);

        meas_flags = zeros(3, 1);
        gnss_meas = zeros(7, 1);
        depth_meas = 0;
        dvl_meas = zeros(3, 1);

        % Cập nhật đo đạc ngoài chuẩn 5 Hz (400 Hz / 5 Hz = 80 bước)
        is_meas_time = (mod(k, 80) == 0);

        if is_meas_time
            meas_flags(2) = 1;
            depth_meas = pos_true(idx_target, 3) + sigma_depth * randn;

            if t_target <= T_gnss_cut
                meas_flags(1) = 1;
                gnss_pos = pos_true(idx_target, :)' + sigma_gps_pos * randn(3, 1);
                gnss_vel = vel_true_ned(idx_target, :)' + sigma_gps_vel * randn(3, 1);
                Cbn_target = quat_to_cbn(quat_true(idx_target, :)');
                true_yaw = atan2(Cbn_target(2, 1), Cbn_target(1, 1));
                gnss_yaw = true_yaw + sigma_gps_yaw * randn;
                gnss_meas = [gnss_pos; gnss_vel; gnss_yaw];
            end

            meas_flags(3) = 1;
            Cbn_target = quat_to_cbn(quat_true(idx_target, :)');
            v_ned_target = vel_true_ned(idx_target, :)';
            w_en_target = [v_ned_target(2)/(radius_n + height); ...
                -v_ned_target(1)/(radius_m + height); ...
                -v_ned_target(2)*tan_lat/(radius_n + height)];
            w_in_b_target = Cbn_target' * (w_ie_ned + w_en_target);

            % Vận tốc góc khung thân thực tế tại mốc thời gian cập nhật t_target
            p_rate_target = deg2rad(6)*sin(0.5*t_target);
            q_rate_target = deg2rad(4)*cos(0.4*t_target);
            r_rate_target = deg2rad(3)*sin(0.15*t_target);
            w_nb_b_target = [p_rate_target; q_rate_target; r_rate_target];

            v_dvl_true = vel_true_body(idx_target, :)' + cross(w_nb_b_target, lever_arm);
            sig_dvl = dvl_noise_floor + dvl_noise_slope * norm(v_dvl_true);
            dvl_meas = (1 + true_scale) * Cbs_true * v_dvl_true + sig_dvl * randn(3, 1);
        end

        [state_est, P_est, update_status, nis] = esekf19_step( ...
            state_est, P_est, Q, w_meas, f_meas, ...
            gnss_meas, depth_meas, dvl_meas, meas_flags, cfg_mc);

        % Thu thập thống kê cổng lọc Chi-square và NIS cho từng cảm biến
        for s_idx = 1:3
            if meas_flags(s_idx) ~= 0
                total_meas_count(s_idx)    = total_meas_count(s_idx) + 1;
                accepted_meas_count(s_idx) = accepted_meas_count(s_idx) + update_status(s_idx);
                sum_nis(s_idx)             = sum_nis(s_idx) + nis(s_idx);
            end
        end

        % Tính toán vector sai số thực e_k đối chiếu với trạng thái chuẩn tại t_{k+1} (chỉ số k+1)
        err_p = state_est(1:3) - pos_true(idx_target, :)';
        err_v = state_est(4:6) - vel_true_ned(idx_target, :)';

        % Sai số góc tư thế chuẩn trong khung Body (Nhân nghịch đảo bên trái)
        q_est = state_est(7:10);
        q_tr  = quat_true(idx_target, :)';
        q_inv_tr = [q_tr(1); -q_tr(2:4)];
        q_diff = quat_multiply(q_inv_tr, q_est);
        if q_diff(1) < 0, q_diff = -q_diff; end
        err_att = 2 * q_diff(2:4);

        err_ba = state_est(11:13) - ba_true_k;
        err_bg = state_est(14:16) - bg_true_k;
        err_scale = state_est(17) - true_scale;

        % Sai số Boresight chuẩn trên SO(3)
        Cbs_cur = rodrigues_so3(-state_est(18:20));
        Cbs_err = Cbs_true' * Cbs_cur;
        err_bs = -so3_log(Cbs_err);

        e_k = [err_p; err_v; err_att; err_ba; err_bg; err_scale; err_bs];

        ci = find(k == chk_idx, 1);
        if ~isempty(ci)
            ep_chk(m,ci) = norm(err_p); ev_chk(m,ci) = norm(err_v);
            ea_chk(m,ci) = rad2deg(norm(err_att)); sp_chk(m,ci) = sqrt(trace(P_est(1:3,1:3)));
        end
        if meas_flags(3) && update_status(3) == 0, rej_dvl(m) = rej_dvl(m) + 1; end

        nees_k = e_k' * (P_est \ e_k);

        % Phân rã NEES theo 5 nhóm trạng thái độc lập
        nees_p_k    = err_p' * (P_est(1:3, 1:3) \ err_p);
        nees_v_k    = err_v' * (P_est(4:6, 4:6) \ err_v);
        nees_att_k  = err_att' * (P_est(7:9, 7:9) \ err_att);
        e_bias_k    = [err_ba; err_bg];
        nees_bias_k = e_bias_k' * (P_est(10:15, 10:15) \ e_bias_k);
        e_cal_k     = [err_scale; err_bs];
        nees_cal_k  = e_cal_k' * (P_est(16:19, 16:19) \ e_cal_k);

        % Tích lũy trực tuyến cho 1.000 chu kỳ
        sum_sq_err(k, :)   = sum_sq_err(k, :) + (e_k.^2)';
        sum_nees(k)        = sum_nees(k) + nees_k;
        sum_nees_pos(k)    = sum_nees_pos(k) + nees_p_k;
        sum_nees_vel(k)    = sum_nees_vel(k) + nees_v_k;
        sum_nees_att(k)    = sum_nees_att(k) + nees_att_k;
        sum_nees_bias(k)   = sum_nees_bias(k) + nees_bias_k;
        sum_nees_cal(k)    = sum_nees_cal(k) + nees_cal_k;
        sum_cov_diag(k, :) = sum_cov_diag(k, :) + diag(P_est)';
        sum_scale_est(k)   = sum_scale_est(k) + state_est(17);
        sum_bs_est(k, :)   = sum_bs_est(k, :) + state_est(18:20)';
    end
end

sim_duration = toc(t_start);
fprintf('Hoàn thành xong trong %.2f giây (%.2f phút)!\n', sim_duration, sim_duration/60);

save('diag_run.mat','ep_chk','ev_chk','ea_chk','sp_chk','rej_dvl');
pf = @(n,x) fprintf('%-12s %s\n', n, sprintf('%9.4f', x));
pf('RMS pos', sqrt(mean(ep_chk.^2))); pf('median pos', median(ep_chk));
pf('max pos', max(ep_chk)); pf('sigma_p', sqrt(mean(sp_chk.^2)));
pf('RMS vel', sqrt(mean(ev_chk.^2))); pf('RMS att deg', sqrt(mean(ea_chk.^2)));
fprintf('DVL rejects/run: mean %.1f, max %d\n', mean(rej_dvl), max(rej_dvl));

%% ========================================================================
% 5. TÍNH TOÁN VÀ IN KẾT QUẢ THỐNG KÊ ANEES, NIS & ARMSE
% =========================================================================
anees          = (sum_nees / M_runs)';
anees_pos      = (sum_nees_pos / M_runs)';
anees_vel      = (sum_nees_vel / M_runs)';
anees_att      = (sum_nees_att / M_runs)';
anees_bias     = (sum_nees_bias / M_runs)';
anees_cal      = (sum_nees_cal / M_runs)';
armse          = sqrt(sum_sq_err / M_runs);
mean_sigma     = sqrt(sum_cov_diag / M_runs);
mean_scale_est = sum_scale_est / M_runs;
mean_bs_est    = sum_bs_est / M_runs;

% Phân tách đánh giá theo pha: Pha 1 (0-15s có GNSS) và Pha 2 (15-30s mất GNSS)
idx_phase1 = (time_eval <= T_gnss_cut);
idx_phase2 = (time_eval > T_gnss_cut);
anees_phase1 = mean(anees(idx_phase1));
anees_phase2 = mean(anees(idx_phase2));

% Tỷ lệ chấp nhận phép đo và NIS trung bình
gate_pass_rate = (accepted_meas_count ./ max(total_meas_count, 1)) * 100;
mean_nis       = sum_nis ./ max(total_meas_count, 1);

dof = M_runs * 19;
r1 = chi2inv_custom(0.025, dof) / M_runs;
r2 = chi2inv_custom(0.975, dof) / M_runs;

pct_anees_pass = mean(anees >= r1 & anees <= r2) * 100;
mean_anees_val = mean(anees);

fprintf('\n================== KẾT QUẢ KIỂM ĐỊNH MONTE CARLO ==================\n');
fprintf('Chế độ kiểm định                  : [%s]\n', test_mode);
if strcmp(test_mode, 'CALIBRATION')
    fprintf('Mục tiêu kiểm định                : Khả năng tự hiệu chuẩn từ 0 (Cold Start)\n');
    fprintf('Khởi tạo Scale & Boresight        : scale_est = 0%%, boresight_est = [0, 0, 0] deg\n');
else
    fprintf('Mục tiêu kiểm định                : Tính nhất quán Kalman (Consistency Test)\n');
    fprintf('Khởi tạo Scale & Boresight        : Phân bố chuẩn quanh giá trị thật (+ delta_x0)\n');
end
fprintf('Số chu kỳ mô phỏng (M)            : %d\n', M_runs);
fprintf('Số biến trạng thái sai số (n_x)   : 19\n');
fprintf('Giá trị lý tưởng của ANEES        : %.2f\n', 19.0);
fprintf('Dải chấp nhận tin cậy 95%% ANEES   : [%.2f, %.2f]\n', r1, r2);
fprintf('Chỉ số ANEES trung bình tổng thể  : %.2f\n', mean_anees_val);
fprintf('  * Pha 1 (t <= 15s, có GNSS)     : %.2f\n', anees_phase1);
fprintf('  * Pha 2 (t > 15s, mất GNSS)     : %.2f\n', anees_phase2);
fprintf('Tỉ lệ bước nằm trong ngưỡng 95%%   : %.1f%%\n', pct_anees_pass);
if pct_anees_pass >= 80.0
    fprintf('ĐÁNH GIÁ TÍNH NHẤT QUÁN (CONSISTENCY): [ĐẠT YÊU CẦU CHUẨN KALMAN]\n');
else
    fprintf('ĐÁNH GIÁ TÍNH NHẤT QUÁN (CONSISTENCY): [CẦN TINH CHỈNH MA TRẬN Q/P]\n');
end
fprintf('-------------------------------------------------------------------\n');
fprintf('ANEES BIÊN THEO TỪNG NHÓM TRẠNG THÁI (MARGINAL ANEES - LÝ TƯỞNG = BẬC TỰ DO):\n');
fprintf('  (Lưu ý: Do hiệp phương sai chéo P_ij ~= 0, tổng ANEES biên không nhất thiết bằng ANEES toàn hệ thống)\n');
fprintf('  1. Vị trí 3D        : %6.2f / 3.00\n', mean(anees_pos));
fprintf('  2. Vận tốc 3D       : %6.2f / 3.00\n', mean(anees_vel));
fprintf('  3. Tư thế 3D        : %6.2f / 3.00\n', mean(anees_att));
fprintf('  4. Bias IMU (6 biến): %6.2f / 6.00\n', mean(anees_bias));
fprintf('  5. Hiệu chuẩn DVL   : %6.2f / 4.00 (Scale & Boresight)\n', mean(anees_cal));
fprintf('-------------------------------------------------------------------\n');
fprintf('CHẤT LƯỢNG CỔNG LỌC CHI-SQUARE & NIS CẢM BIẾN NGOÀI:\n');
fprintf('  * GNSS  : Chấp nhận %5.1f%% | NIS = %5.2f (Kỳ vọng = 7.00)\n', gate_pass_rate(1), mean_nis(1));
fprintf('  * Depth : Chấp nhận %5.1f%% | NIS = %5.2f (Kỳ vọng = 1.00)\n', gate_pass_rate(2), mean_nis(2));
fprintf('  * DVL   : Chấp nhận %5.1f%% | NIS = %5.2f (Kỳ vọng = 3.00)\n', gate_pass_rate(3), mean_nis(3));
fprintf('-------------------------------------------------------------------\n');
fprintf('ARMSE Vị trí cuối (3D)            : %.4f m\n', norm(armse(end, 1:3)));
fprintf('ARMSE Vận tốc cuối (3D)           : %.4f m/s\n', norm(armse(end, 4:6)));
fprintf('ARMSE Scale factor DVL cuối       : %.5f%% (Thực tế = 1.5%%)\n', armse(end, 16)*100);
fprintf('ARMSE Góc Roll Boresight (alpha)  : %.4f deg\n', rad2deg(armse(end, 17)));
fprintf('ARMSE Góc Pitch Boresight (beta)  : %.4f deg\n', rad2deg(armse(end, 18)));
fprintf('ARMSE Góc Yaw Boresight (gamma)   : %.4f deg\n', rad2deg(armse(end, 19)));
if strcmp(test_mode, 'CALIBRATION')
    fprintf('-------------------------------------------------------------------\n');
    fprintf('GIÁ TRỊ ƯỚC LƯỢNG HỘI TỤ ĐƯỢC TỪ [0] VỀ CHÂN LÝ:\n');
    fprintf('  * Scale factor DVL : %.5f%%  --> Thực tế = %.5f%%\n', mean_scale_est(end)*100, true_scale*100);
    fprintf('  * Roll Boresight   : %.4f deg --> Thực tế = %.4f deg\n', rad2deg(mean_bs_est(end, 1)), rad2deg(true_boresight(1)));
    fprintf('  * Pitch Boresight  : %.4f deg --> Thực tế = %.4f deg\n', rad2deg(mean_bs_est(end, 2)), rad2deg(true_boresight(2)));
    fprintf('  * Yaw Boresight    : %.4f deg --> Thực tế = %.4f deg\n', rad2deg(mean_bs_est(end, 3)), rad2deg(true_boresight(3)));
    fprintf('===================================================================\n');
end

%% ========================================================================
% 6. VẼ ĐỒ THỊ TRỰC QUAN HÓA KIỂM ĐỊNH MONTE CARLO
% =========================================================================

% ĐỒ THỊ 1: Kiểm định tính nhất quán ANEES
figure('Name', sprintf('Monte Carlo [%s]: Kiểm định chỉ số ANEES', test_mode), 'Color', 'w', 'Position', [80, 100, 1000, 480]);
fill([time_eval; flipud(time_eval)], [r1*ones(N_steps,1); r2*ones(N_steps,1)], ...
    [0.9 0.95 0.9], 'EdgeColor', 'none', 'DisplayName', 'Dải tin cậy 95% lý thuyết');
hold on; grid on;
yline(19, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Kỳ vọng lý tưởng (E[ANEES] = 19)');
yline(r1, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Ngưỡng dưới r_1 = %.2f', r1));
yline(r2, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Ngưỡng trên r_2 = %.2f', r2));
plot(time_eval, anees, 'b-', 'LineWidth', 1.6, 'DisplayName', sprintf('ANEES thực nghiệm (%d runs)', M_runs));
xline(T_gnss_cut, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS (Bắt đầu lặn ngầm)');
xlabel('Thời gian (giây)');
ylabel('Chỉ số ANEES');
title(sprintf('Kiểm định chỉ số ANEES bộ lọc ES-EKF 19 trạng thái [%s] (%d runs, Trong dải: %.1f%%)', ...
    test_mode, M_runs, pct_anees_pass));
legend('Location', 'best');
ylim([max(0, r1 - 5), r2 + 5]);

% ĐỒ THỊ 2: ARMSE của các trạng thái dẫn đường chính
figure('Name', sprintf('Monte Carlo [%s]: ARMSE Trạng thái dẫn đường', test_mode), 'Color', 'w', 'Position', [120, 80, 1100, 680]);

subplot(2, 2, 1);
pos_3d_armse = sqrt(sum(armse(:, 1:3).^2, 2));
pos_3d_3sig  = 3 * sqrt(sum(mean_sigma(:, 1:3).^2, 2));
plot(time_eval, pos_3d_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
plot(time_eval, pos_3d_3sig, 'r--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Sai số (m)'); title('ARMSE Vị trí 3D vs Biên 3-sigma');
legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

subplot(2, 2, 2);
vel_3d_armse = sqrt(sum(armse(:, 4:6).^2, 2));
vel_3d_3sig  = 3 * sqrt(sum(mean_sigma(:, 4:6).^2, 2));
plot(time_eval, vel_3d_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
plot(time_eval, vel_3d_3sig, 'r--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Sai số (m/s)'); title('ARMSE Vận tốc 3D vs Biên 3-sigma');
legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

subplot(2, 2, 3);
att_armse_deg = rad2deg(armse(:, 7:9));
att_3sig_deg  = 3 * rad2deg(mean_sigma(:, 7:9));
plot(time_eval, att_armse_deg(:, 1), 'r-', 'LineWidth', 1.4); hold on; grid on;
plot(time_eval, att_armse_deg(:, 2), 'g-', 'LineWidth', 1.4);
plot(time_eval, att_armse_deg(:, 3), 'b-', 'LineWidth', 1.4);
plot(time_eval, att_3sig_deg(:, 3), 'k--', 'LineWidth', 1.1);
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Sai số (deg)'); title('ARMSE Tư thế (Roll, Pitch, Yaw)');
legend('Roll ARMSE', 'Pitch ARMSE', 'Yaw ARMSE', 'Biên 3-sigma Yaw', 'Location', 'best');

subplot(2, 2, 4);
bg_armse_degh = rad2deg(armse(:, 13:15)) * 3600;
bg_3sig_degh  = 3 * rad2deg(mean_sigma(:, 13:15)) * 3600;
plot(time_eval, bg_armse_degh(:, 1), 'r-', 'LineWidth', 1.4); hold on; grid on;
plot(time_eval, bg_armse_degh(:, 2), 'g-', 'LineWidth', 1.4);
plot(time_eval, bg_armse_degh(:, 3), 'b-', 'LineWidth', 1.4);
plot(time_eval, bg_3sig_degh(:, 3), 'k--', 'LineWidth', 1.1);
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Bias con quay (deg/h)'); title('ARMSE Bias Con quay FOG');
legend('Bias X', 'Bias Y', 'Bias Z', 'Biên 3-sigma Z', 'Location', 'best');

% ĐỒ THỊ 3: Khả năng hiệu chuẩn hoặc ARMSE của Boresight & Scale Factor
if strcmp(test_mode, 'CALIBRATION')
    figure('Name', 'Monte Carlo [CALIBRATION]: Quỹ đạo hội tụ Boresight & Scale từ 0', 'Color', 'w', 'Position', [160, 60, 1100, 680]);

    subplot(2, 2, 1);
    plot(time_eval, mean_scale_est * 100, 'b-', 'LineWidth', 1.6, 'DisplayName', 'Ước lượng Scale (Hội tụ từ 0%)'); hold on; grid on;
    yline(true_scale * 100, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Chân lý = %.2f%%', true_scale*100));
    plot(time_eval, (mean_scale_est + 3*mean_sigma(:, 16))*100, 'k:', 'LineWidth', 1.0, 'DisplayName', 'Biên +3-sigma');
    plot(time_eval, (mean_scale_est - 3*mean_sigma(:, 16))*100, 'k:', 'LineWidth', 1.0, 'DisplayName', 'Biên -3-sigma');
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Scale factor (%)'); title('Hiệu chuẩn Scale DVL: Hội tụ từ 0% về 1.5%');
    legend('Location', 'best');

    subplot(2, 2, 2);
    plot(time_eval, rad2deg(mean_bs_est(:, 1)), 'b-', 'LineWidth', 1.6, 'DisplayName', 'Ước lượng Roll (Hội tụ từ 0°)'); hold on; grid on;
    yline(rad2deg(true_boresight(1)), 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Chân lý = %.2f°', rad2deg(true_boresight(1))));
    plot(time_eval, rad2deg(mean_bs_est(:, 1) + 3*mean_sigma(:, 17)), 'k:', 'LineWidth', 1.0, 'DisplayName', 'Biên ±3-sigma');
    plot(time_eval, rad2deg(mean_bs_est(:, 1) - 3*mean_sigma(:, 17)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Góc lệch (deg)'); title('Hiệu chuẩn Roll Boresight (alpha): 0° -> 1.5°');
    legend('Location', 'best');

    subplot(2, 2, 3);
    plot(time_eval, rad2deg(mean_bs_est(:, 2)), 'b-', 'LineWidth', 1.6, 'DisplayName', 'Ước lượng Pitch (Hội tụ từ 0°)'); hold on; grid on;
    yline(rad2deg(true_boresight(2)), 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Chân lý = %.2f°', rad2deg(true_boresight(2))));
    plot(time_eval, rad2deg(mean_bs_est(:, 2) + 3*mean_sigma(:, 18)), 'k:', 'LineWidth', 1.0, 'DisplayName', 'Biên ±3-sigma');
    plot(time_eval, rad2deg(mean_bs_est(:, 2) - 3*mean_sigma(:, 18)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Góc lệch (deg)'); title('Hiệu chuẩn Pitch Boresight (beta): 0° -> -2.0°');
    legend('Location', 'best');

    subplot(2, 2, 4);
    plot(time_eval, rad2deg(mean_bs_est(:, 3)), 'b-', 'LineWidth', 1.6, 'DisplayName', 'Ước lượng Yaw (Hội tụ từ 0°)'); hold on; grid on;
    yline(rad2deg(true_boresight(3)), 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Chân lý = %.2f°', rad2deg(true_boresight(3))));
    plot(time_eval, rad2deg(mean_bs_est(:, 3) + 3*mean_sigma(:, 19)), 'k:', 'LineWidth', 1.0, 'DisplayName', 'Biên ±3-sigma');
    plot(time_eval, rad2deg(mean_bs_est(:, 3) - 3*mean_sigma(:, 19)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Góc lệch (deg)'); title('Hiệu chuẩn Yaw Boresight (gamma): 0° -> 3.0°');
    legend('Location', 'best');
else
    figure('Name', 'Monte Carlo [CONSISTENCY]: ARMSE Boresight & Scale Factor', 'Color', 'w', 'Position', [160, 60, 1100, 680]);

    subplot(2, 2, 1);
    scale_armse_pct = armse(:, 16) * 100;
    scale_3sig_pct  = 3 * mean_sigma(:, 16) * 100;
    plot(time_eval, scale_armse_pct, 'b-', 'LineWidth', 1.6); hold on; grid on;
    plot(time_eval, scale_3sig_pct, 'r--', 'LineWidth', 1.2);
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Sai số (%)'); title('ARMSE Hệ số tỉ lệ DVL (Scale s)');
    legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

    subplot(2, 2, 2);
    bs_roll_armse = rad2deg(armse(:, 17));
    bs_roll_3sig  = 3 * rad2deg(mean_sigma(:, 17));
    plot(time_eval, bs_roll_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
    plot(time_eval, bs_roll_3sig, 'r--', 'LineWidth', 1.2);
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Sai số (deg)'); title('ARMSE Góc Roll Boresight (alpha)');
    legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

    subplot(2, 2, 3);
    bs_pitch_armse = rad2deg(armse(:, 18));
    bs_pitch_3sig  = 3 * rad2deg(mean_sigma(:, 18));
    plot(time_eval, bs_pitch_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
    plot(time_eval, bs_pitch_3sig, 'r--', 'LineWidth', 1.2);
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Sai số (deg)'); title('ARMSE Góc Pitch Boresight (beta)');
    legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

    subplot(2, 2, 4);
    bs_yaw_armse = rad2deg(armse(:, 19));
    bs_yaw_3sig  = 3 * rad2deg(mean_sigma(:, 19));
    plot(time_eval, bs_yaw_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
    plot(time_eval, bs_yaw_3sig, 'r--', 'LineWidth', 1.2);
    xline(T_gnss_cut, 'm:', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel('Sai số (deg)'); title('ARMSE Góc Yaw Boresight (gamma)');
    legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');
end

% ĐỒ THỊ 4: ANEES biên theo 5 nhóm trạng thái (Marginal ANEES)
figure('Name', sprintf('Monte Carlo [%s]: ANEES biên 5 nhóm trạng thái (Marginal ANEES)', test_mode), 'Color', 'w', 'Position', [200, 40, 1100, 700]);

subplot(2, 3, 1);
plot(time_eval, anees_pos, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(3, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=3');
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('ANEES'); title('1. Vị trí 3D (Kỳ vọng = 3)');
legend('ANEES', 'Lý tưởng', 'Location', 'best');

subplot(2, 3, 2);
plot(time_eval, anees_vel, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(3, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=3');
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('ANEES'); title('2. Vận tốc 3D (Kỳ vọng = 3)');
legend('ANEES', 'Lý tưởng', 'Location', 'best');

subplot(2, 3, 3);
plot(time_eval, anees_att, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(3, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=3');
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('ANEES'); title('3. Tư thế 3D (Kỳ vọng = 3)');
legend('ANEES', 'Lý tưởng', 'Location', 'best');

subplot(2, 3, 4);
plot(time_eval, anees_bias, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(6, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=6');
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('ANEES'); title('4. Bias IMU (Kỳ vọng = 6)');
legend('ANEES', 'Lý tưởng', 'Location', 'best');

subplot(2, 3, 5);
plot(time_eval, anees_cal, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(4, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=4');
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('ANEES'); title('5. Hiệu chuẩn DVL (Kỳ vọng = 4)');
legend('ANEES', 'Lý tưởng', 'Location', 'best');

subplot(2, 3, 6);
plot(time_eval, anees, 'k-', 'LineWidth', 1.5); hold on; grid on;
yline(19, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=19');
xline(T_gnss_cut, 'm:', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('ANEES'); title('Tổng thể (Kỳ vọng = 19)');
legend('ANEES tổng', 'Lý tưởng', 'Location', 'best');

% Đẩy các biến kết quả chính ra Base Workspace để tiện theo dõi trong cửa sổ Workspace
assignin('base', 'test_mode', test_mode);
assignin('base', 'time_eval', time_eval);
assignin('base', 'anees', anees);
assignin('base', 'anees_pos', anees_pos);
assignin('base', 'anees_vel', anees_vel);
assignin('base', 'anees_att', anees_att);
assignin('base', 'anees_bias', anees_bias);
assignin('base', 'anees_cal', anees_cal);
assignin('base', 'armse', armse);
assignin('base', 'mean_sigma', mean_sigma);
assignin('base', 'mean_scale_est', mean_scale_est);
assignin('base', 'mean_bs_est', mean_bs_est);
assignin('base', 'state_est', state_est);
assignin('base', 'P_est', P_est);

% Tự động lưu kết quả ra file .mat
save('ket_qua_E1_gyro_lever.mat', 'test_mode', 'time_eval', 'anees', ...
    'anees_pos', 'anees_vel', 'anees_att', 'anees_bias', 'anees_cal', ...
    'armse', 'mean_sigma', 'mean_scale_est', 'mean_bs_est');
fprintf('\n[THÀNH CÔNG] Đã tự động lưu kết quả E1 vào file: ket_qua_E1_gyro_lever.mat\n');
end

%% Các hàm phụ trợ cục bộ
function R = rodrigues_so3(phi)
theta = norm(phi);
K = skew_mat(phi);
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
end
end

function phi = so3_log(R)
cos_theta = max(min(0.5*(trace(R) - 1), 1), -1);
theta = acos(cos_theta);
if theta > 1e-8
    phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
else
    phi = 0.5 * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
end
end

function S = skew_mat(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function Cbn = quat_to_cbn(q)
q = q/norm(q); w = q(1); x = q(2); y = q(3); z = q(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
    2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
    2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function q = quat_multiply(p, r)
q = [p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4); ...
    p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3); ...
    p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2); ...
    p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1)];
end

function q = eul2quat_custom(eul)
c = cos(eul/2); s = sin(eul/2);
q = [c(1)*c(2)*c(3)+s(1)*s(2)*s(3); ...
    s(1)*c(2)*c(3)-c(1)*s(2)*s(3); ...
    c(1)*s(2)*c(3)+s(1)*c(2)*s(3); ...
    c(1)*c(2)*s(3)-s(1)*s(2)*c(3)];
end

function q = chi2inv_custom(p, d)
if exist('chi2inv', 'file') == 2
    q = chi2inv(p, d);
else
    z = sqrt(2) * erfinv(2*p - 1);
    q = d * (1 - 2/(9*d) + z*sqrt(2/(9*d)))^3;
end
end

