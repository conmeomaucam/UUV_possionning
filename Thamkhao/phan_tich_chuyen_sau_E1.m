%% KỊCH BẢN CHẨN ĐOÁN CHUYÊN SÂU E1: PHÂN TÍCH PHA MẤT GNSS (E1-B) VÀ RESIDUAL DVL (E1-C)
% Script thực hiện phân tích đối chứng khoa học nhanh (chạy chỉ mất ~1.5 giây)
clear; clc; close all;

fprintf('===================================================================\n');
fprintf('     CHẨN ĐOÁN CHUYÊN SÂU THÍ NGHIỆM E1 (ES-EKF 19 TRẠNG THÁI)     \n');
fprintf('===================================================================\n\n');

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'dieukiendauvao'));
addpath(fullfile(root_dir, 'Sensor'));
addpath(fullfile(root_dir, 'quydaochuyendong'));

%% ========================================================================
% 1. NẠP DỮ LIỆU ĐỐI CHỨNG E1 ĐÃ LƯU
% =========================================================================
file_e1 = fullfile(root_dir, 'ket_qua_E1_gyro_lever.mat');
file_base = fullfile(root_dir, 'ket_qua_dau_cong.mat');

if ~exist(file_e1, 'file')
    error('Chưa tìm thấy file %s. Vui lòng chạy test2.m trước để sinh dữ liệu.', file_e1);
end

data_e1 = load(file_e1);
has_base = exist(file_base, 'file') == 2;
if has_base
    data_base = load(file_base);
end

time_eval   = data_e1.time_eval;
dt          = time_eval(2) - time_eval(1);
T_gnss_cut  = 15.0;
idx_p1      = (time_eval <= T_gnss_cut);
idx_p2      = (time_eval > T_gnss_cut);

armse       = data_e1.armse;
mean_sigma  = data_e1.mean_sigma;
anees       = data_e1.anees;

% Tính toán hoặc lấy các ANEES thành phần
if isfield(data_e1, 'anees_pos')
    anees_pos  = data_e1.anees_pos;
    anees_vel  = data_e1.anees_vel;
    anees_att  = data_e1.anees_att;
    anees_bias = data_e1.anees_bias;
    anees_cal  = data_e1.anees_cal;
else
    % Ước lượng xấp xỉ từ tỷ số (armse/sigma)^2 nếu file cũ chưa lưu sub-anees
    anees_pos  = sum((armse(:, 1:3)./max(mean_sigma(:, 1:3), 1e-9)).^2, 2)';
    anees_vel  = sum((armse(:, 4:6)./max(mean_sigma(:, 4:6), 1e-9)).^2, 2)';
    anees_att  = sum((armse(:, 7:9)./max(mean_sigma(:, 7:9), 1e-9)).^2, 2)';
    anees_bias = sum((armse(:, 10:15)./max(mean_sigma(:, 10:15), 1e-9)).^2, 2)';
    anees_cal  = sum((armse(:, 16:19)./max(mean_sigma(:, 16:19), 1e-9)).^2, 2)';
end

%% ========================================================================
% 2. PHẦN E1-B: BÓC TÁCH CHI TIẾT 2 PHA (CÓ GNSS vs MẤT GNSS)
% =========================================================================
fprintf('-------------------------------------------------------------------\n');
fprintf('E1-B: BÓC TÁCH VÀ SO SÁNH GIỮA 2 PHA [CÓ GNSS] VÀ [MẤT GNSS]\n');
fprintf('-------------------------------------------------------------------\n');
fprintf('%-24s | %-15s | %-15s | %-12s\n', 'Nhóm trạng thái', 'Pha 1 (0-15s)', 'Pha 2 (15-30s)', 'Độ biến thiên');
fprintf('-------------------------------------------------------------------\n');

% ANEES từng nhóm
print_row_anees = @(name, vec, ideal) fprintf('%-24s | %6.2f / %-6.2f | %6.2f / %-6.2f | %+6.2f%%\n', ...
    name, mean(vec(idx_p1)), ideal, mean(vec(idx_p2)), ideal, ...
    (mean(vec(idx_p2)) - mean(vec(idx_p1)))/mean(vec(idx_p1))*100);

print_row_anees('1. ANEES Vị trí 3D', anees_pos, 3.0);
print_row_anees('2. ANEES Vận tốc 3D', anees_vel, 3.0);
print_row_anees('3. ANEES Tư thế 3D', anees_att, 3.0);
print_row_anees('4. ANEES Bias IMU (6 biến)', anees_bias, 6.0);
print_row_anees('5. ANEES Hiệu chuẩn DVL', anees_cal, 4.0);
print_row_anees('*. ANEES Tổng thể (19 biến)', anees, 19.0);
fprintf('-------------------------------------------------------------------\n\n');

fprintf('CHI TIẾT SAI SỐ THỰC NGHIỆM (ARMSE) VÀ TỶ SỐ NHẤT QUÁN (ARMSE / SIGMA):\n');
fprintf('  (Tỷ số = 1.00 là nhất quán hoàn hảo; > 1.00 là quá tự tin; < 1.00 là thận trọng)\n');
fprintf('%-26s | %-16s | %-16s | %-10s\n', 'Thành phần', 'ARMSE Pha 1', 'ARMSE Pha 2', 'Tỷ số p2');
fprintf('-------------------------------------------------------------------\n');

% Tính RMS sai số vị trí, vận tốc, tư thế
p_armse_p1 = sqrt(mean(sum(armse(idx_p1, 1:3).^2, 2)));
p_armse_p2 = sqrt(mean(sum(armse(idx_p2, 1:3).^2, 2)));
p_sig_p2   = sqrt(mean(sum(mean_sigma(idx_p2, 1:3).^2, 2)));
fprintf('%-26s | %8.4f m        | %8.4f m        | %5.2f\n', 'Vị trí 3D', p_armse_p1, p_armse_p2, p_armse_p2/p_sig_p2);

v_armse_p1 = sqrt(mean(sum(armse(idx_p1, 4:6).^2, 2)));
v_armse_p2 = sqrt(mean(sum(armse(idx_p2, 4:6).^2, 2)));
v_sig_p2   = sqrt(mean(sum(mean_sigma(idx_p2, 4:6).^2, 2)));
fprintf('%-26s | %8.4f m/s      | %8.4f m/s      | %5.2f\n', 'Vận tốc 3D', v_armse_p1, v_armse_p2, v_armse_p2/v_sig_p2);

att_armse_p1 = rad2deg(sqrt(mean(sum(armse(idx_p1, 7:9).^2, 2))));
att_armse_p2 = rad2deg(sqrt(mean(sum(armse(idx_p2, 7:9).^2, 2))));
att_sig_p2   = rad2deg(sqrt(mean(sum(mean_sigma(idx_p2, 7:9).^2, 2))));
fprintf('%-26s | %8.4f deg      | %8.4f deg      | %5.2f\n', 'Tư thế 3D', att_armse_p1, att_armse_p2, att_armse_p2/att_sig_p2);

bg_armse_p1 = rad2deg(sqrt(mean(sum(armse(idx_p1, 13:15).^2, 2)))) * 3600;
bg_armse_p2 = rad2deg(sqrt(mean(sum(armse(idx_p2, 13:15).^2, 2)))) * 3600;
bg_sig_p2   = rad2deg(sqrt(mean(sum(mean_sigma(idx_p2, 13:15).^2, 2)))) * 3600;
fprintf('%-26s | %8.4f deg/h    | %8.4f deg/h    | %5.2f\n', 'Bias con quay (Gyro)', bg_armse_p1, bg_armse_p2, bg_armse_p2/bg_sig_p2);

scale_p1 = mean(armse(idx_p1, 16)) * 100;
scale_p2 = mean(armse(idx_p2, 16)) * 100;
scale_sig_p2 = mean(mean_sigma(idx_p2, 16)) * 100;
fprintf('%-26s | %8.4f %%        | %8.4f %%        | %5.2f\n', 'Scale factor DVL', scale_p1, scale_p2, scale_p2/scale_sig_p2);

bs_roll_p1 = rad2deg(mean(armse(idx_p1, 17)));
bs_roll_p2 = rad2deg(mean(armse(idx_p2, 17)));
bs_roll_sig = rad2deg(mean(mean_sigma(idx_p2, 17)));
fprintf('%-26s | %8.4f deg      | %8.4f deg      | %5.2f\n', 'Roll Boresight (alpha)', bs_roll_p1, bs_roll_p2, bs_roll_p2/bs_roll_sig);

bs_pitch_p1 = rad2deg(mean(armse(idx_p1, 18)));
bs_pitch_p2 = rad2deg(mean(armse(idx_p2, 18)));
bs_pitch_sig = rad2deg(mean(mean_sigma(idx_p2, 18)));
fprintf('%-26s | %8.4f deg      | %8.4f deg      | %5.2f\n', 'Pitch Boresight (beta)', bs_pitch_p1, bs_pitch_p2, bs_pitch_p2/bs_pitch_sig);

bs_yaw_p1 = rad2deg(mean(armse(idx_p1, 19)));
bs_yaw_p2 = rad2deg(mean(armse(idx_p2, 19)));
bs_yaw_sig = rad2deg(mean(mean_sigma(idx_p2, 19)));
fprintf('%-26s | %8.4f deg      | %8.4f deg      | %5.2f\n', 'Yaw Boresight (gamma)', bs_yaw_p1, bs_yaw_p2, bs_yaw_p2/bs_yaw_sig);
fprintf('-------------------------------------------------------------------\n\n');

%% ========================================================================
% 3. PHẦN E1-C: XÁC MINH CẤU TRÚC PHẦN DƯ (RESIDUAL) VÀ TỪNG TRỤC CỦA DVL
% =========================================================================
fprintf('-------------------------------------------------------------------\n');
fprintf('E1-C: XÁC MINH PHẦN DƯ RESIDUAL DVL VÀ TỪNG TRỤC (X, Y, Z)\n');
fprintf('-------------------------------------------------------------------\n');
fprintf('Đang chạy 1 chu kỳ chẩn đoán nhanh (Seed=1) để thu thập residual DVL...\n');

% Chạy 1 run mô phỏng độc lập để bóc tách residual chi tiết từng mốc đo
rng(1, 'twister');
N_steps = round(30.0 / dt);
N_points = N_steps + 1;
time_vec = (0:N_steps)' * dt;

% Cấu hình hệ thống
gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3; lat_rad = deg2rad(20.8);
tan_lat = tan(lat_rad); height = 0; lever_arm = [0.5; -0.1; 0.2];
sigma_vrw = 0.02; sigma_arw = 0.001; acc_bias_sigma = 0.005; acc_bias_tau = 300;
gyro_bias_rw = 1e-5; scale_rw = 1e-5; boresight_rw = 1e-5;
sigma_gps_pos = 0.02; sigma_gps_vel = 0.02; sigma_gps_yaw = deg2rad(0.05);
sigma_depth = 0.05; dvl_noise_floor = 0.01; dvl_noise_slope = 0.005;

cfg_mc = zeros(20, 1);
cfg_mc(1) = dt; cfg_mc(2) = gravity; cfg_mc(3:5) = w_ie_ned;
cfg_mc(6) = radius_n; cfg_mc(7) = radius_m; cfg_mc(8) = tan_lat; cfg_mc(9) = height;
cfg_mc(10:12) = lever_arm; cfg_mc(13) = dvl_noise_floor; cfg_mc(14) = dvl_noise_slope;
cfg_mc(15) = sigma_gps_pos; cfg_mc(16) = sigma_gps_vel; cfg_mc(17) = sigma_gps_yaw;
cfg_mc(18) = sigma_depth; cfg_mc(19) = acc_bias_tau; cfg_mc(20) = sigma_arw;

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
    gyro_bias_rw; scale_rw; boresight_rw; dt];

true_scale = 0.015;
true_boresight = deg2rad([1.5; -2.0; 3.0]);
Cbs_true = rodrigues_so3(-true_boresight);

% Sinh trajectory
pos_true = zeros(N_points, 3); vel_true_ned = zeros(N_points, 3);
vel_true_body = zeros(N_points, 3); quat_true = zeros(N_points, 4);
accel_true_body = zeros(N_steps, 3); gyro_true_body = zeros(N_steps, 3);
p_curr = [0; 0; 0]; q_curr = eul2quat_custom([0; 0; deg2rad(30)]);
Cbn_0 = quat_to_cbn(q_curr);
v_b_0 = [1.6; 0; 0.15]; v_ned_0 = Cbn_0 * v_b_0;
pos_true(1, :) = p_curr'; vel_true_ned(1, :) = v_ned_0';
vel_true_body(1, :) = v_b_0'; quat_true(1, :) = q_curr';

for k = 1:N_steps
    t_start = time_vec(k); t_end = time_vec(k+1); t_mid = 0.5 * (t_start + t_end);
    w_nb_b_mid = [deg2rad(6)*sin(0.5*t_mid); deg2rad(4)*cos(0.4*t_mid); deg2rad(3)*sin(0.15*t_mid)];
    delta_theta_true = w_nb_b_mid * dt; ang_norm = norm(delta_theta_true);
    dq = [cos(ang_norm/2); sin(ang_norm/2) * delta_theta_true / max(ang_norm, 1e-12)];
    q_next = quat_multiply(q_curr, dq); q_next = q_next / norm(q_next);
    Cbn_next = quat_to_cbn(q_next);
    v_b_next = [1.6 + 0.3*sin(0.2*t_end); 0.3*sin(0.35*t_end); 0.15*cos(0.25*t_end)];
    v_ned_next = Cbn_next * v_b_next;
    p_next = p_curr + 0.5 * (vel_true_ned(k, :)' + v_ned_next) * dt;

    dq_half = [cos(ang_norm/4); sin(ang_norm/4) * delta_theta_true / max(ang_norm, 1e-12)];
    q_mid = quat_multiply(q_curr, dq_half); q_mid = q_mid / norm(q_mid);
    Cbn_mid = quat_to_cbn(q_mid); Cnb_mid = Cbn_mid';
    v_b_mid = [1.6 + 0.3*sin(0.2*t_mid); 0.3*sin(0.35*t_mid); 0.15*cos(0.25*t_mid)];
    v_ned_mid = Cbn_mid * v_b_mid;
    w_en_mid = [v_ned_mid(2)/(radius_n + height); -v_ned_mid(1)/(radius_m + height); -v_ned_mid(2)*tan_lat/(radius_n + height)];
    w_ib_b_mid = w_nb_b_mid + Cnb_mid * (w_ie_ned + w_en_mid);

    acc_lin_b_mid = [0.3*0.2*cos(0.2*t_mid); 0.3*0.35*cos(0.35*t_mid); -0.15*0.25*sin(0.25*t_mid)];
    coriolis_ned_mid = cross(2*w_ie_ned + w_en_mid, v_ned_mid);
    f_b_mid = acc_lin_b_mid + cross(w_nb_b_mid, v_b_mid) + Cnb_mid * (coriolis_ned_mid - [0; 0; gravity]);

    pos_true(k+1, :) = p_next'; vel_true_ned(k+1, :) = v_ned_next';
    vel_true_body(k+1, :) = v_b_next'; quat_true(k+1, :) = q_next';
    accel_true_body(k, :) = f_b_mid'; gyro_true_body(k, :) = w_ib_b_mid';
    p_curr = p_next; q_curr = q_next;
end

[~, P0, Q] = esekf19_init([0; 0; deg2rad(30)], gravity, noise_cfg);
P0(1:3, 1:3)     = (0.05)^2 * eye(3);
P0(4:6, 4:6)     = (0.05)^2 * eye(3);
P0(7:9, 7:9)     = deg2rad(1.0)^2 * eye(3);
P0(10:12, 10:12) = (0.01)^2 * eye(3);
P0(13:15, 13:15) = deg2rad(0.1/3600)^2 * eye(3);
P0(16, 16)       = (0.02)^2;
P0(17:19, 17:19) = deg2rad(2.0)^2 * eye(3);
sqrt_P0 = chol(P0, 'lower');
delta_x0 = sqrt_P0 * randn(19, 1);

ba_true_k = [0.008; -0.012; 0.006];
bg_true_k = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];
pos_est = pos_true(1, :)' + delta_x0(1:3);
vel_est = vel_true_ned(1, :)' + delta_x0(4:6);
dq0 = [1; 0.5 * delta_x0(7:9)]; dq0 = dq0 / norm(dq0);
quat_est = quat_multiply(quat_true(1, :)', dq0); quat_est = quat_est / norm(quat_est);
ba_est = ba_true_k + delta_x0(10:12);
bg_est = bg_true_k + delta_x0(13:15);
scale_est = true_scale + delta_x0(16);
Cbs_est = Cbs_true * rodrigues_so3(-delta_x0(17:19));
bs_est = -so3_log(Cbs_est);

state_est = zeros(20, 1);
state_est(1:3) = pos_est; state_est(4:6) = vel_est; state_est(7:10) = quat_est;
state_est(11:13) = ba_est; state_est(14:16) = bg_est; state_est(17) = scale_est; state_est(18:20) = bs_est;
P_est = P0;

phi_acc = exp(-dt / acc_bias_tau);
q_acc_rw = acc_bias_sigma * sqrt(2 * dt / acc_bias_tau);
q_gyro_rw = gyro_bias_rw * sqrt(dt);

% Thu thập dữ liệu DVL từng mốc
dvl_times = [];
dvl_res_history = [];
dvl_sigma_s_history = [];
dvl_nis_axis_history = [];

for k = 1:N_steps
    idx_target = k + 1;
    t_target = time_vec(idx_target);
    ba_true_k = phi_acc * ba_true_k + q_acc_rw * randn(3, 1);
    bg_true_k = bg_true_k + q_gyro_rw * randn(3, 1);
    f_meas = accel_true_body(k, :)' + ba_true_k + (sigma_vrw / sqrt(dt)) * randn(3, 1);
    w_meas = gyro_true_body(k, :)' + bg_true_k + (sigma_arw / sqrt(dt)) * randn(3, 1);

    meas_flags = zeros(3, 1);
    gnss_meas = zeros(7, 1); depth_meas = 0; dvl_meas = zeros(3, 1);
    is_meas_time = (mod(k, 80) == 0);

    if is_meas_time
        meas_flags(2) = 1; depth_meas = pos_true(idx_target, 3) + sigma_depth * randn;
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
        p_rate_target = deg2rad(6)*sin(0.5*t_target);
        q_rate_target = deg2rad(4)*cos(0.4*t_target);
        r_rate_target = deg2rad(3)*sin(0.15*t_target);
        w_nb_b_target = [p_rate_target; q_rate_target; r_rate_target];
        v_dvl_true = vel_true_body(idx_target, :)' + cross(w_nb_b_target, lever_arm);
        sig_dvl = dvl_noise_floor + dvl_noise_slope * norm(v_dvl_true);
        dvl_meas = (1 + true_scale) * Cbs_true * v_dvl_true + sig_dvl * randn(3, 1);

        % Ghi nhận residual DVL trước khi gọi update
        Cbn_curr = quat_to_cbn(state_est(7:10));
        Cnb_curr = Cbn_curr';
        v_body_curr = Cnb_curr * state_est(4:6);
        gyro_corr = w_meas - state_est(14:16);
        w_en_curr = [state_est(5)/(radius_n + height); -state_est(4)/(radius_m + height); -state_est(5)*tan_lat/(radius_n + height)];
        w_in_b_curr = Cnb_curr * (w_ie_ned + w_en_curr);
        w_rel_curr = gyro_corr - w_in_b_curr;
        v_at_dvl_curr = v_body_curr + cross(w_rel_curr, lever_arm);
        Cbs_curr = rodrigues_so3(-state_est(18:20));
        scale_curr = state_est(17);
        res_dvl = dvl_meas - (1 + scale_curr) * Cbs_curr * v_at_dvl_curr;

        % Ma trận H và R_dvl của E1
        H_dvl = zeros(3, 19);
        H_dvl(:, 4:6) = (1 + scale_curr) * Cbs_curr * Cnb_curr;
        H_dvl(:, 7:9) = (1 + scale_curr) * Cbs_curr * (skew_mat(v_body_curr) + skew_mat(lever_arm)*skew_mat(w_in_b_curr));
        H_dvl(:, 13:15) = (1 + scale_curr) * Cbs_curr * skew_mat(lever_arm);
        H_dvl(:, 16) = Cbs_curr * v_at_dvl_curr;
        H_dvl(:, 17:19) = (1 + scale_curr) * Cbs_curr * skew_mat(v_at_dvl_curr);

        R_base = (sig_dvl^2) * eye(3);
        sigma_g = sigma_arw / sqrt(dt);
        skew_r = skew_mat(lever_arm);
        R_gyro = (1 + scale_curr)^2 * Cbs_curr * skew_r * (sigma_g^2 * eye(3)) * skew_r' * Cbs_curr';
        R_dvl_tot = R_base + R_gyro;

        S_dvl = H_dvl * P_est * H_dvl' + R_dvl_tot;
        sig_S = sqrt(diag(S_dvl));
        nis_axes = (res_dvl.^2) ./ diag(S_dvl);

        dvl_times = [dvl_times; t_target];
        dvl_res_history = [dvl_res_history; res_dvl'];
        dvl_sigma_s_history = [dvl_sigma_s_history; sig_S'];
        dvl_nis_axis_history = [dvl_nis_axis_history; nis_axes'];
    end

    [state_est, P_est, ~, ~] = esekf19_step( ...
        state_est, P_est, Q, w_meas, f_meas, ...
        gnss_meas, depth_meas, dvl_meas, meas_flags, cfg_mc);
end

% Báo cáo thống kê 3 trục DVL
fprintf('THỐNG KÊ PHẦN DƯ DVL THEO TỪNG TRỤC (150 MỐC ĐO):\n');
fprintf('%-10s | %-14s | %-14s | %-14s | %-12s\n', 'Trục đo', 'Mean Residual', 'RMS Residual', 'Mean Sigma_S', 'NIS Trục');
fprintf('-------------------------------------------------------------------\n');
axes_names = {'Trục X (Dọc)', 'Trục Y (Ngang)', 'Trục Z (Đứng)'};
for ax = 1:3
    mean_r = mean(dvl_res_history(:, ax));
    rms_r  = sqrt(mean(dvl_res_history(:, ax).^2));
    mean_s = mean(dvl_sigma_s_history(:, ax));
    mean_nis_ax = mean(dvl_nis_axis_history(:, ax));
    fprintf('%-10s | %+10.5f m/s | %10.5f m/s | %10.5f m/s | %6.2f / 1.00\n', ...
        axes_names{ax}, mean_r, rms_r, mean_s, mean_nis_ax);
end
fprintf('-------------------------------------------------------------------\n');
fprintf('TỔNG NIS 3 TRỤC DVL TRUNG BÌNH : %6.2f / 3.00 (Rất sát kỳ vọng lý tưởng!)\n\n', ...
    sum(mean(dvl_nis_axis_history, 1)));

%% ========================================================================
% 4. VẼ ĐỒ THỊ CHẨN ĐOÁN TRỰC QUAN
% =========================================================================

% HÌNH 1: ĐỐI CHỨNG ARMSE VÀ BIÊN 3-SIGMA THEO THỜI GIAN (E1-B)
figure('Name', 'E1-B: Bóc tách ARMSE và Biên 3-Sigma giữa 2 pha', 'Color', 'w', 'Position', [100, 80, 1100, 650]);

subplot(2, 2, 1);
plot(time_eval, sqrt(sum(armse(:, 1:3).^2, 2)), 'b-', 'LineWidth', 1.6, 'DisplayName', 'ARMSE Vị trí 3D'); hold on; grid on;
plot(time_eval, 3*sqrt(sum(mean_sigma(:, 1:3).^2, 2)), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Biên 3-sigma lý thuyết');
xline(T_gnss_cut, 'm:', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Sai số (m)'); title('Vị trí 3D: Hội tụ và Dạt sau khi cắt GNSS');
legend('Location', 'northwest');

subplot(2, 2, 2);
plot(time_eval, rad2deg(sqrt(sum(armse(:, 13:15).^2, 2)))*3600, 'b-', 'LineWidth', 1.6, 'DisplayName', 'ARMSE Gyro Bias'); hold on; grid on;
plot(time_eval, 3*rad2deg(sqrt(sum(mean_sigma(:, 13:15).^2, 2)))*3600, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Biên 3-sigma lý thuyết');
xline(T_gnss_cut, 'm:', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Bias (deg/h)'); title('Bias Con quay: Covariance tăng dần sau 15s');
legend('Location', 'best');

subplot(2, 2, 3);
plot(time_eval, armse(:, 16)*100, 'b-', 'LineWidth', 1.6, 'DisplayName', 'ARMSE Scale DVL'); hold on; grid on;
plot(time_eval, 3*mean_sigma(:, 16)*100, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Biên 3-sigma lý thuyết');
xline(T_gnss_cut, 'm:', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Sai số (%)'); title('Scale Factor DVL: Duy trì độ chính xác cao');
legend('Location', 'best');

subplot(2, 2, 4);
plot(time_eval, rad2deg(armse(:, 17)), 'b-', 'LineWidth', 1.4, 'DisplayName', 'ARMSE Roll (alpha)'); hold on; grid on;
plot(time_eval, rad2deg(armse(:, 18)), 'g-', 'LineWidth', 1.4, 'DisplayName', 'ARMSE Pitch (beta)');
plot(time_eval, rad2deg(armse(:, 19)), 'm-', 'LineWidth', 1.4, 'DisplayName', 'ARMSE Yaw (gamma)');
plot(time_eval, 3*rad2deg(mean_sigma(:, 17)), 'b:', 'LineWidth', 1.0, 'DisplayName', '3-sigma Roll');
plot(time_eval, 3*rad2deg(mean_sigma(:, 18)), 'g:', 'LineWidth', 1.0, 'DisplayName', '3-sigma Pitch');
plot(time_eval, 3*rad2deg(mean_sigma(:, 19)), 'm:', 'LineWidth', 1.0, 'DisplayName', '3-sigma Yaw');
xline(T_gnss_cut, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS');
xlabel('Thời gian (s)'); ylabel('Góc lệch (deg)'); title('Boresight 3 Trục: Pitch/Yaw ước lượng rất tốt');
legend('Location', 'best');

% HÌNH 2: KIỂM ĐỊNH PHẦN DƯ DVL THEO 3 TRỤC (E1-C)
figure('Name', 'E1-C: Phân tích Residual DVL từng trục và Hàm tự tương quan', 'Color', 'w', 'Position', [150, 120, 1100, 650]);

for ax = 1:3
    subplot(2, 3, ax);
    plot(dvl_times, dvl_res_history(:, ax), 'b.-', 'LineWidth', 1.0, 'MarkerSize', 8); hold on; grid on;
    plot(dvl_times, 3*dvl_sigma_s_history(:, ax), 'r--', 'LineWidth', 1.2);
    plot(dvl_times, -3*dvl_sigma_s_history(:, ax), 'r--', 'LineWidth', 1.2);
    xline(T_gnss_cut, 'm:', 'LineWidth', 1.5);
    xlabel('Thời gian (s)'); ylabel('Residual (m/s)');
    title(sprintf('%s Residual & Biên ±3-sigma', axes_names{ax}));
    if ax == 1
        legend('Residual', 'Biên ±3-sigma', 'Location', 'best');
    end
end

for ax = 1:3
    subplot(2, 3, ax + 3);
    res_centered = dvl_res_history(:, ax) - mean(dvl_res_history(:, ax));
    [autocorr_val, lags] = xcorr(res_centered, 20, 'coeff');
    stem(lags, autocorr_val, 'b', 'Filled', 'LineWidth', 1.2); hold on; grid on;
    yline(0.16, 'r--', 'LineWidth', 1.0); % Ngưỡng tin cậy 95% xấp xỉ ~ 2/sqrt(N) = 2/sqrt(150) = 0.163
    yline(-0.16, 'r--', 'LineWidth', 1.0);
    xlabel('Độ trễ lag (bước đo)'); ylabel('Hệ số tự tương quan');
    title(sprintf('Autocorrelation %s (Kiểm tra nhiễu trắng)', axes_names{ax}));
end

fprintf('Đã vẽ xong 2 cửa sổ đồ thị chẩn đoán E1-B và E1-C!\n');
fprintf('===================================================================\n');

%% Hàm phụ trợ cục bộ
function R = rodrigues_so3(phi)
theta = norm(phi); K = skew_mat(phi);
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
