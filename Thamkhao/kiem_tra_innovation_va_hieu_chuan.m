%% SCRIPT CHẨN ĐOÁN CHUYÊN SÂU: KIỂM TRA TÍNH TRẮNG INNOVATION VÀ BÓC TÁCH COVARIANCE HIỆU CHUẨN
% Mục tiêu:
%   1. Kiểm tra tính trắng của Innovation DVL chuẩn hóa (Whiteness Test bằng Cholesky)
%   2. Bóc tách chi tiết 4 biến hiệu chuẩn (Scale, Roll, Pitch, Yaw) để tìm nguyên nhân ANEES hiệu chuẩn tăng sau 15s
clear; clc; close all;

fprintf('===================================================================\n');
fprintf('  KIỂM ĐỊNH TÍNH TRẮNG INNOVATION DVL VÀ BÓC TÁCH COVARIANCE HIỆU CHUẨN \n');
fprintf('===================================================================\n\n');

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'dieukiendauvao'));
addpath(fullfile(root_dir, 'Sensor'));
addpath(fullfile(root_dir, 'quydaochuyendong'));

file_e1 = fullfile(root_dir, 'ket_qua_E1_gyro_lever.mat');
if ~exist(file_e1, 'file')
    error('Chưa tìm thấy file %s. Vui lòng chạy test2.m trước.', file_e1);
end
data_e1 = load(file_e1);

time_eval  = data_e1.time_eval;
dt         = time_eval(2) - time_eval(1);
T_gnss_cut = 15.0;
idx_p1     = (time_eval <= T_gnss_cut);
idx_p2     = (time_eval > T_gnss_cut);
armse      = data_e1.armse;
mean_sigma = data_e1.mean_sigma;

%% ========================================================================
% PHẦN 1: BÓC TÁCH CHI TIẾT 4 BIẾN HIỆU CHUẨN DVL (SCALE, ROLL, PITCH, YAW)
% =========================================================================
fprintf('-------------------------------------------------------------------\n');
fprintf('PHẦN 1: BÓC TÁCH SỰ CO COVARIANCE CỦA 4 BIẾN HIỆU CHUẨN DVL\n');
fprintf('  (Giải thích vì sao ARMSE tiếp tục giảm mà ANEES lại tăng từ 4.22 lên 4.65)\n');
fprintf('-------------------------------------------------------------------\n');

cal_names = {'Scale factor DVL (s)', 'Roll Boresight (alpha)', 'Pitch Boresight (beta)', 'Yaw Boresight (gamma)'};
cal_units = {'%', 'deg', 'deg', 'deg'};
cal_factors = [100, 180/pi, 180/pi, 180/pi];

% Tính tỷ số (ARMSE / sigma_est)^2 cho từng biến theo thời gian
% (Tổng 4 tỷ số này chính là xấp xỉ của ANEES hiệu chuẩn khi xét riêng từng trục)
ratio_sq_time = (armse(:, 16:19) ./ max(mean_sigma(:, 16:19), 1e-12)).^2;

fprintf('%-24s | %-16s | %-16s | %-16s\n', 'Biến hiệu chuẩn', 'Pha 1 (Có GNSS)', 'Pha 2 (Mất GNSS)', 'Tỷ số (ARMSE/sig)^2');
fprintf('%-24s | ARMSE -> sigma   | ARMSE -> sigma   | Pha 1   -> Pha 2  \n', '', '', '');
fprintf('-------------------------------------------------------------------\n');

for i = 1:4
    idx_col = 15 + i;
    f = cal_factors(i);
    u = cal_units{i};

    armse_p1 = mean(armse(idx_p1, idx_col)) * f;
    sig_p1   = mean(mean_sigma(idx_p1, idx_col)) * f;

    armse_p2 = mean(armse(idx_p2, idx_col)) * f;
    sig_p2   = mean(mean_sigma(idx_p2, idx_col)) * f;

    r_sq_p1  = mean(ratio_sq_time(idx_p1, i));
    r_sq_p2  = mean(ratio_sq_time(idx_p2, i));

    delta_armse_pct = (armse_p2 - armse_p1) / armse_p1 * 100;
    delta_sig_pct   = (sig_p2 - sig_p1) / sig_p1 * 100;

    fprintf('%-24s | %6.4f -> %6.4f %-3s | %6.4f -> %6.4f %-3s | %5.2f   -> %5.2f\n', ...
        cal_names{i}, armse_p1, sig_p1, u, armse_p2, sig_p2, u, r_sq_p1, r_sq_p2);
end

fprintf('-------------------------------------------------------------------\n');
fprintf('TỔNG 4 TRỤC HIỆU CHUẨN    : Tỷ số Pha 1 = %5.2f  -->  Tỷ số Pha 2 = %5.2f (Kỳ vọng = 4.00)\n', ...
    sum(mean(ratio_sq_time(idx_p1, :), 1)), sum(mean(ratio_sq_time(idx_p2, :), 1)));
fprintf('-------------------------------------------------------------------\n\n');

%% ========================================================================
% PHẦN 2: KIỂM ĐỊNH TÍNH TRẮNG CỦA INNOVATION DVL CHUẨN HÓA (CHOLESKY WHITENING)
% =========================================================================
fprintf('-------------------------------------------------------------------\n');
fprintf('PHẦN 2: KIỂM ĐỊNH TÍNH TRẮNG CỦA INNOVATION CHUẨN HÓA (WHITENESS TEST)\n');
fprintf('  Thu thập vector nu_norm = inv(L) * residual, với S = L*L''\n');
fprintf('-------------------------------------------------------------------\n');

% Chạy mô phỏng 1 seed để trích xuất ma trận S và residual tại 150 mốc đo
rng(1, 'twister');
N_steps = round(30.0 / dt);
N_points = N_steps + 1;
time_vec = (0:N_steps)' * dt;

gravity = 9.80665; w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3; lat_rad = deg2rad(20.8); tan_lat = tan(lat_rad); height = 0;
lever_arm = [0.5; -0.1; 0.2];
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

true_scale = 0.015; true_boresight = deg2rad([1.5; -2.0; 3.0]);
Cbs_true = rodrigues_so3(-true_boresight);

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
pos_est = pos_true(1, :)' + delta_x0(1:3); vel_est = vel_true_ned(1, :)' + delta_x0(4:6);
dq0 = [1; 0.5 * delta_x0(7:9)]; dq0 = dq0 / norm(dq0);
quat_est = quat_multiply(quat_true(1, :)', dq0); quat_est = quat_est / norm(quat_est);
ba_est = ba_true_k + delta_x0(10:12); bg_est = bg_true_k + delta_x0(13:15);
scale_est = true_scale + delta_x0(16);
Cbs_est = Cbs_true * rodrigues_so3(-delta_x0(17:19)); bs_est = -so3_log(Cbs_est);

state_est = zeros(20, 1);
state_est(1:3) = pos_est; state_est(4:6) = vel_est; state_est(7:10) = quat_est;
state_est(11:13) = ba_est; state_est(14:16) = bg_est; state_est(17) = scale_est; state_est(18:20) = bs_est;
P_est = P0;

phi_acc = exp(-dt / acc_bias_tau);
q_acc_rw = acc_bias_sigma * sqrt(2 * dt / acc_bias_tau);
q_gyro_rw = gyro_bias_rw * sqrt(dt);

dvl_meas_times = [];
dvl_nu_white_history = [];

for k = 1:N_steps
    idx_target = k + 1;
    t_target = time_vec(idx_target);
    ba_true_k = phi_acc * ba_true_k + q_acc_rw * randn(3, 1);
    bg_true_k = bg_true_k + q_gyro_rw * randn(3, 1);
    f_meas = accel_true_body(k, :)' + ba_true_k + (sigma_vrw / sqrt(dt)) * randn(3, 1);
    w_meas = gyro_true_body(k, :)' + bg_true_k + (sigma_arw / sqrt(dt)) * randn(3, 1);

    meas_flags = zeros(3, 1); gnss_meas = zeros(7, 1); depth_meas = 0; dvl_meas = zeros(3, 1);
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

        % Tính toán chính xác phần dư và ma trận hiệp phương sai phần dư S
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

        % CHUẨN HÓA CHOLESKY WHITENING: nu_white = inv(L) * res, với S = L*L'
        L_chol = chol(S_dvl, 'lower');
        nu_white = L_chol \ res_dvl;

        dvl_meas_times = [dvl_meas_times; t_target];
        dvl_nu_white_history = [dvl_nu_white_history; nu_white'];
    end

    [state_est, P_est, ~, ~] = esekf19_step( ...
        state_est, P_est, Q, w_meas, f_meas, ...
        gnss_meas, depth_meas, dvl_meas, meas_flags, cfg_mc);
end

% Phân tách Innovation chuẩn hóa theo 2 pha
idx_dvl_p1 = (dvl_meas_times <= T_gnss_cut);
idx_dvl_p2 = (dvl_meas_times > T_gnss_cut);

nu_p1 = dvl_nu_white_history(idx_dvl_p1, :);
nu_p2 = dvl_nu_white_history(idx_dvl_p2, :);

fprintf('THỐNG KÊ INNOVATION CHUẨN HÓA (Kỳ vọng: Mean = [0 0 0], Covariance = eye(3)):\n\n');
fprintf('  * PHA 1 (Có GNSS - 75 mốc đo):\n');
fprintf('    Mean: [X: %+6.3f, Y: %+6.3f, Z: %+6.3f]\n', mean(nu_p1));
cov_p1 = cov(nu_p1);
fprintf('    Ma trận Covariance thực nghiệm:\n');
disp(cov_p1);

fprintf('  * PHA 2 (Mất GNSS - 75 mốc đo):\n');
fprintf('    Mean: [X: %+6.3f, Y: %+6.3f, Z: %+6.3f]\n', mean(nu_p2));
cov_p2 = cov(nu_p2);
fprintf('    Ma trận Covariance thực nghiệm:\n');
disp(cov_p2);

%% ========================================================================
% VẼ ĐỒ THỊ CHẨN ĐOÁN
% =========================================================================

% HÌNH 1: BÓC TÁCH 4 BIẾN HIỆU CHUẨN
figure('Name', 'Chẩn đoán 4 biến hiệu chuẩn DVL (Scale, Roll, Pitch, Yaw)', 'Color', 'w', 'Position', [100, 60, 1150, 700]);

for i = 1:4
    idx_col = 15 + i; f = cal_factors(i); u = cal_units{i};

    subplot(2, 4, i);
    plot(time_eval, armse(:, idx_col)*f, 'b-', 'LineWidth', 1.6, 'DisplayName', 'ARMSE thực nghiệm'); hold on; grid on;
    plot(time_eval, 3*mean_sigma(:, idx_col)*f, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Biên 3-sigma lý thuyết');
    xline(T_gnss_cut, 'm:', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS');
    xlabel('Thời gian (s)'); ylabel(sprintf('Sai số (%s)', u));
    title(sprintf('%s', cal_names{i}));
    if i == 1, legend('ARMSE', '3-sigma', 'Location', 'best'); end

    subplot(2, 4, i + 4);
    plot(time_eval, ratio_sq_time(:, i), 'k-', 'LineWidth', 1.5); hold on; grid on;
    yline(1.0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng E=1.0');
    xline(T_gnss_cut, 'm:', 'LineWidth', 1.5);
    xlabel('Thời gian (s)'); ylabel('Tỷ số (ARMSE / sigma)^2');
    title(sprintf('Độ tự tin trục: %s', cal_names{i}));
    ylim([0, max(max(ratio_sq_time(:, i))*1.2, 3.0)]);
    if i == 1, legend('Tỷ số trục', 'Lý tưởng', 'Location', 'best'); end
end

% HÌNH 2: TỰ TƯƠNG QUAN (AUTOCORRELATION) VÀ TƯƠNG QUAN CHÉO INNOVATION CHUẨN HÓA
figure('Name', 'Whiteness Test: Tự tương quan của Innovation DVL chuẩn hóa', 'Color', 'w', 'Position', [150, 100, 1100, 650]);

axes_lbl = {'Trục X', 'Trục Y', 'Trục Z'};
max_lags = 15;
bound_p1 = 2 / sqrt(size(nu_p1, 1));
bound_p2 = 2 / sqrt(size(nu_p2, 1));

for ax = 1:3
    subplot(2, 3, ax);
    [ac_p1, lags1] = xcorr(nu_p1(:, ax) - mean(nu_p1(:, ax)), max_lags, 'coeff');
    stem(lags1, ac_p1, 'b', 'Filled', 'LineWidth', 1.2); hold on; grid on;
    yline(bound_p1, 'r--', 'LineWidth', 1.0); yline(-bound_p1, 'r--', 'LineWidth', 1.0);
    xlabel('Lag (bước đo)'); ylabel('Hệ số tự tương quan');
    title(sprintf('Pha 1 (Có GNSS): %s', axes_lbl{ax}));
    ylim([-0.5, 1.1]);

    subplot(2, 3, ax + 3);
    [ac_p2, lags2] = xcorr(nu_p2(:, ax) - mean(nu_p2(:, ax)), max_lags, 'coeff');
    stem(lags2, ac_p2, 'm', 'Filled', 'LineWidth', 1.2); hold on; grid on;
    yline(bound_p2, 'r--', 'LineWidth', 1.0); yline(-bound_p2, 'r--', 'LineWidth', 1.0);
    xlabel('Lag (bước đo)'); ylabel('Hệ số tự tương quan');
    title(sprintf('Pha 2 (Mất GNSS): %s', axes_lbl{ax}));
    ylim([-0.5, 1.1]);
end

fprintf('\n[THÀNH CÔNG] Đã hoàn thành chẩn đoán và hiển thị 2 cửa sổ đồ thị!\n');
fprintf('===================================================================\n');

%% Các hàm phụ trợ cục bộ
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
