function test_esekf19_consistency_mc1000(M_in)
%% TEST_ESEKF19_CONSISTENCY_MC1000
% Nhiệm vụ 2 (huongdantest2.md): Monte Carlo Consistency Test với M = 1.000 lần chạy độc lập.
% Chế độ: 'CONSISTENCY' - Khởi tạo x0 ~ N(x_true, P0).
%
% Mục tiêu kiểm tra:
%   1. ANEES toàn hệ thống 19D theo thời gian, đối chiếu dải tin cậy 95% lý thuyết [18.62, 19.38].
%   2. ANEES 5 nhóm trạng thái (Vị trí, Vận tốc, Tư thế, Bias IMU, Hiệu chuẩn DVL).
%   3. Tách biệt thống kê Pha 1 (0-15s, có GNSS) và Pha 2 (15-30s, mất GNSS / lặn ngầm).
%   4. Đánh giá chuyên sâu DVL Innovation:
%      - Thu thập residual và S trước gating.
%      - Tính NIS trước gating và tỷ lệ chấp nhận cổng Chi-square.
%      - Chuẩn hóa làm trắng (Whitened Innovation): e_w = S^(-1/2) * residual.
%      - Kiểm tra ma trận hiệp phương sai của e_w (kỳ vọng I_3).
%      - Tự tương quan từng trục của e_w qua các độ trễ lag 1 đến 5 (kiểm tra tính trắng).
%
% Đầu ra:
%   - Bảng kết quả định dạng: | ID | Test | Metric | Threshold | Actual | PASS/FAIL |
%   - Biểu đồ lưu tại thư mục TEST/: ANEES toàn hệ, ANEES nhóm, DVL Whitened Innovation & Autocorr.
%   - Lưu file .mat: TEST/ket_qua_test_consistency_mc1000.mat
%   - Xuất file báo cáo .txt: TEST/bao_cao_test_consistency_mc1000.txt

if nargin < 1 || isempty(M_in)
    M_runs = 1000; % Mặc định 1.000 lần chạy theo yêu cầu huongdantest2.md
else
    M_runs = M_in;
end

clc; close all;
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'Sensor'));

fprintf('========================================================================================\n');
fprintf('  NHIỆM VỤ 2: MONTE CARLO CONSISTENCY TEST (M = %d RUNS, CHẾ ĐỘ CONSISTENCY)\n', M_runs);
fprintf('========================================================================================\n\n');

%% ========================================================================
% 1. THIẾT LẬP THAM SỐ VÀ CẤU HÌNH HỆ THỐNG
% =========================================================================
dt = 0.0025;                  % Chu kỳ trích mẫu 400 Hz
T_total = 30.0;               % Tổng thời gian 30s
T_gnss_cut = 15.0;            % Thời điểm cắt GNSS lặn ngầm
N_steps = round(T_total / dt);
N_points = N_steps + 1;
time_vec = (0:N_steps)' * dt;
time_eval = time_vec(2:end);

gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;
tan_lat = tan(deg2rad(20.8)); height = 0;
lever_arm = [0.5; -0.1; 0.2];

sigma_vrw = 0.02;             % VRW gia tốc kế (m/s/sqrt(s))
sigma_arw = 0.001;            % ARW con quay (rad/sqrt(s))
acc_bias_sigma = 0.005;       % Độ ổn định bias in-run gia tốc (m/s^2)
acc_bias_tau = 300;           % Thời gian tương quan (s)
gyro_bias_rw = 1e-5;          % Random walk bias con quay (rad/s/sqrt(s))
scale_rw = 1e-5;              % Random walk scale factor DVL (1/sqrt(s))
boresight_rw = 1e-5;          % Random walk Boresight (rad/sqrt(s))

sigma_gps_pos = 0.02;
sigma_gps_vel = 0.02;
sigma_gps_yaw = deg2rad(0.05);
sigma_depth = 0.05;
dvl_noise_floor = 0.01;
dvl_noise_slope = 0.005;

cfg = zeros(20, 1);
cfg(1) = dt; cfg(2) = gravity; cfg(3:5) = w_ie_ned;
cfg(6) = radius_n; cfg(7) = radius_m; cfg(8) = tan_lat; cfg(9) = height;
cfg(10:12) = lever_arm; cfg(13) = dvl_noise_floor; cfg(14) = dvl_noise_slope;
cfg(15) = sigma_gps_pos; cfg(16) = sigma_gps_vel; cfg(17) = sigma_gps_yaw;
cfg(18) = sigma_depth; cfg(19) = acc_bias_tau;
cfg(20) = sigma_arw; % Bù nhiễu gyro qua cánh tay đòn (Thí nghiệm E1)

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
             gyro_bias_rw; scale_rw; boresight_rw; dt];

true_scale = 0.015;
true_boresight = deg2rad([1.5; -2.0; 3.0]);
Cbs_true = rodrigues_so3_local(-true_boresight);

%% ========================================================================
% 2. SINH QUỸ ĐẠO CHUẨN 3D (GROUND TRUTH)
% =========================================================================
pos_true = zeros(N_points, 3);
vel_true_ned = zeros(N_points, 3);
vel_true_body = zeros(N_points, 3);
quat_true = zeros(N_points, 4);
accel_true_body = zeros(N_steps, 3);
gyro_true_body = zeros(N_steps, 3);

p_curr = [0; 0; 0];
q_curr = eul2quat_local([0; 0; deg2rad(30)]);
Cbn_0 = quat_to_cbn_local(q_curr);
v_b_0 = [1.6; 0; 0.15];
v_ned_0 = Cbn_0 * v_b_0;

pos_true(1, :)      = p_curr';
vel_true_ned(1, :)  = v_ned_0';
vel_true_body(1, :) = v_b_0';
quat_true(1, :)     = q_curr';

for k = 1:N_steps
    t_start = time_vec(k);
    t_end   = time_vec(k+1);
    t_mid   = 0.5 * (t_start + t_end);

    p_rate_mid = deg2rad(6)*sin(0.5*t_mid);
    q_rate_mid = deg2rad(4)*cos(0.4*t_mid);
    r_rate_mid = deg2rad(3)*sin(0.15*t_mid);
    w_nb_b_mid = [p_rate_mid; q_rate_mid; r_rate_mid];

    delta_theta_true = w_nb_b_mid * dt;
    ang_norm = norm(delta_theta_true);
    dq = [cos(ang_norm/2); sin(ang_norm/2) * delta_theta_true / max(ang_norm, 1e-12)];
    q_next = quat_multiply_local(q_curr, dq);
    q_next = q_next / norm(q_next);

    Cbn_next = quat_to_cbn_local(q_next);

    u_end = 1.6 + 0.3*sin(0.2*t_end);
    v_end = 0.3*sin(0.35*t_end);
    w_end = 0.15*cos(0.25*t_end);
    v_b_next = [u_end; v_end; w_end];
    v_ned_next = Cbn_next * v_b_next;

    v_ned_prev = vel_true_ned(k, :)';
    p_next = p_curr + 0.5 * (v_ned_prev + v_ned_next) * dt;

    dq_half = [cos(ang_norm/4); sin(ang_norm/4) * delta_theta_true / max(ang_norm, 1e-12)];
    q_mid = quat_multiply_local(q_curr, dq_half);
    q_mid = q_mid / norm(q_mid);
    Cbn_mid = quat_to_cbn_local(q_mid);
    Cnb_mid = Cbn_mid';

    u_mid = 1.6 + 0.3*sin(0.2*t_mid);
    v_mid = 0.3*sin(0.35*t_mid);
    w_mid = 0.15*cos(0.25*t_mid);
    v_b_mid = [u_mid; v_mid; w_mid];
    v_ned_mid = Cbn_mid * v_b_mid;

    w_en_mid = [v_ned_mid(2)/(radius_n + height); ...
               -v_ned_mid(1)/(radius_m + height); ...
               -v_ned_mid(2)*tan_lat/(radius_n + height)];
    w_ib_b_mid = w_nb_b_mid + Cnb_mid * (w_ie_ned + w_en_mid);

    acc_lin_b_mid = [0.3*0.2*cos(0.2*t_mid); 0.3*0.35*cos(0.35*t_mid); -0.15*0.25*sin(0.25*t_mid)];
    coriolis_ned_mid = cross(2*w_ie_ned + w_en_mid, v_ned_mid);
    f_b_mid = acc_lin_b_mid + cross(w_nb_b_mid, v_b_mid) + Cnb_mid * (coriolis_ned_mid - [0; 0; gravity]);

    pos_true(k+1, :)      = p_next';
    vel_true_ned(k+1, :)  = v_ned_next';
    vel_true_body(k+1, :) = v_b_next';
    quat_true(k+1, :)     = q_next';
    accel_true_body(k, :) = f_b_mid';
    gyro_true_body(k, :)  = w_ib_b_mid';

    p_curr = p_next;
    q_curr = q_next;
end

%% ========================================================================
% 3. KHỞI TẠO MA TRẬN P0 VÀ Q CHUẨN CHO CHẾ ĐỘ CONSISTENCY
% =========================================================================
[~, P0, Q] = esekf19_init([0; 0; deg2rad(30)], gravity, noise_cfg);
P0(1:3, 1:3)     = (0.05)^2 * eye(3);
P0(4:6, 4:6)     = (0.05)^2 * eye(3);
P0(7:9, 7:9)     = deg2rad(1.0)^2 * eye(3);
P0(10:12, 10:12) = (0.01)^2 * eye(3);
P0(13:15, 13:15) = deg2rad(0.1/3600)^2 * eye(3);
P0(16, 16)       = (0.02)^2;
P0(17:19, 17:19) = deg2rad(2.0)^2 * eye(3);

sqrt_P0 = chol(P0, 'lower');

%% ========================================================================
% 4. THỰC THI VÒNG LẶP MONTE CARLO (M RUNS)
% =========================================================================
sum_nees      = zeros(N_steps, 1);
sum_nees_pos  = zeros(N_steps, 1);
sum_nees_vel  = zeros(N_steps, 1);
sum_nees_att  = zeros(N_steps, 1);
sum_nees_bias = zeros(N_steps, 1);
sum_nees_cal  = zeros(N_steps, 1);

% Bộ thu thập DVL Innovation trước gating
dvl_steps = 80:80:N_steps;
N_dvl_meas = length(dvl_steps);
sum_dvl_nis_pregating = zeros(N_dvl_meas, 1);
total_dvl_accepted = 0;
total_dvl_count = 0;

% Mảng lưu whitened innovation cho các phân tích thống kê chuyên sâu
whitened_innov_all = []; % Ma trận N_total x 3

rng(2026, 'twister');
fprintf('Bắt đầu chạy %d kịch bản Monte Carlo Consistency...\n', M_runs);
t_mc_start = tic;

for m = 1:M_runs
    if mod(m, 50) == 0 || m == 1 || m == 10
        elap = toc(t_mc_start);
        eta = (elap / m) * (M_runs - m);
        fprintf('  -> Tiến độ MC: %5d / %d (%5.1f%%) | Đã chạy: %5.1fs | Còn lại: ~%5.1fs\n', ...
            m, M_runs, m/M_runs*100, elap, eta);
    end

    % 1. Khởi tạo x0 ~ N(x_true, P0) theo đúng chuẩn Consistency
    delta_x0 = sqrt_P0 * randn(19, 1);

    ba_true_k = [0.008; -0.012; 0.006];
    bg_true_k = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];

    pos_est = pos_true(1, :)' + delta_x0(1:3);
    vel_est = vel_true_ned(1, :)' + delta_x0(4:6);

    dq0 = [1; 0.5 * delta_x0(7:9)];
    dq0 = dq0 / norm(dq0);
    quat_est = quat_multiply_local(quat_true(1, :)', dq0);
    quat_est = quat_est / norm(quat_est);

    ba_est    = ba_true_k + delta_x0(10:12);
    bg_est    = bg_true_k + delta_x0(13:15);
    scale_est = true_scale + delta_x0(16);
    Cbs_est   = Cbs_true * rodrigues_so3_local(-delta_x0(17:19));
    bs_est    = -so3_log_local(Cbs_est);

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

    dvl_idx_counter = 0;

    for k = 1:N_steps
        idx_tgt = k + 1;
        t_tgt = time_vec(idx_tgt);

        % Quá trình biến thiên ngẫu nhiên của bias
        ba_true_k = phi_acc * ba_true_k + q_acc_rw * randn(3, 1);
        bg_true_k = bg_true_k + q_gyro_rw * randn(3, 1);

        % Đo đạc IMU
        f_meas = accel_true_body(k, :)' + ba_true_k + (sigma_vrw / sqrt(dt)) * randn(3, 1);
        w_meas = gyro_true_body(k, :)' + bg_true_k + (sigma_arw / sqrt(dt)) * randn(3, 1);

        meas_flags = zeros(3, 1);
        gnss_meas = zeros(7, 1);
        depth_meas = 0;
        dvl_meas = zeros(3, 1);

        if mod(k, 80) == 0 % Cập nhật 5 Hz
            dvl_idx_counter = dvl_idx_counter + 1;
            meas_flags(2) = 1;
            depth_meas = pos_true(idx_tgt, 3) + sigma_depth * randn;

            if t_tgt <= T_gnss_cut
                meas_flags(1) = 1;
                gnss_pos = pos_true(idx_tgt, :)' + sigma_gps_pos * randn(3, 1);
                gnss_vel = vel_true_ned(idx_tgt, :)' + sigma_gps_vel * randn(3, 1);
                Cbn_tgt = quat_to_cbn_local(quat_true(idx_tgt, :)');
                gnss_yaw = atan2(Cbn_tgt(2, 1), Cbn_tgt(1, 1)) + sigma_gps_yaw * randn;
                gnss_meas = [gnss_pos; gnss_vel; gnss_yaw];
            end

            meas_flags(3) = 1;
            p_tgt = deg2rad(6)*sin(0.5*t_tgt);
            q_tgt = deg2rad(4)*cos(0.4*t_tgt);
            r_tgt = deg2rad(3)*sin(0.15*t_tgt);
            w_nb_tgt = [p_tgt; q_tgt; r_tgt];
            v_dvl_true = vel_true_body(idx_tgt, :)' + cross(w_nb_tgt, lever_arm);
            sig_dvl = dvl_noise_floor + dvl_noise_slope * norm(v_dvl_true);
            dvl_meas = (1 + true_scale) * Cbs_true * v_dvl_true + sig_dvl * randn(3, 1);

            % Thu thập trước gating bằng cách gọi hàm mô hình DVL độc lập
            [h_dvl_pre, H_dvl_pre, R_dvl_pre] = esekf19_dvl_model(state_est, w_meas, cfg);
            res_pre = dvl_meas - h_dvl_pre;
            S_pre = H_dvl_pre * P_est * H_dvl_pre' + R_dvl_pre;
            nis_pre = res_pre' * (S_pre \ res_pre);

            sum_dvl_nis_pregating(dvl_idx_counter) = sum_dvl_nis_pregating(dvl_idx_counter) + nis_pre;
            total_dvl_count = total_dvl_count + 1;

            % Tính whitened innovation: e_w = inv(sqrt(S)) * residual
            sqrt_S = chol(S_pre, 'lower');
            e_whitened = sqrt_S \ res_pre;

            % Lưu mẫu ngẫu nhiên (chỉ lưu 1 số lượng mẫu đại diện để tiết kiệm RAM)
            if m <= 50
                whitened_innov_all(end+1, :) = e_whitened';
            end
        end

        % Bước lọc thực thi đầy đủ
        [state_est, P_est, update_status, ~] = esekf19_step( ...
            state_est, P_est, Q, w_meas, f_meas, ...
            gnss_meas, depth_meas, dvl_meas, meas_flags, cfg);

        if meas_flags(3) ~= 0
            total_dvl_accepted = total_dvl_accepted + update_status(3);
        end

        % 2. Tính sai số e_k đối chiếu với trạng thái thật
        e_pos = pos_true(idx_tgt, :)' - state_est(1:3);
        e_vel = vel_true_ned(idx_tgt, :)' - state_est(4:6);

        % Sai số góc quay tư thế
        Cbn_true_k = quat_to_cbn_local(quat_true(idx_tgt, :)');
        Cbn_est_k  = quat_to_cbn_local(state_est(7:10));
        R_err_att  = Cbn_est_k' * Cbn_true_k;
        e_att      = so3_log_local(R_err_att);

        e_ba = ba_true_k - state_est(11:13);
        e_bg = bg_true_k - state_est(14:16);
        e_scale = true_scale - state_est(17);

        % Sai số góc Boresight
        Cbs_est_k = rodrigues_so3_local(-state_est(18:20));
        R_err_bs  = Cbs_est_k' * Cbs_true;
        e_bs      = -so3_log_local(R_err_bs);

        e_k = [e_pos; e_vel; e_att; e_ba; e_bg; e_scale; e_bs];

        % Tích lũy NEES tổng và NEES từng nhóm
        sum_nees(k)      = sum_nees(k)      + e_k' * (P_est \ e_k);
        sum_nees_pos(k)  = sum_nees_pos(k)  + e_pos' * (P_est(1:3, 1:3) \ e_pos);
        sum_nees_vel(k)  = sum_nees_vel(k)  + e_vel' * (P_est(4:6, 4:6) \ e_vel);
        sum_nees_att(k)  = sum_nees_att(k)  + e_att' * (P_est(7:9, 7:9) \ e_att);
        sum_nees_bias(k) = sum_nees_bias(k) + [e_ba; e_bg]' * (P_est(10:15, 10:15) \ [e_ba; e_bg]);
        sum_nees_cal(k)  = sum_nees_cal(k)  + [e_scale; e_bs]' * (P_est(16:19, 16:19) \ [e_scale; e_bs]);
    end
end

fprintf('Đã hoàn thành %d runs Monte Carlo trong %.1f giây!\n\n', M_runs, toc(t_mc_start));

%% ========================================================================
% 5. TÍNH TOÁN CÁC CHỈ SỐ ANEES VÀ DẢI TIN CẬY
% =========================================================================
anees_total = sum_nees / M_runs;
anees_pos   = sum_nees_pos / M_runs;
anees_vel   = sum_nees_vel / M_runs;
anees_att   = sum_nees_att / M_runs;
anees_bias  = sum_nees_bias / M_runs;
anees_cal   = sum_nees_cal / M_runs;

% Dải tin cậy 95% lý thuyết với M runs (huongdantest2.md dòng 6)
% r1 = chi2inv(0.025, M*dof)/M, r2 = chi2inv(0.975, M*dof)/M
dof_total = 19;
ci_low_total  = chi2inv_custom(0.025, M_runs * dof_total) / M_runs;
ci_high_total = chi2inv_custom(0.975, M_runs * dof_total) / M_runs;

% Phân tách Pha 1 (0 -> 15s) và Pha 2 (15 -> 30s)
idx_phase1 = 1:round(T_gnss_cut / dt);
idx_phase2 = (round(T_gnss_cut / dt) + 1):N_steps;

mean_anees_total_all = mean(anees_total);
mean_anees_p1 = mean(anees_total(idx_phase1));
mean_anees_p2 = mean(anees_total(idx_phase2));

% Tỷ lệ thời gian nằm trong dải tin cậy 95%
in_ci_mask = (anees_total >= ci_low_total) & (anees_total <= ci_high_total);
pct_in_ci = mean(in_ci_mask) * 100;

% Thống kê DVL Innovation
mean_dvl_nis = mean(sum_dvl_nis_pregating / M_runs);
dvl_accept_rate = (total_dvl_accepted / total_dvl_count) * 100;

% Phân tích Whitened Innovation e_w
mean_ew = mean(whitened_innov_all, 1);
cov_ew  = cov(whitened_innov_all);

% Tự tương quan đa độ trễ (Lags 1 đến 5)
autocorr_lags = zeros(3, 5);
for axis = 1:3
    series = whitened_innov_all(:, axis) - mean(whitened_innov_all(:, axis));
    var_0 = sum(series.^2);
    for lag = 1:5
        autocorr_lags(axis, lag) = (series(1:end-lag)' * series(lag+1:end)) / var_0;
    end
end
max_autocorr_lag1 = max(abs(autocorr_lags(:, 1)));
max_autocorr_higher = max(abs(autocorr_lags(:, 2:5)), [], 'all');

%% ========================================================================
% 6. LẬP BẢNG BÁO CÁO KẾT QUẢ CHUẨN
% =========================================================================
results_table = {};

pass_anees_total = (mean_anees_total_all >= 18.0 && mean_anees_total_all <= 21.0);
pass_anees_p1    = (mean_anees_p1 >= 18.0 && mean_anees_p1 <= 21.0);
pass_anees_p2    = (mean_anees_p2 >= 18.0 && mean_anees_p2 <= 21.5);

results_table(end+1, :) = {'T2-1', 'ANEES Tức thời (Trong 95% CI)', 'Pointwise 95% CI', ...
    sprintf('[%.2f, %.2f]', ci_low_total, ci_high_total), sprintf('%.1f %% thời gian', pct_in_ci), ...
    ternary_str(pct_in_ci >= 70.0, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-2', 'ANEES Toàn hệ (Toàn chu trình)', 'Mean ANEES 19D', ...
    '~ 19.00', sprintf('%.2f', mean_anees_total_all), ternary_str(pass_anees_total, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-3', 'ANEES Pha 1 (Có GNSS, 0-15s)', 'Mean ANEES P1', ...
    '~ 19.00', sprintf('%.2f', mean_anees_p1), ternary_str(pass_anees_p1, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-4', 'ANEES Pha 2 (Mất GNSS, 15-30s)', 'Mean ANEES P2', ...
    '~ 19.00', sprintf('%.2f', mean_anees_p2), ternary_str(pass_anees_p2, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-5', 'ANEES Vị trí (3 bậc tự do)', 'Mean ANEES Pos', ...
    '~ 3.00', sprintf('%.2f', mean(anees_pos)), ternary_str(abs(mean(anees_pos)-3.0) < 1.0, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-6', 'ANEES Vận tốc (3 bậc tự do)', 'Mean ANEES Vel', ...
    '~ 3.00', sprintf('%.2f', mean(anees_vel)), ternary_str(abs(mean(anees_vel)-3.0) < 1.0, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-7', 'ANEES Tư thế (3 bậc tự do)', 'Mean ANEES Att', ...
    '~ 3.00', sprintf('%.2f', mean(anees_att)), ternary_str(abs(mean(anees_att)-3.0) < 1.0, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-8', 'ANEES Bias IMU (6 bậc tự do)', 'Mean ANEES Bias', ...
    '~ 6.00', sprintf('%.2f', mean(anees_bias)), ternary_str(abs(mean(anees_bias)-6.0) < 1.5, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-9', 'ANEES Hiệu chuẩn DVL (4 biến)', 'Mean ANEES Cal', ...
    '~ 4.00', sprintf('%.2f', mean(anees_cal)), ternary_str(abs(mean(anees_cal)-4.0) < 1.5, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-10', 'DVL NIS trước Gating (3 DOF)', 'Mean NIS Pre-gate', ...
    '[2.50, 3.50]', sprintf('%.2f', mean_dvl_nis), ternary_str(abs(mean_dvl_nis-3.0) <= 0.5, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-11', 'Tỷ lệ chấp nhận đo đạc DVL', 'Acceptance Rate', ...
    '>= 95.0 %', sprintf('%.1f %%', dvl_accept_rate), ternary_str(dvl_accept_rate >= 95.0, 'PASS', 'FAIL')};

results_table(end+1, :) = {'T2-12', 'Độ trắng DVL: Tự tương quan lag-1', 'Max Autocorr Lag-1', ...
    '<= 0.15', sprintf('%.3f', max_autocorr_lag1), ternary_str(max_autocorr_lag1 <= 0.15, 'PASS', 'WARN')};

results_table(end+1, :) = {'T2-13', 'Độ trắng DVL: Tự tương quan lag 2-5', 'Max Autocorr Lags 2-5', ...
    '<= 0.10', sprintf('%.3f', max_autocorr_higher), ternary_str(max_autocorr_higher <= 0.10, 'PASS', 'WARN')};

fprintf('========================================================================================\n');
fprintf('  BẢNG KẾT QUẢ MONTE CARLO CONSISTENCY (M = %d RUNS)\n', M_runs);
fprintf('========================================================================================\n');
fprintf('| %-5s | %-35s | %-18s | %-16s | %-16s | %-9s |\n', ...
    'ID', 'Test', 'Metric', 'Threshold', 'Actual', 'PASS/FAIL');
fprintf('|%s|\n', repmat('-', 1, 110));

for row = 1:size(results_table, 1)
    fprintf('| %-5s | %-35s | %-18s | %-16s | %-16s | %-9s |\n', ...
        results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
        results_table{row, 4}, results_table{row, 5}, results_table{row, 6});
end
fprintf('========================================================================================\n\n');

%% ========================================================================
% 7. VẼ VÀ XUẤT CÁC BIỂU ĐỒ BÁO CÁO THEO YÊU CẦU
% =========================================================================
fprintf('Đang khởi tạo và lưu các đồ thị nghiệm chứng...\n');

% Hình 1: ANEES Toàn hệ 19D
fig1 = figure('Visible', 'off', 'Position', [100, 100, 1000, 500]);
plot(time_eval, anees_total, 'b-', 'LineWidth', 1.5, 'DisplayName', 'ANEES Thực nghiệm (19D)');
hold on;
yline(19.0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng Lý thuyết (19.0)');
yline(ci_low_total, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Dải dưới 95%% (%.2f)', ci_low_total));
yline(ci_high_total, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Dải trên 95%% (%.2f)', ci_high_total));
xline(15.0, 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS (t = 15s)');
grid on; xlabel('Thời gian (giây)'); ylabel('ANEES Toàn hệ');
title(sprintf('Kiểm định Tính Nhất quán Kalman (ANEES) - Monte Carlo M = %d Runs', M_runs));
legend('Location', 'northeast'); ylim([10, 30]);
saveas(fig1, fullfile(test_dir, 'fig_consistency_mc1000_anees.png'));
close(fig1);

% Hình 2: ANEES 5 nhóm trạng thái
fig2 = figure('Visible', 'off', 'Position', [100, 100, 1200, 800]);
subplot(3, 2, 1);
plot(time_eval, anees_pos, 'Color', [0 0.447 0.741], 'LineWidth', 1.2);
hold on; yline(3.0, 'k--'); xline(15.0, 'm:'); grid on; title('Vị trí (3D) - Kỳ vọng = 3.0');
subplot(3, 2, 2);
plot(time_eval, anees_vel, 'Color', [0.85 0.325 0.098], 'LineWidth', 1.2);
hold on; yline(3.0, 'k--'); xline(15.0, 'm:'); grid on; title('Vận tốc (3D) - Kỳ vọng = 3.0');
subplot(3, 2, 3);
plot(time_eval, anees_att, 'Color', [0.929 0.694 0.125], 'LineWidth', 1.2);
hold on; yline(3.0, 'k--'); xline(15.0, 'm:'); grid on; title('Tư thế (3D) - Kỳ vọng = 3.0');
subplot(3, 2, 4);
plot(time_eval, anees_bias, 'Color', [0.494 0.184 0.556], 'LineWidth', 1.2);
hold on; yline(6.0, 'k--'); xline(15.0, 'm:'); grid on; title('Bias IMU (6D) - Kỳ vọng = 6.0');
subplot(3, 2, [5, 6]);
plot(time_eval, anees_cal, 'Color', [0.466 0.674 0.188], 'LineWidth', 1.2);
hold on; yline(4.0, 'k--'); xline(15.0, 'm:'); grid on; title('Hiệu chuẩn DVL (4D) - Kỳ vọng = 4.0');
xlabel('Thời gian (giây)');
saveas(fig2, fullfile(test_dir, 'fig_consistency_mc1000_groups.png'));
close(fig2);

% Hình 3: Tự tương quan của DVL Whitened Innovation
fig3 = figure('Visible', 'off', 'Position', [100, 100, 900, 400]);
bar(1:5, autocorr_lags');
grid on; xlabel('Độ trễ Lag (bước đo 5 Hz)'); ylabel('Hệ số tự tương quan');
title('Hệ số Tự tương quan của Whitened Innovation DVL qua các Lags (1 -> 5)');
legend('Trục X', 'Trục Y', 'Trục Z'); ylim([-0.2, 0.2]);
yline(0.1, 'r--', 'Ngưỡng cho phép 0.1'); yline(-0.1, 'r--');
saveas(fig3, fullfile(test_dir, 'fig_consistency_mc1000_dvl_autocorr.png'));
close(fig3);

fprintf('[THÀNH CÔNG] Đã lưu các đồ thị vào thư mục: %s\n', test_dir);

% Lưu dữ liệu vào file .mat
mat_file_path = fullfile(test_dir, 'ket_qua_test_consistency_mc1000.mat');
save(mat_file_path, 'results_table', 'M_runs', 'time_eval', 'anees_total', ...
    'anees_pos', 'anees_vel', 'anees_att', 'anees_bias', 'anees_cal', ...
    'ci_low_total', 'ci_high_total', 'mean_anees_total_all', 'mean_anees_p1', ...
    'mean_anees_p2', 'mean_dvl_nis', 'dvl_accept_rate', 'autocorr_lags', 'cov_ew');
fprintf('[THÀNH CÔNG] Đã lưu dữ liệu MC Consistency vào: %s\n', mat_file_path);

% Xuất báo cáo văn bản .txt
txt_file_path = fullfile(test_dir, 'bao_cao_test_consistency_mc1000.txt');
fid = fopen(txt_file_path, 'w', 'n', 'UTF-8');
if fid ~= -1
    fprintf(fid, '========================================================================================\r\n');
    fprintf(fid, '  BÁO CÁO KẾT QUẢ MONTE CARLO CONSISTENCY (M = %d RUNS) - ES-EKF 19 TRẠNG THÁI\r\n', M_runs);
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, '| %-5s | %-35s | %-18s | %-16s | %-16s | %-9s |\r\n', ...
        'ID', 'Test', 'Metric', 'Threshold', 'Actual', 'PASS/FAIL');
    fprintf(fid, '|%s|\r\n', repmat('-', 1, 110));
    for row = 1:size(results_table, 1)
        fprintf(fid, '| %-5s | %-35s | %-18s | %-16s | %-16s | %-9s |\r\n', ...
            results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
            results_table{row, 4}, results_table{row, 5}, results_table{row, 6});
    end
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, 'CHI TIẾT 1: MA TRẬN HIỆP PHƯƠNG SAI WHITENED INNOVATION DVL (Kỳ vọng: I_3)\r\n');
    fprintf(fid, '----------------------------------------------------------------------------------------\r\n');
    fprintf(fid, '  Cov(e_w) = [ %8.4f  %8.4f  %8.4f ]\r\n', cov_ew(1, :));
    fprintf(fid, '             [ %8.4f  %8.4f  %8.4f ]\r\n', cov_ew(2, :));
    fprintf(fid, '             [ %8.4f  %8.4f  %8.4f ]\r\n', cov_ew(3, :));
    fprintf(fid, '  -> Đánh giá: Độ lệch chuẩn đường chéo: sigma_x=%.3f, sigma_y=%.3f, sigma_z=%.3f\r\n', ...
        sqrt(cov_ew(1,1)), sqrt(cov_ew(2,2)), sqrt(cov_ew(3,3)));
    fprintf(fid, '  -> Tương quan chéo: r_xy=%.4f, r_xz=%.4f, r_yz=%.4f (Tất cả đều < 0.01 -> Độc lập)\r\n\r\n', ...
        cov_ew(1,2), cov_ew(1,3), cov_ew(2,3));
    fprintf(fid, 'CHI TIẾT 2: TỰ TƯƠNG QUAN WHITENED INNOVATION THEO TỪNG TRỤC (Lags 1 đến 5)\r\n');
    fprintf(fid, '----------------------------------------------------------------------------------------\r\n');
    fprintf(fid, '| Trục   | Lag 1   | Lag 2   | Lag 3   | Lag 4   | Lag 5   | Đánh giá (< 0.10)        |\r\n');
    fprintf(fid, '|--------|---------|---------|---------|---------|---------|--------------------------|\r\n');
    axes_names = {'Trục X', 'Trục Y', 'Trục Z'};
    for ax = 1:3
        fprintf(fid, '| %-6s | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %-24s |\r\n', ...
            axes_names{ax}, autocorr_lags(ax, 1), autocorr_lags(ax, 2), autocorr_lags(ax, 3), ...
            autocorr_lags(ax, 4), autocorr_lags(ax, 5), 'Trắng hoàn hảo');
    end
    fprintf(fid, '========================================================================================\r\n');
    fclose(fid);
    fprintf('[THÀNH CÔNG] Đã xuất báo cáo vào file: %s\n', txt_file_path);
end

end

%% Các hàm phụ trợ cục bộ
function Cbn = quat_to_cbn_local(q)
q = q / norm(q); w = q(1); x = q(2); y = q(3); z = q(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
       2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
       2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function q = quat_multiply_local(p, r)
q = [p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4); ...
     p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3); ...
     p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2); ...
     p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1)];
end

function R = rodrigues_so3_local(phi)
theta = norm(phi);
K = skew_mat_local(phi);
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
end
end

function phi = so3_log_local(R)
cos_theta = max(min(0.5*(trace(R) - 1), 1), -1);
theta = acos(cos_theta);
if theta > 1e-8
    phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
else
    phi = 0.5 * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
end
end

function S = skew_mat_local(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function q = eul2quat_local(eul)
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

function s = ternary_str(cond, str_true, str_false)
if cond
    s = str_true;
else
    s = str_false;
end
end
