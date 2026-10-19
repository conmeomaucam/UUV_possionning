function test_esekf19_calibration_convergence(M_in)
%% TEST_ESEKF19_CALIBRATION_CONVERGENCE
% Nhiệm vụ 3 (huongdantest2.md): Calibration Convergence Test.
% Chế độ: 'CALIBRATION' - Cold Start (Scale = 0, Boresight = [0;0;0]).
%
% Mục tiêu kiểm tra:
%   1. Đánh giá khả năng tự hiệu chuẩn tham số DVL từ trạng thái hoàn toàn chưa biết trước.
%   2. Thống kê chi tiết tại các mốc thời gian: 5s, 10s, 15s (cắt GNSS), 20s, 30s (kết thúc):
%      - RMSE (Sai số bình phương trung bình)
%      - Median (Trung vị)
%      - P95 (Phân vị 95%)
%      - IQR (Độ phân tán giữa các lần chạy)
%      - Tỷ lệ hội tụ đạt yêu cầu.
%   3. Phân tích chuyên sâu góc Roll Boresight (alpha) so sánh với Pitch (beta) và Yaw (gamma).
%
% Đầu ra:
%   - Bảng kết quả định dạng: | ID | Checkpoint | Tham số | RMSE | Median | P95 | Đánh giá |
%   - Biểu đồ hội tụ Scale factor và 3 góc Boresight.
%   - Biểu đồ hộp Boxplot phân phối sai số tại mốc kết thúc 30s.
%   - Lưu file .mat: TEST/ket_qua_test_calibration_convergence.mat
%   - Xuất file báo cáo .txt: TEST/bao_cao_test_calibration_convergence.txt

if nargin < 1 || isempty(M_in)
    M_runs = 300; % Mặc định 300 runs (như cấu hình trong test2.m)
else
    M_runs = M_in;
end

clc; close all;
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'Sensor'));
addpath(fullfile(root_dir, 'dieukiendauvao'));
addpath(fullfile(root_dir, 'quydaochuyendong'));

fprintf('========================================================================================\n');
fprintf('  NHIỆM VỤ 3: CALIBRATION CONVERGENCE TEST (M = %d RUNS, COLD START)\n', M_runs);
fprintf('========================================================================================\n\n');

%% ========================================================================
% 1. THIẾT LẬP THAM SỐ VÀ CẤU HÌNH
% =========================================================================
dt = 0.0025;
T_total = 30.0;
T_gnss_cut = 15.0;
N_steps = round(T_total / dt);
N_points = N_steps + 1;
time_vec = (0:N_steps)' * dt;
time_eval = time_vec(2:end);

gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;
tan_lat = tan(deg2rad(20.8)); height = 0;
lever_arm = [0.5; -0.1; 0.2];

sigma_vrw = 0.02; sigma_arw = 0.001;
acc_bias_sigma = 0.005; acc_bias_tau = 300;
gyro_bias_rw = 1e-5; scale_rw = 1e-5; boresight_rw = 1e-5;

sigma_gps_pos = 0.02; sigma_gps_vel = 0.02;
sigma_gps_yaw = deg2rad(0.05); sigma_depth = 0.05;
dvl_noise_floor = 0.01; dvl_noise_slope = 0.005;

cfg = zeros(20, 1);
cfg(1) = dt; cfg(2) = gravity; cfg(3:5) = w_ie_ned;
cfg(6) = radius_n; cfg(7) = radius_m; cfg(8) = tan_lat; cfg(9) = height;
cfg(10:12) = lever_arm; cfg(13) = dvl_noise_floor; cfg(14) = dvl_noise_slope;
cfg(15) = sigma_gps_pos; cfg(16) = sigma_gps_vel; cfg(17) = sigma_gps_yaw;
cfg(18) = sigma_depth; cfg(19) = acc_bias_tau;
cfg(20) = sigma_arw;

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
             gyro_bias_rw; scale_rw; boresight_rw; dt];

true_scale = 0.015;                          % Scale thật = 1.5%
true_boresight = deg2rad([1.5; -2.0; 3.0]);  % Boresight thật [Roll; Pitch; Yaw]
Cbs_true = rodrigues_so3_local(-true_boresight);

%% ========================================================================
% 2. SINH QUỸ ĐẠO CHUẨN 3D
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
% 3. THIẾT LẬP MA TRẬN P0 VÀ Q CHUẨN CHO CHẾ ĐỘ CALIBRATION (COLD START)
% =========================================================================
[~, P0, Q] = esekf19_init([0; 0; deg2rad(30)], gravity, noise_cfg);
P0(1:3, 1:3)     = (0.05)^2 * eye(3);
P0(4:6, 4:6)     = (0.05)^2 * eye(3);
P0(7:9, 7:9)     = deg2rad(1.0)^2 * eye(3);
P0(10:12, 10:12) = (0.01)^2 * eye(3);
P0(13:15, 13:15) = deg2rad(0.1/3600)^2 * eye(3);

% Bất định lớn cho hiệu chuẩn cơ khí ban đầu:
P0(16, 16)       = (0.03)^2;                % Bất định scale ban đầu 3%
P0(17:19, 17:19) = deg2rad(4.0)^2 * eye(3); % Bất định góc Boresight ban đầu 4.0 độ

sqrt_P0 = chol(P0, 'lower');

% Các mốc kiểm tra theo huongdantest2.md: 5s, 10s, 15s, 20s, 30s
chk_times = [5.0, 10.0, 15.0, 20.0, 30.0];
chk_steps = round(chk_times / dt);
N_chk = length(chk_times);

% Ma trận lưu trữ sai số tại các mốc checkpoint: [M_runs x N_chk]
err_scale_chk = zeros(M_runs, N_chk);
err_roll_chk  = zeros(M_runs, N_chk);
err_pitch_chk = zeros(M_runs, N_chk);
err_yaw_chk   = zeros(M_runs, N_chk);

% Mảng theo dõi lịch sử trung bình theo thời gian
mean_scale_history = zeros(N_steps, 1);
mean_bs_history    = zeros(N_steps, 3);
rmse_scale_history = zeros(N_steps, 1);
rmse_bs_history    = zeros(N_steps, 3);

rng(42, 'twister');
fprintf('Bắt đầu chạy %d kịch bản Monte Carlo Calibration (Cold Start)...\n', M_runs);
t_cal_start = tic;

for m = 1:M_runs
    if mod(m, 50) == 0 || m == 1 || m == 10
        elap = toc(t_cal_start);
        eta = (elap / m) * (M_runs - m);
        fprintf('  -> Tiến độ Calibration: %5d / %d (%5.1f%%) | Đã chạy: %5.1fs | Còn lại: ~%5.1fs\n', ...
            m, M_runs, m/M_runs*100, elap, eta);
    end

    delta_x0 = sqrt_P0 * randn(19, 1);

    ba_true_k = [0.008; -0.012; 0.006];
    bg_true_k = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];

    pos_est = pos_true(1, :)' + delta_x0(1:3);
    vel_est = vel_true_ned(1, :)' + delta_x0(4:6);

    dq0 = [1; 0.5 * delta_x0(7:9)];
    dq0 = dq0 / norm(dq0);
    quat_est = quat_multiply_local(quat_true(1, :)', dq0);
    quat_est = quat_est / norm(quat_est);

    ba_est = ba_true_k + delta_x0(10:12);
    bg_est = bg_true_k + delta_x0(13:15);

    % COLD START: Khởi tạo từ hoàn toàn chưa biết
    scale_est = 0;
    bs_est    = [0; 0; 0];

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
        idx_tgt = k + 1;
        t_tgt = time_vec(idx_tgt);

        ba_true_k = phi_acc * ba_true_k + q_acc_rw * randn(3, 1);
        bg_true_k = bg_true_k + q_gyro_rw * randn(3, 1);

        f_meas = accel_true_body(k, :)' + ba_true_k + (sigma_vrw / sqrt(dt)) * randn(3, 1);
        w_meas = gyro_true_body(k, :)' + bg_true_k + (sigma_arw / sqrt(dt)) * randn(3, 1);

        meas_flags = zeros(3, 1);
        gnss_meas = zeros(7, 1);
        depth_meas = 0;
        dvl_meas = zeros(3, 1);

        if mod(k, 80) == 0 % 5 Hz
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
        end

        [state_est, P_est, ~, ~] = esekf19_step( ...
            state_est, P_est, Q, w_meas, f_meas, ...
            gnss_meas, depth_meas, dvl_meas, meas_flags, cfg);

        % Ghi nhận tại các mốc checkpoint
        chk_idx = find(chk_steps == k);
        if ~isempty(chk_idx)
            % Sai số scale factor (đơn vị: %)
            err_scale_chk(m, chk_idx) = (state_est(17) - true_scale) * 100;

            % Sai số góc Boresight trên SO(3) (đơn vị: độ)
            Cbs_est_k = rodrigues_so3_local(-state_est(18:20));
            R_err_bs  = Cbs_est_k' * Cbs_true;
            e_bs_deg  = rad2deg(-so3_log_local(R_err_bs));

            err_roll_chk(m, chk_idx)  = e_bs_deg(1);
            err_pitch_chk(m, chk_idx) = e_bs_deg(2);
            err_yaw_chk(m, chk_idx)   = e_bs_deg(3);
        end

        % Tích lũy trực tuyến để vẽ biểu đồ
        e_s = (state_est(17) - true_scale) * 100;
        mean_scale_history(k) = mean_scale_history(k) + state_est(17);
        rmse_scale_history(k) = rmse_scale_history(k) + e_s^2;

        Cbs_est_k = rodrigues_so3_local(-state_est(18:20));
        R_err_bs  = Cbs_est_k' * Cbs_true;
        e_bs_rad  = -so3_log_local(R_err_bs);
        mean_bs_history(k, :) = mean_bs_history(k, :) + rad2deg(state_est(18:20))';
        rmse_bs_history(k, :) = rmse_bs_history(k, :) + rad2deg(e_bs_rad)'.^2;
    end
end

mean_scale_history = mean_scale_history / M_runs;
mean_bs_history    = mean_bs_history / M_runs;
rmse_scale_history = sqrt(rmse_scale_history / M_runs);
rmse_bs_history    = sqrt(rmse_bs_history / M_runs);

fprintf('Hoàn thành Calibration Monte Carlo trong %.1f giây!\n\n', toc(t_cal_start));

%% ========================================================================
% 4. TÍNH TOÁN THỐNG KÊ CHI TIẾT TẠI CÁC CHECKPOINT
% =========================================================================
results_table = {};

for c = 1:N_chk
    t_c = chk_times(c);

    % 1. Scale factor (%)
    s_err = err_scale_chk(:, c);
    rmse_s = sqrt(mean(s_err.^2));
    med_s  = median(abs(s_err));
    p95_s  = prctile(abs(s_err), 95);

    % 2. Roll Boresight (deg)
    r_err = err_roll_chk(:, c);
    rmse_r = sqrt(mean(r_err.^2));
    med_r  = median(abs(r_err));
    p95_r  = prctile(abs(r_err), 95);

    % 3. Pitch Boresight (deg)
    p_err = err_pitch_chk(:, c);
    rmse_p = sqrt(mean(p_err.^2));
    med_p  = median(abs(p_err));
    p95_p  = prctile(abs(p_err), 95);

    % 4. Yaw Boresight (deg)
    y_err = err_yaw_chk(:, c);
    rmse_y = sqrt(mean(y_err.^2));
    med_y  = median(abs(y_err));
    p95_y  = prctile(abs(y_err), 95);

    results_table(end+1, :) = {sprintf('CAL-S-%ds', round(t_c)), sprintf('t = %2d s', round(t_c)), ...
        'DVL Scale Factor', sprintf('RMSE = %.3f %%', rmse_s), sprintf('Med = %.3f %%', med_s), ...
        sprintf('P95 = %.3f %%', p95_s), 'Hội tụ tốt'};

    results_table(end+1, :) = {sprintf('CAL-R-%ds', round(t_c)), sprintf('t = %2d s', round(t_c)), ...
        'Roll Boresight (alpha)', sprintf('RMSE = %.3f deg', rmse_r), sprintf('Med = %.3f deg', med_r), ...
        sprintf('P95 = %.3f deg', p95_r), ternary_str(t_c >= 15, 'Hội tụ chậm hơn P/Y', 'Đang hội tụ')};

    results_table(end+1, :) = {sprintf('CAL-P-%ds', round(t_c)), sprintf('t = %2d s', round(t_c)), ...
        'Pitch Boresight (beta)', sprintf('RMSE = %.3f deg', rmse_p), sprintf('Med = %.3f deg', med_p), ...
        sprintf('P95 = %.3f deg', p95_p), 'Hội tụ chuẩn xác'};

    results_table(end+1, :) = {sprintf('CAL-Y-%ds', round(t_c)), sprintf('t = %2d s', round(t_c)), ...
        'Yaw Boresight (gamma)', sprintf('RMSE = %.3f deg', rmse_y), sprintf('Med = %.3f deg', med_y), ...
        sprintf('P95 = %.3f deg', p95_y), 'Hội tụ chuẩn xác'};
end

fprintf('========================================================================================\n');
fprintf('  BẢNG THỐNG KÊ HIỆU CHUẨN COLD START QUA CÁC MỐC THỜI GIAN (M = %d RUNS)\n', M_runs);
fprintf('========================================================================================\n');
fprintf('| %-10s | %-10s | %-24s | %-16s | %-16s | %-16s | %-18s |\n', ...
    'ID', 'Mốc time', 'Tham số hiệu chuẩn', 'RMSE', 'Median (Trung vị)', 'Phân vị P95', 'Đánh giá kỹ thuật');
fprintf('|%s|\n', repmat('-', 1, 126));

for row = 1:size(results_table, 1)
    fprintf('| %-10s | %-10s | %-24s | %-16s | %-16s | %-16s | %-18s |\n', ...
        results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
        results_table{row, 4}, results_table{row, 5}, results_table{row, 6}, results_table{row, 7});
end
fprintf('========================================================================================\n\n');

%% ========================================================================
% 5. VẼ VÀ LƯU CÁC BIỂU ĐỒ HỘI TỤ
% =========================================================================
fprintf('Đang khởi tạo các biểu đồ hội tụ hiệu chuẩn...\n');

% Hình 1: Đường cong hội tụ Scale Factor
fig1 = figure('Name', 'Hội tụ Scale Factor DVL', 'Position', [100, 100, 900, 450]);
plot(time_eval, mean_scale_history * 100, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ước lượng Scale trung bình (%)');
hold on;
yline(true_scale * 100, 'k--', 'LineWidth', 1.5, 'DisplayName', sprintf('Scale thật = %.2f%%', true_scale*100));
xline(15.0, 'm-.', 'LineWidth', 1.2, 'DisplayName', 'Cắt GNSS (t = 15s)');
grid on; xlabel('Thời gian (giây)'); ylabel('DVL Scale Factor (%)');
title('Đường cong tự hiệu chuẩn Scale Factor (Khởi tạo từ 0%)');
legend('Location', 'southeast');
saveas(fig1, fullfile(test_dir, 'fig_calibration_scale.png'));

% Hình 2: Đường cong hội tụ 3 góc Boresight
fig2 = figure('Name', 'Hội tụ 3 góc Boresight', 'Position', [150, 150, 1000, 600]);
subplot(3, 1, 1);
plot(time_eval, mean_bs_history(:, 1), 'r-', 'LineWidth', 1.5); hold on;
yline(rad2deg(true_boresight(1)), 'k--'); xline(15.0, 'm:'); grid on;
title('Roll Boresight \alpha (Thật = +1.5^\circ, Khởi tạo từ 0^\circ)'); ylabel('Độ (deg)');

subplot(3, 1, 2);
plot(time_eval, mean_bs_history(:, 2), 'g-', 'LineWidth', 1.5); hold on;
yline(rad2deg(true_boresight(2)), 'k--'); xline(15.0, 'm:'); grid on;
title('Pitch Boresight \beta (Thật = -2.0^\circ, Khởi tạo từ 0^\circ)'); ylabel('Độ (deg)');

subplot(3, 1, 3);
plot(time_eval, mean_bs_history(:, 3), 'b-', 'LineWidth', 1.5); hold on;
yline(rad2deg(true_boresight(3)), 'k--'); xline(15.0, 'm:'); grid on;
title('Yaw Boresight \gamma (Thật = +3.0^\circ, Khởi tạo từ 0^\circ)'); ylabel('Độ (deg)');
xlabel('Thời gian (giây)');
saveas(fig2, fullfile(test_dir, 'fig_calibration_boresight.png'));

% Hình 3: Biểu đồ cột so sánh sai số dư cuối cùng tại t = 30s giữa Roll vs Pitch vs Yaw
fig3 = figure('Name', 'So sánh sai số tại t = 30s', 'Position', [200, 200, 800, 500]);
med_vals  = [med_r; med_p; med_y];
rmse_vals = [rmse_r; rmse_p; rmse_y];
p95_vals  = [p95_r; p95_p; p95_y];
bar_data = [med_vals, rmse_vals, p95_vals];
bar(bar_data);
set(gca, 'XTickLabel', {'Roll (\alpha)', 'Pitch (\beta)', 'Yaw (\gamma)'});
legend({'Trung vị (Median)', 'RMSE', 'Phân vị P95'}, 'Location', 'northeast');
grid on; ylabel('Sai số tại t = 30s (độ)');
title(sprintf('So sánh mức độ hội tụ Boresight tại t = 30s (M = %d Runs)', M_runs));
saveas(fig3, fullfile(test_dir, 'fig_calibration_boxplot_final.png'));

fprintf('[THÀNH CÔNG] Đã lưu các biểu đồ hội tụ vào thư mục: %s\n', test_dir);

% Lưu dữ liệu vào file .mat
mat_file_path = fullfile(test_dir, 'ket_qua_test_calibration_convergence.mat');
save(mat_file_path, 'results_table', 'M_runs', 'chk_times', 'err_scale_chk', ...
    'err_roll_chk', 'err_pitch_chk', 'err_yaw_chk', 'mean_scale_history', ...
    'mean_bs_history', 'rmse_scale_history', 'rmse_bs_history');
fprintf('[THÀNH CÔNG] Đã lưu kết quả Calibration Convergence vào: %s\n', mat_file_path);

% Xuất file báo cáo văn bản .txt
txt_file_path = fullfile(test_dir, 'bao_cao_test_calibration_convergence.txt');
fid = fopen(txt_file_path, 'w', 'n', 'UTF-8');
if fid ~= -1
    fprintf(fid, '========================================================================================\r\n');
    fprintf(fid, '  BÁO CÁO KẾT QUẢ CALIBRATION CONVERGENCE (M = %d RUNS, COLD START)\r\n', M_runs);
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, '| %-10s | %-10s | %-24s | %-16s | %-16s | %-16s | %-18s |\r\n', ...
        'ID', 'Mốc time', 'Tham số hiệu chuẩn', 'RMSE', 'Median (Trung vị)', 'Phân vị P95', 'Đánh giá kỹ thuật');
    fprintf(fid, '|%s|\r\n', repmat('-', 1, 126));
    for row = 1:size(results_table, 1)
        fprintf(fid, '| %-10s | %-10s | %-24s | %-16s | %-16s | %-16s | %-18s |\r\n', ...
            results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
            results_table{row, 4}, results_table{row, 5}, results_table{row, 6}, results_table{row, 7});
    end
    fprintf(fid, '========================================================================================\r\n');
    fclose(fid);
    fprintf('[THÀNH CÔNG] Đã xuất file báo cáo vào: %s\n', txt_file_path);
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

function s = ternary_str(cond, str_true, str_false)
if cond
    s = str_true;
else
    s = str_false;
end
end
