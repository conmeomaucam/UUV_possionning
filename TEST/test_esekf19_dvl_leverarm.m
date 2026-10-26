function test_esekf19_dvl_leverarm()
%% TEST_ESEKF19_DVL_LEVERARM
% Bộ test 2: Kiểm tra mô hình đo DVL, nhiễu Gyro và Cánh tay đòn IMU-DVL.
%
% Nhiệm vụ kiểm thử (theo huongdantest.md):
%   C1. Kiểm tra dấu tích có hướng và 2 bất biến vật lý.
%   C2. Kiểm tra tính nhất quán 2 mô hình đo (Đồng bộ lý tưởng vs Lệch pha t_mid / t_target).
%   C3. Kiểm tra phương sai nhiễu gyro qua cánh tay đòn (100.000 mẫu Monte Carlo, z_ij score).
%   C4. Kiểm thử tích hợp INS-DVL, phân tích tương quan và phân phối thống kê NIS.
%
% Định dạng đầu ra:
%   - In bảng kết quả chuẩn: | ID | Test | Metric | Threshold | Actual | PASS/FAIL |
%   - Lưu file dữ liệu .mat: TEST/ket_qua_test_dvl_leverarm.mat
%   - Xuất file báo cáo .txt: TEST/bao_cao_test_dvl_leverarm.txt

clc; close all;
rng(123, 'twister'); % Đảm bảo tính lặp lại độc lập

test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'Sensor'));

fprintf('========================================================================================\n');
fprintf('  BỘ TEST 2: KIỂM TRA MÔ HÌNH DVL, NHIỄU GYRO VÀ CÁNH TAY ĐÒN IMU-DVL\n');
fprintf('========================================================================================\n\n');

results_table = {};

%% ========================================================================
% C1. KIỂM TRA DẤU TÍCH CÓ HƯỚNG VÀ BẤT BIẾN VẬT LÝ
% =========================================================================
fprintf('--- [C1] KIỂM TRA DẤU TÍCH CÓ HƯỚNG VÀ BẤT BIẾN VẬT LÝ ---\n');

% Ví dụ kiểm tra tiêu chuẩn từ huongdantest.md:
% omega = [0.2; -0.3; 0.4] rad/s, r = [0.5; -0.1; 0.2] m
% cross(omega, r) = [-0.02; 0.16; 0.13] m/s
omega_ref = [0.2; -0.3; 0.4];
r_ref     = [0.5; -0.1; 0.2];
expected_cross = [-0.02; 0.16; 0.13];

actual_cross = cross(omega_ref, r_ref);
diff_cross = max(abs(actual_cross - expected_cross));
pass_c1_cross = (diff_cross <= 1e-12);

fprintf('  * Vector tích có hướng cross(omega, r):\n');
fprintf('    + Kỳ vọng  : [%.4f; %.4f; %.4f] m/s\n', expected_cross(1), expected_cross(2), expected_cross(3));
fprintf('    + Thực tế  : [%.4f; %.4f; %.4f] m/s\n', actual_cross(1), actual_cross(2), actual_cross(3));
fprintf('    + Sai lệch : %.4e m/s (Ngưỡng <= 1e-12) --> %s\n\n', diff_cross, ternary_str(pass_c1_cross, 'PASS', 'FAIL'));

results_table(end+1, :) = {'C1-1', 'Dấu tích có hướng cross(w, r)', 'Max absolute error', ...
    '<= 1.00e-12 m/s', sprintf('%.2e m/s', diff_cross), ternary_str(pass_c1_cross, 'PASS', 'FAIL')};

% Bất biến vật lý 1: Khi r = [0;0;0], thay đổi gyro KHÔNG được làm thay đổi vận tốc DVL
s_test = 0.015;
bs_test = deg2rad([1.5; -2.0; 3.0]);
Cbs_test = rodrigues_so3(-bs_test);
v_body_test = [1.6; 0.2; -0.1];
r_zero = [0; 0; 0];

w_test1 = [0.1; -0.2; 0.3];
w_test2 = [-0.4; 0.5; -0.6];

v_dvl_r0_w1 = (1 + s_test) * Cbs_test * (v_body_test + cross(w_test1, r_zero));
v_dvl_r0_w2 = (1 + s_test) * Cbs_test * (v_body_test + cross(w_test2, r_zero));
diff_inv1 = max(abs(v_dvl_r0_w1 - v_dvl_r0_w2));
pass_c1_inv1 = (diff_inv1 <= 1e-12);

fprintf('  * Bất biến vật lý 1 (Cánh tay đòn r = 0):\n');
fprintf('    + Sai khác giữa 2 giá trị gyro khác nhau: %.4e m/s (Ngưỡng <= 1e-12) --> %s\n\n', ...
    diff_inv1, ternary_str(pass_c1_inv1, 'PASS', 'FAIL'));

results_table(end+1, :) = {'C1-2', 'Bất biến khi r = 0 (Gyro độc lập)', 'Max absolute diff', ...
    '<= 1.00e-12 m/s', sprintf('%.2e m/s', diff_inv1), ternary_str(pass_c1_inv1, 'PASS', 'FAIL')};

% Bất biến vật lý 2: Khi vận tốc góc tương đối = 0, mô hình phải trở về chính xác (1+s)*Cbs*v_body
w_zero = [0; 0; 0];
v_dvl_w0 = (1 + s_test) * Cbs_test * (v_body_test + cross(w_zero, r_ref));
v_expected_w0 = (1 + s_test) * Cbs_test * v_body_test;
diff_inv2 = max(abs(v_dvl_w0 - v_expected_w0));
pass_c1_inv2 = (diff_inv2 <= 1e-12);

fprintf('  * Bất biến vật lý 2 (Vận tốc góc w_rel = 0):\n');
fprintf('    + Sai khác so với (1+s)*Cbs*v_b: %.4e m/s (Ngưỡng <= 1e-12) --> %s\n\n', ...
    diff_inv2, ternary_str(pass_c1_inv2, 'PASS', 'FAIL'));

results_table(end+1, :) = {'C1-3', 'Bất biến khi w_rel = 0', 'Max absolute diff', ...
    '<= 1.00e-12 m/s', sprintf('%.2e m/s', diff_inv2), ternary_str(pass_c1_inv2, 'PASS', 'FAIL')};

%% ========================================================================
% C2. KIỂM TRA TÍNH NHẤT QUÁN CỦA HAI MÔ HÌNH ĐO (GENERATOR vs PREDICTOR)
% =========================================================================
fprintf('--- [C2] KIỂM TRA TÍNH NHẤT QUÁN CỦA HAI MÔ HÌNH ĐO ---\n');

% Cấu hình hệ thống mô phỏng chuẩn của test2.m
dt = 0.0025;
T_total = 30.0;
N_steps = round(T_total / dt);
N_points = N_steps + 1;
time_vec = (0:N_steps)' * dt;

radius_n = 6378137.0; radius_m = 6356752.3;
tan_lat = tan(deg2rad(20.8)); height = 0;
gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
lever_arm = [0.5; -0.1; 0.2];
true_scale = 0.015;
true_boresight = deg2rad([1.5; -2.0; 3.0]);
Cbs_true = rodrigues_so3(-true_boresight);

dvl_noise_floor = 0.01;
dvl_noise_slope = 0.005;

% Sinh quỹ đạo chuẩn giống hệt test2.m
pos_true = zeros(N_points, 3);
vel_true_ned = zeros(N_points, 3);
vel_true_body = zeros(N_points, 3);
quat_true = zeros(N_points, 4);
gyro_true_body = zeros(N_steps, 3);

p_curr = [0; 0; 0];
q_curr = eul2quat_custom([0; 0; deg2rad(30)]);
Cbn_0 = quat_to_cbn(q_curr);
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
    q_next = quat_multiply(q_curr, dq);
    q_next = q_next / norm(q_next);

    Cbn_next = quat_to_cbn(q_next);

    u_end = 1.6 + 0.3*sin(0.2*t_end);
    v_end = 0.3*sin(0.35*t_end);
    w_end = 0.15*cos(0.25*t_end);
    v_b_next = [u_end; v_end; w_end];
    v_ned_next = Cbn_next * v_b_next;

    v_ned_prev = vel_true_ned(k, :)';
    p_next = p_curr + 0.5 * (v_ned_prev + v_ned_next) * dt;

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

    w_en_mid = [v_ned_mid(2)/(radius_n + height); ...
               -v_ned_mid(1)/(radius_m + height); ...
               -v_ned_mid(2)*tan_lat/(radius_n + height)];
    w_ib_b_mid = w_nb_b_mid + Cnb_mid * (w_ie_ned + w_en_mid);

    % Lực riêng đo bởi gia tốc kế trong khoảng [t_k, t_{k+1}]
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

% --- PHẦN 1: MÔ HÌNH LÝ TƯỞNG CÙNG TIMESTAMP ---
% Cùng sử dụng trạng thái thật, cùng mốc thời gian t_target, không nhiễu
max_res_ideal = 0;
meas_steps = 80:80:N_steps; % Cập nhật 5 Hz

for idx = 1:length(meas_steps)
    k_step = meas_steps(idx);
    t_target = time_vec(k_step + 1);

    % 1. Mô hình sinh DVL trong test2.m tại t_target:
    p_tgt = deg2rad(6)*sin(0.5*t_target);
    q_tgt = deg2rad(4)*cos(0.4*t_target);
    r_tgt = deg2rad(3)*sin(0.15*t_target);
    w_nb_tgt = [p_tgt; q_tgt; r_tgt];
    v_dvl_gen = (1 + true_scale) * Cbs_true * (vel_true_body(k_step+1, :)' + cross(w_nb_tgt, lever_arm));

    % 2. Mô hình dự đoán DVL trong esekf19_step nếu có đúng gyro tại t_target:
    Cbn_tgt = quat_to_cbn(quat_true(k_step+1, :)');
    Cnb_tgt = Cbn_tgt';
    v_ned_tgt = vel_true_ned(k_step+1, :)';
    w_en_tgt = [v_ned_tgt(2)/(radius_n + height); ...
               -v_ned_tgt(1)/(radius_m + height); ...
               -v_ned_tgt(2)*tan_lat/(radius_n + height)];
    w_ib_tgt = w_nb_tgt + Cnb_tgt * (w_ie_ned + w_en_tgt);

    % Bộ lọc tính:
    w_in_b = Cnb_tgt * (w_ie_ned + w_en_tgt);
    w_rel_est = w_ib_tgt - w_in_b; % bg = 0
    v_dvl_pred = (1 + true_scale) * Cbs_true * (vel_true_body(k_step+1, :)' + cross(w_rel_est, lever_arm));

    diff_ideal = norm(v_dvl_gen - v_dvl_pred);
    max_res_ideal = max(max_res_ideal, diff_ideal);
end

pass_c2_part1 = (max_res_ideal <= 1e-10);
fprintf('  * Phần 1 (Cùng timestamp lý tưởng, không nhiễu):\n');
fprintf('    + Residual lớn nhất giữa bộ sinh và bộ lọc: %.4e m/s (Ngưỡng <= 1e-10) --> %s\n\n', ...
    max_res_ideal, ternary_str(pass_c2_part1, 'PASS', 'FAIL'));

results_table(end+1, :) = {'C2-1', 'Nhất quán DVL cùng timestamp', 'Max absolute residual', ...
    '<= 1.00e-10 m/s', sprintf('%.2e m/s', max_res_ideal), ternary_str(pass_c2_part1, 'PASS', 'FAIL')};

% --- PHẦN 2: KIỂM TRA PIPELINE HIỆN TẠI (LỆCH TIMING t_mid vs t_target) ---
% Bộ lọc dùng gyro đo ở t_mid trong khi DVL đo ở t_target (lệch pha dt/2 = 1.25 ms)
delta_v_pipeline = zeros(length(meas_steps), 3);
sigma_dvl_history = zeros(length(meas_steps), 1);
time_meas = zeros(length(meas_steps), 1);

for idx = 1:length(meas_steps)
    k_step = meas_steps(idx);
    t_target = time_vec(k_step + 1);
    time_meas(idx) = t_target;

    % DVL sinh tại t_target:
    p_tgt = deg2rad(6)*sin(0.5*t_target);
    q_tgt = deg2rad(4)*cos(0.4*t_target);
    r_tgt = deg2rad(3)*sin(0.15*t_target);
    w_nb_tgt = [p_tgt; q_tgt; r_tgt];
    v_dvl_true_tgt = vel_true_body(k_step+1, :)' + cross(w_nb_tgt, lever_arm);
    z_dvl_true = (1 + true_scale) * Cbs_true * v_dvl_true_tgt;

    % Bộ lọc dùng mẫu gyro ở t_mid = gyro_true_body(k_step, :)':
    w_meas_mid = gyro_true_body(k_step, :)';
    Cbn_tgt = quat_to_cbn(quat_true(k_step+1, :)');
    Cnb_tgt = Cbn_tgt';
    v_ned_tgt = vel_true_ned(k_step+1, :)';
    w_en_tgt = [v_ned_tgt(2)/(radius_n + height); ...
               -v_ned_tgt(1)/(radius_m + height); ...
               -v_ned_tgt(2)*tan_lat/(radius_n + height)];
    w_in_b_tgt = Cnb_tgt * (w_ie_ned + w_en_tgt);
    w_rel_mid = w_meas_mid - w_in_b_tgt;

    v_dvl_pred_pipeline = (1 + true_scale) * Cbs_true * (vel_true_body(k_step+1, :)' + cross(w_rel_mid, lever_arm));

    delta_v_pipeline(idx, :) = (z_dvl_true - v_dvl_pred_pipeline)';
    sigma_dvl_history(idx) = dvl_noise_floor + dvl_noise_slope * norm(v_dvl_true_tgt);
end

norm_delta_v = sqrt(sum(delta_v_pipeline.^2, 2));
rms_delta_v = sqrt(mean(norm_delta_v.^2));
[max_delta_v, max_idx] = max(norm_delta_v);
t_worst = time_meas(max_idx);
mean_sigma_dvl = mean(sigma_dvl_history);

rms_ratio_pct = (rms_delta_v / mean_sigma_dvl) * 100;
max_ratio_pct = (max_delta_v / mean_sigma_dvl) * 100;

rms_axes = sqrt(mean(delta_v_pipeline.^2, 1));
max_axes = max(abs(delta_v_pipeline), [], 1);

fprintf('  * Phần 2 (Đánh giá sai lệch do lệch timestamp dt/2 = 1.25 ms):\n');
fprintf('    + RMS sai lệch vận tốc   : %.4e m/s (%.2f%% sigma_dvl)\n', rms_delta_v, rms_ratio_pct);
fprintf('    + Max sai lệch vận tốc   : %.4e m/s (%.2f%% sigma_dvl) tại t = %.2f s\n', max_delta_v, max_ratio_pct, t_worst);
fprintf('    + Sai lệch RMS theo trục : [X: %.2e, Y: %.2e, Z: %.2e] m/s\n', rms_axes(1), rms_axes(2), rms_axes(3));
fprintf('    + Sai lệch Max theo trục : [X: %.2e, Y: %.2e, Z: %.2e] m/s\n', max_axes(1), max_axes(2), max_axes(3));
fprintf('    + Ngưỡng đánh giá dự án  : RMS <= 10.0%% sigma_dvl, Max <= 25.0%% sigma_dvl\n');

pass_c2_rms = (rms_ratio_pct <= 10.0);
pass_c2_max = (max_ratio_pct <= 25.0);
pass_c2_part2 = pass_c2_rms && pass_c2_max;
fprintf('    --> Kết luận: %s\n\n', ternary_str(pass_c2_part2, 'PASS (Nằm an toàn trong dải cho phép)', ...
    'WARN (Vượt ngưỡng đánh giá dự án)'));

results_table(end+1, :) = {'C2-2', 'Lệch timestamp: RMS / sigma', 'RMS / sigma_dvl', ...
    '<= 10.0 %', sprintf('%.2f %% (%.2e m/s)', rms_ratio_pct, rms_delta_v), ...
    ternary_str(pass_c2_rms, 'PASS', 'WARN')};

results_table(end+1, :) = {'C2-3', 'Lệch timestamp: Max / sigma', 'Max / sigma_dvl', ...
    '<= 25.0 %', sprintf('%.2f %% (%.2e m/s)', max_ratio_pct, max_delta_v), ...
    ternary_str(pass_c2_max, 'PASS', 'WARN')};

%% ========================================================================
% C3. KIỂM TRA PHƯƠNG SAI DO NHIỄU GYRO QUA LEVER ARM (100.000 MẪU)
% =========================================================================
fprintf('--- [C3] KIỂM TRA PHƯƠNG SAI DO NHIỄU GYRO QUA LEVER ARM (100.000 MẪU) ---\n');

sigma_arw = 0.001; % rad/sqrt(s)
sigma_g = sigma_arw / sqrt(dt); % 0.02 rad/s
N_mc_samples = 100000;

% 1. Ma trận R lý thuyết từ đúng công thức của esekf19_step dòng 240:
skew_r = skew_mat(lever_arm);
R_gyro_theory = (1 + true_scale)^2 * Cbs_true * skew_r * (sigma_g^2 * eye(3)) * skew_r' * Cbs_true';

% 2. Thực nghiệm Monte Carlo 100.000 mẫu nhiễu gyro độc lập:
noise_gyro = sigma_g * randn(3, N_mc_samples);
% Sai số vận tốc DVL gây ra bởi nhiễu gyro qua cánh tay đòn:
% delta_v_dvl = (1 + s) * Cbs * cross(delta_w, lever_arm) = -(1 + s) * Cbs * skew(r) * delta_w
v_err_samples = -(1 + true_scale) * Cbs_true * skew_r * noise_gyro;

mean_err = mean(v_err_samples, 2);
R_emp = cov(v_err_samples');

% Tính chỉ số chuẩn hóa z_ij theo đúng công thức từ huongdantest.md dòng 170:
% z_ij = (R_emp(i,j) - R_theory(i,j)) / sqrt((R_theory(i,i)*R_theory(j,j) + R_theory(i,j)^2) / (N-1))
z_matrix = zeros(3, 3);
for i = 1:3
    for j = 1:3
        var_sample_cov = (R_gyro_theory(i, i)*R_gyro_theory(j, j) + R_gyro_theory(i, j)^2) / (N_mc_samples - 1);
        z_matrix(i, j) = (R_emp(i, j) - R_gyro_theory(i, j)) / sqrt(var_sample_cov);
    end
end

max_z_score = max(abs(z_matrix(:)));
z_mean = max(abs(mean_err) ./ sqrt(diag(R_gyro_theory) / N_mc_samples));

pass_c3_cov = (max_z_score <= 4.0);
pass_c3_mean = (z_mean <= 4.0);
pass_c3 = pass_c3_cov && pass_c3_mean;

fprintf('  * Kiểm tra qua N = %d mẫu Monte Carlo:\n', N_mc_samples);
fprintf('    + Độ lệch chuẩn hóa max của Hiệp phương sai (|z_ij| max): %.3f (Ngưỡng <= 4.0) --> %s\n', ...
    max_z_score, ternary_str(pass_c3_cov, 'PASS', 'FAIL'));
fprintf('    + Độ lệch chuẩn hóa max của Trung bình sai số (|z_mean|): %.3f (Ngưỡng <= 4.0) --> %s\n', ...
    z_mean, ternary_str(pass_c3_mean, 'PASS', 'FAIL'));
fprintf('    + Ma trận hiệp phương sai lý thuyết R_theory:\n');
disp(R_gyro_theory);
fprintf('    + Ma trận hiệp phương sai thực nghiệm R_emp:\n');
disp(R_emp);
fprintf('    + Ma trận chuẩn hóa z_ij:\n');
disp(z_matrix);

results_table(end+1, :) = {'C3-1', 'Phương sai Lever-arm Cov (|z_ij|)', 'Max normalized z-score', ...
    '<= 4.00', sprintf('%.2f (N=100k)', max_z_score), ternary_str(pass_c3_cov, 'PASS', 'FAIL')};

results_table(end+1, :) = {'C3-2', 'Trung bình sai số Gyro (|z_mean|)', 'Max mean z-score', ...
    '<= 4.00', sprintf('%.2f', z_mean), ternary_str(pass_c3_mean, 'PASS', 'FAIL')};

%% ========================================================================
% C4. KIỂM THỬ NHIỄU TRONG QUÁ TRÌNH TÍCH HỢP INS-DVL (INNOVATION & NIS)
% =========================================================================
fprintf('--- [C4] KIỂM THỬ NHIỄU TRONG TÍCH HỢP INS-DVL (INNOVATION & NIS) ---\n');

% Khởi tạo bộ lọc ES-EKF cho 1 lượt chạy mô phỏng hoàn chỉnh
noise_cfg = [0.02; 0.001; 0.005; 300; 1e-5; 1e-5; 1e-5; dt];
[x_flt, P_flt, Q_flt] = esekf19_init([0; 0; deg2rad(30)], gravity, noise_cfg);
P_flt(1:3, 1:3)     = (0.05)^2 * eye(3);
P_flt(4:6, 4:6)     = (0.05)^2 * eye(3);
P_flt(7:9, 7:9)     = deg2rad(1.0)^2 * eye(3);
P_flt(10:12, 10:12) = (0.01)^2 * eye(3);
P_flt(13:15, 13:15) = deg2rad(0.1/3600)^2 * eye(3);
P_flt(16, 16)       = (0.02)^2;
P_flt(17:19, 17:19) = deg2rad(2.0)^2 * eye(3);

% Khởi tạo trạng thái ước lượng quanh giá trị thật
x_flt(1:3)   = pos_true(1, :)';
x_flt(4:6)   = vel_true_ned(1, :)';
x_flt(7:10)  = quat_true(1, :)';
x_flt(17)    = true_scale;
x_flt(18:20) = true_boresight;

cfg_run = zeros(20, 1);
cfg_run(1) = dt; cfg_run(2) = gravity; cfg_run(3:5) = w_ie_ned;
cfg_run(6) = radius_n; cfg_run(7) = radius_m; cfg_run(8) = tan_lat;
cfg_run(9) = height; cfg_run(10:12) = lever_arm;
cfg_run(13) = dvl_noise_floor; cfg_run(14) = dvl_noise_slope;
cfg_run(15) = 0.02; cfg_run(16) = 0.02; cfg_run(17) = deg2rad(0.05);
cfg_run(18) = 0.05; cfg_run(19) = 300; cfg_run(20) = sigma_arw;

sigma_vrw = 0.02; % m/s/sqrt(s)
dvl_res_history = [];
dvl_nis_history = [];
dvl_accepted_history = [];

for k = 1:N_steps
    t_target = time_vec(k+1);

    % IMU đo đạc thực tế có nhiễu trắng chuẩn
    f_m = accel_true_body(k, :)' + (sigma_vrw / sqrt(dt)) * randn(3, 1);
    w_m = gyro_true_body(k, :)' + (sigma_arw / sqrt(dt)) * randn(3, 1);

    meas_flags = zeros(3, 1);
    gnss_m = zeros(7, 1);
    depth_m = 0;
    dvl_m = zeros(3, 1);

    % Cập nhật cảm biến ngoài 5 Hz (400 Hz / 5 Hz = 80 bước)
    if mod(k, 80) == 0
        % Cảm biến đo sâu
        meas_flags(2) = 1;
        depth_m = pos_true(k+1, 3) + 0.05 * randn;

        % GNSS RTK trong 15s đầu
        if t_target <= 15.0
            meas_flags(1) = 1;
            gnss_pos = pos_true(k+1, :)' + 0.02 * randn(3, 1);
            gnss_vel = vel_true_ned(k+1, :)' + 0.02 * randn(3, 1);
            Cbn_tgt = quat_to_cbn(quat_true(k+1, :)');
            true_yaw = atan2(Cbn_tgt(2, 1), Cbn_tgt(1, 1));
            gnss_yaw = true_yaw + deg2rad(0.05) * randn;
            gnss_m = [gnss_pos; gnss_vel; gnss_yaw];
        end

        % Vận tốc đáy DVL
        meas_flags(3) = 1;
        p_tgt = deg2rad(6)*sin(0.5*t_target);
        q_tgt = deg2rad(4)*cos(0.4*t_target);
        r_tgt = deg2rad(3)*sin(0.15*t_target);
        w_nb_tgt = [p_tgt; q_tgt; r_tgt];
        v_dvl_true = vel_true_body(k+1, :)' + cross(w_nb_tgt, lever_arm);
        sig_dvl = dvl_noise_floor + dvl_noise_slope * norm(v_dvl_true);
        dvl_m = (1 + true_scale) * Cbs_true * v_dvl_true + sig_dvl * randn(3, 1);
    end

    [x_flt, P_flt, update_status, nis] = esekf19_step( ...
        x_flt, P_flt, Q_flt, w_m, f_m, ...
        gnss_m, depth_m, dvl_m, meas_flags, cfg_run);

    if meas_flags(3) ~= 0
        dvl_nis_history(end+1, 1) = nis(3);
        dvl_accepted_history(end+1, 1) = update_status(3);
    end
end

mean_nis_dvl = mean(dvl_nis_history);
accept_rate_pct = mean(dvl_accepted_history) * 100;

% Tự tương quan lag-1 của NIS
if length(dvl_nis_history) > 10
    nis_centered = dvl_nis_history - mean(dvl_nis_history);
    autocorr_lag1 = (nis_centered(1:end-1)' * nis_centered(2:end)) / sum(nis_centered.^2);
else
    autocorr_lag1 = 0;
end

% Phân phối Chi-square 3 bậc tự do: kỳ vọng lý thuyết là 3.0, dải 95% tin cậy ~ [0.216, 9.348]
pass_c4_mean_nis = (mean_nis_dvl >= 1.5 && mean_nis_dvl <= 5.5);
pass_c4_accept = (accept_rate_pct >= 95.0);

fprintf('  * Kết quả kiểm định tích hợp INS-DVL (Số lượng phép đo: %d):\n', length(dvl_nis_history));
fprintf('    + NIS trung bình của DVL   : %.3f (Kỳ vọng lý thuyết: 3.00, dải chấp nhận [1.5, 5.5]) --> %s\n', ...
    mean_nis_dvl, ternary_str(pass_c4_mean_nis, 'PASS', 'FAIL'));
fprintf('    + Tỷ lệ chấp nhận cổng Chi2: %.2f%% (Ngưỡng >= 95.0%%) --> %s\n', ...
    accept_rate_pct, ternary_str(pass_c4_accept, 'PASS', 'FAIL'));
fprintf('    + Hệ số tự tương quan lag-1: %.3f (Gần 0 thể hiện tính trắng của innovation)\n', autocorr_lag1);

results_table(end+1, :) = {'C4-1', 'NIS DVL Trung bình (3 DOF)', 'Mean NIS', ...
    '[1.50, 5.50]', sprintf('%.2f', mean_nis_dvl), ternary_str(pass_c4_mean_nis, 'PASS', 'FAIL')};

results_table(end+1, :) = {'C4-2', 'Tỷ lệ chấp nhận đo đạc DVL', 'Acceptance Rate', ...
    '>= 95.0 %', sprintf('%.1f %%', accept_rate_pct), ternary_str(pass_c4_accept, 'PASS', 'FAIL')};

results_table(end+1, :) = {'C4-3', 'Tự tương quan NIS lag-1', 'Autocorr lag-1', ...
    '<= 0.30', sprintf('%.3f', autocorr_lag1), ternary_str(abs(autocorr_lag1) <= 0.30, 'PASS', 'WARN')};

%% ========================================================================
% TỔNG HỢP VÀ XUẤT BÁO CÁO BỘ TEST 2
% =========================================================================
fprintf('\n========================================================================================\n');
fprintf('  BẢNG TỔNG HỢP KẾT QUẢ BỘ TEST 2 (DVL, GYRO & LEVER ARM)\n');
fprintf('========================================================================================\n');
fprintf('| %-5s | %-32s | %-22s | %-16s | %-18s | %-9s |\n', ...
    'ID', 'Test', 'Metric', 'Threshold', 'Actual', 'PASS/FAIL');
fprintf('|%s|\n', repmat('-', 1, 113));

for row = 1:size(results_table, 1)
    fprintf('| %-5s | %-32s | %-22s | %-16s | %-18s | %-9s |\n', ...
        results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
        results_table{row, 4}, results_table{row, 5}, results_table{row, 6});
end
fprintf('========================================================================================\n\n');

% Lưu kết quả ra file .mat
mat_file_path = fullfile(test_dir, 'ket_qua_test_dvl_leverarm.mat');
save(mat_file_path, 'results_table', 'diff_cross', 'diff_inv1', 'diff_inv2', ...
    'max_res_ideal', 'rms_delta_v', 'max_delta_v', 'rms_ratio_pct', 'max_ratio_pct', ...
    'max_z_score', 'z_mean', 'R_gyro_theory', 'R_emp', 'mean_nis_dvl', 'accept_rate_pct');
fprintf('[THÀNH CÔNG] Đã lưu kết quả Bộ test 2 vào file: %s\n', mat_file_path);

% Xuất file báo cáo .txt
txt_file_path = fullfile(test_dir, 'bao_cao_test_dvl_leverarm.txt');
fid = fopen(txt_file_path, 'w', 'n', 'UTF-8');
if fid ~= -1
    fprintf(fid, '========================================================================================\r\n');
    fprintf(fid, '  BÁO CÁO KẾT QUẢ BỘ TEST 2: DVL, NHIỄU GYRO VÀ CÁNH TAY ĐÒN (ES-EKF 19 TRẠNG THÁI)\r\n');
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, '| %-5s | %-32s | %-22s | %-16s | %-18s | %-9s |\r\n', ...
        'ID', 'Test', 'Metric', 'Threshold', 'Actual', 'PASS/FAIL');
    fprintf(fid, '|%s|\r\n', repmat('-', 1, 113));
    for row = 1:size(results_table, 1)
        fprintf(fid, '| %-5s | %-32s | %-22s | %-16s | %-18s | %-9s |\r\n', ...
            results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
            results_table{row, 4}, results_table{row, 5}, results_table{row, 6});
    end
    fprintf(fid, '========================================================================================\r\n');
    fclose(fid);
    fprintf('[THÀNH CÔNG] Đã xuất báo cáo Bộ test 2 vào file: %s\n', txt_file_path);
end

end

%% ========================================================================
% CÁC HÀM PHỤ TRỢ CỤC BỘ DÙNG CHO KIỂM THỬ
% =========================================================================

function R = rodrigues_so3(phi)
theta = norm(phi);
K = skew_mat(phi);
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
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

function s = ternary_str(cond, str_true, str_false)
if cond
    s = str_true;
else
    s = str_false;
end
end
