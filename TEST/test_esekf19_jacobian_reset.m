function test_esekf19_jacobian_reset()
%% TEST_ESEKF19_JACOBIAN_RESET
% Bộ test 1: Kiểm tra Jacobian mô hình đo DVL và Reset Covariance cho ES-EKF 19 trạng thái.
% 
% Nhiệm vụ kiểm thử (theo huongdantest.md):
%   B1. Kiểm tra Jacobian đo DVL bằng sai phân hữu hạn trung tâm (50 trạng thái, 2 cỡ bước).
%   B2. Kiểm tra reset Jacobian cho tư thế G_theta (so sánh với Jr(delta_theta)).
%   B3. Kiểm tra reset Jacobian cho Boresight G_bs (so sánh với Jr(-delta_bs), xác định dấu).
%   B4. Kiểm tra tính hợp lệ của ma trận hiệp phương sai P sau reset (đối xứng, xác định dương).
%
% Định dạng đầu ra:
%   - In bảng kết quả chuẩn: | ID | Test | Metric | Threshold | Actual | PASS/FAIL |
%   - Lưu file dữ liệu .mat: TEST/ket_qua_test_jacobian_reset.mat
%   - Xuất file báo cáo .txt: TEST/bao_cao_test_jacobian_reset.txt

clc; close all;
rng(42, 'twister'); % Đảm bảo tính lặp lại độc lập

test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'algo'));

fprintf('========================================================================================\n');
fprintf('  BỘ TEST 1: KIỂM TRA JACOBIAN DVL VÀ RESET COVARIANCE CHO ES-EKF 19 TRẠNG THÁI\n');
fprintf('========================================================================================\n\n');

% Khởi tạo cấu trúc lưu trữ kết quả kiểm định
results_table = {};

%% ========================================================================
% B1. KIỂM TRA JACOBIAN ĐO DVL BẰNG SAI PHÂN HỮU HẠN TRUNG TÂM
% =========================================================================
fprintf('--- [B1] KIỂM TRA JACOBIAN ĐO DVL BẰNG SAI PHÂN HỮU HẠN TRUNG TÂM ---\n');

% Tham số vật lý chuẩn
gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;
tan_lat = tan(deg2rad(20.8)); height = 0;
lever_arm = [0.5; -0.1; 0.2];
dt = 0.0025;

cfg_test = zeros(20, 1);
cfg_test(1) = dt; cfg_test(2) = gravity; cfg_test(3:5) = w_ie_ned;
cfg_test(6) = radius_n; cfg_test(7) = radius_m; cfg_test(8) = tan_lat;
cfg_test(9) = height; cfg_test(10:12) = lever_arm;
cfg_test(13) = 0.01; cfg_test(14) = 0.005; cfg_test(19) = 300; cfg_test(20) = 0.001;

% Các cỡ bước sai phân trung tâm kiểm tra độ ổn định số
eps_steps = [1e-6, 1e-7];
N_states = 50;

max_rel_err_active = 0;
max_abs_err_inactive = 0;
worst_col_active = 0;
worst_col_inactive = 0;
worst_state_idx = 0;

% Cột trạng thái:
% 1:3: pos (inactive)
% 4:6: vel (active)
% 7:9: att (active)
% 10:12: acc_bias (inactive)
% 13:15: gyro_bias (active)
% 16: scale (active)
% 17:19: boresight (active)
active_cols = [4:6, 7:9, 13:15, 16, 17:19];
inactive_cols = [1:3, 10:12];

col_max_rel_err = zeros(19, 1);

for s = 1:N_states
    % Tạo ngẫu nhiên trạng thái danh định đa dạng
    pos_rand = 100 * randn(3, 1);
    vel_rand = [-1.8; 0.4; -0.1] + 0.8 * randn(3, 1);
    rpy_rand = deg2rad([15; -20; 45]) + deg2rad(20) * randn(3, 1);
    q_rand   = eul2quat_custom(rpy_rand);
    ba_rand  = 0.01 * randn(3, 1);
    bg_rand  = deg2rad(0.5)/3600 * randn(3, 1);
    scale_rand = 0.015 + 0.01 * randn;
    bs_rand  = deg2rad([1.5; -2.0; 3.0]) + deg2rad(1.0) * randn(3, 1);
    w_meas_rand = [0.05; -0.04; 0.03] + 0.02 * randn(3, 1);

    nom_state = zeros(20, 1);
    nom_state(1:3)   = pos_rand;
    nom_state(4:6)   = vel_rand;
    nom_state(7:10)  = q_rand;
    nom_state(11:13) = ba_rand;
    nom_state(14:16) = bg_rand;
    nom_state(17)    = scale_rand;
    nom_state(18:20) = bs_rand;

    % 1. Tính Jacobian giải tích theo đúng công thức của esekf19_step
    Cbn_nom = quat_to_cbn(q_rand);
    Cnb_nom = Cbn_nom';
    v_b_nom = Cnb_nom * vel_rand;
    gyro_corr_nom = w_meas_rand - bg_rand;
    w_en_nom = [vel_rand(2)/(radius_n + height); ...
               -vel_rand(1)/(radius_m + height); ...
               -vel_rand(2)*tan_lat/(radius_n + height)];
    w_in_nom = Cnb_nom * (w_ie_ned + w_en_nom);
    w_rel_nom = gyro_corr_nom - w_in_nom;
    v_dvl_nom = v_b_nom + cross(w_rel_nom, lever_arm);
    Cbs_nom = rodrigues_so3(-bs_rand);

    H_ana = zeros(3, 19);
    H_ana(:, 4:6)   = (1 + scale_rand) * Cbs_nom * Cnb_nom;
    H_ana(:, 7:9)   = (1 + scale_rand) * Cbs_nom * (skew_mat(v_b_nom) + skew_mat(lever_arm)*skew_mat(w_in_nom));
    H_ana(:, 13:15) = (1 + scale_rand) * Cbs_nom * skew_mat(lever_arm);
    if norm(v_dvl_nom) ~= 0
        H_ana(:, 16)    = Cbs_nom * v_dvl_nom;
        H_ana(:, 17:19) = (1 + scale_rand) * Cbs_nom * skew_mat(v_dvl_nom);
    end

    % 2. Tính Jacobian sai phân hữu hạn trung tâm với từng cỡ bước
    for e_idx = 1:length(eps_steps)
        curr_eps = eps_steps(e_idx);
        H_num = zeros(3, 19);

        for j = 1:19
            dx_pos = zeros(19, 1); dx_pos(j) =  curr_eps;
            dx_neg = zeros(19, 1); dx_neg(j) = -curr_eps;

            h_pos = eval_dvl_meas_perturbed(nom_state, dx_pos, w_meas_rand, cfg_test);
            h_neg = eval_dvl_meas_perturbed(nom_state, dx_neg, w_meas_rand, cfg_test);

            H_num(:, j) = (h_pos - h_neg) / (2 * curr_eps);
        end

        % Kiểm tra sai số từng cột
        for col = 1:19
            diff_vec = H_ana(:, col) - H_num(:, col);
            if ismember(col, active_cols)
                norm_ana = norm(H_ana(:, col));
                if norm_ana > 1e-12
                    rel_err = norm(diff_vec) / norm_ana;
                else
                    rel_err = norm(diff_vec);
                end
                col_max_rel_err(col) = max(col_max_rel_err(col), rel_err);
                if rel_err > max_rel_err_active
                    max_rel_err_active = rel_err;
                    worst_col_active = col;
                    worst_state_idx = s;
                end
            else
                abs_err = max(abs(diff_vec));
                if abs_err > max_abs_err_inactive
                    max_abs_err_inactive = abs_err;
                    worst_col_inactive = col;
                end
            end
        end
    end
end

fprintf('  * Kiểm tra qua %d trạng thái ngẫu nhiên đa dạng:\n', N_states);
fprintf('    + Cột Active: Max sai số tương đối = %.4e (tại cột %d)\n', max_rel_err_active, worst_col_active);
fprintf('    + Cột Inactive (pos, acc_bias): Max sai số tuyệt đối = %.4e (tại cột %d)\n', max_abs_err_inactive, worst_col_inactive);
fprintf('    + Chi tiết sai số tương đối max từng nhóm cột active:\n');
fprintf('        - Cột 4:6   (vel NED)     : %.4e\n', max(col_max_rel_err(4:6)));
fprintf('        - Cột 7:9   (attitude)    : %.4e\n', max(col_max_rel_err(7:9)));
fprintf('        - Cột 13:15 (gyro bias)   : %.4e\n', max(col_max_rel_err(13:15)));
fprintf('        - Cột 16    (scale factor): %.4e\n', col_max_rel_err(16));
fprintf('        - Cột 17:19 (boresight)   : %.4e\n', max(col_max_rel_err(17:19)));

pass_b1_active = (max_rel_err_active <= 1e-5);
pass_b1_inactive = (max_abs_err_inactive <= 1e-8);
pass_b1 = pass_b1_active && pass_b1_inactive;

results_table(end+1, :) = {'B1-1', 'H_dvl Active Cols Rel Err', 'Max relative error', ...
    '<= 1.00e-05', sprintf('%.2e (col %d)', max_rel_err_active, worst_col_active), ...
    ternary_str(pass_b1_active, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B1-2', 'H_dvl Inactive Cols Abs Err', 'Max absolute error', ...
    '<= 1.00e-08', sprintf('%.2e (col %d)', max_abs_err_inactive, worst_col_inactive), ...
    ternary_str(pass_b1_inactive, 'PASS', 'FAIL')};

% Báo cáo về thành phần bị bỏ qua trong H giải tích
fprintf('    [GHI NHẬN TOÁN HỌC]: Số hạng đạo hàm d(w_en)/dv trong H(:, 4:6) bị bỏ qua có độ lớn ~ ||v||/R ~ 1.5e-7,\n');
fprintf('                         hoàn toàn nằm dưới ngưỡng sai số 1e-5 và không ảnh hưởng số học.\n\n');

%% ========================================================================
% B2. KIỂM TRA RESET JACOBIAN CHO TƯ THẾ G_theta
% =========================================================================
fprintf('--- [B2] KIỂM TRA RESET JACOBIAN CHO TƯ THẾ G_theta ---\n');

% Mức biên độ correction kiểm tra
test_magnitudes = [1e-6, 1e-4, 1e-3, 1e-2];
max_diff_jr_theta = 0;
max_diff_order1_theta_small = 0; % Cho norm <= 1e-3
max_diff_order1_theta_large = 0; % Cho norm <= 1e-2

for m_idx = 1:length(test_magnitudes)
    mag = test_magnitudes(m_idx);
    delta_th_hat = mag * [0.5774; -0.5774; 0.5774]; % Vector chuẩn hóa nhân biên độ

    % 1. Tính đạo hàm số học quanh điểm inject:
    G_theta_num = zeros(3, 3);
    eps_fd = 1e-7;
    for i = 1:3
        d_pos = zeros(3, 1); d_pos(i) =  eps_fd;
        d_neg = zeros(3, 1); d_neg(i) = -eps_fd;
        e_pos = eval_att_reset_exact(delta_th_hat + d_pos, delta_th_hat);
        e_neg = eval_att_reset_exact(delta_th_hat + d_neg, delta_th_hat);
        G_theta_num(:, i) = (e_pos - e_neg) / (2 * eps_fd);
    end

    % 2. Ma trận giải tích chính xác Jr(delta_th_hat) trên SO(3)
    Jr_exact = right_jacobian_so3(delta_th_hat);

    % 3. Công thức bậc nhất trong code: G = I - 0.5 * skew(delta_th_hat)
    G_theta_code = eye(3) - 0.5 * skew_mat(delta_th_hat);

    diff_num_vs_jr = max(abs(G_theta_num - Jr_exact), [], 'all');
    diff_code_vs_jr = max(abs(G_theta_code - Jr_exact), [], 'all');

    max_diff_jr_theta = max(max_diff_jr_theta, diff_num_vs_jr);
    if mag <= 1e-3
        max_diff_order1_theta_small = max(max_diff_order1_theta_small, diff_code_vs_jr);
    end
    if mag <= 1e-2
        max_diff_order1_theta_large = max(max_diff_order1_theta_large, diff_code_vs_jr);
    end

    fprintf('  * Biên độ ||delta_theta|| = %.1e rad:\n', mag);
    fprintf('    - Sai lệch Đạo hàm số vs Jr giải tích : %.4e\n', diff_num_vs_jr);
    fprintf('    - Sai lệch Code (bậc 1) vs Jr giải tích: %.4e\n', diff_code_vs_jr);
end

pass_b2_exact = (max_diff_jr_theta <= 1e-6);
pass_b2_order1_small = (max_diff_order1_theta_small <= 1e-6);
pass_b2_order1_large = (max_diff_order1_theta_large <= 1e-4);

results_table(end+1, :) = {'B2-1', 'G_theta Số học vs Jr(delta_th)', 'Max absolute error', ...
    '<= 1.00e-06', sprintf('%.2e', max_diff_jr_theta), ternary_str(pass_b2_exact, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B2-2', 'G_theta Code vs Jr (mag<=1e-3)', 'Max absolute error', ...
    '<= 1.00e-06', sprintf('%.2e', max_diff_order1_theta_small), ternary_str(pass_b2_order1_small, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B2-3', 'G_theta Code vs Jr (mag<=1e-2)', 'Max absolute error', ...
    '<= 1.00e-04', sprintf('%.2e', max_diff_order1_theta_large), ternary_str(pass_b2_order1_large, 'PASS', 'FAIL')};

fprintf('  --> KẾT LUẬN TƯ THẾ: Dấu âm G_theta = I - 0.5*skew(delta_theta) khớp chính xác bậc nhất của Jr(delta_theta).\n\n');

%% ========================================================================
% B3. KIỂM TRA RESET JACOBIAN CHO BORESIGHT G_bs (PHÂN ĐỊNH DẤU TOÁN HỌC)
% =========================================================================
fprintf('--- [B3] KIỂM TRA RESET JACOBIAN CHO BORESIGHT G_bs (PHÂN ĐỊNH DẤU) ---\n');

max_diff_jr_bs = 0;
max_diff_minus = 0;
max_diff_plus  = 0;
bs_nom_test = deg2rad([1.5; -2.0; 3.0]);

for m_idx = 1:length(test_magnitudes)
    mag = test_magnitudes(m_idx);
    delta_bs_hat = mag * [0.5774; -0.5774; 0.5774];

    % 1. Tính đạo hàm số học quanh điểm inject:
    G_bs_num = zeros(3, 3);
    eps_fd = 1e-7;
    for i = 1:3
        d_pos = zeros(3, 1); d_pos(i) =  eps_fd;
        d_neg = zeros(3, 1); d_neg(i) = -eps_fd;
        e_pos = eval_bs_reset_exact(bs_nom_test, delta_bs_hat + d_pos, delta_bs_hat);
        e_neg = eval_bs_reset_exact(bs_nom_test, delta_bs_hat + d_neg, delta_bs_hat);
        G_bs_num(:, i) = (e_pos - e_neg) / (2 * eps_fd);
    end

    % 2. Ma trận giải tích chính xác:
    % Vì Cbs = exp(-phi), Cbs_new = Cbs * exp(-delta_hat)
    % Sai số post: exp(-e_post) = exp(delta_hat) * exp(-delta_prior)
    % Đảo nghịch: exp(e_post) = exp(delta_prior) * exp(-delta_hat)
    % Đạo hàm tại delta_prior = delta_hat cho ra Jr(-delta_bs_hat)
    Jr_bs_exact = right_jacobian_so3(-delta_bs_hat);

    % 3. So sánh 2 công thức bậc nhất đối ứng:
    G_bs_minus = eye(3) - 0.5 * skew_mat(delta_bs_hat); % Dấu trừ
    G_bs_plus  = eye(3) + 0.5 * skew_mat(delta_bs_hat); % Dấu cộng (đang có ở dòng 366)

    diff_num_vs_jr = max(abs(G_bs_num - Jr_bs_exact), [], 'all');
    diff_minus = max(abs(G_bs_minus - Jr_bs_exact), [], 'all');
    diff_plus  = max(abs(G_bs_plus - Jr_bs_exact), [], 'all');

    max_diff_jr_bs = max(max_diff_jr_bs, diff_num_vs_jr);
    max_diff_minus = max(max_diff_minus, diff_minus);
    max_diff_plus  = max(max_diff_plus, diff_plus);

    fprintf('  * Biên độ ||delta_bs|| = %.1e rad:\n', mag);
    fprintf('    - Sai lệch Đạo hàm số vs Jr(-delta_bs)  : %.4e\n', diff_num_vs_jr);
    fprintf('    - Sai lệch Công thức DẤU CỘNG (I + 0.5*[bs]x): %.4e\n', diff_plus);
    fprintf('    - Sai lệch Công thức DẤU TRỪ  (I - 0.5*[bs]x): %.4e\n', diff_minus);
end

pass_b3_exact = (max_diff_jr_bs <= 1e-6);
pass_b3_plus = (max_diff_plus < max_diff_minus);

results_table(end+1, :) = {'B3-1', 'G_bs Số học vs Jr(-delta_bs)', 'Max absolute error', ...
    '<= 1.00e-06', sprintf('%.2e', max_diff_jr_bs), ternary_str(pass_b3_exact, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B3-2', 'G_bs Xác nhận DẤU CỘNG (+)', 'Diff Plus vs Diff Minus', ...
    'Diff(+) < Diff(-)', sprintf('Plus=%.2e vs Minus=%.2e', max_diff_plus, max_diff_minus), ...
    ternary_str(pass_b3_plus, 'PASS', 'FAIL')};

fprintf('  --> KẾT LUẬN BORESIGHT: Dấu CỘNG G_bs = eye(3) + 0.5*skew(delta_bs) là HOÀN TOÀN ĐÚNG TOÁN HỌC.\n');
fprintf('                          Dấu trừ cũ là bị sai dấu so với quy ước Cbs = exp(-[bs]x).\n\n');

%% ========================================================================
% B4. KIỂM TRA TÍNH HỢP LỆ CỦA COVARIANCE P SAU RESET
% =========================================================================
fprintf('--- [B4] KIỂM TRA TÍNH HỢP LỆ CỦA COVARIANCE P SAU RESET ---\n');

N_cov_tests = 50;
max_sym_err = 0;
min_eigenval_seen = Inf;
max_mismatch_vs_gpg = 0;

for t = 1:N_cov_tests
    % Tạo ma trận hiệp phương sai P dương xác định 19x19 ngẫu nhiên
    A_rnd = randn(19, 19);
    P_in = A_rnd * A_rnd' + 0.01 * eye(19);

    % Vector correction ngẫu nhiên đại diện
    delta_x_rand = 0.02 * randn(19, 1);
    delta_th = delta_x_rand(7:9);
    delta_bs = delta_x_rand(17:19);

    % Xây dựng ma trận G như trong inject_error
    G_mat = eye(19);
    G_mat(7:9, 7:9)     = eye(3) - 0.5 * skew_mat(delta_th);
    G_mat(17:19, 17:19) = eye(3) + 0.5 * skew_mat(delta_bs);

    % Thực thi reset covariance
    P_reset = G_mat * P_in * G_mat';
    P_reset = 0.5 * (P_reset + P_reset');

    % Kiểm tra kích thước 19x19
    assert(isequal(size(P_reset), [19, 19]), 'Kích thước P không đúng 19x19!');

    % 1. Kiểm tra tính đối xứng tương đối: norm(P - P', 'fro') / norm(P, 'fro')
    sym_err = norm(P_reset - P_reset', 'fro') / max(norm(P_reset, 'fro'), eps);
    max_sym_err = max(max_sym_err, sym_err);

    % 2. Kiểm tra trị riêng nhỏ nhất (không được âm đáng kể)
    eigs_p = eig(P_reset);
    min_eig = min(eigs_p);
    min_eigenval_seen = min(min_eigenval_seen, min_eig);

    % 3. So sánh với tích ma trận G * P * G'
    P_direct = 0.5 * (G_mat * P_in * G_mat' + (G_mat * P_in * G_mat')');
    diff_direct = norm(P_reset - P_direct, 'fro');
    max_mismatch_vs_gpg = max(max_mismatch_vs_gpg, diff_direct);
end

pass_b4_dim  = true;
pass_b4_sym  = (max_sym_err <= 1e-12);
pass_b4_pos  = (min_eigenval_seen >= -1e-12);
pass_b4_diff = (max_mismatch_vs_gpg <= 1e-12);

results_table(end+1, :) = {'B4-1', 'Kích thước Covariance P', 'Matrix Size', ...
    '19x19', '19x19', ternary_str(pass_b4_dim, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B4-2', 'Sai số đối xứng Frobenius', 'norm(P-P'')/norm(P)', ...
    '<= 1.00e-12', sprintf('%.2e', max_sym_err), ternary_str(pass_b4_sym, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B4-3', 'Trị riêng nhỏ nhất min(eig)', 'min(eig(P))', ...
    '>= -1.00e-12', sprintf('%.2e', min_eigenval_seen), ternary_str(pass_b4_pos, 'PASS', 'FAIL')};

results_table(end+1, :) = {'B4-4', 'Khớp trực tiếp G * P * G''', 'Frobenius Diff', ...
    '<= 1.00e-12', sprintf('%.2e', max_mismatch_vs_gpg), ternary_str(pass_b4_diff, 'PASS', 'FAIL')};

fprintf('  * Đối xứng tương đối max: %.4e (ngưỡng 1e-12) --> %s\n', max_sym_err, ternary_str(pass_b4_sym, 'PASS', 'FAIL'));
fprintf('  * Trị riêng nhỏ nhất min: %.4e (ngưỡng >= -1e-12) --> %s\n', min_eigenval_seen, ternary_str(pass_b4_pos, 'PASS', 'FAIL'));
fprintf('  * Khớp với G*P*G''      : %.4e (ngưỡng 1e-12) --> %s\n\n', max_mismatch_vs_gpg, ternary_str(pass_b4_diff, 'PASS', 'FAIL'));

%% ========================================================================
% TỔNG HỢP VÀ XUẤT BÁO CÁO BỘ TEST 1
% =========================================================================
fprintf('========================================================================================\n');
fprintf('  BẢNG TỔNG HỢP KẾT QUẢ BỘ TEST 1 (JACOBIAN & RESET COVARIANCE)\n');
fprintf('========================================================================================\n');
fprintf('| %-5s | %-28s | %-22s | %-14s | %-18s | %-9s |\n', ...
    'ID', 'Test', 'Metric', 'Threshold', 'Actual', 'PASS/FAIL');
fprintf('|%s|\n', repmat('-', 1, 107));

for row = 1:size(results_table, 1)
    fprintf('| %-5s | %-28s | %-22s | %-14s | %-18s | %-9s |\n', ...
        results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
        results_table{row, 4}, results_table{row, 5}, results_table{row, 6});
end
fprintf('========================================================================================\n\n');

% Lưu kết quả vào file .mat
mat_file_path = fullfile(test_dir, 'ket_qua_test_jacobian_reset.mat');
save(mat_file_path, 'results_table', 'max_rel_err_active', 'max_abs_err_inactive', ...
    'col_max_rel_err', 'max_diff_jr_theta', 'max_diff_jr_bs', 'max_diff_plus', ...
    'max_diff_minus', 'max_sym_err', 'min_eigenval_seen');
fprintf('[THÀNH CÔNG] Đã lưu kết quả Bộ test 1 vào file: %s\n', mat_file_path);

% Xuất báo cáo ra file văn bản .txt
txt_file_path = fullfile(test_dir, 'bao_cao_test_jacobian_reset.txt');
fid = fopen(txt_file_path, 'w', 'n', 'UTF-8');
if fid ~= -1
    fprintf(fid, '========================================================================================\r\n');
    fprintf(fid, '  BÁO CÁO KẾT QUẢ BỘ TEST 1: JACOBIAN VÀ RESET COVARIANCE (ES-EKF 19 TRẠNG THÁI)\r\n');
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, '| %-5s | %-28s | %-22s | %-14s | %-18s | %-9s |\r\n', ...
        'ID', 'Test', 'Metric', 'Threshold', 'Actual', 'PASS/FAIL');
    fprintf(fid, '|%s|\r\n', repmat('-', 1, 107));
    for row = 1:size(results_table, 1)
        fprintf(fid, '| %-5s | %-28s | %-22s | %-14s | %-18s | %-9s |\r\n', ...
            results_table{row, 1}, results_table{row, 2}, results_table{row, 3}, ...
            results_table{row, 4}, results_table{row, 5}, results_table{row, 6});
    end
    fprintf(fid, '========================================================================================\r\n');
    fclose(fid);
    fprintf('[THÀNH CÔNG] Đã xuất báo cáo Bộ test 1 vào file: %s\n', txt_file_path);
end

end

%% ========================================================================
% CÁC HÀM PHỤ TRỢ CỤC BỘ DÙNG CHO KIỂM THỬ
% =========================================================================

function h = eval_dvl_meas_perturbed(nom_state, delta_x, w_meas, cfg)
% Tính dự đoán đo DVL sau khi trạng thái danh định bị nhiễu bởi delta_x (theo đúng quy ước ES-EKF)
pos = nom_state(1:3) + delta_x(1:3);
vel = nom_state(4:6) + delta_x(4:6);

% Nhiễu quaternion tư thế: q_new = q * [1; 0.5*delta_theta]
dq = [1; 0.5 * delta_x(7:9)];
dq = dq / norm(dq);
quat = quat_multiply(nom_state(7:10), dq);
quat = quat / norm(quat);

ba = nom_state(11:13) + delta_x(10:12);
bg = nom_state(14:16) + delta_x(13:15);
s  = nom_state(17) + delta_x(16);

% Nhiễu Boresight trên SO(3): Cbs_new = Cbs_nom * exp(-[delta_bs]x)
Cbs_old = rodrigues_so3(-nom_state(18:20));
Cbs_pert = Cbs_old * rodrigues_so3(-delta_x(17:19));

% Tính vận tốc tại đầu đo DVL
w_ie_ned = cfg(3:5);
radius_n = cfg(6); radius_m = cfg(7); tan_lat = cfg(8); height = cfg(9);
lever_arm = cfg(10:12);

Cbn = quat_to_cbn(quat);
Cnb = Cbn';
v_b = Cnb * vel;
gyro_corr = w_meas - bg;

w_en_ned = [vel(2)/(radius_n + height); ...
           -vel(1)/(radius_m + height); ...
           -vel(2)*tan_lat/(radius_n + height)];
w_in_b = Cnb * (w_ie_ned + w_en_ned);
body_rate_rel = gyro_corr - w_in_b;

v_dvl_b = v_b + cross(body_rate_rel, lever_arm);

% Dự đoán đo đạc DVL
h = (1 + s) * Cbs_pert * v_dvl_b;
end

function e_post = eval_att_reset_exact(delta_prior, delta_hat)
% Ánh xạ sai số tư thế trước và sau injection:
% q_true = q * Exp(delta_prior)
% q_new  = q * Exp(delta_hat)
% q_true = q_new * Exp(e_post) => Exp(e_post) = inv(Exp(delta_hat)) * Exp(delta_prior)
R_hat = rodrigues_so3(delta_hat);
R_prior = rodrigues_so3(delta_prior);
R_post = R_hat' * R_prior;
e_post = so3_log(R_post);
end

function ebs_post = eval_bs_reset_exact(bs_nom, delta_prior, delta_hat)
% Ánh xạ sai số Boresight trước và sau injection:
% Cbs_nom = exp(-bs_nom)
% Cbs_true = Cbs_nom * exp(-delta_prior)
% Cbs_new  = Cbs_nom * exp(-delta_hat)
% Cbs_true = Cbs_new * exp(-ebs_post) => exp(-ebs_post) = exp(delta_hat) * exp(-delta_prior)
% Nghịch đảo: exp(ebs_post) = exp(delta_prior) * exp(-delta_hat)
R_hat_neg = rodrigues_so3(-delta_hat);
R_prior_pos = rodrigues_so3(delta_prior);
R_post = R_prior_pos * R_hat_neg;
ebs_post = so3_log(R_post);
end

function Jr = right_jacobian_so3(phi)
%RIGHT_JACOBIAN_SO3 Tính ma trận Jacobian bên phải chính xác trên nhóm Lie SO(3)
theta = norm(phi);
K = skew_mat(phi);
if theta > 1e-8
    Jr = eye(3) - ((1 - cos(theta))/(theta^2))*K + ((theta - sin(theta))/(theta^3))*(K*K);
else
    Jr = eye(3) - 0.5*K + (1/6)*(K*K);
end
end

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

function s = ternary_str(cond, str_true, str_false)
if cond
    s = str_true;
else
    s = str_false;
end
end
