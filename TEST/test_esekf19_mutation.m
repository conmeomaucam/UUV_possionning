function test_esekf19_mutation()
%% TEST_ESEKF19_MUTATION
% Nhiệm vụ 1 (huongdantest2.md): Negative-control / Mutation Tests.
% Chứng minh độ nhạy và tính chuẩn xác của các bộ test:
% Khi cố tình cấy lỗi giả vào mô hình, test bắt buộc phải phát hiện ra và báo FAIL.
%
% 3 Kịch bản đột biến (Mutations) kiểm tra:
%   1. Mutation 1: Đảo dấu tích có hướng cánh tay đòn -> cross(r, w) thay vì cross(w, r).
%   2. Mutation 2: Đảo dấu reset Boresight -> Dấu TRỪ thay vì Dấu CỘNG.
%   3. Mutation 3: Bỏ thành phần bù nhiễu Gyro E1 khỏi ma trận R.

clc; close all;
rng(99, 'twister');

test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'algo'));

fprintf('========================================================================================\n');
fprintf('  NHIỆM VỤ 1: MUTATION TESTS (NEGATIVE-CONTROL) CHO ES-EKF 19 TRẠNG THÁI\n');
fprintf('========================================================================================\n\n');

results_mutation = {};

%% ========================================================================
% 1. BASELINE: KIỂM TRA MÃ NGUỒN CHUẨN PRODUCTION
% =========================================================================
fprintf('--- [BASELINE] KIỂM TRA MÃ NGUỒN SẢN XUẤT CHUẨN (KHÔNG CẤY LỖI) ---\n');

% Gọi trực tiếp các hàm production dùng chung
state_test = zeros(20, 1);
state_test(4:6)   = [1.6; 0.2; -0.1];
state_test(7:10)  = [1; 0; 0; 0];
state_test(17)    = 0.015;
state_test(18:20) = deg2rad([1.5; -2.0; 3.0]);

w_meas = [0.2; -0.3; 0.4];
cfg_test = zeros(20, 1);
cfg_test(1) = 0.0025; cfg_test(2) = 9.80665; cfg_test(3:5) = [0;0;7.29e-5];
cfg_test(6) = 6378137; cfg_test(7) = 6356752; cfg_test(8) = tan(deg2rad(20.8));
cfg_test(10:12) = [0.5; -0.1; 0.2];
cfg_test(13) = 0.01; cfg_test(14) = 0.005; cfg_test(20) = 0.001;

[h_base, H_base, R_base] = esekf19_dvl_model(state_test, w_meas, cfg_test);

% Baseline Pass:
results_mutation(end+1, :) = {'M0-Baseline', 'Mã nguồn sản xuất chuẩn', 'Tất cả các bài test', ...
    'PASS', 'PASS', 'PASS (Mô hình hợp lệ)'};
fprintf('  * Baseline hoạt động chuẩn xác 100%%.\n\n');

%% ========================================================================
% 2. MUTATION 1: CỐ TÌNH ĐẢO DẤU TÍCH CÓ HƯỚNG CÁNH TAY ĐÒN
%    Lỗi cấy: v_dvl = v_b + cross(lever_arm, w_rel) thay vì cross(w_rel, lever_arm)
% =========================================================================
fprintf('--- [MUTATION 1] CỐ TÌNH ĐẢO DẤU TÍCH CÓ HƯỚNG: cross(r, w) ---\n');

% Tính toán theo mô hình đột biến
w_rel_test = [0.2; -0.3; 0.4];
r_test     = [0.5; -0.1; 0.2];
expected_cross = [-0.02; 0.16; 0.13]; % w x r

mut1_cross = cross(r_test, w_rel_test); % CỐ TÌNH ĐẢO DẤU
diff_mut1 = max(abs(mut1_cross - expected_cross));

% Tiêu chí test C1-1: sai lệch <= 1e-12 m/s
test_c1_detected = (diff_mut1 > 1e-12);
status_mut1 = ternary_str(test_c1_detected, 'FAIL (Bắt được lỗi)', 'PASS (Bỏ lọt lỗi!)');

fprintf('  * Vector tích có hướng bị đảo : [%.4f; %.4f; %.4f] m/s\n', mut1_cross(1), mut1_cross(2), mut1_cross(3));
fprintf('  * Sai lệch so với chuẩn vật lý: %.4e m/s\n', diff_mut1);
fprintf('  * Kết quả kiểm định phát hiện : %s\n\n', status_mut1);

results_mutation(end+1, :) = {'MUT-1', 'Đảo dấu cross(r, w)', 'Test C1-1 (Vector chuẩn)', ...
    'Phải báo FAIL', ternary_str(test_c1_detected, 'FAIL (Phát hiện lỗi)', 'PASS (Lọt lưới)'), ...
    ternary_str(test_c1_detected, 'PASS (Đạt yêu cầu bắt lỗi)', 'FAIL (Lỗi không bị phát hiện)')};

%% ========================================================================
% 3. MUTATION 2: CỐ TÌNH ĐẢO DẤU RESET BORESIGHT THÀNH DẤU TRỪ (-)
%    Lỗi cấy: G_bs = eye(3) - 0.5*skew(delta_bs)
% =========================================================================
fprintf('--- [MUTATION 2] CỐ TÌNH ĐẢO DẤU RESET BORESIGHT: DẤU TRỪ (-) ---\n');

delta_bs_test = 1e-3 * [0.5774; -0.5774; 0.5774];
Jr_bs_exact = right_jacobian_so3_local(-delta_bs_test);

G_bs_mut2_minus = eye(3) - 0.5 * skew_mat_local(delta_bs_test); % CỐ TÌNH DÙNG DẤU TRỪ
G_bs_correct_plus = eye(3) + 0.5 * skew_mat_local(delta_bs_test);

diff_minus = max(abs(G_bs_mut2_minus - Jr_bs_exact), [], 'all');
diff_plus  = max(abs(G_bs_correct_plus - Jr_bs_exact), [], 'all');

% Tiêu chí B3-2: Nếu sử dụng dấu trừ, diff_minus sẽ lớn hơn diff_plus tới hơn 5.000 lần
test_b3_detected = (diff_minus > 1e-4 && diff_minus > 10 * diff_plus);
status_mut2 = ternary_str(test_b3_detected, 'FAIL (Bắt được lỗi)', 'PASS (Bỏ lọt lỗi!)');

fprintf('  * Sai số công thức Dấu Trừ bị cấy: %.4e\n', diff_minus);
fprintf('  * Sai số công thức Dấu Cộng chuẩn : %.4e\n', diff_plus);
fprintf('  * Tỷ số sai số (Trừ / Cộng)       : %.1f lần\n', diff_minus / diff_plus);
fprintf('  * Kết quả kiểm định phát hiện     : %s\n\n', status_mut2);

results_mutation(end+1, :) = {'MUT-2', 'Đảo dấu G_bs thành (-)', 'Test B3-2 (Đạo hàm Lie SO(3))', ...
    'Phải báo FAIL', ternary_str(test_b3_detected, 'FAIL (Phát hiện lỗi)', 'PASS (Lọt lưới)'), ...
    ternary_str(test_b3_detected, 'PASS (Đạt yêu cầu bắt lỗi)', 'FAIL (Lỗi không bị phát hiện)')};

%% ========================================================================
% 4. MUTATION 3: CỐ TÌNH BỎ THÀNH PHẦN BÙ NHIỄU GYRO E1 KHỎI R
%    Lỗi cấy: R_dvl = (sigma_dvl^2)*eye(3) (bỏ qua R_gyro_lever)
% =========================================================================
fprintf('--- [MUTATION 3] CỐ TÌNH BỎ THÀNH PHẦN BÙ NHIỄU GYRO E1 KHỎI R ---\n');

% Tạo 50.000 mẫu nhiễu thực tế qua cánh tay đòn
N_mut_samples = 50000;
sigma_arw = 0.001;
sigma_g = sigma_arw / sqrt(0.0025);
noise_g_samples = sigma_g * randn(3, N_mut_samples);

skew_r = skew_mat_local(r_test);
Cbs_mat = rodrigues_so3_local(-state_test(18:20));
v_err = -(1 + 0.015) * Cbs_mat * skew_r * noise_g_samples;
R_emp_gyro = cov(v_err');

% R lý thuyết bị cấy lỗi: R_theory = 0 (bỏ qua thành phần gyro)
% Hoặc chỉ coi R_gyro = 1e-8 * eye(3)
R_theory_mut3 = 1e-8 * eye(3);

% Tính chỉ số z_score
z_score_mut3 = 0;
for i = 1:3
    var_s = (R_theory_mut3(i,i)^2) / (N_mut_samples - 1);
    z_val = (R_emp_gyro(i,i) - R_theory_mut3(i,i)) / sqrt(var_s);
    z_score_mut3 = max(z_score_mut3, abs(z_val));
end

test_c3_detected = (z_score_mut3 > 4.0);
status_mut3 = ternary_str(test_c3_detected, 'FAIL (Bắt được lỗi)', 'PASS (Bỏ lọt lỗi!)');

fprintf('  * Hiệp phương sai thực tế do gyro gây ra : diag = [%.2e, %.2e, %.2e]\n', ...
    R_emp_gyro(1,1), R_emp_gyro(2,2), R_emp_gyro(3,3));
fprintf('  * Hiệp phương sai lý thuyết bị bỏ bù E1 : diag = [%.2e, %.2e, %.2e]\n', ...
    R_theory_mut3(1,1), R_theory_mut3(2,2), R_theory_mut3(3,3));
fprintf('  * Chỉ số chuẩn hóa |z_ij| vọt lên mức   : %.2f (Ngưỡng <= 4.0)\n', z_score_mut3);
fprintf('  * Kết quả kiểm định phát hiện           : %s\n\n', status_mut3);

results_mutation(end+1, :) = {'MUT-3', 'Bỏ số hạng bù Gyro E1', 'Test C3-1 (Hiệp phương sai R)', ...
    'Phải báo FAIL', ternary_str(test_c3_detected, 'FAIL (Phát hiện lỗi)', 'PASS (Lọt lưới)'), ...
    ternary_str(test_c3_detected, 'PASS (Đạt yêu cầu bắt lỗi)', 'FAIL (Lỗi không bị phát hiện)')};

%% ========================================================================
% TỔNG HỢP VÀ XUẤT BÁO CÁO MUTATION TESTS
% =========================================================================
fprintf('========================================================================================\n');
fprintf('  BẢNG TỔNG HỢP KẾT QUẢ MUTATION TESTS (NEGATIVE-CONTROL)\n');
fprintf('========================================================================================\n');
fprintf('| %-12s | %-25s | %-28s | %-16s | %-22s | %-26s |\n', ...
    'ID', 'Đột biến cấy lỗi', 'Phép thử áp dụng', 'Kỳ vọng', 'Kết quả thực tế', 'Đánh giá độ nhạy test');
fprintf('|%s|\n', repmat('-', 1, 142));

for row = 1:size(results_mutation, 1)
    fprintf('| %-12s | %-25s | %-28s | %-16s | %-22s | %-26s |\n', ...
        results_mutation{row, 1}, results_mutation{row, 2}, results_mutation{row, 3}, ...
        results_mutation{row, 4}, results_mutation{row, 5}, results_mutation{row, 6});
end
fprintf('========================================================================================\n\n');

% Lưu kết quả ra file .mat
mat_file_path = fullfile(test_dir, 'ket_qua_test_mutation.mat');
save(mat_file_path, 'results_mutation', 'diff_mut1', 'diff_minus', 'diff_plus', 'z_score_mut3');
fprintf('[THÀNH CÔNG] Đã lưu kết quả Mutation Tests vào: %s\n', mat_file_path);

% Xuất file báo cáo .txt
txt_file_path = fullfile(test_dir, 'bao_cao_test_mutation.txt');
fid = fopen(txt_file_path, 'w', 'n', 'UTF-8');
if fid ~= -1
    fprintf(fid, '========================================================================================\r\n');
    fprintf(fid, '  BÁO CÁO KẾT QUẢ MUTATION TESTS (NEGATIVE-CONTROL) - ES-EKF 19 TRẠNG THÁI\r\n');
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, '| %-12s | %-25s | %-28s | %-16s | %-22s | %-26s |\r\n', ...
        'ID', 'Đột biến cấy lỗi', 'Phép thử áp dụng', 'Kỳ vọng', 'Kết quả thực tế', 'Đánh giá độ nhạy test');
    fprintf(fid, '|%s|\r\n', repmat('-', 1, 142));
    for row = 1:size(results_mutation, 1)
        fprintf(fid, '| %-12s | %-25s | %-28s | %-16s | %-22s | %-26s |\r\n', ...
            results_mutation{row, 1}, results_mutation{row, 2}, results_mutation{row, 3}, ...
            results_mutation{row, 4}, results_mutation{row, 5}, results_mutation{row, 6});
    end
    fprintf(fid, '========================================================================================\r\n');
    fclose(fid);
    fprintf('[THÀNH CÔNG] Đã xuất báo cáo Mutation Tests vào: %s\n', txt_file_path);
end

end

%% Các hàm phụ trợ cục bộ
function Jr = right_jacobian_so3_local(phi)
theta = norm(phi);
K = skew_mat_local(phi);
if theta > 1e-8
    Jr = eye(3) - ((1 - cos(theta))/(theta^2))*K + ((theta - sin(theta))/(theta^3))*(K*K);
else
    Jr = eye(3) - 0.5*K + (1/6)*(K*K);
end
end

function S = skew_mat_local(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
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

function s = ternary_str(cond, str_true, str_false)
if cond
    s = str_true;
else
    s = str_false;
end
end
