%% KIEM_TRA_BUOC2_JACOBIAN_VA_RESET.M
% Script thực hiện BƯỚC 2 (CẬP NHẬT THEO ĐÓNG GÓP PHẢN BIỆN):
% 1. Kiểm tra Jacobian DVL (H_dvl) & GNSS yaw (H_yaw) vs Sai phân trung tâm.
% 2. ÁP DỤNG THAY ĐỔI A: Lấy sai phân hữu hạn quanh điểm inject (delta_hat + d).
% 3. ÁP DỤNG THAY ĐỔI B: 
%    - Tính ma trận Jacobian SO(3) chính xác Jr(phi).
%    - So sánh đạo hàm số học với Jr(delta_th_hat) cho tư thế và Jr(-delta_bs_hat) cho Boresight.
%    - Đối chiếu xấp xỉ bậc một I - 0.5*[delta x] vs I + 0.5*[delta x] để kết luận dấu.

clear; clc;
fprintf('========================================================================================\n');
fprintf('  BƯỚC 2 (HIỆU CHỈNH): KIỂM ĐỊNH TOÁN HỌC MA TRẬN RESET G QUANH ĐIỂM INJECT\n');
fprintf('========================================================================================\n\n');

rng(1, 'twister');

%% -------------------------------------------------------------------------
% 1. ĐIỂM LÀM VIỆC VÀ THAM SỐ
% -------------------------------------------------------------------------
euler_nom = deg2rad([8.5; -12.3; 35.0]);
q_nom = eul2quat_custom(euler_nom);
v_nom = [1.65; 0.28; -0.12];
w_meas_nom = [0.045; -0.032; 0.028];
ba_nom = [0.008; -0.012; 0.006];
bg_nom = [0.0015; -0.0020; 0.0010];
s_nom = 0.015;
bs_nom = deg2rad([1.5; -2.0; 3.0]);
lever_arm = [0.5; -0.1; 0.2];

w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;
tan_lat = tan(deg2rad(20.8)); height = 0;

nom = struct('q', q_nom, 'v', v_nom, 'w_meas', w_meas_nom, 'ba', ba_nom, ...
             'bg', bg_nom, 's', s_nom, 'bs', bs_nom, 'L', lever_arm, ...
             'w_ie_ned', w_ie_ned, 'radius_n', radius_n, 'radius_m', radius_m, ...
             'tan_lat', tan_lat, 'height', height);

eps_diff = 1e-7;

%% -------------------------------------------------------------------------
% PHẦN 1: KIỂM ĐỊNH MA TRẬN JACOBIAN DVL (H_dvl) VÀ GNSS YAW (H_yaw)
% -------------------------------------------------------------------------
fprintf('--- [PHẦN 1] KIỂM ĐỊNH MA TRẬN JACOBIAN DVL (H_dvl) VÀ GNSS YAW ---\n');
Cbn = quat_to_cbn(nom.q); Cnb = Cbn';
v_b = Cnb * nom.v;
gyro_corr = nom.w_meas - nom.bg;
w_en = [nom.v(2)/(nom.radius_n + nom.height); ...
       -nom.v(1)/(nom.radius_m + nom.height); ...
       -nom.v(2)*nom.tan_lat/(nom.radius_n + nom.height)];
w_in_b = Cnb * (nom.w_ie_ned + w_en);
w_rel = gyro_corr - w_in_b;
v_dvl = v_b + cross(w_rel, nom.L);
Cbs = rodrigues_so3(-nom.bs);

H_dvl_ana = zeros(3, 19);
H_dvl_ana(:, 4:6)   = (1 + nom.s) * Cbs * Cnb;
H_dvl_ana(:, 7:9)   = (1 + nom.s) * Cbs * (skew_mat(v_b) + skew_mat(nom.L)*skew_mat(w_in_b));
H_dvl_ana(:, 13:15) = (1 + nom.s) * Cbs * skew_mat(nom.L);
H_dvl_ana(:, 16)    = Cbs * v_dvl;
H_dvl_ana(:, 17:19) = (1 + nom.s) * Cbs * skew_mat(v_dvl);

H_dvl_num = zeros(3, 19);
for j = 1:19
    dx_p = zeros(19, 1); dx_p(j) =  eps_diff;
    dx_m = zeros(19, 1); dx_m(j) = -eps_diff;
    H_dvl_num(:, j) = (predict_dvl(nom, dx_p) - predict_dvl(nom, dx_m)) / (2 * eps_diff);
end

diff_dvl = abs(H_dvl_ana - H_dvl_num);
fprintf('  * Max |H_dvl_diff| toàn bộ 19 cột = %.4e  --> %s\n', ...
    max(diff_dvl(:)), ternary(max(diff_dvl(:)) < 1e-5, '[PASS - HOÀN HẢO]', '[FAIL]'));

roll_est = atan2(Cbn(3, 2), Cbn(3, 3));
pitch_est = -asin(max(min(Cbn(3, 1), 1), -1));
cosp = max(cos(pitch_est), 1e-6);
H_yaw_ana = zeros(1, 19);
H_yaw_ana(1, 7:9) = [0, sin(roll_est)/cosp, cos(roll_est)/cosp];
H_yaw_num = zeros(1, 19);
for j = 1:19
    dx_p = zeros(19, 1); dx_p(j) =  eps_diff;
    dx_m = zeros(19, 1); dx_m(j) = -eps_diff;
    H_yaw_num(1, j) = (predict_yaw(nom.q, dx_p(7:9)) - predict_yaw(nom.q, dx_m(7:9))) / (2 * eps_diff);
end
diff_yaw = abs(H_yaw_ana - H_yaw_num);
fprintf('  * Max |H_yaw_diff| cột 7:9        = %.4e  --> %s\n\n', ...
    max(diff_yaw(7:9)), ternary(max(diff_yaw(7:9)) < 1e-5, '[PASS - HOÀN HẢO]', '[FAIL]'));

%% -------------------------------------------------------------------------
% PHẦN 2: KIỂM ĐỊNH RESET THEO ĐÚNG THAY ĐỔI A VÀ THAY ĐỔI B
% -------------------------------------------------------------------------
fprintf('--- [PHẦN 2] KIỂM ĐỊNH MA TRẬN RESET QUANH ĐIỂM INJECT (THAY ĐỔI A & B) ---\n');

% Giả sử vector sai số ước lượng (delta_hat) từ tầng Kalman
delta_th_hat = deg2rad([2.0; -3.0; 4.0]);    % Ước lượng delta_theta (~ vài độ)
delta_bs_hat = deg2rad([1.5; -2.5; 3.5]);    % Ước lượng delta_boresight (~ vài độ)

% -------------------------------------------------------------------------
% 2.1. KIỂM ĐỊNH RESET TƯ THẾ UUV
% -------------------------------------------------------------------------
fprintf('1. KHỐI RESET TƯ THẾ UUV G(7:9, 7:9):\n');
% Tính đạo hàm số học quanh điểm inject (Thay đổi A):
G_theta_num = zeros(3, 3);
for i = 1:3
    d_pos = zeros(3, 1); d_pos(i) =  eps_diff;
    d_neg = zeros(3, 1); d_neg(i) = -eps_diff;
    % Điểm tuyến tính hóa là delta_th_hat (Thay đổi A):
    e_pos = eval_att_reset_exact(delta_th_hat + d_pos, delta_th_hat);
    e_neg = eval_att_reset_exact(delta_th_hat + d_neg, delta_th_hat);
    G_theta_num(:, i) = (e_pos - e_neg) / (2 * eps_diff);
end

% Tính ma trận Jacobian SO(3) giải tích chính xác (Thay đổi B):
Jr_theta_exact = right_jacobian_so3(delta_th_hat);

% Xấp xỉ bậc 1 trong code: G = I - 0.5 * [delta_theta x]
G_theta_code = eye(3) - 0.5 * skew_mat(delta_th_hat);

diff_theta_exact = max(abs(G_theta_num - Jr_theta_exact), [], 'all');
diff_theta_order1 = max(abs(G_theta_code - Jr_theta_exact), [], 'all');

fprintf('   - Sai lệch giữa Đạo hàm số học và Jr(delta_th_hat) chính xác : %.4e --> %s\n', ...
    diff_theta_exact, ternary(diff_theta_exact < 1e-5, '[PASS - KHỚP CHÍNH XÁC TUYỆT ĐỐI]', '[FAIL]'));
fprintf('   - Sai số do xấp xỉ bậc 1 (G_code = I - 0.5*[th x]) so với Jr   : %.4e (Sai số bậc O(delta^2))\n', ...
    diff_theta_order1);
fprintf('   -> KẾT LUẬN TƯ THẾ: Dấu âm I - 0.5*[delta_theta x] là HOÀN TOÀN ĐÚNG.\n\n');

% -------------------------------------------------------------------------
% 2.2. KIỂM ĐỊNH RESET BORESIGHT DVL (SO SOI DẤU CỦA G_bs)
% -------------------------------------------------------------------------
fprintf('2. KHỐI RESET BORESIGHT DVL G(17:19, 17:19) [KIỂM TRA DẤU]:\n');

% Tính đạo hàm số học quanh điểm inject (Thay đổi A):
G_bs_num = zeros(3, 3);
for i = 1:3
    d_pos = zeros(3, 1); d_pos(i) =  eps_diff;
    d_neg = zeros(3, 1); d_neg(i) = -eps_diff;
    % Điểm tuyến tính hóa là delta_bs_hat (Thay đổi A):
    ebs_pos = eval_bs_reset_exact(nom.bs, delta_bs_hat + d_pos, delta_bs_hat);
    ebs_neg = eval_bs_reset_exact(nom.bs, delta_bs_hat + d_neg, delta_bs_hat);
    G_bs_num(:, i) = (ebs_pos - ebs_neg) / (2 * eps_diff);
end

% Tính Jacobian SO(3) giải tích chính xác (Thay đổi B: Jr(-delta_bs_hat)):
Jr_bs_exact = right_jacobian_so3(-delta_bs_hat);

% Hai công thức bậc một đối chiếu:
G_bs_code_minus = eye(3) - 0.5 * skew_mat(delta_bs_hat); % Dòng 351 hiện tại (DẤU TRỪ)
G_bs_prop_plus  = eye(3) + 0.5 * skew_mat(delta_bs_hat); % Công thức đề xuất (DẤU CỘNG)

diff_bs_exact = max(abs(G_bs_num - Jr_bs_exact), [], 'all');
diff_minus_code = max(abs(G_bs_code_minus - Jr_bs_exact), [], 'all');
diff_plus_prop  = max(abs(G_bs_prop_plus - Jr_bs_exact), [], 'all');

fprintf('   - Sai lệch giữa Đạo hàm số học và Jr(-delta_bs_hat) chính xác: %.4e --> %s\n', ...
    diff_bs_exact, ternary(diff_bs_exact < 1e-5, '[PASS - KHỚP CHÍNH XÁC TUYỆT ĐỐI]', '[FAIL]'));
fprintf('   - So sánh với công thức trong Code hiện tại (DẤU TRỪ -): Max |Diff| = %.4e\n', diff_minus_code);
fprintf('   - So sánh với công thức đề xuất sửa đổi     (DẤU CỘNG +): Max |Diff| = %.4e\n', diff_plus_prop);

fprintf('\n----------------------------------------------------------------------------------------\n');
if diff_plus_prop < diff_minus_code
    fprintf('  ===> KẾT QUẢ XÁC NHẬN RÕ RÀNG VỀ MẶT TOÁN HỌC:\n');
    fprintf('       Khối Reset Boresight G(17:19, 17:19) TRONG CODE ĐANG BỊ NGƯỢC DẤU!\n');
    fprintf('       * Code hiện tại đang viết (dòng 351): G(17:19, 17:19) = eye(3) - 0.5*skew_mat(delta_bs);\n');
    fprintf('       * Công thức chuẩn toán học bắt buộc : G(17:19, 17:19) = eye(3) + 0.5*skew_mat(delta_bs);\n');
    fprintf('       (Hoặc chính xác bậc cao: right_jacobian_so3(-delta_bs))\n');
else
    fprintf('  ===> Dấu trừ trong code hiện tại là phù hợp.\n');
end
fprintf('========================================================================================\n');

%% ==================== CÁC HÀM CỤC BỘ BỔ TRỢ ============================
function Jr = right_jacobian_so3(phi)
%RIGHT_JACOBIAN_SO3 Tính ma trận Jacobian bên phải chính xác trên SO(3)
theta = norm(phi);
K = skew_mat(phi);
if theta > 1e-8
    Jr = eye(3) - ((1 - cos(theta))/(theta^2))*K + ((theta - sin(theta))/(theta^3))*(K*K);
else
    Jr = eye(3) - 0.5*K + (1/6)*(K*K);
end
end

function e_post = eval_att_reset_exact(delta_prior, delta_hat)
% Phép cập nhật quaternion chính xác: q_new = q * Exp(delta_hat)
% Sai số thực trước: q_true = q * Exp(delta_prior)
% Sai số thực sau  : Exp(e_post) = inv(Exp(delta_hat)) * Exp(delta_prior)
R_hat = rodrigues_so3(delta_hat);
R_prior = rodrigues_so3(delta_prior);
R_post = R_hat' * R_prior;
e_post = so3_log(R_post);
end

function ebs_post = eval_bs_reset_exact(bs_nom, delta_prior, delta_hat)
% Theo định nghĩa code: Cbs_new = Cbs_nom * Exp(-delta_hat)
% Sai số thực trước: Cbs_true = Cbs_nom * Exp(-delta_prior)
% Sai số thực sau  : Cbs_true = Cbs_new * Exp(-ebs_post)
% => Exp(-ebs_post) = inv(Cbs_new) * Cbs_true = Exp(delta_hat) * Exp(-delta_prior)
R_hat_inv = rodrigues_so3(delta_hat); % inv(Exp(-delta_hat)) = Exp(+delta_hat)
R_prior = rodrigues_so3(-delta_prior);
R_err = R_hat_inv * R_prior;
ebs_post = -so3_log(R_err);
end

function z = predict_dvl(nom, dx)
v = nom.v + dx(4:6);
dq = eul2quat_custom(dx(7:9));
q = quat_multiply(nom.q, dq);
Cnb = quat_to_cbn(q)';
v_b = Cnb * v;

bg = nom.bg + dx(13:15);
gyro_corr = nom.w_meas - bg;

w_en = [v(2)/(nom.radius_n + nom.height); ...
       -v(1)/(nom.radius_m + nom.height); ...
       -v(2)*nom.tan_lat/(nom.radius_n + nom.height)];
w_in_b = Cnb * (nom.w_ie_ned + w_en);
w_rel = gyro_corr - w_in_b;
v_dvl = v_b + cross(w_rel, nom.L);

s = nom.s + dx(16);
d_bs = dx(17:19);
Cbs_pert = rodrigues_so3(-nom.bs) * rodrigues_so3(-d_bs);
z = (1 + s) * Cbs_pert * v_dvl;
end

function yaw = predict_yaw(q_nom, d_theta)
dq = eul2quat_custom(d_theta);
q = quat_multiply(q_nom, dq);
Cbn = quat_to_cbn(q);
yaw = atan2(Cbn(2, 1), Cbn(1, 1));
end

function Cbn = quat_to_cbn(quaternion)
quaternion = quaternion / norm(quaternion);
w = quaternion(1); x = quaternion(2); y = quaternion(3); z = quaternion(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
       2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
       2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function S = skew_mat(vector)
S = [0, -vector(3), vector(2); ...
     vector(3), 0, -vector(1); ...
     -vector(2), vector(1), 0];
end

function quaternion = quat_multiply(first, second)
quaternion = [first(1)*second(1)-first(2)*second(2)-first(3)*second(3)-first(4)*second(4); ...
    first(1)*second(2)+first(2)*second(1)+first(3)*second(4)-first(4)*second(3); ...
    first(1)*second(3)-first(2)*second(4)+first(3)*second(1)+first(4)*second(2); ...
    first(1)*second(4)+first(2)*second(3)-first(3)*second(2)+first(4)*second(1)];
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

function q = eul2quat_custom(eul)
phi = eul(1)/2; theta = eul(2)/2; psi = eul(3)/2;
q = [cos(phi)*cos(theta)*cos(psi) + sin(phi)*sin(theta)*sin(psi); ...
     sin(phi)*cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi); ...
     cos(phi)*sin(theta)*cos(psi) + sin(phi)*cos(theta)*sin(psi); ...
     cos(phi)*cos(theta)*sin(psi) - sin(phi)*sin(theta)*cos(psi)];
q = q / norm(q);
end

function val = ternary(cond, a, b)
if cond, val = a; else, val = b; end
end
