% kichban_calib.m
% Kịch bản Calibration Offline: Chạy hình số 8 trong 4 phút kết hợp tăng giảm tốc độ.
% Sử dụng Bình phương tối thiểu (BPTTT) trên toàn bộ dữ liệu để tính k, beta, gamma.

clear all; close all; clc;

disp('1. Khởi tạo cấu hình chạy Calibration (Hình Số 8, 4 phút)...');
p = thong_so_he_thong_chuan();

% Thời gian kịch bản
T_can_chinh = 60;   % 60s đứng yên căn chỉnh
T_run = 240;        % 4 phút chạy hình số 8
T_total = T_can_chinh + T_run;

rng(1); % Cố định nhiễu IMU/DVL

disp('2.  mô phỏng dữ liệu IMU & DVL ...');
%% ================= PHẦN 1: MÔ PHỎNG DỮ LIỆU =================
dt  = p.si.dt_ins;
fs  = p.ins.gyro.tan_so_Hz;
g   = p.dat.g;
gn  = [0; 0; g];                         
W   = p.dat.omega_ie;
a_e = p.dat.a;
e2  = p.dat.e2;
s   = p.si;

N  = round(T_total * fs);
t  = (0:N-1)' * dt;

pha = zeros(N, 1);
pha(t >= T_can_chinh) = 3; % Pha 3: Chạy động
k0 = find(pha == 0, 1, 'last');

% TẠO QUỸ ĐẠO HÌNH SỐ 8 VÀ TĂNG GIẢM TỐC
V = zeros(N, 1);
yaw_rate = zeros(N, 1);
for i = 1:N
    if t(i) >= T_can_chinh
        tr = t(i) - T_can_chinh;
        % Hình số 8: yaw_rate dao động hình sin, chu kỳ 120s, biên độ 8 độ/s
        yaw_rate(i) = (8 * pi/180) * sin(2 * pi * tr / 120);
        % Tăng giảm tốc độ: dao động hình sin, chu kỳ 60s, từ 1.0 đến 2.0 m/s
        V(i) = 1.5 + 0.5 * sin(2 * pi * tr / 60);
    end
end

yaw = zeros(N, 1);
yaw(1) = 30 * pi/180; % Hướng ban đầu 30 độ
for i = 2:N
    yaw(i) = yaw(i-1) + yaw_rate(i) * dt;
end

v_n = zeros(N, 3);
for i = 1:N
    cy = cos(yaw(i)); sy = sin(yaw(i));
    Cnb_y = [cy sy 0; -sy cy 0; 0 0 1];
    v_n(i, :) = (Cnb_y' * [V(i); 0; 0])';
end

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

bg = s.gyro_bias_sigma * randn(3, 1);
Kg = eye(3) + diag(s.gyro_scale_err * randn(3, 1)) + s.gyro_cross_axis * (randn(3) .* ~eye(3));
ba = s.acc_bias_sigma * randn(3, 1);
Ka = eye(3) + diag(s.acc_scale_err * randn(3, 1)) + s.acc_cross_axis * (randn(3) .* ~eye(3));
phi_gm = exp(-dt / s.acc_bias_tau);
q_gm   = s.acc_bias_inrun_sigma * sqrt(1 - phi_gm^2);
b_gm   = s.acc_bias_inrun_sigma * randn(3, 1);
sd_g   = s.gyro_arw / sqrt(dt);
sd_a   = s.acc_vrw  / sqrt(dt);

w_do = zeros(N, 3);
f_do = zeros(N, 3);
for i = 1:N
    b_gm = phi_gm * b_gm + q_gm * randn(3, 1);
    w_do(i, :) = (Kg * w_that(i, :)' + bg + sd_g * randn(3, 1))';
    f_do(i, :) = (Ka * f_that(i, :)' + ba + b_gm + sd_a * randn(3, 1))';
end

f_tb = mean(f_do(1:k0, :), 1)';
w_tb = mean(w_do(1:k0, :), 1)';
t2_n = cross([0; 0; 1], [cos(lat(1)); 0; -sin(lat(1))]); t2_n = t2_n / norm(t2_n);
Tn = [[0; 0; 1], t2_n, cross([0; 0; 1], t2_n)];
u1_b = -f_tb / norm(f_tb); u2_b = w_tb / norm(w_tb);
t2_b = cross(u1_b, u2_b); t2_b = t2_b / norm(t2_b);
Tb = [u1_b, t2_b, cross(u1_b, t2_b)];
Cbn0 = Tn * Tb';
q0 = [0.5*sqrt(trace(Cbn0)+1); (Cbn0(3,2)-Cbn0(2,3))/(2*sqrt(trace(Cbn0)+1)); (Cbn0(1,3)-Cbn0(3,1))/(2*sqrt(trace(Cbn0)+1)); (Cbn0(2,1)-Cbn0(1,2))/(2*sqrt(trace(Cbn0)+1))];
q0 = q0 / norm(q0);

Q_ins  = repmat(q0', N, 1);
Vn_ins = zeros(N, 3);
q  = q0;
v  = [0; 0; 0];
la = lat(k0); lo = lon(k0); hh = h(k0);
for i = k0+1:N
    sl2 = sin(la)^2;
    RN = a_e / sqrt(1 - e2 * sl2);
    RM = a_e * (1 - e2) / (1 - e2 * sl2)^1.5;
    w_ie = W * [cos(la); 0; -sin(la)];
    w_en = [v(2) / (RN + hh); -v(1) / (RM + hh); -v(2) * tan(la) / (RN + hh)];
    
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    Cnb = [1-2*(qy*qy+qz*qz), 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy); ...
         2*(qx*qy-qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz+qw*qx); ...
         2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), 1-2*(qx*qx+qy*qy)];
         
    w_nb = w_do(i, :)' - Cnb * (w_ie + w_en);
    dq = [1; 0.5 * w_nb * dt];
    q = [ q(1) -q(2) -q(3) -q(4); q(2) q(1) -q(4) q(3); q(3) q(4) q(1) -q(2); q(4) -q(3) q(2) q(1) ] * dq;
    q = q / norm(q);
    Cbn = [1-2*(q(3)*q(3)+q(4)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3)); ...
         2*(q(2)*q(3)+q(1)*q(4)), 1-2*(q(2)*q(2)+q(4)*q(4)), 2*(q(3)*q(4)-q(1)*q(2)); ...
         2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 1-2*(q(2)*q(2)+q(3)*q(3))];
    
    v_moi = v + (Cbn * f_do(i, :)' - cross(2*w_ie + w_en, v) + gn) * dt;
    la_moi = la + v(1) / (RM + hh) * dt;
    lo = lo + v(2) / ((RN + hh) * cos(la)) * dt;
    hh = hh - v(3) * dt;
    la = la_moi;
    v  = v_moi;
    Q_ins(i, :)  = q';
    Vn_ins(i, :) = v';
end

buoc    = round(fs / p.dvl.ping_Hz);
idx_dvl = (1:buoc:N)';
ndvl    = numel(idx_dvl);
t_dvl   = t(idx_dvl);
pha_dvl = pha(idx_dvl);
bs = s.boresight_that_rad;
Cbs_that = eye(3) - [0 -bs(3) bs(2); bs(3) 0 -bs(1); -bs(2) bs(1) 0];
Z_dvl = zeros(ndvl, 3);
for j = 1:ndvl
    i  = idx_dvl(j);
    cy = cos(yaw(i)); sy = sin(yaw(i));
    Cnb_y = [cy sy 0; -sy cy 0; 0 0 1];
    vb = Cnb_y * v_n(i, :)';
    Z_dvl(j, :) = ((1 + s.k_that) * Cbs_that * vb + s.sigma_dvl_fn(norm(vb)) * randn(3, 1))';
end

disp('3. Giải Offline và vẽ Quá trình hội tụ BPTTT...');
%% ================= PHẦN 2: GIẢI VÀ TÍNH HỘI TỤ =================
m3 = find(pha_dvl == 3);
n3 = numel(m3);
t_run_dvl = t_dvl(m3) - T_can_chinh;

v_ins_b = zeros(n3, 3);
for j = 1:n3
    i = idx_dvl(m3(j));
    q = Q_ins(i, :)';
    w = q(1); x = q(2); y = q(3); z = q(4);
    Cnb = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y); ...
         2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x); ...
         2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)];
    v_ins_b(j, :) = (Cnb * Vn_ins(i, :)')';
end

A = zeros(3*n3, 3);
b = zeros(3*n3, 1);

k_cum = NaN(n3, 1);
beta_cum = NaN(n3, 1);
gamma_cum = NaN(n3, 1);

for j = 1:n3
    v_b = v_ins_b(j, :)';
    r = 3*(j-1) + (1:3);
    A(r, :) = [v_b, [-v_b(3); 0; v_b(1)], [v_b(2); -v_b(1); 0]];
    b(r) = Z_dvl(m3(j), :)' - v_b;
    
    if j > 20 % Đợi khoảng 4 giây
        x_step = A(1:3*j, :) \ b(1:3*j);
        k_cum(j) = x_step(1) * 100;
        beta_cum(j) = (x_step(2) / (1 + x_step(1))) * 180/pi;
        gamma_cum(j) = (x_step(3) / (1 + x_step(1))) * 180/pi;
    end
end

% Giá trị cuối cùng
x_est = A(1:3*n3, :) \ b(1:3*n3);
k_est = x_est(1) * 100;
beta_est = (x_est(2) / (1 + x_est(1))) * 180/pi;
gamma_est = (x_est(3) / (1 + x_est(1))) * 180/pi;

k_that = p.si.k_that * 100;
beta_that = p.dvl.boresight_that_deg(2);
gamma_that = p.dvl.boresight_that_deg(3);

fprintf('\n======= KẾT QUẢ CALIBRATION OFFLINE (4 PHÚT SỐ 8) =======\n');
fprintf('Tham số            | Giá trị thật | Ước lượng BPTTT | Sai số\n');
fprintf('-------------------------------------------------------------\n');
fprintf('Hệ số k (%%)        | %12.4f | %15.4f | %8.4f\n', k_that, k_est, k_est - k_that);
fprintf('Pitch Beta (deg)   | %12.4f | %15.4f | %8.4f\n', beta_that, beta_est, beta_est - beta_that);
fprintf('Yaw Gamma (deg)    | %12.4f | %15.4f | %8.4f\n', gamma_that, gamma_est, gamma_est - gamma_that);
fprintf('=============================================================\n');

%% ================= PHẦN 3: VẼ ĐỒ THỊ =================
figure('Name', 'CALIBRATION OFFLINE (QUỸ ĐẠO VÀ ĐỘNG HỌC)', 'Color', 'w', 'Position', [50 100 700 600]);

% (a) Quỹ đạo tàu
subplot(1, 2, 1);
dt_ins = p.si.dt_ins;
pos_N = cumsum(Vn_ins(:, 1)) * dt_ins;
pos_E = cumsum(Vn_ins(:, 2)) * dt_ins;
plot(pos_E, pos_N, 'b-', 'LineWidth', 1.5);
hold on; grid on; axis equal;
plot(pos_E(1), pos_N(1), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
title('Quỹ đạo Calibration (Hình Số 8)');
xlabel('Đông (m)');
ylabel('Bắc (m)');
legend('Quỹ đạo', 'Điểm xuất phát', 'Location', 'best');

% (b) Động học tàu
subplot(2, 2, 2);
t_run_axis = t(t >= T_can_chinh) - T_can_chinh;
plot(t_run_axis, V(t >= T_can_chinh), 'b-', 'LineWidth', 1.5);
grid on;
title('Vận tốc dọc trục Surge (m/s)');
ylabel('V (m/s)');

subplot(2, 2, 4);
plot(t_run_axis, yaw_rate(t >= T_can_chinh) * 180/pi, 'r-', 'LineWidth', 1.5);
grid on;
title('Tốc độ quay Yaw (deg/s)');
xlabel('Thời gian (s)');
ylabel('\omega_z (deg/s)');

sgtitle('KỊCH BẢN CALIBRATION: HÌNH SỐ 8 (4 PHÚT)', 'FontSize', 14, 'FontWeight', 'bold');

% ĐỒ THỊ 2: HỘI TỤ
figure('Name', 'QUÁ TRÌNH HỘI TỤ BORESIGHT VÀ SCALE FACTOR', 'Color', 'w', 'Position', [800 100 600 600]);

subplot(3, 1, 1);
plot(t_run_dvl, k_cum, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(k_that, 'r--', 'LineWidth', 1.5);
title('Hội tụ Hệ số Tỷ lệ k (%)');
ylabel('k (%)');

subplot(3, 1, 2);
plot(t_run_dvl, beta_cum, 'g-', 'LineWidth', 1.5); hold on; grid on;
yline(beta_that, 'r--', 'LineWidth', 1.5);
title('Hội tụ Pitch Boresight \beta');
ylabel('\beta (độ)');

subplot(3, 1, 3);
plot(t_run_dvl, gamma_cum, 'm-', 'LineWidth', 1.5); hold on; grid on;
yline(gamma_that, 'r--', 'LineWidth', 1.5);
title('Hội tụ Yaw Boresight \gamma');
xlabel('Thời gian Calibration (s)');
ylabel('\gamma (độ)');

sgtitle('QUÁ TRÌNH HỘI TỤ BPTTT DỮ LIỆU OFFLINE', 'FontSize', 14, 'FontWeight', 'bold');
