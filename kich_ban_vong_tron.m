% kich_ban_vong_tron.m
% Sinh dữ liệu 20 phút chạy vòng tròn, sau đó tính EKF/BPTTT và vẽ đồ thị hội tụ
% Kịch bản này độc lập hoàn toàn, không phụ thuộc vào mo_phong_sinh_du_lieu_full.m

clear all; close all; clc;

% 1. Thiết lập tham số kịch bản
disp('1. Khởi tạo cấu hình chạy vòng tròn 20 phút...');
p = thong_so_he_thong_chuan();
p.kich_ban.t_can_chinh_s = 60;    % Căn chỉnh thô 60s
p.kich_ban.t_tang_toc_s = 10;     % Tăng tốc 10s
p.kich_ban.t_thang_s = 10;        % Chạy thẳng lấy đà 10s
p.kich_ban.t_quay_s = 1200;       % Quay vòng 20 phút (1200s)
p.kich_ban.toc_do_quay_deg_s = 6; % Quay 6 độ/s
p.kich_ban.van_toc_mps = 1.5;     % Chạy 1.5 m/s

rng(1); % Cố định nhiễu

disp('2. Đang mô phỏng dữ liệu IMU & DVL (có thể mất vài chục giây)...');
%% ================= TỪ ĐÂY LÀ PHẦN MÔ PHỎNG DỮ LIỆU =================
dt  = p.si.dt_ins;
fs  = p.ins.gyro.tan_so_Hz;
g   = p.dat.g;
gn  = [0; 0; g];                         % trọng lực khung NED, z xuống
W   = p.dat.omega_ie;
a_e = p.dat.a;
e2  = p.dat.e2;
kb  = p.kich_ban;
s   = p.si;

T0 = kb.t_can_chinh_s;                   
T1 = T0 + kb.t_tang_toc_s;               
T2 = T1 + kb.t_thang_s;                  
T3 = T2 + kb.t_quay_s;                   
N  = round(T3 * fs);
t  = (0:N-1)' * dt;

pha = zeros(N, 1);
pha(t >= T0) = 1;
pha(t >= T1) = 2;
pha(t >= T2) = 3;
k0 = find(pha == 0, 1, 'last');

% Quỹ đạo thật
V = zeros(N, 1);
yaw_rate = zeros(N, 1);
V(pha == 1) = kb.van_toc_mps * (t(pha == 1) - T0) / kb.t_tang_toc_s;
V(pha >= 2) = kb.van_toc_mps;
yaw_rate(pha == 3) = kb.toc_do_quay_deg_s * pi/180;

yaw = zeros(N, 1);
yaw(1) = kb.huong_dau_deg * pi/180;
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

disp('3. Bắt đầu tính toán ước lượng BPTTT trên quỹ đạo vòng tròn...');
%% ================= TỪ ĐÂY LÀ PHẦN ƯỚC LƯỢNG VÀ VẼ ĐỒ THỊ =================
m3 = find(pha_dvl == 3);
n3 = numel(m3);
t_pha3 = t_dvl(m3);

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

k_cum = NaN(n3, 1);
beta_cum = NaN(n3, 1);
gamma_cum = NaN(n3, 1);

A = zeros(3*n3, 3);
b = zeros(3*n3, 1);

for k_step = 1:n3
    v = v_ins_b(k_step, :)';
    r = 3*(k_step-1) + (1:3);
    A(r, :) = [v, [-v(3); 0; v(1)], [v(2); -v(1); 0]];
    b(r) = Z_dvl(m3(k_step), :)' - v;
    
    if k_step > 50
        x_est = A(1:3*k_step, :) \ b(1:3*k_step);
        k_cum(k_step) = x_est(1) * 100;
        beta_cum(k_step) = (x_est(2) / (1 + x_est(1))) * 180/pi;
        gamma_cum(k_step) = (x_est(3) / (1 + x_est(1))) * 180/pi;
    end
end

disp('4. Đang hiển thị Đồ thị...');
k_that_pct = p.si.k_that * 100;
beta_that_deg = p.dvl.boresight_that_deg(2);
gamma_that_deg = p.dvl.boresight_that_deg(3);

figure('Name', 'QUỸ ĐẠO VÀ HỘI TỤ (VÒNG TRÒN 20 PHÚT)', 'Color', 'w', 'Position', [50 50 1200 600]);

% (a) Quỹ đạo
subplot(1, 2, 1);
dt_ins = p.si.dt_ins;
pos_N = cumsum(Vn_ins(:, 1)) * dt_ins;
pos_E = cumsum(Vn_ins(:, 2)) * dt_ins;
plot(pos_E, pos_N, 'b-', 'LineWidth', 1.5);
hold on; grid on; axis equal;
plot(pos_E(1), pos_N(1), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
title('Quỹ đạo tàu (Local NED)');
xlabel('Đông (m)');
ylabel('Bắc (m)');
legend('Quỹ đạo INS', 'Điểm xuất phát');

% (b) Đồ thị hội tụ
time_x = t_pha3 - t_pha3(1);

subplot(3, 2, 2);
plot(time_x, k_cum, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(k_that_pct, 'r--', 'LineWidth', 1.5);
title('Hội tụ Hệ số Tỷ lệ k (%)');
ylabel('k (%)');

subplot(3, 2, 4);
plot(time_x, beta_cum, 'g-', 'LineWidth', 1.5); hold on; grid on;
yline(beta_that_deg, 'r--', 'LineWidth', 1.5);
title('Hội tụ Pitch Boresight \beta');
ylabel('\beta (độ)');

subplot(3, 2, 6);
plot(time_x, gamma_cum, 'm-', 'LineWidth', 1.5); hold on; grid on;
yline(gamma_that_deg, 'r--', 'LineWidth', 1.5);
title('Hội tụ Yaw Boresight \gamma');
xlabel('Thời gian mô phỏng vòng tròn (s)');
ylabel('\gamma (độ)');

sgtitle('MÔ PHỎNG ƯỚC LƯỢNG K VÀ BORESIGHT - QUỸ ĐẠO VÒNG TRÒN 20 PHÚT', 'FontSize', 14, 'FontWeight', 'bold');
disp('Hoàn thành! Kịch bản vòng tròn 20 phút đã chạy xong.');
