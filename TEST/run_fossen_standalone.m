function run_fossen_standalone()
%% RUN_FOSSEN_STANDALONE - Mô phỏng độc lập REMUS 100 theo chuẩn GS. Thor I. Fossen
% Kịch bản thuần Fossen (không can thiệp ES-EKF):
% - Sử dụng mô hình động lực học 6 bậc tự do remus100.m
% - Dẫn đường 3-D ALOS (Adaptive Line-of-Sight) bám các điểm tọa độ Waypoint
% - Điều khiển hướng bằng SMC (Integral Sliding Mode Control)
% - Điều khiển độ sâu bằng cánh lặn đuôi (Stern planes PID)
% - Ghi nhận toàn bộ quỹ đạo chuẩn (Ground Truth) để làm mốc đối chứng

clc; close all;
fprintf('=== GIAI DOAN 1: CHAY MO PHONG REMUS 100 THUAN FOSSEN (STANDALONE) ===\n');

% Thiết lập đường dẫn MSS toolbox
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
if ~exist(fullfile(project_root, 'AUV_simulation'), 'dir')
    project_root = pwd;
    script_dir = fullfile(pwd, 'TEST');
end
mss_dir = fullfile(project_root, 'AUV_simulation', 'MSS');

addpath(fullfile(project_root, 'algo'));
addpath(fullfile(mss_dir, 'GNC'));
addpath(fullfile(mss_dir, 'LIBRARY'));
addpath(fullfile(mss_dir, 'CRAFT', 'AUV'));
addpath(fullfile(mss_dir, 'CRAFT', 'AUV', 'models'));

% Xóa trạng thái lưu trữ của các hàm điều khiển
clear integralSMCheading ALOS3D ALOSpsi LOSobserver;

%% 1. THIẾT LẬP THAM SỐ MÔ PHỎNG VÀ VẬT LÝ
h = 0.05;              % Chu kỳ trích mẫu h = 0.05 s (20 Hz)
T_final = 1880;        % Tổng thời gian mô phỏng: 1880 s (480s mặt nước + 1400s lặn)
t = 0:h:T_final;
nTimeSteps = length(t);

% Định nghĩa các điểm chuyển hướng 3D Waypoints (Gồm 8 phút mặt nước + Nhiệm vụ lặn)
% 1. Pha mặt nước 8 phút (z = 0 m): chạy thẳng và đổi hướng nhịp nhàng (S-curve nhẹ) để kích thích Boresight
wpt_surface_x = [  0,   0, -30,   30, -20,   0 ];  % North (m)
wpt_surface_y = [  0, 120, 270,  430, 580, 720 ];  % East (m)
wpt_surface_z = [  0,   0,   0,    0,   0,   0 ];  % Down = 0m (Mặt nước)

% 2. Pha lặn sâu nhiệm vụ cũ (tịnh tiến bắt đầu từ East = 720 m)
wpt_mission_x = [ -20,  -100,     0,   200,  200,   400 ];
wpt_mission_y = [ 920,  1320,  1670,  2020, 2520,  2920 ];
wpt_mission_z = [  10,   100,   100,    50,   50,    50 ];

wpt.pos.x = [wpt_surface_x, wpt_mission_x];
wpt.pos.y = [wpt_surface_y, wpt_mission_y];
wpt.pos.z = [wpt_surface_z, wpt_mission_z];

% Khởi tạo trạng thái ban đầu của tàu
xn = 0; yn = 0; zn = 0;
phi = 0; theta = 0;
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));
U = 1.5;               % Tốc độ hành trình chuẩn REMUS 100 (1.5 m/s)

% Biểu diễn góc Euler 12 trạng thái: [u v w p q r x y z phi theta psi]'
x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

% Tham số chân vịt
n_max = 1525;          % RPM tối đa
n_rate = 0.1;          % Giới hạn gia tốc vòng quay chân vịt
n_p = 1300;            % Vòng quay ban đầu đạt ngay tốc độ hành trình 1.5 m/s
n_d = 1300;            % Vòng quay đặt danh định (đạt tốc độ ~1.5 - 1.8 m/s)

% Tham số dòng chảy đại dương
Vc = 0.5;
betaVc = deg2rad(150);
wc = 0.1;

%% 2. THIẾT LẬP BỘ ĐIỀU KHIỂN FOSSEN
delta_max = deg2rad(20); % Giới hạn góc bẻ bánh lái và cánh lặn (20 deg)

% Điều khiển độ sâu qua cánh lặn (Stern plane PID)
[~,~,Mauv] = remus100();
w_theta = 0.8;
Kp_theta = Mauv(5,5) * w_theta^2;
Kd_theta = Mauv(5,5) * 2 * 0.8 * w_theta;
Ki_theta = Kp_theta * w_theta / 10;
theta_int = 0;
theta_d = 0;
q_d = 0;

% Điều khiển hướng bằng SMC (Nomoto model)
K_yaw = 5 / 20;
T_yaw = 1.0;
r_max = deg2rad(5.0);
phi_b = 0.1;
K_sigma = 0.05;
K_d = 0;
psi_d = psi;
r_d = 0;
a_d = 0;

% Tham số dẫn đường 3-D ALOS (Fossen & Aguiar 2024)
Delta_h = 25;
Delta_v = 20;
gamma_h = 0.001;
gamma_v = 0.001;
M_theta = deg2rad(20);
R_switch = 20;
K_f = 0.5;

%% 3. BỘ ĐỆM LƯU TRỮ KẾT QUẢ
pos_history = zeros(nTimeSteps, 3);      % [North, East, Down] (m)
vel_body_history = zeros(nTimeSteps, 3); % [u, v, w] (m/s)
rates_history = zeros(nTimeSteps, 3);    % [p, q, r] (rad/s)
euler_history = zeros(nTimeSteps, 3);    % [phi, theta, psi] (rad)
ui_history = zeros(nTimeSteps, 3);       % [delta_r, delta_s, n_p]
alos_history = zeros(nTimeSteps, 4);     % [y_e, z_e, alpha_c_hat, beta_c_hat]
f_b_ideal = zeros(nTimeSteps, 3);
w_ib_b_ideal = zeros(nTimeSteps, 3);
vel_ned_ideal = zeros(nTimeSteps, 3);

lat_rad = deg2rad(20.8);
tan_lat = tan(lat_rad); height = 0;
gravity = gravity_fossen(lat_rad, height); % Tính trọng lực chuẩn theo vĩ độ WGS-84 Somigliana
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;

fprintf('Bắt đầu chạy mô phỏng %d bước thời gian (T_final = %.0f s)...\n', nTimeSteps, T_final);
tic_sim = tic;

%% 4. VÒNG LẶP MÔ PHỎNG ĐỘNG HỌC VÀ ĐIỀU KHIỂN
for i = 1:nTimeSteps
    % Trích xuất trạng thái thực từ Fossen (Ground Truth)
    u = x(1); v = x(2); w = x(3);
    p = x(4); q = x(5); r = x(6);
    xn = x(7); yn = x(8); zn = x(9);
    phi = x(10); theta = x(11); psi = x(12);

    % Bộ điều khiển hướng: Integral SMC (Fossen 2027)
    delta_r = integralSMCheading(psi, r, psi_d, r_d, a_d, ...
        K_d, K_sigma, 1, phi_b, K_yaw, T_yaw, h);
    delta_r = sat(delta_r, delta_max);

    % Bộ điều khiển độ sâu: Stern planes PID
    delta_s = -Kp_theta * ssa(theta - theta_d) - Kd_theta * q - Ki_theta * theta_int;
    delta_s = sat(delta_s, delta_max);

    % Dẫn đường 3-D ALOS bám đường thẳng qua Waypoints
    [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat] = ...
        ALOS3D(xn, yn, zn, Delta_h, Delta_v, gamma_h, gamma_v, ...
        M_theta, h, R_switch, wpt);

    % Bộ lọc quan sát LOS observer làm mịn tín hiệu đặt
    [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
    [psi_d, r_d]   = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
    r_d = sat(r_d, r_max);

    % Biến đổi dòng chảy đại dương
    if t(i) > 800
        Vc_d = 0.65;
        w_V = 0.05;
        Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
    else
        Vc = 0.5;
    end

    if t(i) > 500
        betaVc_d = deg2rad(160);
        w_beta = 0.1;
        betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
    else
        betaVc = deg2rad(150);
    end

    % Động học vòng quay chân vịt
    if n_p < n_d
        n_p = n_p + n_rate;
    elseif n_p > n_d
        n_p = n_p - n_rate;
    end
    n_p = sat(n_p, n_max);

    % Vector tín hiệu điều khiển [delta_r; -delta_s; n_p]
    ui = [delta_r; -delta_s; n_p];

    % Lưu dữ liệu tại bước i
    pos_history(i, :)      = [xn, yn, zn];
    vel_body_history(i, :) = [u, v, w];
    rates_history(i, :)    = [p, q, r];
    euler_history(i, :)    = [phi, theta, psi];
    ui_history(i, :)       = [delta_r, delta_s, n_p];
    alos_history(i, :)     = [y_e, z_e, alpha_c_hat, beta_c_hat];

    % Tính gia tốc và lực riêng động lực học chính xác từ remus100
    [xdot_true, ~] = remus100(x, ui, Vc, betaVc, wc);
    v_body = [u; v; w];
    omega_b = [p; q; r];
    a_b_true = xdot_true(1:3) + cross(omega_b, v_body);
    
    Cbn_true = eul2cbn_local(phi, theta, psi);
    v_ned_true = Cbn_true * v_body;
    vel_ned_ideal(i, :) = v_ned_true';
    
    w_en_ned = [v_ned_true(2)/(radius_n + height); ...
               -v_ned_true(1)/(radius_m + height); ...
               -v_ned_true(2)*tan_lat/(radius_n + height)];
           
    [g_curr, g_ned_curr] = gravity_fossen(lat_rad, zn);
    f_b_ideal(i, :)    = (a_b_true - Cbn_true' * g_ned_curr + Cbn_true' * cross(2*w_ie_ned + w_en_ned, v_ned_true))';
    w_ib_b_ideal(i, :) = (omega_b + Cbn_true' * (w_ie_ned + w_en_ned))';

    % Tích phân Runge-Kutta bậc 4 (RK4) mô hình thủy động lực học REMUS 100
    x = rk4(@remus100, h, x, ui, Vc, betaVc, wc);

    % Cập nhật khâu tích phân điều khiển độ sâu
    theta_int = theta_int + h * ssa(theta - theta_d);
end

time_elapsed = toc(tic_sim);
fprintf('Hoàn thành mô phỏng Fossen Standalone trong %.2f giây.\n', time_elapsed);

%% 5. LƯU KẾT QUẢ RA FILE .MAT
save_path = fullfile(script_dir, 'ket_qua_fossen_standalone.mat');
save(save_path, 't', 'pos_history', 'vel_body_history', 'rates_history', ...
    'euler_history', 'ui_history', 'alos_history', 'wpt', 'h', 'T_final', ...
    'f_b_ideal', 'w_ib_b_ideal', 'vel_ned_ideal');
fprintf('Đã lưu dữ liệu mốc đối chứng vào: %s\n', save_path);

%% 6. VẼ VÀ LƯU ĐỒ THỊ QUỸ ĐẠO FOSSEN STANDALONE
fig = figure('Name', 'Fossen REMUS 100 Standalone Trajectory', ...
    'Position', [100, 100, 1200, 800], 'Visible', 'off');

% Đồ thị 3D không gian
subplot(2, 2, [1, 3]);
plot3(pos_history(:, 2), pos_history(:, 1), pos_history(:, 3), 'b-', 'LineWidth', 2);
hold on;
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'k--', 'LineWidth', 1);
grid on;
set(gca, 'ZDir', 'reverse');
xlabel('East (m)', 'FontSize', 11);
ylabel('North (m)', 'FontSize', 11);
zlabel('Down - Depth (m)', 'FontSize', 11);
title('Quy dao 3D Fossen REMUS 100 ALOS (Standalone)', 'FontSize', 13, 'FontWeight', 'bold');
legend('Quy dao thuc te REMUS 100', 'Cac diem Waypoint', 'Duong noi danh dinh', 'Location', 'best');
view(45, 25);

% Đồ thị 2D mặt bằng (North - East)
subplot(2, 2, 2);
plot(pos_history(:, 2), pos_history(:, 1), 'b-', 'LineWidth', 1.8);
hold on;
plot(wpt.pos.y, wpt.pos.x, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(wpt.pos.y, wpt.pos.x, 'k--', 'LineWidth', 1);
grid on;
axis equal;
xlabel('East (m)', 'FontSize', 10);
ylabel('North (m)', 'FontSize', 10);
title('Mat bang mat ngang (North - East)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Tau chay', 'Waypoints', 'Location', 'best');

% Đồ thị độ sâu theo thời gian
subplot(2, 2, 4);
plot(t, pos_history(:, 3), 'b-', 'LineWidth', 1.8);
hold on;
grid on;
set(gca, 'YDir', 'reverse');
xlabel('Thoi gian (s)', 'FontSize', 10);
ylabel('Do sau (m)', 'FontSize', 10);
title('Dien bien do sau lan theo thoi gian', 'FontSize', 11, 'FontWeight', 'bold');

img_path = fullfile(script_dir, 'fig_fossen_standalone.png');
saveas(fig, img_path);
close(fig);
fprintf('Đã xuất hình ảnh đồ thị vào: %s\n', img_path);

%% 7. TỔNG KẾT THÔNG SỐ VẬN HÀNH
max_cross_track = max(abs(alos_history(:, 1)));
mean_cross_track = mean(abs(alos_history(:, 1)));
max_depth_track = max(abs(alos_history(:, 2)));
mean_depth_track = mean(abs(alos_history(:, 2)));
fprintf('\n--- KET QUA VAN HANH FOSSEN STANDALONE ---\n');
fprintf('Quang duong East cuoi cung: %.2f m\n', pos_history(end, 2));
fprintf('Quang duong North cuoi cung: %.2f m\n', pos_history(end, 1));
fprintf('Do sau cuoi cung: %.2f m\n', pos_history(end, 3));
fprintf('Sai so dạt ngang Cross-track cuc dai: %.2f m (Trung binh: %.2f m)\n', max_cross_track, mean_cross_track);
fprintf('Sai so do sau Vertical-track cuc dai: %.2f m (Trung binh: %.2f m)\n', max_depth_track, mean_depth_track);
fprintf('=========================================================================\n');
end

function Cbn = eul2cbn_local(phi, theta, psi)
cr = cos(phi); sr = sin(phi);
cp = cos(theta); sp = sin(theta);
cy = cos(psi); sy = sin(psi);
Cbn = [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy; ...
       cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy; ...
       -sp,   sr*cp,            cr*cp];
end
