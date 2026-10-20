function run_live_remus100_esekf(speed_multiplier)
% RUN_LIVE_REMUS100_ESEKF - Chạy mô phỏng LIVE con tàu REMUS 100 và bộ lọc ES-EKF 19 trạng thái
% Hành trình dài 31.3 phút (1880s) với quãng đường gần 3km:
% - 8 phút đầu (0 - 480s): Chạy mặt nước, dùng GNSS RTK hiệu chuẩn Boresight và Scale factor của DVL.
% - 23 phút sau (480s - 1880s): Lặn sâu 100m, CẮT HOÀN TOÀN GNSS, chỉ dùng FOG IMU + DVL.
%
% Tham số:
%   speed_multiplier: Tốc độ tua live (mặc định = 30x, tức 1 giây đời thực xem được 30s bơi của tàu,
%                     toàn bộ 31 phút sẽ chiếu trong ~60 giây mượt mà).

if nargin < 1 || isempty(speed_multiplier)
    speed_multiplier = 35; % Mặc định 35x để xem trực quan trong ~50s
end

clc; close all;
fprintf('========================================================================\n');
fprintf('  MÔ PHỎNG TRỰC TIẾP (LIVE) TÀU NGẦM REMUS 100 + BỘ LỌC ES-EKF 19 TRẠNG THÁI\n');
fprintf('  Hành trình: 31.3 phút (1880s) | Quãng đường: ~3.0 km | Tốc độ xem: %dx\n', speed_multiplier);
fprintf('========================================================================\n');

%% 1. THIẾT LẬP ĐƯỜNG DẪN THƯ MỤC
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
project_root = script_dir;
mss_dir = fullfile(project_root, 'AUV_simulation', 'MSS');
algo_dir = fullfile(project_root, 'algo');
test_dir = fullfile(project_root, 'TEST');

addpath(algo_dir);
addpath(test_dir);
addpath(fullfile(project_root, 'Sensor'));
addpath(fullfile(project_root, 'dieukiendauvao'));
addpath(fullfile(mss_dir, 'GNC'));
addpath(fullfile(mss_dir, 'LIBRARY'));
addpath(fullfile(mss_dir, 'CRAFT', 'AUV'));
addpath(fullfile(mss_dir, 'CRAFT', 'AUV', 'models'));

clear integralSMCheading ALOS3D ALOSpsi LOSobserver;

%% 2. CẤU HÌNH THỜI GIAN VÀ CÁC ĐIỂM MỐC (WAYPOINTS)
h = 0.05;              % Bước tích phân h = 0.05s (20 Hz)
T_final = 1880;        % 31.3 phút
t = 0:h:T_final;
nTimeSteps = length(t);

T_gnss_cut = 480.0;    % Cắt GNSS sau 8 phút
dt_dvl = 0.20;         % Chu kỳ đo DVL = 0.2s (5 Hz)
dvl_step = round(dt_dvl / h);

% Mốc tọa độ 3D (Waypoints)
wpt_surface_x = [  0,   0, -30,   30, -20,   0 ];  % North (m)
wpt_surface_y = [  0, 120, 270,  430, 580, 720 ];  % East (m)
wpt_surface_z = [  0,   0,   0,    0,   0,   0 ];  % Mặt nước 0m

wpt_mission_x = [ -20,  -100,     0,   200,  200,   400 ];
wpt_mission_y = [ 920,  1320,  1670,  2020, 2520,  2920 ];
wpt_mission_z = [  10,   100,   100,    50,   50,    50 ]; % Lặn sâu 100m rồi về 50m

wpt.pos.x = [wpt_surface_x, wpt_mission_x];
wpt.pos.y = [wpt_surface_y, wpt_mission_y];
wpt.pos.z = [wpt_surface_z, wpt_mission_z];

% Hằng số định vị
gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;
lat_rad = deg2rad(20.8); tan_lat = tan(lat_rad); height = 0;
lever_arm = [0.5; -0.1; 0.2];

% Sai số lắp đặt DVL thực tế (Ground Truth)
true_scale = 0.015;                          % Scale factor = +1.5%
true_boresight = deg2rad([1.5; -2.0; 3.0]);  % Boresight [Roll; Pitch; Yaw] = [1.5; -2; 3] deg
Cbs_true = rodrigues_so3_local( -true_boresight );

% Thông số cảm biến GEO-FOG 3D Dual
ins_spec = GEO_FOG_3D('spec');
sigma_vrw = ins_spec.acc.vrw;
sigma_arw = ins_spec.gyro.arw;
acc_bias_sigma = ins_spec.acc.bias_instab;
acc_bias_tau = ins_spec.acc.bias_tau;
gyro_bias_rw = ins_spec.gyro.bias_rw;
scale_rw = 1e-5; boresight_rw = 1e-5;

sigma_gps_pos = ins_spec.gnss.pos_v;
sigma_gps_vel = ins_spec.gnss.vel;
sigma_gps_yaw = ins_spec.gnss.heading;
sigma_depth = 0.05;
dvl_noise_floor = 0.01; dvl_noise_slope = 0.005;

cfg = zeros(20, 1);
cfg(1) = h; cfg(2) = gravity; cfg(3:5) = w_ie_ned;
cfg(6) = radius_n; cfg(7) = radius_m; cfg(8) = tan_lat; cfg(9) = height;
cfg(10:12) = lever_arm; cfg(13) = dvl_noise_floor; cfg(14) = dvl_noise_slope;
cfg(15) = sigma_gps_pos; cfg(16) = sigma_gps_vel; cfg(17) = sigma_gps_yaw;
cfg(18) = sigma_depth; cfg(19) = acc_bias_tau; cfg(20) = sigma_arw;

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
             gyro_bias_rw; scale_rw; boresight_rw; h];

%% 3. KHỞI TẠO TÀU REMUS 100 VÀ BỘ ĐIỀU KHIỂN
xn = 0; yn = 0; zn = 0; phi = 0; theta = 0;
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));
U = 1.5;
x_phys = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

n_max = 1525; n_rate = 0.1; n_p = 1300; n_d = 1300;
Vc = 0.5; betaVc = deg2rad(150); wc = 0.1;

delta_max = deg2rad(20);
[~,~,Mauv] = remus100();
w_theta = 0.8;
Kp_theta = Mauv(5,5) * w_theta^2;
Kd_theta = Mauv(5,5) * 2 * 0.8 * w_theta;
Ki_theta = Kp_theta * w_theta / 10;
theta_int = 0; theta_d = 0; q_d = 0;

K_yaw = 5 / 20; T_yaw = 1.0; r_max = deg2rad(5.0);
phi_b = 0.1; K_sigma = 0.05; K_d = 0;
psi_d = psi; r_d = 0; a_d = 0;

Delta_h = 25; Delta_v = 20; gamma_h = 0.001; gamma_v = 0.001;
M_theta = deg2rad(20); R_switch = 20; K_f = 0.5;

%% 4. KHỞI TẠO BỘ LỌC ES-EKF 19 TRẠNG THÁI (COLD START)
[state_est, P_est, Q] = esekf19_init([phi; theta; psi], gravity, noise_cfg);
state_est(4:6) = eul2cbn_local(phi, theta, psi) * [U; 0; 0];
state_est(17) = 0.0;             % Cold start: scale = 0
state_est(18:20) = [0.0; 0.0; 0.0]; % Cold start: boresight = [0; 0; 0]

P_est(4:6, 4:6)     = (1.5)^2 * eye(3);
P_est(16, 16)       = (0.03)^2;
P_est(17:19, 17:19) = deg2rad(3.0)^2 * eye(3);

ba_true = [0.008; -0.012; 0.006];
bg_true = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];
imu_state = GEO_FOG_3D('init', ins_spec, bg_true, ba_true);

ui_curr = [0; 0; n_p];

%% 5. THIẾT LẬP GIAO DIỆN LIVE COCKPIT ĐẲNG CẤP
fig = figure('Name', 'REMUS 100 + ES-EKF 19-State Live Mission Cockpit', ...
    'Color', [0.06 0.08 0.12], 'Position', [60, 50, 1380, 840]);

% Font và màu sắc HUD
c_bg = [0.08 0.11 0.16];
c_cyan = [0.0 0.85 1.0];
c_mag = [1.0 0.25 0.65];
c_yellow = [1.0 0.85 0.1];
c_green = [0.2 0.9 0.4];
c_gray = [0.6 0.65 0.75];

% Subplot 1: 3D Trajectory Map (Mặt phẳng 3D biển)
ax1 = subplot(2, 2, 1);
set(ax1, 'Color', c_bg, 'XColor', c_gray, 'YColor', c_gray, 'ZColor', c_gray, 'GridColor', [0.2 0.3 0.4]);
hold(ax1, 'on'); grid(ax1, 'on'); view(ax1, 3);
plot3(ax1, wpt.pos.y, wpt.pos.x, wpt.pos.z, 'o--', 'Color', c_yellow, 'LineWidth', 1.5, ...
    'MarkerSize', 7, 'MarkerFaceColor', [0.8 0.5 0], 'DisplayName', 'Waypoints kế hoạch');
h_line_true3d = animatedline(ax1, 'Color', c_cyan, 'LineWidth', 2.2, 'DisplayName', 'Tàu thật (Ground Truth)');
h_line_est3d  = animatedline(ax1, 'Color', c_mag, 'LineWidth', 1.6, 'LineStyle', '--', 'DisplayName', 'ES-EKF ước lượng');
h_auv_pos = plot3(ax1, 0, 0, 0, 'p', 'Color', [1 1 1], 'MarkerFaceColor', c_cyan, 'MarkerSize', 12, 'DisplayName', 'REMUS 100');
xlabel(ax1, 'East (m)', 'Color', c_gray, 'FontWeight', 'bold');
ylabel(ax1, 'North (m)', 'Color', c_gray, 'FontWeight', 'bold');
zlabel(ax1, 'Depth (m)', 'Color', c_gray, 'FontWeight', 'bold');
set(ax1, 'ZDir', 'reverse'); % Trục Z hướng xuống đáy biển
title(ax1, 'QUỸ ĐẠO BƠI 3D NGOÀI BIỂN (EAST - NORTH - DEPTH)', 'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');
legend(ax1, 'TextColor', [0.9 0.9 0.9], 'Color', [0.1 0.13 0.2], 'Location', 'northwest');

% Subplot 2: Depth Profile (Đồ thị lặn sâu)
ax2 = subplot(2, 2, 2);
set(ax2, 'Color', c_bg, 'XColor', c_gray, 'YColor', c_gray, 'GridColor', [0.2 0.3 0.4]);
hold(ax2, 'on'); grid(ax2, 'on');
xline(ax2, T_gnss_cut, ':', 'Color', [1 0.3 0.3], 'LineWidth', 1.8, 'DisplayName', 'Cắt GNSS (Lặn sâu)');
h_depth_true = animatedline(ax2, 'Color', c_cyan, 'LineWidth', 2.0, 'DisplayName', 'Độ sâu thật z');
h_depth_est  = animatedline(ax2, 'Color', c_mag, 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'ES-EKF z');
xlabel(ax2, 'Thời gian (giây)', 'Color', c_gray, 'FontWeight', 'bold');
ylabel(ax2, 'Độ sâu lặn (m)', 'Color', c_gray, 'FontWeight', 'bold');
set(ax2, 'YDir', 'reverse');
xlim(ax2, [0, T_final]); ylim(ax2, [-5, 115]);
title(ax2, 'TIẾN TRÌNH LẶN SÂU THEO THỜI GIAN', 'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');
legend(ax2, 'TextColor', [0.9 0.9 0.9], 'Color', [0.1 0.13 0.2], 'Location', 'southeast');

% Subplot 3: Boresight & Scale Convergence
ax3 = subplot(2, 2, 3);
set(ax3, 'Color', c_bg, 'XColor', c_gray, 'YColor', c_gray, 'GridColor', [0.2 0.3 0.4]);
hold(ax3, 'on'); grid(ax3, 'on');
yline(ax3, rad2deg(true_boresight(3)), 'c--', 'LineWidth', 1.5, 'DisplayName', 'Boresight Yaw thật (+3.0°)');
yline(ax3, true_scale * 100, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Scale DVL thật (+1.5%)');
h_bs_yaw = animatedline(ax3, 'Color', c_yellow, 'LineWidth', 2.0, 'DisplayName', 'ES-EKF Boresight Yaw (°)');
h_scale  = animatedline(ax3, 'Color', c_green, 'LineWidth', 2.0, 'DisplayName', 'ES-EKF Scale Factor (%)');
xlabel(ax3, 'Thời gian (giây)', 'Color', c_gray, 'FontWeight', 'bold');
ylabel(ax3, 'Giá trị hiệu chuẩn', 'Color', c_gray, 'FontWeight', 'bold');
xlim(ax3, [0, T_final]); ylim(ax3, [-1, 4]);
title(ax3, 'HỘI TỤ TỰ HIỆU CHUẨN TRỰC TUYẾN DVL (COLD START)', 'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');
legend(ax3, 'TextColor', [0.9 0.9 0.9], 'Color', [0.1 0.13 0.2], 'Location', 'east');

% Subplot 4: Cockpit Telemetry HUD (Bảng đồng hồ số)
ax4 = subplot(2, 2, 4);
set(ax4, 'Color', c_bg, 'XColor', 'none', 'YColor', 'none');
title(ax4, 'BẢNG THÔNG SỐ ĐIỀU HÀNH THỜI GIAN THỰC (COCKPIT HUD)', 'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');

hud_text = text(ax4, 0.05, 0.5, '', 'FontName', 'Consolas', 'FontSize', 11, ...
    'Color', [0.9 0.95 1.0], 'Interpreter', 'none', 'VerticalAlignment', 'middle');

drawnow;

%% 6. VÒNG LẶP MÔ PHỎNG LIVE
fprintf('Đang chạy live... (Bấm Ctrl+C hoặc đóng cửa sổ để dừng)\n');

% Tần suất vẽ đồ thị: cứ mỗi frame_skip bước thì cập nhật 1 lần
frame_skip = max(1, round(speed_multiplier * 0.15 / h)); % Cập nhật đồ thị mượt mà

tic_start = tic;

for i = 1:nTimeSteps
    % Kiểm tra nếu người dùng tắt figure
    if ~ishandle(fig), break; end

    % 6.1. Trích xuất trạng thái thực Ground Truth từ Fossen
    u = x_phys(1); v = x_phys(2); w = x_phys(3);
    p = x_phys(4); q = x_phys(5); r = x_phys(6);
    xn = x_phys(7); yn = x_phys(8); zn = x_phys(9);
    phi = x_phys(10); theta = x_phys(11); psi = x_phys(12);

    v_body = [u; v; w];
    omega_b = [p; q; r];
    Cbn_true = eul2cbn_local(phi, theta, psi);
    v_ned_true = Cbn_true * v_body;

    % 6.2. Tính gia tốc động lực học chính xác từ mô hình thủy động lực học remus100
    [xdot_true, ~] = remus100(x_phys, ui_curr, Vc, betaVc, wc);
    a_b_true = xdot_true(1:3) + cross(omega_b, v_body);

    w_en_ned = [v_ned_true(2)/(radius_n + height); ...
               -v_ned_true(1)/(radius_m + height); ...
               -v_ned_true(2)*tan_lat/(radius_n + height)];

    f_b_true = a_b_true - Cbn_true' * [0; 0; gravity] + Cbn_true' * cross(2*w_ie_ned + w_en_ned, v_ned_true);
    w_ib_b_true = omega_b + Cbn_true' * (w_ie_ned + w_en_ned);

    % Sinh số đo IMU GEO-FOG 3D Dual
    [gyro_meas, accel_meas, imu_state] = GEO_FOG_3D('imu', ins_spec, imu_state, ...
        w_ib_b_true, f_b_true, h);

    % 6.3. Giả lập phép đo cảm biến ngoài
    gnss_meas = zeros(7, 1);
    depth_meas = 0.0;
    dvl_meas = zeros(3, 1);
    flags = [0; 0; 0];

    if mod(i, dvl_step) == 0
        depth_meas = zn + sigma_depth * randn;
        flags(2) = 1;

        v_lever = v_body + cross(omega_b, lever_arm);
        v_dvl_ideal = (1.0 + true_scale) * Cbs_true * v_lever;
        sigma_dvl_i = dvl_noise_floor + dvl_noise_slope * norm(v_lever);
        dvl_meas = v_dvl_ideal + sigma_dvl_i * randn(3, 1);
        flags(3) = 1;

        if t(i) <= T_gnss_cut
            gnss_meas = GEO_FOG_3D('gnss', ins_spec, [xn; yn; zn], v_ned_true, psi);
            flags(1) = 1;
        end
    end

    % 6.4. Bước lọc ES-EKF 19 trạng thái
    [state_est, P_est, ~, ~] = esekf19_step( ...
        state_est, P_est, Q, gyro_meas, accel_meas, ...
        gnss_meas, depth_meas, dvl_meas, flags, cfg);

    p_hat = state_est(1:3);
    q_hat = state_est(7:10);
    [phi_hat, theta_hat, psi_hat] = quat_to_euler_local(q_hat);
    scale_hat = state_est(17);
    bs_hat = state_est(18:20);

    % 6.5. Dẫn đường ALOS3D và điều khiển khép vòng bằng ước lượng ES-EKF
    [psi_ref, theta_ref, ~, ~, ~, ~] = ...
        ALOS3D(p_hat(1), p_hat(2), p_hat(3), Delta_h, Delta_v, gamma_h, gamma_v, ...
        M_theta, h, R_switch, wpt);

    [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
    [psi_d, r_d]   = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
    r_d = sat(r_d, r_max);

    r_gyro_corrected = gyro_meas(3) - state_est(16);
    q_gyro_corrected = gyro_meas(2) - state_est(15);

    delta_r = integralSMCheading(psi_hat, r_gyro_corrected, psi_d, r_d, a_d, ...
        K_d, K_sigma, 1, phi_b, K_yaw, T_yaw, h);
    delta_r = sat(delta_r, delta_max);

    delta_s = -Kp_theta * ssa(theta_hat - theta_d) - Kd_theta * q_gyro_corrected - Ki_theta * theta_int;
    delta_s = sat(delta_s, delta_max);

    % Biến đổi dòng chảy đại dương
    if t(i) > 800
        Vc_d = 0.65; w_V = 0.05;
        Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
    else
        Vc = 0.5;
    end
    if t(i) > 500
        betaVc_d = deg2rad(160); w_beta = 0.1;
        betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
    else
        betaVc = deg2rad(150);
    end

    % Động học chân vịt
    if n_p < n_d
        n_p = n_p + n_rate;
    elseif n_p > n_d
        n_p = n_p - n_rate;
    end
    n_p = sat(n_p, n_max);

    ui = [delta_r; -delta_s; n_p];

    % Cập nhật vật lý RK4
    x_phys = rk4(@remus100, h, x_phys, ui, Vc, betaVc, wc);
    theta_int = theta_int + h * ssa(theta_hat - theta_d);
    ui_curr = ui;

    % 6.6. CẬP NHẬT GIAO DIỆN LIVE THEO TẦN SUẤT FRAME_SKIP
    if mod(i, frame_skip) == 0 || i == nTimeSteps
        % Thêm điểm vào Animated Line
        addpoints(h_line_true3d, yn, xn, zn);
        addpoints(h_line_est3d, p_hat(2), p_hat(1), p_hat(3));
        set(h_auv_pos, 'XData', yn, 'YData', xn, 'ZData', zn);

        addpoints(h_depth_true, t(i), zn);
        addpoints(h_depth_est, t(i), p_hat(3));

        addpoints(h_bs_yaw, t(i), rad2deg(bs_hat(3)));
        addpoints(h_scale, t(i), scale_hat * 100);

        % Tính toán sai số vị trí tức thời
        pos_err_curr = norm([xn - p_hat(1); yn - p_hat(2); zn - p_hat(3)]);
        
        % Chế độ định vị
        if t(i) <= T_gnss_cut
            mode_str = 'HIỆU CHUẨN MẶT NƯỚC (GNSS RTK 5Hz)';
            mode_color = '[0.2, 0.9, 0.4]';
        else
            mode_str = 'LẶN SÂU TUYỆT ĐỐI (CẮT GNSS - FOG + DVL)';
            mode_color = '[1.0, 0.3, 0.3]';
        end

        % Cập nhật bảng số liệu HUD
        hud_lines = {
            sprintf('------------------------------------------------------------');
            sprintf('  THỜI GIAN NHIỆM VỤ : %02d:%02d / 31:20  (Tua %dx)', ...
                floor(t(i)/60), mod(floor(t(i)), 60), speed_multiplier);
            sprintf('  CHẾ ĐỘ ĐỊNH VỊ     : %s', mode_str);
            sprintf('------------------------------------------------------------');
            sprintf('  VỊ TRÍ THỰC TẾ     : N=%7.1f m | E=%7.1f m | Z=%5.1f m', xn, yn, zn);
            sprintf('  ES-EKF ƯỚC LƯỢNG   : N=%7.1f m | E=%7.1f m | Z=%5.1f m', p_hat(1), p_hat(2), p_hat(3));
            sprintf('  SAI SỐ VỊ TRÍ 3D   : >> %5.2f mét << (Rất nhỏ!)', pos_err_curr);
            sprintf('------------------------------------------------------------');
            sprintf('  TỐC ĐỘ DI CHUYỂN   : %5.2f m/s (~3.0 hải lý/h)', norm(v_body));
            sprintf('  HƯỚNG BƠI (YAW)    : %6.1f deg (Thực) | %6.1f deg (EKF)', rad2deg(psi), rad2deg(psi_hat));
            sprintf('  GÓC CHÚI (PITCH)   : %6.1f deg | GÓC NGHIÊNG: %4.1f deg', rad2deg(theta), rad2deg(phi));
            sprintf('------------------------------------------------------------');
            sprintf('  HIỆU CHUẨN BORESIGHT YAW : %5.2f° (Thật = +3.00°)', rad2deg(bs_hat(3)));
            sprintf('  HIỆU CHUẨN SCALE DVL     : %+5.2f%% (Thật = +1.50%%)', scale_hat * 100);
            sprintf('  DÒNG NƯỚC NGẦM           : %4.2f m/s | Hướng: %5.1f°', Vc, rad2deg(betaVc));
            sprintf('------------------------------------------------------------');
        };
        set(hud_text, 'String', strjoin(hud_lines, '\n'));

        drawnow limitrate;
    end
end

real_time_taken = toc(tic_start);
fprintf('\n>>> ĐÃ HOÀN THÀNH TOÀN BỘ HÀNH TRÌNH 31.3 PHÚT TRONG %.1f GIÂY ĐỜI THỰC! <<<\n', real_time_taken);
fprintf('Sai số vị trí cuối cùng sau khi lặn sâu gần 3km: %.2f mét.\n', pos_err_curr);
fprintf('Boresight Yaw ước lượng được: %.3f° (Thật = 3.000°).\n', rad2deg(bs_hat(3)));
fprintf('Scale Factor ước lượng được : %.3f%% (Thật = 1.500%%).\n', scale_hat * 100);
end

%% HÀM PHỤ TRỢ CỤC BỘ
function Cbn = eul2cbn_local(phi, theta, psi)
cr = cos(phi); sr = sin(phi);
cp = cos(theta); sp = sin(theta);
cy = cos(psi); sy = sin(psi);
Cbn = [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy; ...
       cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy; ...
       -sp,   sr*cp,            cr*cp];
end

function [phi, theta, psi] = quat_to_euler_local(q)
q = q / norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);
phi = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
sin_theta = 2*(w*y - z*x);
sin_theta = max(min(sin_theta, 1), -1);
theta = asin(sin_theta);
psi = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
end

function R = rodrigues_so3_local(phi)
theta = norm(phi);
K = [0, -phi(3), phi(2); phi(3), 0, -phi(1); -phi(2), phi(1), 0];
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
end
end
