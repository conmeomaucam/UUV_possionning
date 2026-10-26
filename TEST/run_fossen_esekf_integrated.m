function run_fossen_esekf_integrated(sensor_model, save_main)
% sensor_model: 'GEO_FOG' (mặc định, GEO-FOG 3D Dual, KHÔNG từ kế) hoặc 'INS_FI' (INS-DM-FI)
% save_main   : true -> ghi đè file kết quả chính dùng cho compare_fossen_vs_esekf
if nargin < 1 || isempty(sensor_model), sensor_model = 'GEO_FOG'; end
if nargin < 2 || isempty(save_main), save_main = true; end
sensor_model = upper(sensor_model);
rng(2026);  % Cùng chuỗi ngẫu nhiên cho mọi cảm biến -> so sánh công bằng
%% RUN_FOSSEN_ESEKF_INTEGRATED - Mô phỏng kết hợp REMUS 100 và ES-EKF 19 trạng thái
% Kịch bản tích hợp:
% - Mô hình chuyển động vật lý 6 bậc tự do: remus100.m (Fossen)
% - Giả lập cảm biến: IMU (400 Hz/20 Hz), DVL (5 Hz, có lệch trục Boresight & Scale factor thật),
%   Cảm biến đo sâu (5 Hz), GNSS RTK mặt nước (5 Hz, chỉ có trong 30s đầu trước khi lặn).
% - Bộ lọc ES-EKF 19 trạng thái: Tự hiệu chuẩn trực tuyến Boresight [Roll, Pitch, Yaw] và Scale factor
%   từ trạng thái nguội (Cold Start: Boresight = [0;0;0], Scale = 0).
% - Vòng phản hồi kín (Closed-loop): Tín hiệu ước lượng vị trí, hướng bơi từ ES-EKF được cấp
%   trực tiếp cho bộ dẫn đường ALOS3D và bộ lái SMC để tàu bám quỹ đạo Waypoints.

clc; close all;
fprintf('=== GIAI DOAN 2: CHAY MO PHONG KET HOP REMUS 100 VA THUAT TOAN ES-EKF ===\n');

% Thiết lập đường dẫn thư mục
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
if ~exist(fullfile(project_root, 'algo'), 'dir')
    project_root = pwd;
    script_dir = fullfile(pwd, 'TEST');
end
mss_dir = fullfile(project_root, 'AUV_simulation', 'MSS');
algo_dir = fullfile(project_root, 'algo');

addpath(algo_dir);
addpath(script_dir);
addpath(fullfile(project_root, 'Sensor'));
addpath(fullfile(project_root, 'dieukiendauvao'));
addpath(fullfile(mss_dir, 'GNC'));
addpath(fullfile(mss_dir, 'LIBRARY'));
addpath(fullfile(mss_dir, 'CRAFT', 'AUV'));
addpath(fullfile(mss_dir, 'CRAFT', 'AUV', 'models'));

% Xóa trạng thái lưu trữ của các bộ điều khiển
clear integralSMCheading ALOS3D ALOSpsi LOSobserver;

%% 1. CẤU HÌNH THỜI GIAN VÀ THÔNG SỐ VẬT LÝ
h = 0.05;              % Chu kỳ trích mẫu h = 0.05 s (20 Hz)
T_final = 1880;        % Tổng thời gian: 1880 s (480s mặt nước + 1400s lặn)
t = 0:h:T_final;
nTimeSteps = length(t);

T_gnss_cut = 480.0;    % Cắt GNSS sau 480 giây (8 phút căn chỉnh mặt nước có GNSS)
dt_dvl = 0.20;         % Chu kỳ đo DVL = 0.2 s (5 Hz)
dvl_step = round(dt_dvl / h); % Cập nhật DVL mỗi 4 bước mô phỏng 20 Hz

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

% Hằng số vật lý và định vị WGS-84
lat_rad = deg2rad(20.8);
tan_lat = tan(lat_rad);
height = 0;
gravity = gravity_fossen(lat_rad, height); % Tính trọng lực chuẩn theo vĩ độ WGS-84 Somigliana
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0;
radius_m = 6356752.3;

% Cánh tay đòn lắp đặt từ IMU đến DVL (khung thân tàu, đơn vị m)
lever_arm = [0.5; -0.1; 0.2];

% Sai số lắp đặt DVL thực tế (Ground Truth cần hiệu chuẩn)
true_scale = 0.015;                          % Scale factor thật = +1.5%
true_boresight = deg2rad([1.5; -2.0; 3.0]);  % Boresight thật [Roll; Pitch; Yaw] (rad)
Cbs_true = rodrigues_so3_local(-true_boresight);

% Đặc tính nhiễu cảm biến quán tính theo datasheet thiết bị được chọn (KHÔNG từ kế)
switch sensor_model
    case 'GEO_FOG'
        ins_spec = GEO_FOG_3D('spec');
    case 'INS_FI'
        ins_spec = ins_fi_spec_local();
    otherwise
        error('sensor_model khong hop le: %s', sensor_model);
end
fprintf('Cam bien quan tinh: %s (tu ke: TAT)\n', ins_spec.ten);

sigma_vrw = ins_spec.acc.vrw;              % VRW gia tốc kế (m/s/sqrt(s))
sigma_arw = ins_spec.gyro.arw;             % ARW con quay FOG (rad/sqrt(s))
acc_bias_sigma = ins_spec.acc.bias_instab; % Độ ổn định bias in-run gia tốc kế (m/s^2)
acc_bias_tau = ins_spec.acc.bias_tau;      % Thời gian tương quan Gauss-Markov (s)
gyro_bias_rw = ins_spec.gyro.bias_rw;      % Random walk bias con quay tương đương (rad/s/sqrt(s))
scale_rw = 1e-5;              % Random walk scale factor (1/sqrt(s))
boresight_rw = 1e-5;          % Random walk boresight (rad/sqrt(s))

sigma_gps_pos = ins_spec.gnss.pos_v;       % Sai số vị trí GNSS RTK dùng trong R (lấy trục đứng, thận trọng)
sigma_gps_vel = ins_spec.gnss.vel;         % Sai số vận tốc GNSS (m/s)
sigma_gps_yaw = ins_spec.gnss.heading;     % Sai số góc hướng GNSS 2 ăng-ten (rad)
sigma_depth = 0.05;           % Sai số cảm biến độ sâu (m)
dvl_spec = DVL_A125('spec');
dvl_noise_floor = dvl_spec.noise_floor;       % Nhiễu sàn DVL A125 (m/s)
dvl_noise_slope = dvl_spec.noise_slope;      % Nhiễu tỉ lệ vận tốc DVL

% Vector cấu hình 20 phần tử truyền cho esekf19_step
cfg = zeros(20, 1);
cfg(1) = h;
cfg(2) = gravity;
cfg(3:5) = w_ie_ned;
cfg(6) = radius_n;
cfg(7) = radius_m;
cfg(8) = tan_lat;
cfg(9) = height;
cfg(10:12) = lever_arm;
cfg(13) = dvl_noise_floor;
cfg(14) = dvl_noise_slope;
cfg(15) = sigma_gps_pos;
cfg(16) = sigma_gps_vel;
cfg(17) = sigma_gps_yaw;
cfg(18) = sigma_depth;
cfg(19) = acc_bias_tau;
cfg(20) = sigma_arw;

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
    gyro_bias_rw; scale_rw; boresight_rw; h];

%% 2. KHỞI TẠO BỘ ĐIỀU KHIỂN VÀ THỦY ĐỘNG LỰC HỌC REMUS 100
xn = 0; yn = 0; zn = 0;
phi = 0; theta = 0;
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));
U = 1.5;               % Tốc độ hành trình chuẩn REMUS 100 (1.5 m/s)

% Biểu diễn trạng thái Fossen 12x1
x_phys = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

% Tham số chân vịt
n_max = 1525;
n_rate = 0.1;
n_p = 1300;            % Vòng quay ban đầu đạt ngay tốc độ hành trình 1.5 m/s
n_d = 1300;

% Dòng chảy đại dương
Vc = 0.5;
betaVc = deg2rad(150);
wc = 0.1;

% Bộ điều khiển bánh lái và cánh lặn
delta_max = deg2rad(20);
[~,~,Mauv] = remus100();
w_theta = 0.8;
Kp_theta = Mauv(5,5) * w_theta^2;
Kd_theta = Mauv(5,5) * 2 * 0.8 * w_theta;
Ki_theta = Kp_theta * w_theta / 10;
theta_int = 0;
theta_d = 0;
q_d = 0;

K_yaw = 5 / 20;
T_yaw = 1.0;
r_max = deg2rad(5.0);
phi_b = 0.1;
K_sigma = 0.05;
K_d = 0;
psi_d = psi;
r_d = 0;
a_d = 0;

Delta_h = 25;
Delta_v = 20;
gamma_h = 0.001;
gamma_v = 0.001;
M_theta = deg2rad(20);
R_switch = 20;
K_f = 0.5;

%% 3. KHỞI TẠO BỘ LỌC ES-EKF 19 TRẠNG THÁI (COLD START)
[state_est, P_est, Q] = esekf19_init([phi; theta; psi], gravity, noise_cfg);

% Đồng bộ trạng thái vận tốc danh định ban đầu theo tốc độ hành trình và hướng ban đầu của tàu
state_est(4:6) = eul2cbn_local(phi, theta, psi) * [U; 0; 0];

% Thiết lập Cold Start: Chưa biết Boresight và Scale factor
state_est(17) = 0.0;             % Scale factor khởi tạo = 0 (Sai lệch thật = +1.5%)
state_est(18:20) = [0.0; 0.0; 0.0]; % Boresight khởi tạo = [0; 0; 0] (Thật = [1.5; -2.0; 3.0] deg)

% Bất định tiên nghiệm ma trận P0 chuẩn hóa cho tốc độ 1.5 m/s
P_est(4:6, 4:6)     = (1.5)^2 * eye(3);         % Bất định vận tốc ban đầu 1.5 m/s mở cổng Chi-square nhận GNSS/DVL
P_est(16, 16)       = (0.03)^2;                % Độ lệch chuẩn scale 3%
P_est(17:19, 17:19) = deg2rad(3.0)^2 * eye(3); % Độ lệch chuẩn Boresight 3.0 deg

% Khởi tạo các bias cảm biến thực tế
% Bias bật máy (turn-on) GIẢ ĐỊNH, dùng chung cho mọi cảm biến để so sánh công bằng
ba_true = [0.008; -0.012; 0.006];
bg_true = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];
imu_state = GEO_FOG_3D('init', ins_spec, bg_true, ba_true);

% Bộ đệm chẩn đoán
bg_true_hist = zeros(nTimeSteps, 3);
bg_est_hist  = zeros(nTimeSteps, 3);
nis_hist     = zeros(nTimeSteps, 3);
acc_hist     = nan(nTimeSteps, 3);   % 1 = chấp nhận, 0 = từ chối, NaN = không có phép đo

% Bộ đệm lưu vết trạng thái tích hợp
pos_true_hist = zeros(nTimeSteps, 3);
pos_est_hist  = zeros(nTimeSteps, 3);
vel_true_hist = zeros(nTimeSteps, 3);
vel_est_hist  = zeros(nTimeSteps, 3);
euler_true_hist = zeros(nTimeSteps, 3);
euler_est_hist  = zeros(nTimeSteps, 3);
boresight_est_hist = zeros(nTimeSteps, 3);
scale_est_hist = zeros(nTimeSteps, 1);
cov_bs_hist = zeros(nTimeSteps, 3);
cov_scale_hist = zeros(nTimeSteps, 1);
ui_hist = zeros(nTimeSteps, 3);
alos_hist = zeros(nTimeSteps, 4);

ui_curr = [0; 0; n_p];

fprintf('Bắt đầu chạy mô phỏng tích hợp REMUS 100 + ES-EKF (%d bước)...\n', nTimeSteps);
tic_sim = tic;

%% 4. VÒNG LẶP MÔ PHỎNG TÍCH HỢP (CLOSED-LOOP GUIDANCE)
for i = 1:nTimeSteps
    % 4.1. Trích xuất trạng thái thực Ground Truth từ Fossen
    u = x_phys(1); v = x_phys(2); w = x_phys(3);
    p = x_phys(4); q = x_phys(5); r = x_phys(6);
    xn = x_phys(7); yn = x_phys(8); zn = x_phys(9);
    phi = x_phys(10); theta = x_phys(11); psi = x_phys(12);

    v_body = [u; v; w];
    omega_b = [p; q; r];
    Cbn_true = eul2cbn_local(phi, theta, psi);
    v_ned_true = Cbn_true * v_body;

    % 4.2. Tính gia tốc động lực học chính xác từ mô hình thủy động lực học remus100
    [xdot_true, ~] = remus100(x_phys, ui_curr, Vc, betaVc, wc);
    a_b_true = xdot_true(1:3) + cross(omega_b, v_body);

    w_en_ned = [v_ned_true(2)/(radius_n + height); ...
        -v_ned_true(1)/(radius_m + height); ...
        -v_ned_true(2)*tan_lat/(radius_n + height)];

    % Trọng lực chuẩn theo vĩ độ và độ sâu tức thời qua algo/gravity_fossen.m
    [g_curr, g_ned_curr] = gravity_fossen(lat_rad, zn);
    f_b_true = a_b_true - Cbn_true' * g_ned_curr + Cbn_true' * cross(2*w_ie_ned + w_en_ned, v_ned_true);
    w_ib_b_true = omega_b + Cbn_true' * (w_ie_ned + w_en_ned);

    % Sinh số đo IMU theo datasheet (bias bật máy + in-run Gauss-Markov + ARW/VRW)
    [gyro_meas, accel_meas, imu_state] = GEO_FOG_3D('imu', ins_spec, imu_state, ...
        w_ib_b_true, f_b_true, h);

    % 4.3. Giả lập phép đo cảm biến ngoài (GNSS, Depth, DVL)
    gnss_meas = zeros(7, 1);
    depth_meas = 0.0;
    dvl_meas = zeros(3, 1);
    flags = [0; 0; 0];

    % Cập nhật DVL và Depth theo chu kỳ trích mẫu 5 Hz
    if mod(i, dvl_step) == 0
        % Cảm biến đo sâu
        depth_meas = zn + sigma_depth * randn;
        flags(2) = 1;

        % Cảm biến DVL A125 gọi qua module Sensor/DVL_A125
        dvl_meas = DVL_A125('step', v_body, omega_b, lever_arm, true_scale, Cbs_true, dvl_spec);
        flags(3) = 1;

        % GNSS RTK ở mặt nước (chỉ có khi t <= T_gnss_cut)
        if t(i) <= T_gnss_cut
            gnss_meas = GEO_FOG_3D('gnss', ins_spec, [xn; yn; zn], v_ned_true, psi);
            flags(1) = 1;
        end
    end

    % 4.4. Thực hiện 1 bước lọc ES-EKF 19 trạng thái
    cfg(2) = g_curr; % Cập nhật trọng lực theo độ sâu cho bộ lọc
    [state_est, P_est, update_status, nis] = esekf19_step( ...
        state_est, P_est, Q, gyro_meas, accel_meas, ...
        gnss_meas, depth_meas, dvl_meas, flags, cfg);

    % 4.5. Trích xuất thông tin định vị và hiệu chuẩn từ ES-EKF
    p_hat = state_est(1:3);
    v_hat = state_est(4:6);
    q_hat = state_est(7:10);
    [phi_hat, theta_hat, psi_hat] = quat_to_euler_local(q_hat);
    scale_hat = state_est(17);
    bs_hat = state_est(18:20);

    % 4.6. Dẫn đường và điều khiển khép vòng bằng ước lượng ES-EKF
    % Chú ý: Dẫn đường nhận [p_hat(1), p_hat(2), p_hat(3)] thay vì vị trí thật
    [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat] = ...
        ALOS3D(p_hat(1), p_hat(2), p_hat(3), Delta_h, Delta_v, gamma_h, gamma_v, ...
        M_theta, h, R_switch, wpt);

    [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
    [psi_d, r_d]   = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
    r_d = sat(r_d, r_max);

    % Autopilot nhận góc hướng psi_hat và góc chúi theta_hat từ ES-EKF
    r_gyro_corrected = gyro_meas(3) - state_est(16);
    q_gyro_corrected = gyro_meas(2) - state_est(15);

    delta_r = integralSMCheading(psi_hat, r_gyro_corrected, psi_d, r_d, a_d, ...
        K_d, K_sigma, 1, phi_b, K_yaw, T_yaw, h);
    delta_r = sat(delta_r, delta_max);

    delta_s = -Kp_theta * ssa(theta_hat - theta_d) - Kd_theta * q_gyro_corrected - Ki_theta * theta_int;
    delta_s = sat(delta_s, delta_max);

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

    % Động học chân vịt
    if n_p < n_d
        n_p = n_p + n_rate;
    elseif n_p > n_d
        n_p = n_p - n_rate;
    end
    n_p = sat(n_p, n_max);

    ui = [delta_r; -delta_s; n_p];

    % 4.7. Ghi nhận dữ liệu lịch sử
    pos_true_hist(i, :)      = [xn, yn, zn];
    pos_est_hist(i, :)       = p_hat';
    vel_true_hist(i, :)      = v_ned_true';
    vel_est_hist(i, :)       = v_hat';
    euler_true_hist(i, :)    = [phi, theta, psi];
    euler_est_hist(i, :)     = [phi_hat, theta_hat, psi_hat];
    boresight_est_hist(i, :) = bs_hat';
    scale_est_hist(i)        = scale_hat;
    cov_bs_hist(i, :)        = sqrt(diag(P_est(17:19, 17:19)))';
    cov_scale_hist(i)        = sqrt(P_est(16, 16));
    ui_hist(i, :)            = [delta_r, delta_s, n_p];
    alos_hist(i, :)          = [y_e, z_e, alpha_c_hat, beta_c_hat];
    bg_true_hist(i, :)       = imu_state.bg_total';
    bg_est_hist(i, :)        = state_est(14:16)';
    nis_hist(i, :)           = nis';
    acc_row = nan(1, 3);
    acc_row(flags' ~= 0) = update_status(flags ~= 0)';
    acc_hist(i, :)           = acc_row;

    % 4.8. Cập nhật mô hình vật lý Fossen REMUS 100 (RK4)
    x_phys = rk4(@remus100, h, x_phys, ui, Vc, betaVc, wc);
    theta_int = theta_int + h * ssa(theta_hat - theta_d);
    ui_curr = ui;
end

time_elapsed = toc(tic_sim);
fprintf('Hoàn thành mô phỏng tích hợp Fossen + ES-EKF trong %.2f giây.\n', time_elapsed);

%% 5. LƯU KẾT QUẢ TÍCH HỢP RA FILE .MAT
sensor_name = ins_spec.ten;
save_vars = {'t', 'pos_true_hist', 'pos_est_hist', 'vel_true_hist', 'vel_est_hist', ...
    'euler_true_hist', 'euler_est_hist', 'boresight_est_hist', 'scale_est_hist', ...
    'cov_bs_hist', 'cov_scale_hist', 'ui_hist', 'alos_hist', 'wpt', 'true_scale', ...
    'true_boresight', 'h', 'T_final', 'T_gnss_cut', ...
    'bg_true_hist', 'bg_est_hist', 'nis_hist', 'acc_hist', 'sensor_model', 'sensor_name'};
save(fullfile(script_dir, sprintf('ket_qua_esekf_%s.mat', sensor_model)), save_vars{:});
save_path = fullfile(script_dir, 'ket_qua_fossen_esekf_integrated.mat');
if save_main
    save(save_path, save_vars{:});
end
fprintf('Đã lưu dữ liệu tích hợp vào: %s\n', save_path);

%% 6. VẼ ĐỒ THỊ KẾT QUẢ TÍCH HỢP VÀ HỘI TỤ HIỆU CHUẨN
fig = figure('Name', 'Integrated Fossen REMUS 100 with ES-EKF Calibration', ...
    'Position', [50, 50, 1300, 850], 'Visible', 'off');

% Đồ thị 3D quỹ đạo thực tế và quỹ đạo ước lượng
subplot(2, 2, 1);
plot3(pos_true_hist(:, 2), pos_true_hist(:, 1), pos_true_hist(:, 3), 'b-', 'LineWidth', 2);
hold on;
plot3(pos_est_hist(:, 2), pos_est_hist(:, 1), pos_est_hist(:, 3), 'm--', 'LineWidth', 1.5);
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'k:', 'LineWidth', 1);
grid on;
set(gca, 'ZDir', 'reverse');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Down (m)');
title('Quy dao 3D: REMUS 100 dan duong boi ES-EKF', 'FontWeight', 'bold');
legend('Tau chay thuc te', 'Uoc luong ES-EKF', 'Waypoints', 'Location', 'best');
view(45, 25);

% Đồ thị hội tụ góc Boresight DVL (Roll, Pitch, Yaw)
subplot(2, 2, 2);
plot(t, rad2deg(boresight_est_hist(:, 1)), 'r-', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(boresight_est_hist(:, 2)), 'g-', 'LineWidth', 1.8);
plot(t, rad2deg(boresight_est_hist(:, 3)), 'b-', 'LineWidth', 1.8);
yline(rad2deg(true_boresight(1)), 'r--', 'LineWidth', 1.5);
yline(rad2deg(true_boresight(2)), 'g--', 'LineWidth', 1.5);
yline(rad2deg(true_boresight(3)), 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Thoi gian (s)'); ylabel('Goc Boresight (deg)');
title('Hoi tu goc lech truc Boresight DVL (Roll, Pitch, Yaw)', 'FontWeight', 'bold');
legend('Roll est', 'Pitch est', 'Yaw est', 'Roll true', 'Pitch true', 'Yaw true', 'Location', 'best');

% Đồ thị hội tụ hệ số tỉ lệ Scale Factor DVL
subplot(2, 2, 3);
plot(t, scale_est_hist * 100, 'b-', 'LineWidth', 2); hold on;
plot(t, (scale_est_hist + 3*cov_scale_hist) * 100, 'k:', 'LineWidth', 1.2);
plot(t, (scale_est_hist - 3*cov_scale_hist) * 100, 'k:', 'LineWidth', 1.2);
yline(true_scale * 100, 'r--', 'LineWidth', 1.8);
grid on;
xlabel('Thoi gian (s)'); ylabel('Scale Factor (%)');
title('Hoi tu he so ti le Scale Factor DVL (Cold Start)', 'FontWeight', 'bold');
legend('Scale est', '+3\sigma', '-3\sigma', 'Scale true (+1.5%)', 'Location', 'best');

% Đồ thị sai số định vị vị trí NED
subplot(2, 2, 4);
pos_err = pos_true_hist - pos_est_hist;
pos_err_norm = sqrt(sum(pos_err.^2, 2));
plot(t, pos_err(:, 1), 'r-', 'LineWidth', 1.3); hold on;
plot(t, pos_err(:, 2), 'g-', 'LineWidth', 1.3);
plot(t, pos_err(:, 3), 'b-', 'LineWidth', 1.3);
plot(t, pos_err_norm, 'k-', 'LineWidth', 2);
xline(T_gnss_cut, 'm--', sprintf('Cut GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.5);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so vi tri (m)');
title('Sai so vi tri UUV giua Ground Truth va ES-EKF', 'FontWeight', 'bold');
legend('Err North', 'Err East', 'Err Down', 'Tong sai so 3D', 'Location', 'best');

if save_main
    img_path = fullfile(script_dir, 'fig_fossen_esekf_integrated.png');
else
    img_path = fullfile(script_dir, sprintf('fig_fossen_esekf_integrated_%s.png', sensor_model));
end
saveas(fig, img_path);
close(fig);
fprintf('Đã xuất hình ảnh tích hợp vào: %s\n', img_path);

%% 7. TỔNG KẾT THỐNG KÊ HIỆU CHUẨN VÀ ĐỊNH VỊ
final_bs_deg = rad2deg(boresight_est_hist(end, :)');
true_bs_deg  = rad2deg(true_boresight);
err_bs_deg   = abs(final_bs_deg - true_bs_deg);
final_scale_pct = scale_est_hist(end) * 100;
true_scale_pct  = true_scale * 100;
err_scale_pct   = abs(final_scale_pct - true_scale_pct);

fprintf('\n--- KET QUA HIEU CHUAN TRUC TUYEN ES-EKF ---\n');
fprintf('Boresight Roll  : Uoc luong = %6.3f deg | That = %6.3f deg | Sai so = %6.3f deg\n', ...
    final_bs_deg(1), true_bs_deg(1), err_bs_deg(1));
fprintf('Boresight Pitch : Uoc luong = %6.3f deg | That = %6.3f deg | Sai so = %6.3f deg\n', ...
    final_bs_deg(2), true_bs_deg(2), err_bs_deg(2));
fprintf('Boresight Yaw   : Uoc luong = %6.3f deg | That = %6.3f deg | Sai so = %6.3f deg\n', ...
    final_bs_deg(3), true_bs_deg(3), err_bs_deg(3));
fprintf('Scale Factor DVL: Uoc luong = %6.3f %%   | That = %6.3f %%   | Sai so = %6.3f %%\n', ...
    final_scale_pct, true_scale_pct, err_scale_pct);
fprintf('Sai so vi tri 3D cuoi cung: %.2f m (Sau %.0fs hanh trinh va mat GNSS tu giay %.0f)\n', ...
    pos_err_norm(end), T_final, T_gnss_cut);
fprintf('=========================================================================\n');
end

%% CÁC HÀM PHỤ TRỢ CỤC BỘ
function s = ins_fi_spec_local()
%INS_FI_SPEC_LOCAL Thông số INS-DM-FI (Inertial Labs) cùng cấu trúc với GEO_FOG_3D('spec').
% Nguồn: thong_so_he_thong_chuan.m (datasheet INS-DM-FI rev 1.8) và bảng so sánh .docx.
p = thong_so_he_thong_chuan();
d2r = pi/180;
s.ten = 'INS-DM-FI (Inertial Labs)';
s.co_tu_ke = false;
s.gyro.arw         = p.ins.gyro.arw_deg_sqrt_h * d2r / 60;        % 0.025 deg/sqrt(h)
s.gyro.bias_instab = p.ins.gyro.bias_in_run_deg_h * d2r / 3600;   % 0.025 deg/h (trang bán hàng, không có trong datasheet IMU)
s.gyro.bias_tau    = 300;                                         % GIẢ ĐỊNH, giống GEO-FOG
s.gyro.bias_rw     = s.gyro.bias_instab * sqrt(2 / s.gyro.bias_tau);
s.gyro.dai_do      = p.ins.gyro.dai_do_deg_s * d2r;
s.acc.vrw          = p.ins.acc.vrw_mps_sqrt_h / 60;               % 0.015 m/s/sqrt(h)
s.acc.bias_instab  = p.ins.acc.bias_in_run_mg * 1e-3 * p.dat.g;   % 0.005 mg
s.acc.bias_tau     = p.ins.acc.tau_in_run_s;                      % 100 s (GIẢ ĐỊNH trong thong_so)
s.acc.dai_do       = p.ins.acc.dai_do_g * p.dat.g;
s.gnss.pos_h       = 0.01;                                        % .docx: RTK 0.01 m ngang
s.gnss.pos_v       = 0.02;                                        % .docx: RTK 0.02 m đứng
s.gnss.vel         = 0.03;                                        % .docx: 0.03 m/s
s.gnss.heading     = 0.15 * d2r;                                  % .docx: 0.15 deg khi 2 anten cách 1 m (GIẢ ĐỊNH baseline 1 m trên thân 1.6 m)
end

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
