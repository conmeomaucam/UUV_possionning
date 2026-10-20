function plot_danh_gia_geofog_esekf()
%% PLOT_DANH_GIA_GEOFOG_ESEKF - Đánh giá sai số giữa Tàu thật và ES-EKF (GEO-FOG 3D Dual)
% Mục tiêu:
% 1. Loại bỏ hoàn toàn đồ thị so sánh với INS-DM-FI cũ.
% 2. Tính toán và vẽ chi tiết độ lệch giữa Tàu thật (Ground Truth) và ước lượng của ES-EKF:
%    - Sai số vị trí từng phương (North, East, Down) và tổng sai số 3D (thang đo mét).
%    - Sai số góc tư thế (Roll, Pitch, Yaw) theo độ.
%    - Sai số vận tốc 3 trục (u, v, w) theo m/s.
%    - Quá trình tự hội tụ hiệu chuẩn Boresight (Roll, Pitch, Yaw) và Scale Factor DVL.
%    - Quỹ đạo 3D và quỹ đạo mặt bằng 2D của tàu thật bám theo Waypoints.

clc; close all;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

mat_file = fullfile(script_dir, 'ket_qua_esekf_GEO_FOG.mat');
if ~exist(mat_file, 'file')
    mat_file = fullfile(script_dir, 'ket_qua_fossen_esekf_integrated.mat');
end

if ~exist(mat_file, 'file')
    error('Khong tim thay file ket qua: %s', mat_file);
end

data = load(mat_file);
t = data.t;
T_gnss_cut = data.T_gnss_cut;
wpt = data.wpt;

% 1. Trích xuất vị trí và sai số vị trí
pos_true = data.pos_true_hist;  % [N, E, D]
pos_est  = data.pos_est_hist;   % [N, E, D]
err_pos = pos_true - pos_est;
err_pos_3d = sqrt(sum(err_pos.^2, 2));

% 2. Trích xuất góc Euler và tính sai số góc
euler_true_deg = rad2deg(data.euler_true_hist); % [phi, theta, psi]
euler_est_deg  = rad2deg(data.euler_est_hist);
err_euler_deg = euler_true_deg - euler_est_deg;
% Un-wrap sai số góc yaw
err_euler_deg(:, 3) = mod(err_euler_deg(:, 3) + 180, 360) - 180;

% 3. Trích xuất vận tốc và sai số vận tốc
vel_true = data.vel_true_hist;
vel_est  = data.vel_est_hist;
err_vel  = vel_true - vel_est;
err_vel_norm = sqrt(sum(err_vel.^2, 2));

% 4. Trích xuất Boresight và Scale Factor
bs_est_deg = rad2deg(data.boresight_est_hist);
bs_true_deg = rad2deg(data.true_boresight(:)');
scale_est_pct = data.scale_est_hist * 100;
scale_true_pct = data.true_scale * 100;
cov_bs_deg = rad2deg(data.cov_bs_hist);
cov_scale_pct = data.cov_scale_hist * 100;

%% HÌNH 1: TỔNG HỢP SAI SỐ CHI TIẾT (TÀU THẬT vs ES-EKF)
fig1 = figure('Name', 'Danh Gia Sai So: Tau That vs ES-EKF (GEO-FOG 3D)', ...
    'Position', [50, 50, 1400, 900], 'Visible', 'off');

% Ô 1: Sai số vị trí từng phương và 3D
subplot(2, 3, 1);
plot(t, err_pos(:, 1), 'r-', 'LineWidth', 1.4); hold on;
plot(t, err_pos(:, 2), 'g-', 'LineWidth', 1.4);
plot(t, err_pos(:, 3), 'b-', 'LineWidth', 1.4);
plot(t, err_pos_3d, 'k-', 'LineWidth', 2.0);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so vi tri (m)');
title('1. Sai so vi tri: Tau that - ES-EKF', 'FontWeight', 'bold');
legend('Sai so North', 'Sai so East', 'Sai so Down (Do sau)', 'Tong sai so 3D', 'Location', 'best');

% Ô 2: Sai số góc tư thế (Roll, Pitch, Yaw)
subplot(2, 3, 2);
plot(t, err_euler_deg(:, 1), 'r-', 'LineWidth', 1.3); hold on;
plot(t, err_euler_deg(:, 2), 'g-', 'LineWidth', 1.3);
plot(t, err_euler_deg(:, 3), 'b-', 'LineWidth', 1.6);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so goc (deg)');
title('2. Sai so goc huong & tu the (Euler)', 'FontWeight', 'bold');
legend('Sai so Roll', 'Sai so Pitch', 'Sai so Yaw (Huong)', 'Location', 'best');

% Ô 3: Sai số vận tốc 3 trục
subplot(2, 3, 3);
plot(t, err_vel(:, 1), 'r-', 'LineWidth', 1.2); hold on;
plot(t, err_vel(:, 2), 'g-', 'LineWidth', 1.2);
plot(t, err_vel(:, 3), 'b-', 'LineWidth', 1.2);
plot(t, err_vel_norm, 'k-', 'LineWidth', 1.8);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so van toc (m/s)');
title('3. Sai so van toc UUV (m/s)', 'FontWeight', 'bold');
legend('Err v_x', 'Err v_y', 'Err v_z', 'Tong |v|', 'Location', 'best');

% Ô 4: Hội tụ góc lệch Boresight DVL (Roll, Pitch, Yaw)
subplot(2, 3, 4);
plot(t, bs_est_deg(:, 1), 'r-', 'LineWidth', 1.6); hold on;
plot(t, bs_est_deg(:, 2), 'g-', 'LineWidth', 1.6);
plot(t, bs_est_deg(:, 3), 'b-', 'LineWidth', 1.6);
yline(bs_true_deg(1), 'r--', sprintf('Roll that = %.1f deg', bs_true_deg(1)), 'LineWidth', 1.3);
yline(bs_true_deg(2), 'g--', sprintf('Pitch that = %.1f deg', bs_true_deg(2)), 'LineWidth', 1.3);
yline(bs_true_deg(3), 'b--', sprintf('Yaw that = %.1f deg', bs_true_deg(3)), 'LineWidth', 1.3);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Goc Boresight (deg)');
title('4. Hoi tu 3 goc lech truc Boresight DVL', 'FontWeight', 'bold');
legend('Roll est', 'Pitch est', 'Yaw est', 'Location', 'best');

% Ô 5: Hội tụ hệ số tỉ lệ Scale Factor DVL
subplot(2, 3, 5);
plot(t, scale_est_pct, 'm-', 'LineWidth', 2.0); hold on;
plot(t, scale_est_pct + 3*cov_scale_pct, 'k:', 'LineWidth', 1.2);
plot(t, scale_est_pct - 3*cov_scale_pct, 'k:', 'LineWidth', 1.2);
yline(scale_true_pct, 'b--', sprintf('Scale that = %.2f %%', scale_true_pct), 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Scale Factor (%)');
title('5. Hoi tu he so Scale Factor DVL', 'FontWeight', 'bold');
legend('Uoc luong Scale', '+3\sigma', '-3\sigma', 'Gia tri that', 'Location', 'best');

% Ô 6: Mặt bằng 2D North - East: Tàu thật vs ES-EKF vs Waypoints
subplot(2, 3, 6);
plot(pos_true(:, 2), pos_true(:, 1), 'b-', 'LineWidth', 2.2); hold on;
plot(pos_est(:, 2), pos_est(:, 1), 'r--', 'LineWidth', 1.5);
plot(wpt.pos.y, wpt.pos.x, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'y');
plot(wpt.pos.y, wpt.pos.x, 'k:', 'LineWidth', 1.0);
grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)');
title('6. Mat bang North - East (Nhin tu tren xuong)', 'FontWeight', 'bold');
legend('Tau chay thuc te', 'ES-EKF uoc luong', 'Waypoints', 'Location', 'best');

fig1_path = fullfile(script_dir, 'fig_danh_gia_tau_that_vs_esekf_geofog.png');
saveas(fig1, fig1_path);
close(fig1);
fprintf('Da luu bieu do danh gia Tau that vs ES-EKF vao: %s\n', fig1_path);

%% HÌNH 2: PHÓNG TO CHI TIẾT ĐỘ TRÔI KHI LẶN NGẦM (SAU KHI CẮT GNSS)
fig2 = figure('Name', 'Chi Tiet Do Troi Khi Lan Khong GNSS', ...
    'Position', [100, 100, 1200, 750], 'Visible', 'off');

idx_sub = (t >= T_gnss_cut);
t_sub = t(idx_sub) - T_gnss_cut;

subplot(2, 2, 1);
plot(t_sub, err_pos(idx_sub, 1), 'r-', 'LineWidth', 1.8); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Do lech North (m)');
title('Do troi phuong Bac (North Drift) khi lan', 'FontWeight', 'bold');

subplot(2, 2, 2);
plot(t_sub, err_pos(idx_sub, 2), 'b-', 'LineWidth', 1.8); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Do lech East (m)');
title('Do troi phuong Dong (East Drift) khi lan', 'FontWeight', 'bold');

subplot(2, 2, 3);
plot(t_sub, err_pos(idx_sub, 3), 'Color', [0, 0.6, 0], 'LineWidth', 1.8); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Do lech Down (m)');
title('Do troi phuong thang dung / Do sau (Down Drift)', 'FontWeight', 'bold');

subplot(2, 2, 4);
plot(t_sub, err_pos_3d(idx_sub), 'k-', 'LineWidth', 2.0); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Tong sai so 3D (m)');
title('Tong do troi khong gian 3D khi lan', 'FontWeight', 'bold');

fig2_path = fullfile(script_dir, 'fig_chi_tiet_do_troi_khi_lan_geofog.png');
saveas(fig2, fig2_path);
close(fig2);
fprintf('Da luu bieu do chi tiet do troi khi lan vao: %s\n', fig2_path);

%% BÁO CÁO THỐNG KÊ CHI TIẾT
fprintf('\n=========================================================================\n');
fprintf('  KET QUA DANH GIA ES-EKF VOI GEO-FOG 3D DUAL (SO VOI TAU THAT)\n');
fprintf('=========================================================================\n');
fprintf('Tong thoi gian hanh trinh       : %.0f s (Lặn ngầm mất GNSS: %.0f s)\n', t(end), t(end) - T_gnss_cut);
fprintf('Quang duong di chuyen 3D       : %.2f m\n', sum(sqrt(sum(diff(pos_true).^2, 2))));
fprintf('Sai so vi tri 3D luc cat GNSS   : %.4f m\n', err_pos_3d(find(idx_sub, 1)));
fprintf('Sai so vi tri 3D cuc dai khi lan: %.4f m\n', max(err_pos_3d(idx_sub)));
fprintf('Sai so vi tri 3D ket thuc       : %.4f m\n', err_pos_3d(end));
fprintf('Do lech phuong North ket thuc   : %.4f m\n', err_pos(end, 1));
fprintf('Do lech phuong East ket thuc    : %.4f m\n', err_pos(end, 2));
fprintf('Do lech do sau Down ket thuc    : %.4f m\n', err_pos(end, 3));
fprintf('Sai so goc huong Yaw ket thuc   : %.4f deg\n', abs(err_euler_deg(end, 3)));
fprintf('Sai so Boresight Roll           : %.4f deg (Uoc luong = %.3f, That = %.3f)\n', ...
    abs(bs_est_deg(end, 1) - bs_true_deg(1)), bs_est_deg(end, 1), bs_true_deg(1));
fprintf('Sai so Boresight Pitch          : %.4f deg (Uoc luong = %.3f, That = %.3f)\n', ...
    abs(bs_est_deg(end, 2) - bs_true_deg(2)), bs_est_deg(end, 2), bs_true_deg(2));
fprintf('Sai so Boresight Yaw            : %.4f deg (Uoc luong = %.3f, That = %.3f)\n', ...
    abs(bs_est_deg(end, 3) - bs_true_deg(3)), bs_est_deg(end, 3), bs_true_deg(3));
fprintf('Sai so Scale Factor DVL         : %.4f %%  (Uoc luong = %.3f %%, That = %.3f %%)\n', ...
    abs(scale_est_pct(end) - scale_true_pct), scale_est_pct(end), scale_true_pct);
fprintf('=========================================================================\n');
end
