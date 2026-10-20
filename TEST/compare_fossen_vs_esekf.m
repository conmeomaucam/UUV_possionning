function compare_fossen_vs_esekf()
%% COMPARE_FOSSEN_VS_ESEKF - So sánh trực tiếp Fossen Standalone và Fossen + ES-EKF
% Mục tiêu:
% 1. So sánh 3 đường quỹ đạo riêng biệt:
%    - Đường 1: Fossen Standalone (Mô hình chuẩn lý tưởng theo tool Fossen MSS).
%    - Đường 2: Tàu thật (Tàu REMUS 100 thực tế chịu nhiễu, được dẫn đường khép vòng bởi ES-EKF).
%    - Đường 3: ES-EKF ước lượng (Quỹ đạo do thuật toán ES-EKF ước lượng nội bộ).
%    - Các điểm Waypoints.
% 2. So sánh sai số theo từng phương (North, East) và sai số độ sâu (Down) giữa 3 đường.
% 3. So sánh vận tốc theo 3 phương (v_North, v_East, v_Down) của cả 3 đường và sai số vận tốc.
% 4. Khảo sát quá trình tự hội tụ hiệu chuẩn trực tuyến của ES-EKF (Boresight và Scale Factor DVL).

clc; close all;
fprintf('=== GIAI DOAN 3: SO SANH TRUC TIEP FOSSEN STANDALONE VA FOSSEN + ES-EKF ===\n');

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
if ~exist(fullfile(script_dir, 'ket_qua_fossen_standalone.mat'), 'file') && exist(fullfile(pwd, 'TEST', 'ket_qua_fossen_standalone.mat'), 'file')
    script_dir = fullfile(pwd, 'TEST');
end
standalone_mat = fullfile(script_dir, 'ket_qua_fossen_standalone.mat');
integrated_mat = fullfile(script_dir, 'ket_qua_fossen_esekf_integrated.mat');

if ~exist(standalone_mat, 'file') || ~exist(integrated_mat, 'file')
    error('Chua tim thay day du file ket qua. Hay chay run_fossen_standalone va run_fossen_esekf_integrated truoc!');
end

% Nạp dữ liệu Fossen Standalone
f_data = load(standalone_mat);
% Nạp dữ liệu Fossen + ES-EKF
e_data = load(integrated_mat);

t = e_data.t;
wpt = e_data.wpt;
T_gnss_cut = e_data.T_gnss_cut;
N_steps = length(t);

%% 1. TRÍCH XUẤT VÀ TÍNH TOÁN DỮ LIỆU SO SÁNH 3 ĐƯỜNG
% Vị trí 3 đường [N, E, D]
pos_fossen   = f_data.pos_history;      % 1. Fossen Standalone (Ground Truth lý tưởng)
pos_true_ekf = e_data.pos_true_hist;    % 2. Tàu thật (có ES-EKF khép vòng)
pos_est_ekf  = e_data.pos_est_hist;     % 3. ES-EKF ước lượng

% Vận tốc thân tàu & chuyển đổi sang NED cho Fossen Standalone
v_ned_fossen = zeros(N_steps, 3);
for k = 1:N_steps
    phi_f   = f_data.euler_history(k, 1);
    theta_f = f_data.euler_history(k, 2);
    psi_f   = f_data.euler_history(k, 3);
    cr = cos(phi_f); sr = sin(phi_f);
    cp = cos(theta_f); sp = sin(theta_f);
    cy = cos(psi_f); sy = sin(psi_f);
    Cbn_f = [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy; ...
             cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy; ...
             -sp,   sr*cp,            cr*cp];
    v_ned_fossen(k, :) = (Cbn_f * f_data.vel_body_history(k, :)')';
end

% Vận tốc NED của Tàu thật và ES-EKF ước lượng
vel_true_ekf = e_data.vel_true_hist;    % [v_N, v_E, v_D] Tàu thật
vel_est_ekf  = e_data.vel_est_hist;     % [v_N, v_E, v_D] ES-EKF ước lượng

% Sai số vị trí
err_pos_ekf = pos_true_ekf - pos_est_ekf;            % Tàu thật - ES-EKF
err_pos_norm = sqrt(sum(err_pos_ekf.^2, 2));

diff_true_fossen = pos_true_ekf - pos_fossen;        % Tàu thật - Fossen lý tưởng
diff_true_norm = sqrt(sum(diff_true_fossen.^2, 2));

% Sai số vận tốc
err_vel_ekf = vel_true_ekf - vel_est_ekf;
err_vel_norm = sqrt(sum(err_vel_ekf.^2, 2));

% Dữ liệu Boresight & Scale
bs_est_deg = rad2deg(e_data.boresight_est_hist);
bs_true_deg = rad2deg(e_data.true_boresight(:)');
scale_est_pct = e_data.scale_est_hist * 100;
scale_true_pct = e_data.true_scale * 100;
cov_bs_deg = rad2deg(e_data.cov_bs_hist);
cov_scale_pct = e_data.cov_scale_hist * 100;

%% 2. BIỂU ĐỒ 1: QUỸ ĐẠO 3 ĐƯỜNG RIÊNG BIỆT (3D, MẶT BẰNG 2D, TRẮC DỌC VÀ ĐỘ SÂU THEO THỜI GIAN)
fig1 = figure('Name', 'So Sanh 3 Duong Quy Dao: Fossen vs Tau That vs ES-EKF', ...
    'Position', [40, 40, 1400, 900], 'Visible', 'off');

% Subplot 1: Không gian 3D
subplot(2, 2, 1);
plot3(pos_fossen(:, 2), pos_fossen(:, 1), pos_fossen(:, 3), 'b-', 'LineWidth', 2.0); hold on;
plot3(pos_true_ekf(:, 2), pos_true_ekf(:, 1), pos_true_ekf(:, 3), 'r-', 'LineWidth', 2.0);
plot3(pos_est_ekf(:, 2), pos_est_ekf(:, 1), pos_est_ekf(:, 3), 'm--', 'LineWidth', 1.5);
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'k:', 'LineWidth', 1);
grid on; set(gca, 'ZDir', 'reverse');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Down - Do sau (m)');
title('1. Quy dao 3D (3 duong rieng biet)', 'FontWeight', 'bold');
legend('1. Fossen Standalone (Ly tuong)', ...
       '2. Tau that (Tich hop ES-EKF)', ...
       '3. ES-EKF uoc luong', ...
       'Waypoints', 'Location', 'best');
view(40, 25);

% Subplot 2: Mặt bằng 2D (North - East)
subplot(2, 2, 2);
plot(pos_fossen(:, 2), pos_fossen(:, 1), 'b-', 'LineWidth', 2.0); hold on;
plot(pos_true_ekf(:, 2), pos_true_ekf(:, 1), 'r-', 'LineWidth', 2.0);
plot(pos_est_ekf(:, 2), pos_est_ekf(:, 1), 'm--', 'LineWidth', 1.5);
plot(wpt.pos.y, wpt.pos.x, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
plot(wpt.pos.y, wpt.pos.x, 'k:', 'LineWidth', 1);
grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)');
title('2. Mat bang North - East (Nhin tu tren xuong)', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Waypoints', 'Location', 'best');

% Subplot 3: Trắc dọc độ sâu theo East
subplot(2, 2, 3);
plot(pos_fossen(:, 2), pos_fossen(:, 3), 'b-', 'LineWidth', 2.0); hold on;
plot(pos_true_ekf(:, 2), pos_true_ekf(:, 3), 'r-', 'LineWidth', 2.0);
plot(pos_est_ekf(:, 2), pos_est_ekf(:, 3), 'm--', 'LineWidth', 1.5);
plot(wpt.pos.y, wpt.pos.z, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
grid on; set(gca, 'YDir', 'reverse');
xlabel('East (m)'); ylabel('Down - Do sau (m)');
title('3. Trac doc do sau theo quang duong East', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Waypoints', 'Location', 'best');

% Subplot 4: Diễn biến độ sâu theo thời gian (Depth vs Time)
subplot(2, 2, 4);
plot(t, pos_fossen(:, 3), 'b-', 'LineWidth', 2.0); hold on;
plot(t, pos_true_ekf(:, 3), 'r-', 'LineWidth', 2.0);
plot(t, pos_est_ekf(:, 3), 'm--', 'LineWidth', 1.5);
xline(T_gnss_cut, 'k--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.3);
grid on; set(gca, 'YDir', 'reverse');
xlabel('Thoi gian (s)'); ylabel('Do sau (m)');
title('4. Dien bien do sau Down theo thoi gian', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Cat GNSS', 'Location', 'best');

fig1_path = fullfile(script_dir, 'fig_so_sanh_quy_dao.png');
saveas(fig1, fig1_path);
close(fig1);
fprintf('Đã lưu biểu đồ 3 đường quỹ đạo vào: %s\n', fig1_path);

%% 3. BIỂU ĐỒ 2: SAI SỐ THEO TỪNG PHƯƠNG (NORTH, EAST) VÀ ĐỘ SÂU (DOWN)
fig2 = figure('Name', 'Sai So Tung Phuong Va Do Sau', ...
    'Position', [60, 60, 1350, 850], 'Visible', 'off');

% 1. Phương Bắc (North Drift)
subplot(2, 2, 1);
plot(t, diff_true_fossen(:, 1), 'r-', 'LineWidth', 1.8); hold on;
plot(t, err_pos_ekf(:, 1), 'k--', 'LineWidth', 1.3);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so North (m)');
title('1. Sai so phuong Bac (North Drift)', 'FontWeight', 'bold');
legend('Tau that - Fossen ly tuong', 'Tau that - ES-EKF uoc luong', 'Location', 'best');

% 2. Phương Đông (East Drift)
subplot(2, 2, 2);
plot(t, diff_true_fossen(:, 2), 'b-', 'LineWidth', 1.8); hold on;
plot(t, err_pos_ekf(:, 2), 'k--', 'LineWidth', 1.3);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so East (m)');
title('2. Sai so phuong Dong (East Drift)', 'FontWeight', 'bold');
legend('Tau that - Fossen ly tuong', 'Tau that - ES-EKF uoc luong', 'Location', 'best');

% 3. Phương thẳng đứng / Độ sâu (Down Drift / Sai số độ sâu)
subplot(2, 2, 3);
plot(t, diff_true_fossen(:, 3), 'Color', [0, 0.6, 0], 'LineWidth', 1.8); hold on;
plot(t, err_pos_ekf(:, 3), 'k--', 'LineWidth', 1.3);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so do sau Down (m)');
title('3. Sai so do sau (Down Drift)', 'FontWeight', 'bold');
legend('Tau that - Fossen ly tuong', 'Tau that - ES-EKF uoc luong', 'Location', 'best');

% 4. Tổng sai số vị trí 3D
subplot(2, 2, 4);
plot(t, diff_true_norm, 'r-', 'LineWidth', 1.8); hold on;
plot(t, err_pos_norm, 'k-', 'LineWidth', 2.0);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Tong sai so 3D (m)');
title('4. Tong hop sai so vi tri khong gian 3D', 'FontWeight', 'bold');
legend('Tong lech: Tau that - Fossen', 'Sai so dinh vi: Tau that - ES-EKF', 'Location', 'best');

fig2_path = fullfile(script_dir, 'fig_so_sanh_sai_so_tung_phuong.png');
saveas(fig2, fig2_path);
close(fig2);
fprintf('Đã lưu biểu đồ sai số từng phương và độ sâu vào: %s\n', fig2_path);

%% 4. BIỂU ĐỒ 3: SO SÁNH VẬN TỐC THEO 3 PHƯƠNG (v_North, v_East, v_Down)
fig3 = figure('Name', 'So Sanh Van Toc Theo 3 Phuong', ...
    'Position', [80, 80, 1350, 850], 'Visible', 'off');

% 1. Vận tốc phương Bắc (v_North)
subplot(2, 2, 1);
plot(t, v_ned_fossen(:, 1), 'b-', 'LineWidth', 1.8); hold on;
plot(t, vel_true_ekf(:, 1), 'r-', 'LineWidth', 1.6);
plot(t, vel_est_ekf(:, 1), 'm--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'k--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.2);
grid on;
xlabel('Thoi gian (s)'); ylabel('v_{North} (m/s)');
title('1. Van toc phuong Bac (v_{North})', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Location', 'best');

% 2. Vận tốc phương Đông (v_East)
subplot(2, 2, 2);
plot(t, v_ned_fossen(:, 2), 'b-', 'LineWidth', 1.8); hold on;
plot(t, vel_true_ekf(:, 2), 'r-', 'LineWidth', 1.6);
plot(t, vel_est_ekf(:, 2), 'm--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'k--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.2);
grid on;
xlabel('Thoi gian (s)'); ylabel('v_{East} (m/s)');
title('2. Van toc phuong Dong (v_{East})', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Location', 'best');

% 3. Vận tốc phương thẳng đứng / Chìm nổi (v_Down)
subplot(2, 2, 3);
plot(t, v_ned_fossen(:, 3), 'b-', 'LineWidth', 1.8); hold on;
plot(t, vel_true_ekf(:, 3), 'r-', 'LineWidth', 1.6);
plot(t, vel_est_ekf(:, 3), 'm--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'k--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.2);
grid on;
xlabel('Thoi gian (s)'); ylabel('v_{Down} (m/s)');
title('3. Van toc phuong thang dung (v_{Down})', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Location', 'best');

% 4. Sai số vận tốc giữa Tàu thật và ES-EKF (v_true - v_est)
subplot(2, 2, 4);
plot(t, err_vel_ekf(:, 1), 'r-', 'LineWidth', 1.3); hold on;
plot(t, err_vel_ekf(:, 2), 'g-', 'LineWidth', 1.3);
plot(t, err_vel_ekf(:, 3), 'b-', 'LineWidth', 1.3);
plot(t, err_vel_norm, 'k-', 'LineWidth', 1.8);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on;
xlabel('Thoi gian (s)'); ylabel('Sai so van toc (m/s)');
title('4. Sai so van toc ES-EKF (Tau that - Uoc luong)', 'FontWeight', 'bold');
legend('Err v_N', 'Err v_E', 'Err v_D', 'Tong |v_{err}|', 'Location', 'best');

fig3_path = fullfile(script_dir, 'fig_so_sanh_van_toc_3_phuong.png');
saveas(fig3, fig3_path);
close(fig3);
fprintf('Đã lưu biểu đồ vận tốc theo 3 phương vào: %s\n', fig3_path);

%% 5. BIỂU ĐỒ 4: HỘI TỤ BORESIGHT VÀ SCALE FACTOR DVL
fig4 = figure('Name', 'Hoi Tu Hieu Chuan Boresight va Scale Factor', ...
    'Position', [100, 100, 1200, 800], 'Visible', 'off');

% Boresight Roll
subplot(2, 2, 1);
plot(t, bs_est_deg(:, 1), 'r-', 'LineWidth', 1.8); hold on;
plot(t, bs_est_deg(:, 1) + 3*cov_bs_deg(:, 1), 'k:', 'LineWidth', 1);
plot(t, bs_est_deg(:, 1) - 3*cov_bs_deg(:, 1), 'k:', 'LineWidth', 1);
yline(bs_true_deg(1), 'b--', sprintf('True = %.2f deg', bs_true_deg(1)), 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut));
grid on;
xlabel('Thoi gian (s)'); ylabel('Boresight Roll (deg)');
title('Hieu chuan goc lech truc Roll Boresight', 'FontWeight', 'bold');
legend('Uoc luong Roll', '+3\sigma', '-3\sigma', 'Gia tri that', 'Location', 'best');

% Boresight Pitch
subplot(2, 2, 2);
plot(t, bs_est_deg(:, 2), 'g-', 'LineWidth', 1.8); hold on;
plot(t, bs_est_deg(:, 2) + 3*cov_bs_deg(:, 2), 'k:', 'LineWidth', 1);
plot(t, bs_est_deg(:, 2) - 3*cov_bs_deg(:, 2), 'k:', 'LineWidth', 1);
yline(bs_true_deg(2), 'b--', sprintf('True = %.2f deg', bs_true_deg(2)), 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut));
grid on;
xlabel('Thoi gian (s)'); ylabel('Boresight Pitch (deg)');
title('Hieu chuan goc lech truc Pitch Boresight', 'FontWeight', 'bold');
legend('Uoc luong Pitch', '+3\sigma', '-3\sigma', 'Gia tri that', 'Location', 'best');

% Boresight Yaw
subplot(2, 2, 3);
plot(t, bs_est_deg(:, 3), 'b-', 'LineWidth', 1.8); hold on;
plot(t, bs_est_deg(:, 3) + 3*cov_bs_deg(:, 3), 'k:', 'LineWidth', 1);
plot(t, bs_est_deg(:, 3) - 3*cov_bs_deg(:, 3), 'k:', 'LineWidth', 1);
yline(bs_true_deg(3), 'r--', sprintf('True = %.2f deg', bs_true_deg(3)), 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut));
grid on;
xlabel('Thoi gian (s)'); ylabel('Boresight Yaw (deg)');
title('Hieu chuan goc lech truc Yaw Boresight', 'FontWeight', 'bold');
legend('Uoc luong Yaw', '+3\sigma', '-3\sigma', 'Gia tri that', 'Location', 'best');

% DVL Scale Factor
subplot(2, 2, 4);
plot(t, scale_est_pct, 'm-', 'LineWidth', 2.0); hold on;
plot(t, scale_est_pct + 3*cov_scale_pct, 'k:', 'LineWidth', 1);
plot(t, scale_est_pct - 3*cov_scale_pct, 'k:', 'LineWidth', 1);
yline(scale_true_pct, 'b--', sprintf('True = %.2f %%', scale_true_pct), 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut));
grid on;
xlabel('Thoi gian (s)'); ylabel('Scale Factor (%)');
title('Hieu chuan he so ti le Scale Factor DVL', 'FontWeight', 'bold');
legend('Uoc luong Scale', '+3\sigma', '-3\sigma', 'Gia tri that', 'Location', 'best');

fig4_path = fullfile(script_dir, 'fig_so_sanh_boresight_scale.png');
saveas(fig4, fig4_path);
close(fig4);
fprintf('Đã lưu biểu đồ hội tụ hiệu chuẩn vào: %s\n', fig4_path);

%% 6. BÁO CÁO TỔNG HỢP VÀ GHI FILE
final_err_bs_deg = abs(bs_est_deg(end, :) - bs_true_deg);
final_err_scale_pct = abs(scale_est_pct(end) - scale_true_pct);

fprintf('\n========================================================================================\n');
fprintf('  BANG SO SANH TONG HOP 3 DUONG: FOSSEN STANDALONE vs TAU THAT vs ES-EKF\n');
fprintf('========================================================================================\n');
fprintf('| %-32s | %-16s | %-16s | %-16s |\n', 'Dai luong danh gia', 'Fossen Standalone', 'Tau that', 'ES-EKF uoc luong');
fprintf('|----------------------------------|------------------|------------------|------------------|\n');
fprintf('| Vi tri North cuoi cung (m)       | %16.2f | %16.2f | %16.2f |\n', pos_fossen(end, 1), pos_true_ekf(end, 1), pos_est_ekf(end, 1));
fprintf('| Vi tri East cuoi cung (m)        | %16.2f | %16.2f | %16.2f |\n', pos_fossen(end, 2), pos_true_ekf(end, 2), pos_est_ekf(end, 2));
fprintf('| Do sau Down cuoi cung (m)        | %16.2f | %16.2f | %16.2f |\n', pos_fossen(end, 3), pos_true_ekf(end, 3), pos_est_ekf(end, 3));
fprintf('| Van toc v_North cuoi (m/s)       | %16.3f | %16.3f | %16.3f |\n', v_ned_fossen(end, 1), vel_true_ekf(end, 1), vel_est_ekf(end, 1));
fprintf('| Van toc v_East cuoi (m/s)        | %16.3f | %16.3f | %16.3f |\n', v_ned_fossen(end, 2), vel_true_ekf(end, 2), vel_est_ekf(end, 2));
fprintf('| Van toc v_Down cuoi (m/s)        | %16.3f | %16.3f | %16.3f |\n', v_ned_fossen(end, 3), vel_true_ekf(end, 3), vel_est_ekf(end, 3));
fprintf('| Sai so vi tri 3D cuoi (m)        | %16s | Lech Fossen: %5.2f | Sai so EKF: %5.2f |\n', ...
    'Goc chuan', diff_true_norm(end), err_pos_norm(end));
fprintf('| Sai so Boresight Roll / Pitch / Yaw : %5.3f / %5.3f / %5.3f deg\n', ...
    final_err_bs_deg(1), final_err_bs_deg(2), final_err_bs_deg(3));
fprintf('| Sai so Scale Factor DVL             : %5.3f %%\n', final_err_scale_pct);
fprintf('========================================================================================\n');

txt_file = fullfile(script_dir, 'bao_cao_so_sanh_fossen_vs_esekf.txt');
fid = fopen(txt_file, 'w', 'n', 'UTF-8');
if fid ~= -1
    fprintf(fid, '========================================================================================\r\n');
    fprintf(fid, '  BAO CAO SO SANH 3 DUONG: FOSSEN STANDALONE vs TAU THAT vs ES-EKF (GEO-FOG 3D)\r\n');
    fprintf(fid, '========================================================================================\r\n\r\n');
    fprintf(fid, '1. TONG QUAN:\r\n');
    fprintf(fid, '   - Thoi gian chay: %.0f s (Lan mat GNSS tu giay %.0f)\r\n', t(end), T_gnss_cut);
    fprintf(fid, '   - Cam bien: GEO-FOG 3D Dual INS (Khong dung tu ke)\r\n');
    fprintf(fid, '   - 3 duong danh gia:\r\n');
    fprintf(fid, '     + Duong 1: Fossen Standalone (Ly tuong, biet truoc vi tri)\r\n');
    fprintf(fid, '     + Duong 2: Tau that (Chay duoi nuoc, co nhieu cam bien, lai theo ES-EKF)\r\n');
    fprintf(fid, '     + Duong 3: ES-EKF uoc luong (Thuat toan loc 19 trang thai)\r\n\r\n');
    fprintf(fid, '2. KET QUA HIEN CHUAN TRUC TUYEN:\r\n');
    fprintf(fid, '   - Boresight Roll : Uoc = %6.3f deg | That = %6.3f deg | Sai so = %6.3f deg\r\n', bs_est_deg(end, 1), bs_true_deg(1), final_err_bs_deg(1));
    fprintf(fid, '   - Boresight Pitch: Uoc = %6.3f deg | That = %6.3f deg | Sai so = %6.3f deg\r\n', bs_est_deg(end, 2), bs_true_deg(2), final_err_bs_deg(2));
    fprintf(fid, '   - Boresight Yaw  : Uoc = %6.3f deg | That = %6.3f deg | Sai so = %6.3f deg\r\n', bs_est_deg(end, 3), bs_true_deg(3), final_err_bs_deg(3));
    fprintf(fid, '   - Scale Factor   : Uoc = %6.3f %%   | That = %6.3f %%   | Sai so = %6.3f %%\r\n\r\n', scale_est_pct(end), scale_true_pct, final_err_scale_pct);
    fprintf(fid, '3. SAI SO VI TRI VA VAN TOC CUOI CUNG:\r\n');
    fprintf(fid, '   - Sai so vi tri 3D noi bo (Tau that - ES-EKF): %.4f m\r\n', err_pos_norm(end));
    fprintf(fid, '   - Do lech vi tri 3D (Tau that - Fossen ly tuong) : %.4f m\r\n', diff_true_norm(end));
    fprintf(fid, '   - Sai so van toc 3D noi bo (Tau that - ES-EKF): %.4f m/s\r\n', err_vel_norm(end));
    fprintf(fid, '========================================================================================\r\n');
    fclose(fid);
end

end
