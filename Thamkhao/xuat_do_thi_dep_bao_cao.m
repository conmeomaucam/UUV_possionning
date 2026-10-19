%% SCRIPT XUẤT ĐỒ THỊ CHUYÊN NGHIỆP XUẤT BẢN KHOA HỌC (BỘ DỮ LIỆU CŨ E1)
% Tự động nạp ket_qua_E1_gyro_lever.mat và ket_qua_dau_cong.mat (hoặc ket_qua_goc_M200.mat)
% Thiết kế chuẩn IEEE/Elsevier: Độ phân giải 300 dpi, màu sắc sang trọng, tương phản cao.

clear; clc; close all;

root_dir = fileparts(mfilename('fullpath'));
file_e1 = fullfile(root_dir, 'ket_qua_E1_gyro_lever.mat');
file_base = fullfile(root_dir, 'ket_qua_dau_cong.mat');
if ~exist(file_base, 'file')
    file_base = fullfile(root_dir, 'ket_qua_goc_M200.mat');
end

if ~exist(file_e1, 'file')
    error('Không tìm thấy file %s!', file_e1);
end

fprintf('Đang nạp dữ liệu thí nghiệm cũ...\n');
data_e1 = load(file_e1);
has_base = (exist(file_base, 'file') == 2);
if has_base
    data_base = load(file_base);
    fprintf('  * Đã nạp thành công Baseline: %s\n', file_base);
end
fprintf('  * Đã nạp thành công E1: %s\n', file_e1);

time_eval = data_e1.time_eval;
N_steps = length(time_eval);
T_gnss_cut = 15.0;
idx_p1 = (time_eval <= T_gnss_cut);
idx_p2 = (time_eval > T_gnss_cut);

% Dải tin cậy 95% lý thuyết cho M = 200 runs, DOF = 19
M_runs = 200;
dof = 19;
r1 = 18.1552;
r2 = 19.8638;

% Màu sắc thiết kế cao cấp
c_blue   = [0.00, 0.45, 0.74];
c_red    = [0.85, 0.33, 0.10];
c_green  = [0.13, 0.55, 0.13];
c_gray   = [0.45, 0.45, 0.45];
c_band   = [0.88, 0.94, 0.88]; % Xanh nhạt dịu mắt cho dải tin cậy
c_purple = [0.49, 0.18, 0.56];

%% ========================================================================
% HÌNH 1: ĐỐI CHỨNG ANEES TOÀN HỆ THỐNG 19 TRẠNG THÁI (BASELINE vs E1)
% =========================================================================
fig1 = figure('Name', 'Hinh1_ANEES_DoiChung_ToanHe', 'Color', 'w', 'Position', [100, 100, 960, 520]);
fill([time_eval; flipud(time_eval)], [r1*ones(N_steps,1); r2*ones(N_steps,1)], ...
    c_band, 'EdgeColor', 'none', 'DisplayName', sprintf('Dải tin cậy 95%% lý thuyết [%.2f, %.2f]', r1, r2));
hold on; grid on; set(gca, 'FontSize', 11, 'GridAlpha', 0.25, 'LineWidth', 1.0);

yline(19, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Kỳ vọng lý tưởng E[ANEES] = 19.00');
yline(r1, ':', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(r2, ':', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2, 'HandleVisibility', 'off');

if has_base && isfield(data_base, 'anees')
    plot(time_eval, data_base.anees, 'Color', c_red, 'LineWidth', 1.6, ...
        'DisplayName', sprintf('Baseline cũ (R_{gyro} TẮT): TB = %.2f', mean(data_base.anees)));
end

plot(time_eval, data_e1.anees, 'Color', c_blue, 'LineWidth', 2.0, ...
    'DisplayName', sprintf('Thí nghiệm E1 (Bù R_{gyro\\_lever}): TB = %.2f', mean(data_e1.anees)));

xline(T_gnss_cut, 'm-.', 'LineWidth', 1.8, 'DisplayName', 'Cắt GNSS (t = 15s) -> Chuyển sang Dead-Reckoning');

xlabel('Thời gian mô phỏng (giây)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Chỉ số ANEES toàn hệ thống (19 DOF)', 'FontSize', 12, 'FontWeight', 'bold');
title('KIỂM ĐỊNH TÍNH NHẤT QUÁN ANEES 19 TRẠNG THÁI (M = 200 Chu kỳ Monte Carlo)', ...
    'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 10, 'NumColumns', 2);
ylim([16.5, 23.5]);

exportgraphics(fig1, fullfile(root_dir, 'Hinh1_ANEES_DoiChung_ToanHe.png'), 'Resolution', 300);
fprintf('  -> Đã lưu: Hinh1_ANEES_DoiChung_ToanHe.png (300 dpi)\n');

%% ========================================================================
% HÌNH 2: ANEES BIÊN 5 NHÓM TRẠNG THÁI (E1)
% =========================================================================
fig2 = figure('Name', 'Hinh2_ANEES_5Nhom_TrangThai', 'Color', 'w', 'Position', [120, 80, 1100, 680]);

armse = data_e1.armse;
mean_sigma = data_e1.mean_sigma;

if isfield(data_e1, 'anees_pos')
    sub_data = {data_e1.anees_pos, data_e1.anees_vel, data_e1.anees_att, data_e1.anees_bias, data_e1.anees_cal};
else
    sub_data = { ...
        sum((armse(:, 1:3)./max(mean_sigma(:, 1:3), 1e-9)).^2, 2)', ...
        sum((armse(:, 4:6)./max(mean_sigma(:, 4:6), 1e-9)).^2, 2)', ...
        sum((armse(:, 7:9)./max(mean_sigma(:, 7:9), 1e-9)).^2, 2)', ...
        sum((armse(:, 10:15)./max(mean_sigma(:, 10:15), 1e-9)).^2, 2)', ...
        sum((armse(:, 16:19)./max(mean_sigma(:, 16:19), 1e-9)).^2, 2)'};
end

sub_titles = { ...
    '1. Vị trí 3D (Kỳ vọng = 3.00)', ...
    '2. Vận tốc 3D (Kỳ vọng = 3.00)', ...
    '3. Tư thế 3D (Kỳ vọng = 3.00)', ...
    '4. Bias IMU 6D (Kỳ vọng = 6.00)', ...
    '5. Hiệu chuẩn DVL 4D (Kỳ vọng = 4.00)'};
sub_ideals = [3, 3, 3, 6, 4];
sub_ylims = {[1.5, 4.5], [1.5, 4.5], [1.5, 4.5], [4.0, 8.0], [2.5, 6.5]};

for s = 1:5
    subplot(2, 3, s);
    yline(sub_ideals(s), 'k--', 'LineWidth', 1.5); hold on; grid on;
    xline(T_gnss_cut, 'm-.', 'LineWidth', 1.3);
    plot(time_eval, sub_data{s}, 'Color', c_blue, 'LineWidth', 1.6);
    xlabel('Thời gian (s)', 'FontSize', 10);
    ylabel('ANEES', 'FontSize', 10);
    title(sub_titles{s}, 'FontSize', 11, 'FontWeight', 'bold');
    ylim(sub_ylims{s});
    set(gca, 'FontSize', 10, 'GridAlpha', 0.25);
end

% Subplot 6: ANEES tổng thể tóm tắt
subplot(2, 3, 6);
yline(19, 'k--', 'LineWidth', 1.5); hold on; grid on;
xline(T_gnss_cut, 'm-.', 'LineWidth', 1.3);
plot(time_eval, data_e1.anees, 'Color', c_purple, 'LineWidth', 1.8);
xlabel('Thời gian (s)', 'FontSize', 10);
ylabel('ANEES', 'FontSize', 10);
title('*. Toàn hệ 19 biến (Kỳ vọng = 19.00)', 'FontSize', 11, 'FontWeight', 'bold');
ylim([16.5, 23.5]);
set(gca, 'FontSize', 10, 'GridAlpha', 0.25);

exportgraphics(fig2, fullfile(root_dir, 'Hinh2_ANEES_5Nhom_TrangThai.png'), 'Resolution', 300);
fprintf('  -> Đã lưu: Hinh2_ANEES_5Nhom_TrangThai.png (300 dpi)\n');

%% ========================================================================
% HÌNH 3: ĐỘ HỘI TỤ 4 THÔNG SỐ HIỆU CHUẨN DVL VÀ DẢI ±3-SIGMA
% =========================================================================
fig3 = figure('Name', 'Hinh3_HieuChuan_DVL_4Bien', 'Color', 'w', 'Position', [140, 100, 1000, 640]);

% Scale factor (%)
subplot(2, 2, 1);
plot(time_eval, armse(:, 16)*100, 'Color', c_blue, 'LineWidth', 1.8, 'DisplayName', 'ARMSE thực nghiệm'); hold on; grid on;
plot(time_eval, 3*mean_sigma(:, 16)*100, 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4, 'DisplayName', 'Dải dự báo +3\sigma');
plot(time_eval, -3*mean_sigma(:, 16)*100, 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4, 'DisplayName', '-3\sigma');
xline(T_gnss_cut, 'm-.', 'LineWidth', 1.3);
xlabel('Thời gian (s)', 'FontSize', 10); ylabel('Sai số Scale factor (%)', 'FontSize', 10);
title('(a) Hệ số tỉ lệ DVL (Scale Factor)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
set(gca, 'FontSize', 10, 'GridAlpha', 0.25);

% Roll Boresight (deg)
subplot(2, 2, 2);
plot(time_eval, rad2deg(armse(:, 17)), 'Color', c_blue, 'LineWidth', 1.8); hold on; grid on;
plot(time_eval, 3*rad2deg(mean_sigma(:, 17)), 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4);
plot(time_eval, -3*rad2deg(mean_sigma(:, 17)), 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4);
xline(T_gnss_cut, 'm-.', 'LineWidth', 1.3);
xlabel('Thời gian (s)', 'FontSize', 10); ylabel('Sai số góc (độ)', 'FontSize', 10);
title('(b) Roll Boresight (\alpha - Trục dọc)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'FontSize', 10, 'GridAlpha', 0.25);

% Pitch Boresight (deg)
subplot(2, 2, 3);
plot(time_eval, rad2deg(armse(:, 18)), 'Color', c_blue, 'LineWidth', 1.8); hold on; grid on;
plot(time_eval, 3*rad2deg(mean_sigma(:, 18)), 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4);
plot(time_eval, -3*rad2deg(mean_sigma(:, 18)), 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4);
xline(T_gnss_cut, 'm-.', 'LineWidth', 1.3);
xlabel('Thời gian (s)', 'FontSize', 10); ylabel('Sai số góc (độ)', 'FontSize', 10);
title('(c) Pitch Boresight (\beta - Trục ngang)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'FontSize', 10, 'GridAlpha', 0.25);

% Yaw Boresight (deg)
subplot(2, 2, 4);
plot(time_eval, rad2deg(armse(:, 19)), 'Color', c_blue, 'LineWidth', 1.8); hold on; grid on;
plot(time_eval, 3*rad2deg(mean_sigma(:, 19)), 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4);
plot(time_eval, -3*rad2deg(mean_sigma(:, 19)), 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.4);
xline(T_gnss_cut, 'm-.', 'LineWidth', 1.3);
xlabel('Thời gian (s)', 'FontSize', 10); ylabel('Sai số góc (độ)', 'FontSize', 10);
title('(d) Yaw Boresight (\gamma - Trục đứng)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'FontSize', 10, 'GridAlpha', 0.25);

exportgraphics(fig3, fullfile(root_dir, 'Hinh3_HieuChuan_DVL_4Bien.png'), 'Resolution', 300);
fprintf('  -> Đã lưu: Hinh3_HieuChuan_DVL_4Bien.png (300 dpi)\n');

%% ========================================================================
% HÌNH 4: SAI SỐ VỊ TRÍ 3D THEO THỜI GIAN VÀ PHA MẤT GNSS
% =========================================================================
fig4 = figure('Name', 'Hinh4_SaiSo_ViTri_3D', 'Color', 'w', 'Position', [160, 120, 920, 480]);

pos_armse_3d = sqrt(sum(armse(:, 1:3).^2, 2));
pos_3sig_3d  = 3 * sqrt(sum(mean_sigma(:, 1:3).^2, 2));

plot(time_eval, pos_armse_3d, 'Color', c_blue, 'LineWidth', 2.0, 'DisplayName', 'ARMSE Vị trí 3D thực tế'); hold on; grid on;
plot(time_eval, pos_3sig_3d, 'Color', c_red, 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Biên bao bọc 3\sigma của bộ lọc');
xline(T_gnss_cut, 'm-.', 'LineWidth', 1.8, 'DisplayName', 'Cắt GNSS (t = 15s)');

% Đánh dấu sai số tại t = 15s và t = 30s
idx_15 = round(15.0 / (time_eval(2)-time_eval(1)));
idx_30 = length(time_eval);
plot(time_eval(idx_15), pos_armse_3d(idx_15), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'HandleVisibility', 'off');
text(time_eval(idx_15)+0.5, pos_armse_3d(idx_15), sprintf('t=15s: %.3f m', pos_armse_3d(idx_15)), 'FontSize', 10, 'FontWeight', 'bold');

plot(time_eval(idx_30), pos_armse_3d(idx_30), 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'HandleVisibility', 'off');
text(time_eval(idx_30)-4.0, pos_armse_3d(idx_30)+0.005, sprintf('t=30s: %.3f m', pos_armse_3d(idx_30)), 'FontSize', 10, 'FontWeight', 'bold');

xlabel('Thời gian mô phỏng (giây)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Sai số vị trí 3D (mét)', 'FontSize', 12, 'FontWeight', 'bold');
title('SAI SỐ VỊ TRÍ 3D (ARMSE) QUA 2 PHA: CÓ GNSS VÀ MẤT GNSS (DEAD-RECKONING)', ...
    'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
set(gca, 'FontSize', 11, 'GridAlpha', 0.25);

exportgraphics(fig4, fullfile(root_dir, 'Hinh4_SaiSo_ViTri_3D.png'), 'Resolution', 300);
fprintf('  -> Đã lưu: Hinh4_SaiSo_ViTri_3D.png (300 dpi)\n');

fprintf('\n[HOÀN TẤT] Toàn bộ 4 đồ thị chất lượng cao 300 dpi đã được xuất thành công!\n');
