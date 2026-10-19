function plot_consistency_results()
%% PLOT_CONSISTENCY_RESULTS
% Tiện ích vẽ lại ngay lập tức 3 cửa sổ biểu đồ Consistency Test từ dữ liệu Monte Carlo
% đã lưu trong TEST/ket_qua_test_consistency_mc1000.mat (không cần chạy lại mô phỏng).

test_dir = fileparts(mfilename('fullpath'));
mat_file = fullfile(test_dir, 'ket_qua_test_consistency_mc1000.mat');

if ~exist(mat_file, 'file')
    error('Chưa tìm thấy file kết quả: %s. Hãy chạy test_esekf19_consistency_mc1000 trước!', mat_file);
end

data = load(mat_file);
fprintf('Đang tải dữ liệu Consistency Test (%d runs) từ: %s...\n', data.M_runs, mat_file);

time_eval = data.time_eval;

% Hình 1: ANEES Toàn hệ 19D
fig1 = figure('Name', 'ANEES Toàn hệ 19D', 'Position', [100, 450, 900, 450]);
plot(time_eval, data.anees_total, 'b-', 'LineWidth', 1.2, 'DisplayName', 'ANEES thực tế');
hold on;
yline(19.0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Kỳ vọng (19.0)');
yline(data.ci_low_total, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Dải dưới 95%% (%.2f)', data.ci_low_total));
yline(data.ci_high_total, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Dải trên 95%% (%.2f)', data.ci_high_total));
xline(15.0, 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS (t = 15s)');
grid on; xlabel('Thời gian (giây)'); ylabel('ANEES Toàn hệ');
title(sprintf('Kiểm định Tính Nhất quán Kalman (ANEES) - Monte Carlo M = %d Runs', data.M_runs), 'FontWeight', 'bold');
legend('Location', 'northeast'); ylim([15, 23]);

% Hình 2: ANEES 5 nhóm trạng thái
fig2 = figure('Name', 'ANEES 5 Nhóm trạng thái', 'Position', [150, 200, 1100, 750]);
subplot(3, 2, 1);
plot(time_eval, data.anees_pos, 'Color', [0 0.447 0.741], 'LineWidth', 1.2);
hold on; yline(3.0, 'k--'); xline(15.0, 'm:'); grid on; title('Vị trí (3D) - Kỳ vọng = 3.0', 'FontWeight', 'bold');
subplot(3, 2, 2);
plot(time_eval, data.anees_vel, 'Color', [0.85 0.325 0.098], 'LineWidth', 1.2);
hold on; yline(3.0, 'k--'); xline(15.0, 'm:'); grid on; title('Vận tốc (3D) - Kỳ vọng = 3.0', 'FontWeight', 'bold');
subplot(3, 2, 3);
plot(time_eval, data.anees_att, 'Color', [0.929 0.694 0.125], 'LineWidth', 1.2);
hold on; yline(3.0, 'k--'); xline(15.0, 'm:'); grid on; title('Tư thế (3D) - Kỳ vọng = 3.0', 'FontWeight', 'bold');
subplot(3, 2, 4);
plot(time_eval, data.anees_bias, 'Color', [0.494 0.184 0.556], 'LineWidth', 1.2);
hold on; yline(6.0, 'k--'); xline(15.0, 'm:'); grid on; title('Bias IMU (6D) - Kỳ vọng = 6.0', 'FontWeight', 'bold');
subplot(3, 2, [5, 6]);
plot(time_eval, data.anees_cal, 'Color', [0.466 0.674 0.188], 'LineWidth', 1.2);
hold on; yline(4.0, 'k--'); xline(15.0, 'm:'); grid on; title('Hiệu chuẩn DVL (4D) - Kỳ vọng = 4.0', 'FontWeight', 'bold');
xlabel('Thời gian (giây)');

% Hình 3: Tự tương quan của DVL Whitened Innovation
fig3 = figure('Name', 'Độ trắng DVL Innovation (Autocorr)', 'Position', [200, 100, 850, 400]);
bar(1:5, data.autocorr_lags');
grid on; xlabel('Độ trễ Lag (bước đo 5 Hz)'); ylabel('Hệ số tự tương quan');
title('Hệ số Tự tương quan của Whitened Innovation DVL qua các Lags (1 -> 5)', 'FontWeight', 'bold');
legend({'Trục X', 'Trục Y', 'Trục Z'}, 'Location', 'northeast'); ylim([-0.1, 0.15]);
yline(0.1, 'r--', 'Ngưỡng cho phép 0.1'); yline(-0.1, 'r--');

fprintf('[THÀNH CÔNG] Đã hiển thị đầy đủ 3 cửa sổ biểu đồ Consistency trên màn hình!\n');
end
