function plot_calibration_results()
%% PLOT_CALIBRATION_RESULTS
% Tiện ích vẽ lại ngay lập tức 3 cửa sổ biểu đồ hội tụ từ dữ liệu Monte Carlo
% đã lưu trong TEST/ket_qua_test_calibration_convergence.mat (không cần chạy lại mô phỏng).

test_dir = fileparts(mfilename('fullpath'));
mat_file = fullfile(test_dir, 'ket_qua_test_calibration_convergence.mat');

if ~exist(mat_file, 'file')
    error('Chưa tìm thấy file kết quả: %s. Hãy chạy test_esekf19_calibration_convergence trước!', mat_file);
end

data = load(mat_file);
fprintf('Đang tải dữ liệu %d lần chạy Monte Carlo từ: %s...\n', data.M_runs, mat_file);

dt = 0.0025;
N_steps = size(data.mean_scale_history, 1);
time_eval = (1:N_steps)' * dt;
true_scale = 0.015;
true_boresight = deg2rad([1.5; -2.0; 3.0]);

%% ========================================================================
% HÌNH 1: ĐƯỜNG CONG HỘI TỤ SCALE FACTOR
% =========================================================================
fig1 = figure('Name', 'Hội tụ Scale Factor DVL', 'NumberTitle', 'off', ...
    'Position', [100, 450, 800, 420]);

% Vẽ dải độ lệch chuẩn (+- 1 RMSE)
upper_scale = (data.mean_scale_history + data.rmse_scale_history) * 100;
lower_scale = (data.mean_scale_history - data.rmse_scale_history) * 100;
fill([time_eval; flipud(time_eval)], [upper_scale; flipud(lower_scale)], ...
    [0.8 0.85 1.0], 'EdgeColor', 'none', 'DisplayName', 'Dải phân tán \pm 1 RMSE');
hold on;

plot(time_eval, data.mean_scale_history * 100, 'b-', 'LineWidth', 2.0, ...
    'DisplayName', 'Ước lượng Scale trung bình (%)');
yline(true_scale * 100, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Scale thật = %.2f%%', true_scale*100));
xline(15.0, 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS (t = 15s)');
grid on; xlabel('Thời gian (giây)'); ylabel('DVL Scale Factor (%)');
title(sprintf('Đường cong tự hiệu chuẩn Scale Factor (Cold Start, M = %d Runs)', data.M_runs), ...
    'FontWeight', 'bold');
legend('Location', 'southeast');
ylim([0, 2.0]);

%% ========================================================================
% HÌNH 2: ĐƯỜNG CONG HỘI TỤ 3 GÓC BORESIGHT
% =========================================================================
fig2 = figure('Name', 'Hội tụ 3 góc Boresight DVL', 'NumberTitle', 'off', ...
    'Position', [150, 150, 1000, 650]);

% Roll
subplot(3, 1, 1);
r_up = data.mean_bs_history(:, 1) + rad2deg(data.rmse_bs_history(:, 1));
r_lo = data.mean_bs_history(:, 1) - rad2deg(data.rmse_bs_history(:, 1));
fill([time_eval; flipud(time_eval)], [r_up; flipud(r_lo)], ...
    [1.0 0.85 0.85], 'EdgeColor', 'none', 'DisplayName', '\pm 1 RMSE');
hold on;
plot(time_eval, data.mean_bs_history(:, 1), 'r-', 'LineWidth', 1.8, 'DisplayName', 'Trung bình ước lượng');
yline(rad2deg(true_boresight(1)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Giá trị thật (+1.5^\circ)');
xline(15.0, 'm:', 'LineWidth', 1.2, 'DisplayName', 'Cắt GNSS');
grid on; ylabel('Độ (deg)');
title('Roll Boresight \alpha (Thật = +1.5^\circ, Khởi tạo từ 0^\circ)', 'FontWeight', 'bold');
legend('Location', 'southeast');

% Pitch
subplot(3, 1, 2);
p_up = data.mean_bs_history(:, 2) + rad2deg(data.rmse_bs_history(:, 2));
p_lo = data.mean_bs_history(:, 2) - rad2deg(data.rmse_bs_history(:, 2));
fill([time_eval; flipud(time_eval)], [p_up; flipud(p_lo)], ...
    [0.85 1.0 0.85], 'EdgeColor', 'none', 'DisplayName', '\pm 1 RMSE');
hold on;
plot(time_eval, data.mean_bs_history(:, 2), 'g-', 'LineWidth', 1.8, 'DisplayName', 'Trung bình ước lượng');
yline(rad2deg(true_boresight(2)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Giá trị thật (-2.0^\circ)');
xline(15.0, 'm:', 'LineWidth', 1.2, 'DisplayName', 'Cắt GNSS');
grid on; ylabel('Độ (deg)');
title('Pitch Boresight \beta (Thật = -2.0^\circ, Khởi tạo từ 0^\circ)', 'FontWeight', 'bold');
legend('Location', 'southeast');

% Yaw
subplot(3, 1, 3);
y_up = data.mean_bs_history(:, 3) + rad2deg(data.rmse_bs_history(:, 3));
y_lo = data.mean_bs_history(:, 3) - rad2deg(data.rmse_bs_history(:, 3));
fill([time_eval; flipud(time_eval)], [y_up; flipud(y_lo)], ...
    [0.85 0.85 1.0], 'EdgeColor', 'none', 'DisplayName', '\pm 1 RMSE');
hold on;
plot(time_eval, data.mean_bs_history(:, 3), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Trung bình ước lượng');
yline(rad2deg(true_boresight(3)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Giá trị thật (+3.0^\circ)');
xline(15.0, 'm:', 'LineWidth', 1.2, 'DisplayName', 'Cắt GNSS');
grid on; xlabel('Thời gian (giây)'); ylabel('Độ (deg)');
title('Yaw Boresight \gamma (Thật = +3.0^\circ, Khởi tạo từ 0^\circ)', 'FontWeight', 'bold');
legend('Location', 'southeast');

%% ========================================================================
% HÌNH 3: SO SÁNH SAI SỐ TẠI MỐC CUỐI CÙNG t = 30s
% =========================================================================
fig3 = figure('Name', 'Phân phối sai số tại t = 30s', 'NumberTitle', 'off', ...
    'Position', [200, 100, 850, 480]);

idx_30s = 5; % Mốc t = 30s
r_err = abs(data.err_roll_chk(:, idx_30s));
p_err = abs(data.err_pitch_chk(:, idx_30s));
y_err = abs(data.err_yaw_chk(:, idx_30s));

med_vals  = [median(r_err); median(p_err); median(y_err)];
rmse_vals = [sqrt(mean(r_err.^2)); sqrt(mean(p_err.^2)); sqrt(mean(y_err.^2))];
p95_vals  = [prctile(r_err, 95); prctile(p_err, 95); prctile(y_err, 95)];

bar_data = [med_vals, rmse_vals, p95_vals];
h_bar = bar(bar_data);
h_bar(1).FaceColor = [0.2 0.6 0.8];
h_bar(2).FaceColor = [0.85 0.35 0.2];
h_bar(3).FaceColor = [0.9 0.7 0.1];

set(gca, 'XTickLabel', {'Roll (\alpha)', 'Pitch (\beta)', 'Yaw (\gamma)'});
legend({'Trung vị (Median)', 'RMSE', 'Phân vị P95'}, 'Location', 'northeast');
grid on; ylabel('Sai số tuyệt đối (độ)');
title(sprintf('So sánh mức độ hội tụ Boresight tại t = 30s (M = %d Runs)', data.M_runs), ...
    'FontWeight', 'bold');

% Gắn nhãn số liệu lên các cột
for b = 1:3
    x_pos = h_bar(b).XEndPoints;
    y_pos = h_bar(b).YEndPoints;
    labels = string(round(y_pos, 3));
    text(x_pos, y_pos + 0.03, labels, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 9, 'FontWeight', 'bold');
end
ylim([0, max(p95_vals) * 1.25]);

fprintf('[THÀNH CÔNG] Đã hiển thị đầy đủ 3 cửa sổ biểu đồ trên màn hình!\n');
end
