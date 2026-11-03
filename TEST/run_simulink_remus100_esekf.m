function run_simulink_remus100_esekf(arg1, arg2)
%RUN_SIMULINK_REMUS100_ESEKF Chạy kiểm chứng mô hình Simulink tích hợp REMUS 100 + ES-EKF 100 Hz
%   Mô hình: TEST/remus100_esekf_integrated.slx
%   Cú pháp gọi linh hoạt trên MATLAB Command Line:
%       run_simulink_remus100_esekf()              % Chạy với góc lệch mặc định 0.5 độ (2668s)
%       run_simulink_remus100_esekf(0.5)           % Gõ trực tiếp góc lệch mong muốn (0.5 độ) từ Command Line
%       run_simulink_remus100_esekf(0.5, 2668)     % Truyền góc lệch (0.5 độ) và thời gian mô phỏng (2668s)
%       run_simulink_remus100_esekf(2668, 0.5)     % Hoặc truyền thời gian mô phỏng trước, góc lệch sau
%       run_simulink_remus100_esekf([], 0.5)       % Thời gian mặc định, góc lệch 0.5 độ

default_sim_time = 2668; % Thời gian hoàn thành 12 Waypoints
default_yaw_deg  = 0.5;  % Mặc định 0.5 độ cho kịch bản Stress Test

% Tự động nhận diện tham số truyền vào từ Command Line
if nargin == 0
    sim_time = default_sim_time;
    if evalin('base', 'exist(''injected_yaw_submerged_deg'', ''var'')')
        injected_yaw_deg = evalin('base', 'injected_yaw_submerged_deg');
    else
        injected_yaw_deg = default_yaw_deg;
    end
elseif nargin == 1
    if isempty(arg1)
        sim_time = default_sim_time;
        injected_yaw_deg = default_yaw_deg;
    elseif abs(arg1) <= 45
        % Người dùng truyền trực tiếp góc lệch mong muốn, ví dụ: run_simulink_remus100_esekf(0.5)
        injected_yaw_deg = arg1;
        sim_time = default_sim_time;
    else
        % Người dùng truyền thời gian chạy mô phỏng, ví dụ: run_simulink_remus100_esekf(600)
        sim_time = arg1;
        if evalin('base', 'exist(''injected_yaw_submerged_deg'', ''var'')')
            injected_yaw_deg = evalin('base', 'injected_yaw_submerged_deg');
        else
            injected_yaw_deg = default_yaw_deg;
        end
    end
else % nargin >= 2
    if isempty(arg1)
        sim_time = default_sim_time;
        injected_yaw_deg = arg2;
    elseif abs(arg1) <= 45 && ~isempty(arg2) && arg2 > 45
        % Thứ tự: (injected_yaw_deg, sim_time), ví dụ: (0.5, 2668)
        injected_yaw_deg = arg1;
        sim_time = arg2;
    else
        % Thứ tự: (sim_time, injected_yaw_deg), ví dụ: (2668, 0.5)
        sim_time = arg1;
        if isempty(arg2)
            injected_yaw_deg = default_yaw_deg;
        else
            injected_yaw_deg = arg2;
        end
    end
end

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);

addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'Sensor'));
addpath(fullfile(root_dir, 'MISSION_PLANNER'));
addpath(fullfile(root_dir, 'TEST'));
addpath(genpath(fullfile(root_dir, 'AUV_simulation', 'MSS')));

fprintf('=== CHẠY MÔ PHỎNG SIMULINK TÍCH HỢP REMUS 100 + ES-EKF (100 HZ) ===\n');
fprintf('Kịch bản: Bơm sai số góc hướng khi chìm = %+.2f độ\n', injected_yaw_deg);
fprintf('Hello World\n');

% 1. Nạp toàn bộ tham số vào Workspace
init_remus100_esekf_simulink(injected_yaw_deg);

% 2. Mở và chạy mô hình Simulink
mdl = 'remus100_esekf_integrated';
load_system(mdl);
set_param(mdl, 'StopTime', num2str(sim_time));
set_param(mdl, 'ReturnWorkspaceOutputs', 'on');

fprintf('Bắt đầu chạy mô phỏng Simulink "%s" với StopTime = %.1f s...\n', mdl, sim_time);
tic_sim = tic;
sim_out = sim(mdl);
t_elapsed = toc(tic_sim);
fprintf('Hoàn thành mô phỏng Simulink trong %.2f giây thực tế!\n', t_elapsed);

% 3. Thu thập dữ liệu mô phỏng từ Workspace / sim_out
pos_true_hist      = get_sim_var(sim_out, 'pos_true_sim');
pos_est_hist       = get_sim_var(sim_out, 'pos_est_sim');
euler_true_hist    = get_sim_var(sim_out, 'euler_true_sim');
euler_est_hist     = get_sim_var(sim_out, 'euler_est_sim');
boresight_est_hist = get_sim_var(sim_out, 'boresight_sim');
scale_est_hist     = get_sim_var(sim_out, 'scale_sim');
cov_diag_hist      = get_sim_var(sim_out, 'cov_diag_sim');
ui_hist            = get_sim_var(sim_out, 'ui_sim');

wpt = evalin('base', 'wpt');
true_boresight = evalin('base', 'true_boresight');
true_scale     = evalin('base', 'true_scale');

N = size(pos_true_hist, 1);
t = (0:N-1)' * 0.01; % Chu kỳ 100 Hz

% Cắt bỏ phần NaN hoặc chưa hoàn tất
pos_err = pos_true_hist - pos_est_hist;
pos_err_norm = sqrt(sum(pos_err.^2, 2));

% 4. VẼ ĐỒ THỊ KẾT QUẢ ĐỊNH LƯỢNG (4 TAB RIÊNG BIỆT)
% -------------------------------------------------------------------------
% TAB 1: QUỸ ĐẠO 3D (GROUND TRUTH VS ES-EKF VS WAYPOINTS)
% -------------------------------------------------------------------------
fig1 = figure('Name', 'Simulink Tab 1: Quy Dao 3D REMUS 100 + ES-EKF', ...
    'Color', 'w', 'Position', [40, 50, 1400, 850], 'WindowState', 'maximized');
plot3(pos_true_hist(:, 2), pos_true_hist(:, 1), pos_true_hist(:, 3), 'b-', 'LineWidth', 2.2); hold on;
plot3(pos_est_hist(:, 2), pos_est_hist(:, 1), pos_est_hist(:, 3), 'm-.', 'LineWidth', 1.8);
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'k:', 'LineWidth', 1.2);
set(gca, 'ZDir', 'reverse');
grid on; box on;
xlabel('East (m)', 'FontWeight', 'bold');
ylabel('North (m)', 'FontWeight', 'bold');
zlabel('Down - Do sau (m)', 'FontWeight', 'bold');
title('Mô phỏng Simulink: Quỹ đạo 3D REMUS 100 khép vòng với thuật toán ES-EKF 100 Hz', 'FontWeight', 'bold');
legend('Tàu chạy thực tế (Fossen 6-DOF)', 'Ước lượng ES-EKF 100 Hz', 'Waypoints nhiệm vụ', 'Location', 'best');
view(45, 25);
rotate3d(fig1, 'on');
if injected_yaw_deg ~= 0
    fig_pfx = sprintf('simulink_stress%gdeg_', abs(injected_yaw_deg));
else
    fig_pfx = 'simulink_';
end

saveas(fig1, fullfile(script_dir, [fig_pfx 'fig1_quy_dao_3d.png']));

% -------------------------------------------------------------------------
% TAB 2: HỘI TỤ GÓC BORESIGHT DVL
% -------------------------------------------------------------------------
fig2 = figure('Name', 'Simulink Tab 2: Hoi Tu Boresight DVL', 'Color', 'w', 'Position', [70, 70, 1100, 700]);
plot(t, boresight_est_hist(:, 1), 'r-', 'LineWidth', 2.0); hold on;
plot(t, boresight_est_hist(:, 2), 'g-', 'LineWidth', 2.0);
plot(t, boresight_est_hist(:, 3), 'b-', 'LineWidth', 2.0);
yline(rad2deg(true_boresight(1)), 'r--', 'LineWidth', 1.5);
yline(rad2deg(true_boresight(2)), 'g--', 'LineWidth', 1.5);
yline(rad2deg(true_boresight(3)), 'b--', 'LineWidth', 1.5);
grid on; box on;
xlabel('Thời gian (s)', 'FontWeight', 'bold');
ylabel('Góc Boresight (deg)', 'FontWeight', 'bold');
title(sprintf('Simulink: Quá trình hiệu chuẩn Boresight (Lệch góc hướng lặn = %+.1f°)', injected_yaw_deg), 'FontWeight', 'bold');
legend('Roll est', 'Pitch est', 'Yaw est', 'Roll true (1.5 deg)', 'Pitch true (-2.0 deg)', 'Yaw true (3.0 deg)', 'Location', 'best');
saveas(fig2, fullfile(script_dir, [fig_pfx 'fig2_boresight.png']));

% -------------------------------------------------------------------------
% TAB 3: HỘI TỤ HỆ SỐ TỈ LỆ SCALE FACTOR DVL
% -------------------------------------------------------------------------
fig3 = figure('Name', 'Simulink Tab 3: Hoi Tu Scale Factor DVL', 'Color', 'w', 'Position', [100, 90, 1100, 700]);
plot(t, scale_est_hist, 'b-', 'LineWidth', 2.2); hold on;
yline(true_scale * 100, 'r--', 'LineWidth', 1.8);
grid on; box on;
xlabel('Thời gian (s)', 'FontWeight', 'bold');
ylabel('Scale Factor (%)', 'FontWeight', 'bold');
title(sprintf('Simulink: Hội tụ hệ số tỉ lệ tốc độ DVL (Lệch góc hướng lặn = %+.1f°)', injected_yaw_deg), 'FontWeight', 'bold');
legend('Scale Factor est (%)', 'Scale Factor true (+1.5%)', 'Location', 'best');
saveas(fig3, fullfile(script_dir, [fig_pfx 'fig3_scale_factor.png']));

% -------------------------------------------------------------------------
% TAB 4: SAI SỐ VỊ TRÍ ĐỊNH VỊ THEO THỜI GIAN
% -------------------------------------------------------------------------
fig4 = figure('Name', 'Simulink Tab 4: Sai So Vi Tri Theo Thoi Gian', 'Color', 'w', 'Position', [130, 110, 1150, 700]);
plot(t, pos_err(:, 1), 'r-', 'LineWidth', 1.5); hold on;
plot(t, pos_err(:, 2), 'g-', 'LineWidth', 1.5);
plot(t, pos_err(:, 3), 'b-', 'LineWidth', 1.5);
plot(t, pos_err_norm, 'k-', 'LineWidth', 2.2);
grid on; box on;
xlabel('Thời gian (s)', 'FontWeight', 'bold');
ylabel('Sai số vị trí (m)', 'FontWeight', 'bold');
title(sprintf('Simulink: Sai số vị trí giữa Tàu thực tế và ES-EKF (Lệch hướng khi chìm = %+.1f°)', injected_yaw_deg), 'FontWeight', 'bold');
legend('Err North', 'Err East', 'Err Down', 'Tổng sai số 3D', 'Location', 'best');
saveas(fig4, fullfile(script_dir, [fig_pfx 'fig4_sai_so_vi_tri.png']));

% 5. BÁO CÁO TỔNG HỢP CHỈ TIÊU ĐỊNH LƯỢNG (KPI)
final_bs = boresight_est_hist(end, :)';
true_bs_deg = rad2deg(true_boresight);
err_bs = abs(final_bs - true_bs_deg);
final_scale = scale_est_hist(end);
true_scale_pct = true_scale * 100;
err_scale = abs(final_scale - true_scale_pct);

diff_pos = diff(pos_true_hist, 1, 1);
dist_step = sqrt(sum(diff_pos.^2, 2));
total_dist = sum(dist_step);

% Xác định chính xác chặng lặn ngầm (>0.5m) sau chặng chạy nổi hiệu chuẩn (t >= 410s)
idx_dive = find(t >= 410.0 & pos_true_hist(:, 3) > 0.5, 1);
if isempty(idx_dive)
    dist_surface = total_dist;
    dist_dive    = 0;
    pct_dt_dive  = 0;
    t_dive       = t(end);
else
    t_dive       = t(idx_dive);
    dist_surface = sum(dist_step(1:max(1, idx_dive-1)));
    dist_dive    = sum(dist_step(idx_dive:end));
    pct_dt_dive  = (pos_err_norm(end) / max(dist_dive, 1)) * 100;
end
pct_dt_total = (pos_err_norm(end) / max(total_dist, 1)) * 100;

fprintf('\n================== KẾT QUẢ ĐỊNH LƯỢNG MÔ PHỎNG SIMULINK ==================\n');
fprintf('Kịch bản kiểm thử             : Bơm sai số góc hướng khi chìm = %+.2f độ\n', injected_yaw_deg);
fprintf('Boresight Roll                : Ước lượng = %6.3f deg | Thật = %6.3f deg | Sai số = %6.3f deg (%s)\n', ...
    final_bs(1), true_bs_deg(1), err_bs(1), pass_str(err_bs(1) < 0.6));
fprintf('Boresight Pitch               : Ước lượng = %6.3f deg | Thật = %6.3f deg | Sai số = %6.3f deg (%s)\n', ...
    final_bs(2), true_bs_deg(2), err_bs(2), pass_str(err_bs(2) < 0.6));
fprintf('Boresight Yaw                 : Ước lượng = %6.3f deg | Thật = %6.3f deg | Sai số = %6.3f deg (%s)\n', ...
    final_bs(3), true_bs_deg(3), err_bs(3), pass_str(err_bs(3) < 0.6));
fprintf('Scale Factor DVL              : Ước lượng = %6.3f %%   | Thật = %6.3f %%   | Sai số = %6.3f %%   (%s)\n', ...
    final_scale, true_scale_pct, err_scale, pass_str(err_scale < 1.0));
fprintf('-------------------------------------------------------------------------\n');
fprintf('Chặng chạy nổi hiệu chuẩn (WP1-7): %.2f km (%.1f m) trong %.1f s (có GNSS RTK)\n', ...
    dist_surface / 1000, dist_surface, t_dive);
fprintf('Chặng lặn ngầm mất GNSS (z>0.5m) : %.2f km (%.1f m) từ t = %.1f s đến %.1f s\n', ...
    dist_dive / 1000, dist_dive, t_dive, t(end));
fprintf('Tổng cự ly hành trình Remus 100 : %.2f km (%.1f m) trong %.1f s (~%.1f phút)\n', ...
    total_dist / 1000, total_dist, t(end), t(end)/60);
fprintf('Sai số vị trí 3D tại điểm đích: %.2f m (Đích đến WP12)\n', pos_err_norm(end));
fprintf('Tỉ lệ dạt / Cự ly lặn ngầm    : %.3f %% DT (Chuẩn hàng hải < 1.0 %% DT: %s)\n', ...
    pct_dt_dive, pass_str(pct_dt_dive < 1.0));
fprintf('Độ trôi ngang lý thuyết       : %.2f m (theo công thức D_ngầm x sin(%.1f°))\n', ...
    dist_dive * abs(sin(deg2rad(injected_yaw_deg))), injected_yaw_deg);
fprintf('=========================================================================\n');

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end

function s = pass_str(cond)
if cond
    s = 'ĐẠT';
else
    s = 'CHƯA ĐẠT';
end
end

function val = get_sim_var(sim_out, var_name)
if isa(sim_out, 'Simulink.SimulationOutput')
    try
        val = sim_out.get(var_name);
        if ~isempty(val), return; end
    catch
    end
end
if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    val = evalin('base', var_name);
else
    error('Không tìm thấy biến mô phỏng: %s', var_name);
end
end
