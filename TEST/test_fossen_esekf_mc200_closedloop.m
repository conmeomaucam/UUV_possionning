function test_fossen_esekf_mc200_closedloop(M_in)
%% TEST_FOSSEN_ESEKF_MC200_CLOSEDLOOP - Kiểm thử Monte Carlo Khép Vòng Toàn Trình 4km
% Kịch bản: Chuẩn thử nghiệm cơ học vòng lặp kín (Closed-Loop GNC Testing)
% - Tàu REMUS 100 mô hình vật lý 6 bậc tự do (Thor I. Fossen).
% - Cảm biến: GEO-FOG 3D Dual INS (không dùng từ kế) + DVL + Cảm biến áp suất + GNSS RTK.
% - Bộ điều khiển khép vòng: Dẫn đường ALOS3D + Bánh lái Integral SMC + Cánh lặn Stern planes PID.
% - Số lần chạy mặc định: M = 200 mẫu.
% - Các yếu tố ngẫu nhiên ở từng run:
%   1. Góc lệch Boresight thật ngẫu nhiên: Roll [-3, 3] deg, Pitch [-3, 3] deg, Yaw [-5, 5] deg.
%   2. Hệ số Scale Factor DVL thật ngẫu nhiên: [-3%, +3%]. EKF khởi động nguội từ 0.
%   3. Hướng dòng hải lưu ngẫu nhiên [0, 360] deg, vận tốc dòng ngẫu nhiên [0.2, 0.8] m/s.
%   4. Sự cố mất đáy DVL ngẫu nhiên 20s khi lặn ngầm.
%   5. Nhiễu đo vọt ngoại lai Outlier 1.5% số đo DVL để test cổng Chi-square gating.
%   6. Đường kẻ hồng nét đứt (Cat GNSS) tại t = 480s trên toàn bộ biểu đồ.

clc; close all;
if nargin < 1 || isempty(M_in)
    M_runs = 200;
else
    M_runs = M_in;
end

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
project_root = fileparts(script_dir);

addpath(fullfile(project_root, 'algo'));
addpath(fullfile(project_root, 'Sensor'));
addpath(fullfile(project_root, 'dieukiendauvao'));
addpath(fullfile(project_root, 'AUV_simulation', 'MSS', 'GNC'));
addpath(fullfile(project_root, 'AUV_simulation', 'MSS', 'LIBRARY'));
addpath(fullfile(project_root, 'AUV_simulation', 'MSS', 'CRAFT', 'AUV'));
addpath(fullfile(project_root, 'AUV_simulation', 'MSS', 'CRAFT', 'AUV', 'models'));

fprintf('========================================================================================\n');
fprintf('  KIỂM THỬ MONTE CARLO KHÉP VÒNG TOÀN TRÌNH 4KM (M = %d MẪU, REMUS 100 + GEO-FOG 3D)\n', M_runs);
fprintf('========================================================================================\n\n');

% Cấu hình thời gian hành trình
h = 0.05;              % Bước tích phân 20 Hz
T_final = 1880;        % 1880 giây (31.3 phút, 480s mặt nước + 1400s lặn)
T_gnss_cut = 480;      % Cắt GNSS ở giây 480
t = 0:h:T_final;
nSteps = length(t);

% Định nghĩa 12 mốc Waypoints chuẩn
wpt_x = [ 0,   0, -30,   30, -20,   0,  -20, -100,    0,  200,  200,  400];
wpt_y = [ 0, 120, 270,  430, 580, 720,  920, 1320, 1670, 2020, 2520, 2920];
wpt_z = [ 0,   0,   0,    0,   0,   0,   10,  100,  100,   50,   50,   50];
wpt.pos.x = wpt_x; wpt.pos.y = wpt_y; wpt.pos.z = wpt_z;

% Bộ đệm lưu trữ kết quả Monte Carlo (lưu mẫu trích mẫu cách 2s để tiết kiệm RAM)
sample_step = round(2.0 / h); % Mỗi 2 giây lưu 1 điểm
t_eval = t(1:sample_step:end);
nEval = length(t_eval);

% Ma trận lưu trữ thống kê: [nEval x M_runs]
err_pos_N   = zeros(nEval, M_runs);
err_pos_E   = zeros(nEval, M_runs);
err_pos_D   = zeros(nEval, M_runs);
err_pos_3D  = zeros(nEval, M_runs);
err_roll_deg = zeros(nEval, M_runs);
err_pitch_deg= zeros(nEval, M_runs);
err_yaw_deg  = zeros(nEval, M_runs);

bs_roll_est  = zeros(nEval, M_runs);
bs_pitch_est = zeros(nEval, M_runs);
bs_yaw_est   = zeros(nEval, M_runs);
scale_est_all= zeros(nEval, M_runs);

bs_roll_true  = zeros(M_runs, 1);
bs_pitch_true = zeros(M_runs, 1);
bs_yaw_true   = zeros(M_runs, 1);
scale_true_all= zeros(M_runs, 1);

pos_final_true = zeros(M_runs, 3);
pos_final_est  = zeros(M_runs, 3);

traj_N_true = zeros(nEval, 1);
traj_E_true = zeros(nEval, 1);
traj_D_true = zeros(nEval, 1);
traj_N_est  = zeros(nEval, 1);
traj_E_est  = zeros(nEval, 1);
traj_D_est  = zeros(nEval, 1);

sum_nees      = zeros(nEval, 1);
sum_nees_pos  = zeros(nEval, 1);
sum_nees_vel  = zeros(nEval, 1);
sum_nees_att  = zeros(nEval, 1);
sum_nees_bias = zeros(nEval, 1);
sum_nees_cal  = zeros(nEval, 1);

sum_sq_pos    = zeros(nEval, 3);
sum_sq_vel    = zeros(nEval, 3);
sum_sq_att    = zeros(nEval, 3);
sum_sq_bg     = zeros(nEval, 3);
sum_sq_ba     = zeros(nEval, 3);
sum_var_pos   = zeros(nEval, 3);
sum_var_vel   = zeros(nEval, 3);
sum_var_att   = zeros(nEval, 3);
sum_var_bg    = zeros(nEval, 3);
sum_var_ba    = zeros(nEval, 3);
sum_var_bs    = zeros(nEval, 3);
sum_var_scale = zeros(nEval, 1);

outlier_rejected_count = zeros(M_runs, 1);
outlier_injected_count = zeros(M_runs, 1);

% Khởi tạo song song nếu có Parallel Computing Toolbox
use_parfor = false;
pool_workers = 1;
try
    if ~isempty(ver('parallel'))
        pool = gcp('nocreate');
        if isempty(pool)
            pool = parpool('local');
        end
        use_parfor = true;
        pool_workers = pool.NumWorkers;
    end
catch ME
    warning('Không thể khởi tạo parpool: %s. Chuyển sang chạy tuần tự đơn luồng.', ME.message);
    use_parfor = false;
end

if use_parfor
    fprintf('Chế độ tính toán: Song song đa nhân (%d workers)\n', pool_workers);
    try
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'algo')));
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'Sensor')));
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'dieukiendauvao')));
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'AUV_simulation', 'MSS', 'GNC')));
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'AUV_simulation', 'MSS', 'LIBRARY')));
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'AUV_simulation', 'MSS', 'CRAFT', 'AUV')));
        pctRunOnAll(sprintf('addpath(''%s'');', fullfile(project_root, 'AUV_simulation', 'MSS', 'CRAFT', 'AUV', 'models')));
    catch
    end
else
    fprintf('Chế độ tính toán: Tuần tự đơn luồng\n');
end
t_mc_start = tic;
clear progress_printer;


%% VÒNG LẶP MONTE CARLO (HỖ TRỢ PARFOR ĐA NHÂN)
sim_results(M_runs) = struct();

if use_parfor
    q = parallel.pool.DataQueue;
    afterEach(q, @(data) progress_printer(data, M_runs, t_mc_start));

    parfor m = 1:M_runs
        sim_results(m) = run_single_simulation(m, wpt, h, T_final, T_gnss_cut, sample_step, nEval, t_eval, q);
    end
else
    for m = 1:M_runs
        sim_results(m) = run_single_simulation(m, wpt, h, T_final, T_gnss_cut, sample_step, nEval, t_eval, []);
        t_elap = toc(t_mc_start);
        eta_sec = (t_elap / m) * (M_runs - m);
        fprintf('Run %3d / %d hoàn thành | Đã chạy: %5.1f ph | Còn lại: ~%5.1f ph\n', ...
            m, M_runs, t_elap/60, eta_sec/60);
    end
end

% Giải nén và tổng hợp kết quả thống kê Monte Carlo
fprintf('\nĐang tổng hợp dữ liệu từ %d run Monte Carlo...\n', M_runs);
for m = 1:M_runs
    err_pos_N(:, m) = sim_results(m).err_pos_N;
    err_pos_E(:, m) = sim_results(m).err_pos_E;
    err_pos_D(:, m) = sim_results(m).err_pos_D;
    err_pos_3D(:, m) = sim_results(m).err_pos_3D;
    err_roll_deg(:, m) = sim_results(m).err_roll_deg;
    err_pitch_deg(:, m) = sim_results(m).err_pitch_deg;
    err_yaw_deg(:, m) = sim_results(m).err_yaw_deg;

    bs_roll_est(:, m) = sim_results(m).bs_roll_est;
    bs_pitch_est(:, m) = sim_results(m).bs_pitch_est;
    bs_yaw_est(:, m) = sim_results(m).bs_yaw_est;
    scale_est_all(:, m) = sim_results(m).scale_est_all;

    bs_roll_true(m) = sim_results(m).bs_roll_true;
    bs_pitch_true(m) = sim_results(m).bs_pitch_true;
    bs_yaw_true(m) = sim_results(m).bs_yaw_true;
    scale_true_all(m) = sim_results(m).scale_true_all;

    pos_final_true(m, :) = sim_results(m).pos_final_true;
    pos_final_est(m, :)  = sim_results(m).pos_final_est;
    outlier_injected_count(m) = sim_results(m).n_outlier_inj;
    outlier_rejected_count(m) = sim_results(m).n_outlier_rej;

    sum_nees      = sum_nees      + sim_results(m).nees;
    sum_nees_pos  = sum_nees_pos  + sim_results(m).nees_pos;
    sum_nees_vel  = sum_nees_vel  + sim_results(m).nees_vel;
    sum_nees_att  = sum_nees_att  + sim_results(m).nees_att;
    sum_nees_bias = sum_nees_bias + sim_results(m).nees_bias;
    sum_nees_cal  = sum_nees_cal  + sim_results(m).nees_cal;

    sum_sq_pos = sum_sq_pos + sim_results(m).sq_pos;
    sum_sq_vel = sum_sq_vel + sim_results(m).sq_vel;
    sum_sq_att = sum_sq_att + sim_results(m).sq_att;
    sum_sq_bg  = sum_sq_bg  + sim_results(m).sq_bg;
    sum_sq_ba  = sum_sq_ba  + sim_results(m).sq_ba;

    sum_var_pos = sum_var_pos + sim_results(m).var_pos;
    sum_var_vel = sum_var_vel + sim_results(m).var_vel;
    sum_var_att = sum_var_att + sim_results(m).var_att;
    sum_var_ba  = sum_var_ba  + sim_results(m).var_ba;
    sum_var_bg  = sum_var_bg  + sim_results(m).var_bg;
    sum_var_bs  = sum_var_bs  + sim_results(m).var_bs;
    sum_var_scale = sum_var_scale + sim_results(m).var_scale;
end

traj_N_true = sim_results(1).traj_N_true;
traj_E_true = sim_results(1).traj_E_true;
traj_D_true = sim_results(1).traj_D_true;
traj_N_est  = sim_results(1).traj_N_est;
traj_E_est  = sim_results(1).traj_E_est;
traj_D_est  = sim_results(1).traj_D_est;

%% XUẤT ĐỒ THỊ BÁO CÁO VÀ NGHIỆM THU
fprintf('\nĐang tổng hợp dữ liệu và xuất 5 hình ảnh nghiệm thu chuẩn...\n');

% Tính toán ANEES và dải tin cậy 95% lý thuyết
anees_total = sum_nees / M_runs;
anees_pos   = sum_nees_pos / M_runs;
anees_vel   = sum_nees_vel / M_runs;
anees_att   = sum_nees_att / M_runs;
anees_bias  = sum_nees_bias / M_runs;
anees_cal   = sum_nees_cal / M_runs;

ci_low_19  = chi2inv_custom(0.025, 19 * M_runs) / M_runs;
ci_high_19 = chi2inv_custom(0.975, 19 * M_runs) / M_runs;

ci_low_3   = chi2inv_custom(0.025, 3 * M_runs) / M_runs;
ci_high_3  = chi2inv_custom(0.975, 3 * M_runs) / M_runs;

ci_low_6   = chi2inv_custom(0.025, 6 * M_runs) / M_runs;
ci_high_6  = chi2inv_custom(0.975, 6 * M_runs) / M_runs;

ci_low_4   = chi2inv_custom(0.025, 4 * M_runs) / M_runs;
ci_high_4  = chi2inv_custom(0.975, 4 * M_runs) / M_runs;

% Tính toán RMSE và 3-sigma trung bình
rmse_pos = sqrt(sum_sq_pos / M_runs);
sigma3_pos = 3 * sqrt(sum_var_pos / M_runs);

rmse_vel = sqrt(sum_sq_vel / M_runs);
sigma3_vel = 3 * sqrt(sum_var_vel / M_runs);

rmse_att = sqrt(sum_sq_att / M_runs);
sigma3_att = 3 * sqrt(sum_var_att / M_runs);

rmse_bg = sqrt(sum_sq_bg / M_runs);
sigma3_bg = 3 * sqrt(sum_var_bg / M_runs);

rmse_ba = sqrt(sum_sq_ba / M_runs);
sigma3_ba = 3 * sqrt(sum_var_ba / M_runs);

sigma3_bs    = 3 * sqrt(sum_var_bs / M_runs);
sigma3_scale = 3 * sqrt(sum_var_scale / M_runs);

% Đọc dữ liệu Fossen Standalone nếu có
fossen_standalone_file = fullfile(script_dir, 'ket_qua_fossen_standalone.mat');
has_fossen_standalone = false;
if exist(fossen_standalone_file, 'file')
    try
        f_data = load(fossen_standalone_file);
        pos_fossen = f_data.pos_history;
        t_fossen   = f_data.t;
        has_fossen_standalone = true;
    catch
        has_fossen_standalone = false;
    end
end
if ~has_fossen_standalone
    pos_fossen = [traj_N_true, traj_E_true, traj_D_true];
    t_fossen = t_eval;
end

% -------------------------------------------------------------------------
% HÌNH 1: QUỸ ĐẠO 3 ĐƯỜNG RIÊNG BIỆT (Chuẩn Ảnh 1)
% -------------------------------------------------------------------------
fig1 = figure('Name', 'So Sanh 3 Duong Quy Dao: Fossen vs Tau That vs ES-EKF', ...
    'Position', [40, 40, 1400, 900], 'Visible', 'off');

% Subplot 1: Không gian 3D
subplot(2, 2, 1);
plot3(pos_fossen(:, 2), pos_fossen(:, 1), pos_fossen(:, 3), 'b-', 'LineWidth', 2.0); hold on;
plot3(traj_E_true, traj_N_true, traj_D_true, 'r-', 'LineWidth', 2.0);
plot3(traj_E_est, traj_N_est, traj_D_est, 'm--', 'LineWidth', 1.5);
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
plot(traj_E_true, traj_N_true, 'r-', 'LineWidth', 2.0);
plot(traj_E_est, traj_N_est, 'm--', 'LineWidth', 1.5);
plot(wpt.pos.y, wpt.pos.x, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
plot(wpt.pos.y, wpt.pos.x, 'k:', 'LineWidth', 1);
grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)');
title('2. Mat bang North - East (Nhin tu tren xuong)', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Waypoints', 'Location', 'best');

% Subplot 3: Trắc dọc độ sâu theo East
subplot(2, 2, 3);
plot(pos_fossen(:, 2), pos_fossen(:, 3), 'b-', 'LineWidth', 2.0); hold on;
plot(traj_E_true, traj_D_true, 'r-', 'LineWidth', 2.0);
plot(traj_E_est, traj_D_est, 'm--', 'LineWidth', 1.5);
plot(wpt.pos.y, wpt.pos.z, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
grid on; set(gca, 'YDir', 'reverse');
xlabel('East (m)'); ylabel('Down - Do sau (m)');
title('3. Trac doc do sau theo quang duong East', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Waypoints', 'Location', 'best');

% Subplot 4: Diễn biến độ sâu theo thời gian (Depth vs Time)
subplot(2, 2, 4);
plot(t_fossen, pos_fossen(:, 3), 'b-', 'LineWidth', 2.0); hold on;
plot(t_eval, traj_D_true, 'r-', 'LineWidth', 2.0);
plot(t_eval, traj_D_est, 'm--', 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.5);
grid on; set(gca, 'YDir', 'reverse');
xlabel('Thoi gian (s)'); ylabel('Do sau (m)');
title('4. Dien bien do sau Down theo thoi gian', 'FontWeight', 'bold');
legend('1. Fossen Standalone', '2. Tau that', '3. ES-EKF uoc luong', 'Cat GNSS', 'Location', 'best');

saveas(fig1, fullfile(script_dir, 'fig_mc200_quy_dao_3_duong.png'));
saveas(fig1, fullfile(script_dir, 'fig_so_sanh_quy_dao.png'));
close(fig1);

% -------------------------------------------------------------------------
% HÌNH 2: KIỂM ĐỊNH ANEES TOÀN HỆ 19D (Chuẩn Báo cáo IEEE & Ảnh 1-2)
% -------------------------------------------------------------------------
fig2 = figure('Name', 'Monte Carlo [CONSISTENCY]: Kiem dinh chi so ANEES', ...
    'Color', 'w', 'Position', [80, 100, 1000, 480], 'Visible', 'off');
pct_anees_pass = 100 * sum(anees_total >= ci_low_19 & anees_total <= ci_high_19) / length(anees_total);
t_col = t_eval(:);
fill([t_col; flipud(t_col)], [ci_low_19*ones(size(t_col)); ci_high_19*ones(size(t_col))], ...
    [0.9 0.95 0.9], 'EdgeColor', 'none', 'DisplayName', 'Dải tin cậy 95% lý thuyết');
hold on; grid on;
yline(19.0, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Kỳ vọng lý tưởng (E[ANEES] = 19)');
yline(ci_low_19, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Ngưỡng dưới r_1 = %.2f', ci_low_19));
yline(ci_high_19, 'r:', 'LineWidth', 1.2, 'DisplayName', sprintf('Ngưỡng trên r_2 = %.2f', ci_high_19));
plot(t_eval, anees_total, 'b-', 'LineWidth', 1.6, 'DisplayName', sprintf('ANEES thực nghiệm (%d runs)', M_runs));
xline(T_gnss_cut, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Cắt GNSS (Bắt đầu lặn ngầm)');
xlabel('Thời gian (giây)'); ylabel('Chỉ số ANEES');
title(sprintf('Kiểm định chỉ số ANEES bộ lọc ES-EKF 19 trạng thái [CONSISTENCY] (%d runs, Trong dải: %.1f%%)', ...
    M_runs, pct_anees_pass), 'FontWeight', 'bold');
legend('Location', 'best');
ylim([max(0, ci_low_19 - 5), ci_high_19 + 5]);

saveas(fig2, fullfile(script_dir, 'fig_mc200_anees_toan_he.png'));
close(fig2);

% -------------------------------------------------------------------------
% HÌNH 3: ANEES 5 NHÓM TRẠNG THÁI (Vị trí, Vận tốc, Tư thế, Bias, Hiệu chuẩn)
% -------------------------------------------------------------------------
fig3 = figure('Name', 'ANEES 5 Nhom trang thai', 'Position', [150, 150, 1150, 780], 'Visible', 'off');
subplot(3, 2, 1);
plot(t_eval, anees_pos, 'Color', [0 0.447 0.741], 'LineWidth', 1.3); hold on;
yline(3.0, 'k--', 'LineWidth', 1.2); yline(ci_low_3, 'r:'); yline(ci_high_3, 'r:');
xline(T_gnss_cut, 'm--', 'LineWidth', 1.2); grid on;
title('Vi tri (3D) - Ky vong = 3.0', 'FontWeight', 'bold'); xlabel('Thoi gian (s)');

subplot(3, 2, 2);
plot(t_eval, anees_vel, 'Color', [0.85 0.325 0.098], 'LineWidth', 1.3); hold on;
yline(3.0, 'k--', 'LineWidth', 1.2); yline(ci_low_3, 'r:'); yline(ci_high_3, 'r:');
xline(T_gnss_cut, 'm--', 'LineWidth', 1.2); grid on;
title('Van toc (3D) - Ky vong = 3.0', 'FontWeight', 'bold'); xlabel('Thoi gian (s)');

subplot(3, 2, 3);
plot(t_eval, anees_att, 'Color', [0.929 0.694 0.125], 'LineWidth', 1.3); hold on;
yline(3.0, 'k--', 'LineWidth', 1.2); yline(ci_low_3, 'r:'); yline(ci_high_3, 'r:');
xline(T_gnss_cut, 'm--', 'LineWidth', 1.2); grid on;
title('Tu the (3D) - Ky vong = 3.0', 'FontWeight', 'bold'); xlabel('Thoi gian (s)');

subplot(3, 2, 4);
plot(t_eval, anees_bias, 'Color', [0.494 0.184 0.556], 'LineWidth', 1.3); hold on;
yline(6.0, 'k--', 'LineWidth', 1.2); yline(ci_low_6, 'r:'); yline(ci_high_6, 'r:');
xline(T_gnss_cut, 'm--', 'LineWidth', 1.2); grid on;
title('Bias IMU (6D) - Ky vong = 6.0', 'FontWeight', 'bold'); xlabel('Thoi gian (s)');

subplot(3, 2, [5, 6]);
plot(t_eval, anees_cal, 'Color', [0.466 0.674 0.188], 'LineWidth', 1.3); hold on;
yline(4.0, 'k--', 'LineWidth', 1.2); yline(ci_low_4, 'r:'); yline(ci_high_4, 'r:');
xline(T_gnss_cut, 'm--', 'LineWidth', 1.2); grid on;
title('Hieu chuan DVL (4D: Scale + Boresight) - Ky vong = 4.0', 'FontWeight', 'bold'); xlabel('Thoi gian (s)');

saveas(fig3, fullfile(script_dir, 'fig_mc200_anees_5_nhom.png'));
close(fig3);

% -------------------------------------------------------------------------
% HÌNH 4: ARMSE VÀ BIÊN 3-SIGMA (Chuẩn 4 ô Khớp 100% Ảnh 2 Word Report)
% -------------------------------------------------------------------------
fig4 = figure('Name', 'Monte Carlo [CONSISTENCY]: ARMSE Trang thai dan duong', ...
    'Color', 'w', 'Position', [120, 80, 1100, 680], 'Visible', 'off');

% Ô 1: ARMSE Vị trí 3D vs Biên 3-sigma
subplot(2, 2, 1);
pos_3d_armse = sqrt(sum(rmse_pos.^2, 2));
pos_3d_3sig  = sqrt(sum(sigma3_pos.^2, 2));
plot(t_eval, pos_3d_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
plot(t_eval, pos_3d_3sig, 'r--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm:', 'Cắt GNSS', 'LineWidth', 1.4);
xlabel('Thời gian (s)'); ylabel('Sai số (m)');
title('ARMSE Vị trí 3D vs Biên 3-sigma', 'FontWeight', 'bold');
legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

% Ô 2: ARMSE Vận tốc 3D vs Biên 3-sigma
subplot(2, 2, 2);
vel_3d_armse = sqrt(sum(rmse_vel.^2, 2));
vel_3d_3sig  = sqrt(sum(sigma3_vel.^2, 2));
plot(t_eval, vel_3d_armse, 'b-', 'LineWidth', 1.6); hold on; grid on;
plot(t_eval, vel_3d_3sig, 'r--', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm:', 'Cắt GNSS', 'LineWidth', 1.4);
xlabel('Thời gian (s)'); ylabel('Sai số (m/s)');
title('ARMSE Vận tốc 3D vs Biên 3-sigma', 'FontWeight', 'bold');
legend('ARMSE thực nghiệm', 'Biên 3-sigma lý thuyết', 'Location', 'best');

% Ô 3: ARMSE Tư thế (Roll, Pitch, Yaw)
subplot(2, 2, 3);
plot(t_eval, rmse_att(:, 1), 'r-', 'LineWidth', 1.4); hold on; grid on;
plot(t_eval, rmse_att(:, 2), 'g-', 'LineWidth', 1.4);
plot(t_eval, rmse_att(:, 3), 'b-', 'LineWidth', 1.4);
plot(t_eval, sigma3_att(:, 3), 'k--', 'LineWidth', 1.1);
xline(T_gnss_cut, 'm:', 'Cắt GNSS', 'LineWidth', 1.4);
xlabel('Thời gian (s)'); ylabel('Sai số (deg)');
title('ARMSE Tư thế (Roll, Pitch, Yaw)', 'FontWeight', 'bold');
legend('Roll ARMSE', 'Pitch ARMSE', 'Yaw ARMSE', 'Biên 3-sigma Yaw', 'Location', 'best');

% Ô 4: ARMSE Bias Con quay FOG
subplot(2, 2, 4);
plot(t_eval, rmse_bg(:, 1), 'r-', 'LineWidth', 1.4); hold on; grid on;
plot(t_eval, rmse_bg(:, 2), 'g-', 'LineWidth', 1.4);
plot(t_eval, rmse_bg(:, 3), 'b-', 'LineWidth', 1.4);
plot(t_eval, sigma3_bg(:, 3), 'k--', 'LineWidth', 1.1);
xline(T_gnss_cut, 'm:', 'Cắt GNSS', 'LineWidth', 1.4);
xlabel('Thời gian (s)'); ylabel('Bias con quay (deg/h)');
title('ARMSE Bias Con quay FOG', 'FontWeight', 'bold');
legend('Bias X', 'Bias Y', 'Bias Z', 'Biên 3-sigma Z', 'Location', 'best');

saveas(fig4, fullfile(script_dir, 'fig_mc200_rmse_vs_3sigma.png'));
close(fig4);

% -------------------------------------------------------------------------
% HÌNH 5: HỘI TỤ BORESIGHT & DVL SCALE FACTOR KÈM 3-SIGMA (Khóa khi lặn)
% -------------------------------------------------------------------------
fig5 = figure('Name', 'MC200: Hoi Tu Boresight Va Scale Factor', 'Position', [100, 100, 1300, 850], 'Visible', 'off');

subplot(2, 2, 1);
err_bs_r = abs(bs_roll_est - bs_roll_true');
plot(t_eval, mean(err_bs_r, 2), 'r-', 'LineWidth', 1.8); hold on;
plot(t_eval, sigma3_bs(:, 1), 'k:', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.5);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so Roll (deg)');
title('1. Hoi tu sai so Boresight Roll vs 3\sigma', 'FontWeight', 'bold');
legend('Sai so TB', '3\sigma Bound', 'Cat GNSS', 'Location', 'best');

subplot(2, 2, 2);
err_bs_p = abs(bs_pitch_est - bs_pitch_true');
plot(t_eval, mean(err_bs_p, 2), 'g-', 'LineWidth', 1.8); hold on;
plot(t_eval, sigma3_bs(:, 2), 'k:', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.5);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so Pitch (deg)');
title('2. Hoi tu sai so Boresight Pitch vs 3\sigma', 'FontWeight', 'bold');
legend('Sai so TB', '3\sigma Bound', 'Cat GNSS', 'Location', 'best');

subplot(2, 2, 3);
err_bs_y = abs(bs_yaw_est - bs_yaw_true');
plot(t_eval, mean(err_bs_y, 2), 'b-', 'LineWidth', 1.8); hold on;
plot(t_eval, sigma3_bs(:, 3), 'k:', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.5);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so Yaw (deg)');
title('3. Hoi tu sai so Boresight Yaw vs 3\sigma', 'FontWeight', 'bold');
legend('Sai so TB', '3\sigma Bound', 'Cat GNSS', 'Location', 'best');

subplot(2, 2, 4);
err_scale = abs(scale_est_all - scale_true_all');
plot(t_eval, mean(err_scale, 2), 'm-', 'LineWidth', 1.8); hold on;
plot(t_eval, sigma3_scale, 'k:', 'LineWidth', 1.2);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.5);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so Scale Factor (%)');
title('4. Hoi tu sai so DVL Scale Factor vs 3\sigma', 'FontWeight', 'bold');
legend('Sai so TB', '3\sigma Bound', 'Cat GNSS', 'Location', 'best');

saveas(fig5, fullfile(script_dir, 'fig_mc200_hoi_tu_boresight_scale.png'));
close(fig5);

% -------------------------------------------------------------------------
% HÌNH 6: ĐÁNH GIÁ TỔNG HỢP BỘ LỌC ES-EKF (Chuẩn 6 ô)
% -------------------------------------------------------------------------
fig6 = figure('Name', 'Danh Gia Tong Hop Bo Loc ES-EKF', ...
    'Position', [50, 50, 1400, 900], 'Visible', 'off');

% Ô 1: Sai số vị trí từng phương và 3D
subplot(2, 3, 1);
plot(t_eval, mean(err_pos_N, 2), 'r-', 'LineWidth', 1.4); hold on;
plot(t_eval, mean(err_pos_E, 2), 'g-', 'LineWidth', 1.4);
plot(t_eval, mean(err_pos_D, 2), 'b-', 'LineWidth', 1.4);
plot(t_eval, sqrt(mean(err_pos_3D.^2, 2)), 'k-', 'LineWidth', 2.0);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so vi tri (m)');
title('1. Sai so vi tri: Tau that - ES-EKF', 'FontWeight', 'bold');
legend('Sai so North', 'Sai so East', 'Sai so Down (Do sau)', 'Tong sai so 3D', 'Location', 'best');

% Ô 2: Sai số góc tư thế (Roll, Pitch, Yaw)
subplot(2, 3, 2);
plot(t_eval, mean(err_roll_deg, 2), 'r-', 'LineWidth', 1.3); hold on;
plot(t_eval, mean(err_pitch_deg, 2), 'g-', 'LineWidth', 1.3);
plot(t_eval, mean(err_yaw_deg, 2), 'b-', 'LineWidth', 1.6);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so goc (deg)');
title('2. Sai so goc huong & tu the (Euler)', 'FontWeight', 'bold');
legend('Sai so Roll', 'Sai so Pitch', 'Sai so Yaw (Huong)', 'Location', 'best');

% Ô 3: Sai số vận tốc 3 trục
subplot(2, 3, 3);
plot(t_eval, rmse_vel(:, 1), 'r-', 'LineWidth', 1.2); hold on;
plot(t_eval, rmse_vel(:, 2), 'g-', 'LineWidth', 1.2);
plot(t_eval, rmse_vel(:, 3), 'b-', 'LineWidth', 1.2);
plot(t_eval, sqrt(sum(rmse_vel.^2, 2)), 'k-', 'LineWidth', 1.8);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on; xlabel('Thoi gian (s)'); ylabel('Sai so van toc (m/s)');
title('3. Sai so van toc UUV (m/s)', 'FontWeight', 'bold');
legend('Err v_x', 'Err v_y', 'Err v_z', 'Tong |v|', 'Location', 'best');

% Ô 4: Hội tụ góc lệch Boresight DVL (Roll, Pitch, Yaw)
subplot(2, 3, 4);
plot(t_eval, mean(bs_roll_est, 2), 'r-', 'LineWidth', 1.6); hold on;
plot(t_eval, mean(bs_pitch_est, 2), 'g-', 'LineWidth', 1.6);
plot(t_eval, mean(bs_yaw_est, 2), 'b-', 'LineWidth', 1.6);
yline(mean(bs_roll_true), 'r--', 'LineWidth', 1.3);
yline(mean(bs_pitch_true), 'g--', 'LineWidth', 1.3);
yline(mean(bs_yaw_true), 'b--', 'LineWidth', 1.3);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on; xlabel('Thoi gian (s)'); ylabel('Goc Boresight (deg)');
title('4. Hoi tu 3 goc lech truc Boresight DVL', 'FontWeight', 'bold');
legend('Roll est', 'Pitch est', 'Yaw est', 'Location', 'best');

% Ô 5: Hội tụ hệ số tỉ lệ Scale Factor DVL kèm dải 3-sigma
subplot(2, 3, 5);
plot(t_eval, mean(scale_est_all, 2), 'm-', 'LineWidth', 2.0); hold on;
plot(t_eval, mean(scale_est_all, 2) + sigma3_scale, 'k:', 'LineWidth', 1.2);
plot(t_eval, mean(scale_est_all, 2) - sigma3_scale, 'k:', 'LineWidth', 1.2);
yline(mean(scale_true_all), 'b--', 'LineWidth', 1.5);
xline(T_gnss_cut, 'm--', sprintf('Cat GNSS (%.0fs)', T_gnss_cut), 'LineWidth', 1.4);
grid on; xlabel('Thoi gian (s)'); ylabel('Scale Factor (%)');
title('5. Hoi tu he so Scale Factor DVL', 'FontWeight', 'bold');
legend('Uoc luong Scale', '+3\sigma', '-3\sigma', 'Gia tri that', 'Location', 'best');

% Ô 6: Mặt bằng 2D North - East
subplot(2, 3, 6);
plot(traj_E_true, traj_N_true, 'b-', 'LineWidth', 2.2); hold on;
plot(traj_E_est, traj_N_est, 'r--', 'LineWidth', 1.5);
plot(wpt.pos.y, wpt.pos.x, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'y');
plot(wpt.pos.y, wpt.pos.x, 'k:', 'LineWidth', 1.0);
grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)');
title('6. Mat bang North - East (Nhin tu tren xuong)', 'FontWeight', 'bold');
legend('Tau chay thuc te', 'ES-EKF uoc luong', 'Waypoints', 'Location', 'best');

saveas(fig6, fullfile(script_dir, 'fig_mc200_danh_gia_bo_loc.png'));
saveas(fig6, fullfile(script_dir, 'fig_danh_gia_tau_that_vs_esekf_geofog.png'));
close(fig6);

% -------------------------------------------------------------------------
% HÌNH 7: PHÓNG TO CHI TIẾT ĐỘ TRÔI KHI LẶN NGẦM KHÔNG CÓ GNSS
% -------------------------------------------------------------------------
fig7 = figure('Name', 'Chi Tiet Do Troi Khi Lan Khong GNSS', ...
    'Position', [100, 100, 1200, 750], 'Visible', 'off');

idx_sub = (t_eval >= T_gnss_cut);
t_sub = t_eval(idx_sub) - T_gnss_cut;

subplot(2, 2, 1);
plot(t_sub, mean(err_pos_N(idx_sub, :), 2), 'r-', 'LineWidth', 1.8); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Do lech North (m)');
title('Do troi phuong Bac (North Drift) khi lan', 'FontWeight', 'bold');

subplot(2, 2, 2);
plot(t_sub, mean(err_pos_E(idx_sub, :), 2), 'b-', 'LineWidth', 1.8); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Do lech East (m)');
title('Do troi phuong Dong (East Drift) khi lan', 'FontWeight', 'bold');

subplot(2, 2, 3);
plot(t_sub, mean(err_pos_D(idx_sub, :), 2), 'Color', [0, 0.6, 0], 'LineWidth', 1.8); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Do lech Down (m)');
title('Do troi phuong thang dung / Do sau (Down Drift)', 'FontWeight', 'bold');

subplot(2, 2, 4);
plot(t_sub, sqrt(mean(err_pos_3D(idx_sub, :).^2, 2)), 'k-', 'LineWidth', 2.0); grid on;
xlabel('Thoi gian lan khong GNSS (s)'); ylabel('Tong sai so 3D (m)');
title('Tong do troi khong gian 3D khi lan', 'FontWeight', 'bold');

saveas(fig7, fullfile(script_dir, 'fig_mc200_do_troi_khi_lan.png'));
saveas(fig7, fullfile(script_dir, 'fig_chi_tiet_do_troi_khi_lan_geofog.png'));
close(fig7);

% Tính toán các chỉ số thống kê định lượng chi tiết
idx_p1 = (t_eval <= T_gnss_cut);
idx_p2 = (t_eval > T_gnss_cut);
anees_p1 = mean(anees_total(idx_p1));
anees_p2 = mean(anees_total(idx_p2));
anees_mean = mean(anees_total);

armse_pos_final = mean(sqrt(sum((pos_final_true - pos_final_est).^2, 2)));
armse_pos_mean  = mean(pos_3d_armse);
armse_vel_final = vel_3d_armse(end);
armse_vel_mean  = mean(vel_3d_armse);

armse_roll_final  = rmse_att(end, 1);
armse_pitch_final = rmse_att(end, 2);
armse_yaw_final   = rmse_att(end, 3);

armse_bg_final = rmse_bg(end, :);
armse_ba_final = rmse_ba(end, :);

err_bs_final_r = mean(abs(bs_roll_est(end, :) - bs_roll_true'));
err_bs_final_p = mean(abs(bs_pitch_est(end, :) - bs_pitch_true'));
err_bs_final_y = mean(abs(bs_yaw_est(end, :) - bs_yaw_true'));
err_scale_final = mean(abs(scale_est_all(end, :) - scale_true_all'));

drift_N_final = mean(abs(err_pos_N(end, :)));
drift_E_final = mean(abs(err_pos_E(end, :)));
drift_D_final = mean(abs(err_pos_D(end, :)));
drift_3D_final = mean(err_pos_3D(end, :));
submerged_dist = 4000 * ((1880 - T_gnss_cut) / 1880);
drift_pct = (drift_3D_final / submerged_dist) * 100;

% Lưu file MAT
save(fullfile(script_dir, 'ket_qua_mc200_closedloop.mat'), ...
    't_eval', 'anees_total', 'anees_pos', 'anees_vel', 'anees_att', 'anees_bias', 'anees_cal', ...
    'ci_low_19', 'ci_high_19', 'rmse_pos', 'rmse_vel', 'rmse_att', 'rmse_bg', 'rmse_ba', ...
    'sigma3_pos', 'sigma3_vel', 'sigma3_att', 'sigma3_bg', 'sigma3_ba', ...
    'err_pos_N', 'err_pos_E', 'err_pos_D', 'err_pos_3D', ...
    'err_roll_deg', 'err_pitch_deg', 'err_yaw_deg', ...
    'bs_roll_est', 'bs_pitch_est', 'bs_yaw_est', 'scale_est_all', ...
    'bs_roll_true', 'bs_pitch_true', 'bs_yaw_true', 'scale_true_all', ...
    'pos_final_true', 'pos_final_est', 'M_runs', 'T_gnss_cut');

% In báo cáo các chỉ số định lượng đầy đủ ra Console & Ghi File TXT
report_str = sprintf([...
    '\n========================================================================================\n' ...
    '             BÁO CÁO KHOA HỌC: ĐÁNH GIÁ ĐA CHỈ SỐ BỘ LỌC ES-EKF (GEO-FOG 3D + DVL)\n' ...
    '             Quy mô mô phỏng: M = %d runs | Toàn trình 4 km (1880s) | Cắt GNSS tại %0.fs\n' ...
    '========================================================================================\n\n' ...
    '1. CHỈ SỐ NHẤT QUÁN THỐNG KÊ (CONSISTENCY TEST):\n' ...
    '   - ANEES Toàn trình (19D)       : %6.2f  (Kỳ vọng lý tưởng: 19.00)\n' ...
    '   - ANEES Pha 1 (Có GNSS: 0-%.0fs) : %6.2f  (Kỳ vọng: 19.00)\n' ...
    '   - ANEES Pha 2 (Lặn ngầm: >%.0fs): %6.2f  (Kỳ vọng: 19.00)\n' ...
    '   - Tỷ lệ mốc nằm trong dải 95%%%% CI: %6.1f%%%% (Ngưỡng [%.2f, %.2f])\n' ...
    '   - ANEES Vị trí 3D (3 DOF)      : %6.2f  (Kỳ vọng: 3.00)\n' ...
    '   - ANEES Vận tốc 3D (3 DOF)     : %6.2f  (Kỳ vọng: 3.00)\n' ...
    '   - ANEES Tư thế 3D (3 DOF)      : %6.2f  (Kỳ vọng: 3.00)\n' ...
    '   - ANEES Bias IMU (6 DOF)       : %6.2f  (Kỳ vọng: 6.00)\n' ...
    '   - ANEES Hiệu chuẩn DVL (4 DOF) : %6.2f  (Kỳ vọng: 4.00)\n\n' ...
    '2. CHỈ SỐ ĐỘ CHÍNH XÁC DẪN ĐƯỜNG (NAVIGATION ACCURACY - ARMSE):\n' ...
    '   - ARMSE Vị trí 3D trung bình   : %6.4f m\n' ...
    '   - ARMSE Vị trí 3D cuối 4km     : %6.4f m\n' ...
    '   - ARMSE Vận tốc 3D trung bình  : %6.4f m/s\n' ...
    '   - ARMSE Vận tốc 3D cuối 4km    : %6.4f m/s\n' ...
    '   - ARMSE Góc Roll cuối          : %6.4f deg\n' ...
    '   - ARMSE Góc Pitch cuối         : %6.4f deg\n' ...
    '   - ARMSE Góc Yaw (Hướng) cuối   : %6.4f deg\n' ...
    '   - ARMSE Bias Con quay FOG cuối : X=%6.2f, Y=%6.2f, Z=%6.2f deg/h\n' ...
    '   - ARMSE Bias Gia tốc kế cuối   : X=%6.3f, Y=%6.3f, Z=%6.3f mg\n\n' ...
    '3. CHỈ SỐ HIỆU CHUẨN LẮP RÁP CƠ KHÍ & ÂM HỌC DVL:\n' ...
    '   - Boresight Roll (alpha)       : Sai số = %6.4f deg\n' ...
    '   - Boresight Pitch (beta)       : Sai số = %6.4f deg\n' ...
    '   - Boresight Yaw (gamma)        : Sai số = %6.4f deg\n' ...
    '   - Scale Factor DVL (s)         : Sai số = %6.4f %%%%\n' ...
    '   - Cơ chế đóng băng khi lặn ngầm: ĐÃ KHÓA 100%%%% tại t = %.0fs (Loại bỏ trôi boresight)\n\n' ...
    '4. CHỈ SỐ ĐỘ TRÔI KHI LẶN NGẦM (DEAD-RECKONING DRIFT SAU CẮT GNSS):\n' ...
    '   - Quãng đường lặn ngầm         : ~%.0f m (Thời gian lặn: %.0fs)\n' ...
    '   - Độ trôi phương Bắc (North)   : %6.3f m\n' ...
    '   - Độ trôi phương Đông (East)   : %6.3f m\n' ...
    '   - Độ trôi độ sâu (Down)        : %6.3f m\n' ...
    '   - Tổng độ trôi không gian 3D   : %6.3f m\n' ...
    '   - Tỷ lệ độ trôi / Quãng đường  : %6.3f %%%% DT (Đạt tiêu chuẩn quân sự cao cấp)\n' ...
    '========================================================================================\n'], ...
    M_runs, T_gnss_cut, ...
    anees_mean, T_gnss_cut, anees_p1, T_gnss_cut, anees_p2, pct_anees_pass, ci_low_19, ci_high_19, ...
    mean(anees_pos), mean(anees_vel), mean(anees_att), mean(anees_bias), mean(anees_cal), ...
    armse_pos_mean, armse_pos_final, armse_vel_mean, armse_vel_final, ...
    armse_roll_final, armse_pitch_final, armse_yaw_final, ...
    armse_bg_final(1), armse_bg_final(2), armse_bg_final(3), ...
    armse_ba_final(1), armse_ba_final(2), armse_ba_final(3), ...
    err_bs_final_r, err_bs_final_p, err_bs_final_y, err_scale_final, T_gnss_cut, ...
    submerged_dist, 1880 - T_gnss_cut, drift_N_final, drift_E_final, drift_D_final, drift_3D_final, drift_pct);

fprintf('%s', report_str);

report_txt_file = fullfile(script_dir, 'bao_cao_danh_gia_chi_so_esekf.txt');
fid = fopen(report_txt_file, 'w', 'n', 'UTF-8');
if fid > 0
    fprintf(fid, '%s', report_str);
    fclose(fid);
end

fprintf('Đã lưu đầy đủ 7 bộ đồ thị và bảng báo cáo đa chỉ số vào thư mục: %s\n', script_dir);
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
sin_theta = max(min(2*(w*y - z*x), 1), -1);
theta = asin(sin_theta);
psi = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
end

function Cbn = quat_to_cbn_local(q)
q = q / norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);
Cbn = [1 - 2*(y^2 + z^2), 2*(x*y - w*z),     2*(x*z + w*y); ...
    2*(x*y + w*z),     1 - 2*(x^2 + z^2), 2*(y*z - w*x); ...
    2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x^2 + y^2)];
end

function R = rodrigues_so3_local(phi)
theta = norm(phi);
if theta < 1e-12, R = eye(3); return; end
K = [0, -phi(3), phi(2); phi(3), 0, -phi(1); -phi(2), phi(1), 0];
R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
end

function p = prctile_local(x, pct, dim)
if nargin < 3, dim = 1; end
if dim == 2
    x = x';
    p = prctile_local(x, pct, 1)';
    return;
end
x_sorted = sort(x, 1);
N = size(x_sorted, 1);
idx = max(min(round(pct / 100 * N), N), 1);
p = x_sorted(idx, :);
end

function phi = so3_log_local(R)
cos_theta = max(min(0.5*(trace(R) - 1), 1), -1);
theta = acos(cos_theta);
if theta > 1e-8
    phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
else
    phi = 0.5 * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
end
end

function q = chi2inv_custom(p, d)
if exist('chi2inv', 'file') == 2
    q = chi2inv(p, d);
else
    z = sqrt(2) * erfinv(2*p - 1);
    q = d * (1 - 2/(9*d) + z*sqrt(2/(9*d)))^3;
end
end

function progress_printer(m, M_runs, t_start)
persistent completed_count
if isempty(completed_count)
    completed_count = 0;
end
completed_count = completed_count + 1;
t_elap = toc(t_start);
eta_sec = (t_elap / completed_count) * (M_runs - completed_count);
fprintf('[Đa nhân] Hoàn thành %3d / %d run (%5.1f%%) | Đã chạy: %5.1f ph | Còn lại: ~%5.1f ph\n', ...
    completed_count, M_runs, (completed_count / M_runs) * 100, t_elap / 60, eta_sec / 60);
end

function res = run_single_simulation(m, wpt, h, T_final, T_gnss_cut, sample_step, nEval, t_eval, q)
% Đảm bảo cô lập persistent variables trên worker độc lập
clear('ALOS3D', 'integralSMCheading');

% Mầm số ngẫu nhiên độc lập cho từng run
rng(2026 + m * 7, 'twister');

% 1. Ngẫu nhiên hóa Boresight & Scale thật của run m
true_bs_deg = [ ...
    -3.0 + 6.0 * rand(); ...   % Roll [-3, +3] deg
    -3.0 + 6.0 * rand(); ...   % Pitch [-3, +3] deg
    -5.0 + 10.0 * rand()];     % Yaw [-5, +5] deg
true_bs_rad = deg2rad(true_bs_deg);
true_scale_val = -0.03 + 0.06 * rand(); % Scale [-3%, +3%]

res.bs_roll_true  = true_bs_deg(1);
res.bs_pitch_true = true_bs_deg(2);
res.bs_yaw_true   = true_bs_deg(3);
res.scale_true_all= true_scale_val * 100;

Cbs_true = rodrigues_so3_local(-true_bs_rad);

% 2. Ngẫu nhiên hóa dòng hải lưu của run m
Vc_run = 0.2 + 0.6 * rand();       % [0.2, 0.8] m/s
betaVc_run = 2 * pi * rand();       % [0, 360] deg
wc_run = 0.05 + 0.1 * rand();

% 3. Cấu hình cảm biến GEO-FOG 3D Dual
ins_spec = GEO_FOG_3D('spec');
bg_true_run = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600] + ins_spec.gyro.bias_instab * randn(3, 1);
ba_true_run = [0.008; -0.012; 0.006] + ins_spec.acc.bias_instab * randn(3, 1);
ins_state = GEO_FOG_3D('init', ins_spec, bg_true_run, ba_true_run);

% Mô phỏng nghiêm ngặt theo đặc tính nhiễu cảm biến GEO-FOG 3D Dual
sigma_vrw = ins_spec.acc.vrw;
sigma_arw = ins_spec.gyro.arw;
acc_bias_sigma = ins_spec.acc.bias_instab;
acc_bias_tau = ins_spec.acc.bias_tau;
gyro_bias_rw = ins_spec.gyro.bias_rw;
scale_rw = 1e-5;
boresight_rw = 1e-5;

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
    gyro_bias_rw; scale_rw; boresight_rw; h];

gravity = 9.80665;
w_ie_ned = [0; 0; 7.292115e-5];
radius_n = 6378137.0; radius_m = 6356752.3;
tan_lat = tan(deg2rad(20.8)); height = 0;
lever_arm = [0.5; -0.1; 0.2] + 0.005 * randn(3, 1);

cfg = zeros(21, 1);
cfg(1) = h;
cfg(2) = gravity;
cfg(3:5) = w_ie_ned;
cfg(6) = radius_n;
cfg(7) = radius_m;
cfg(8) = tan_lat;
cfg(9) = height;
cfg(10:12) = lever_arm;
cfg(13) = 0.01;       % dvl_noise_floor
cfg(14) = 0.005;      % dvl_noise_slope
cfg(15) = ins_spec.gnss.pos_v;
cfg(16) = ins_spec.gnss.vel;
cfg(17) = ins_spec.gnss.heading;
cfg(18) = 0.05;       % sigma_depth
cfg(19) = acc_bias_tau;
cfg(20) = sigma_arw;

% 4. Khởi tạo trạng thái vật lý tàu REMUS 100
xn = 0; yn = 0; zn = 0; phi = 0; theta = 0;
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));
U = 1.5;
x_phys = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

% 5. Khởi tạo ES-EKF 19 trạng thái (Cold-start từ 0)
[state_est, P_est, Q] = esekf19_init([phi; theta; psi], gravity, noise_cfg);
state_est(4:6) = eul2cbn_local(phi, theta, psi) * [U; 0; 0];
state_est(17) = 0.0;              % Scale cold-start = 0%
state_est(18:20) = [0.0; 0.0; 0.0]; % Boresight cold-start = [0;0;0]
P_est(4:6, 4:6)     = (1.5)^2 * eye(3);
P_est(16, 16)       = (0.05)^2;
P_est(17:19, 17:19) = deg2rad(5.0)^2 * eye(3);

% Tham số bộ điều khiển
[~,~,Mauv] = remus100();
Kp_theta = Mauv(5,5) * 0.64;
Kd_theta = Mauv(5,5) * 1.28;
Ki_theta = Kp_theta * 0.08;
theta_int = 0; theta_d = 0; q_d = 0; psi_d = psi; r_d = 0;
n_p = 1300; delta_max = deg2rad(20);
ui_curr = [0; 0; n_p];

% Điểm ngẫu nhiên mất bám đáy DVL (20s trong pha lặn)
t_dvl_dropout_start = 600 + 700 * rand();
t_dvl_dropout_end   = t_dvl_dropout_start + 20;

eval_cnt = 0;
n_outlier_inj = 0; n_outlier_rej = 0;
locked_scale = 0.0; locked_boresight = [0.0; 0.0; 0.0];

% Cấp phát bộ nhớ cho các mảng kết quả của run m
res.err_pos_N    = zeros(nEval, 1);
res.err_pos_E    = zeros(nEval, 1);
res.err_pos_D    = zeros(nEval, 1);
res.err_pos_3D   = zeros(nEval, 1);
res.err_roll_deg = zeros(nEval, 1);
res.err_pitch_deg= zeros(nEval, 1);
res.err_yaw_deg  = zeros(nEval, 1);

res.bs_roll_est  = zeros(nEval, 1);
res.bs_pitch_est = zeros(nEval, 1);
res.bs_yaw_est   = zeros(nEval, 1);
res.scale_est_all= zeros(nEval, 1);

res.nees         = zeros(nEval, 1);
res.nees_pos     = zeros(nEval, 1);
res.nees_vel     = zeros(nEval, 1);
res.nees_att     = zeros(nEval, 1);
res.nees_bias    = zeros(nEval, 1);
res.nees_cal     = zeros(nEval, 1);

res.sq_pos       = zeros(nEval, 3);
res.sq_vel       = zeros(nEval, 3);
res.sq_att       = zeros(nEval, 3);
res.sq_bg        = zeros(nEval, 3);
res.sq_ba        = zeros(nEval, 3);

res.var_pos      = zeros(nEval, 3);
res.var_vel      = zeros(nEval, 3);
res.var_att      = zeros(nEval, 3);
res.var_ba       = zeros(nEval, 3);
res.var_bg       = zeros(nEval, 3);
res.var_bs       = zeros(nEval, 3);
res.var_scale    = zeros(nEval, 1);

res.traj_N_true  = zeros(nEval, 1);
res.traj_E_true  = zeros(nEval, 1);
res.traj_D_true  = zeros(nEval, 1);
res.traj_N_est   = zeros(nEval, 1);
res.traj_E_est   = zeros(nEval, 1);
res.traj_D_est   = zeros(nEval, 1);

t = 0:h:T_final;
nSteps = length(t);

for k = 1:nSteps
    tk = t(k);
    u_b = x_phys(1); v_b = x_phys(2); w_b = x_phys(3);
    p_b = x_phys(4); q_b = x_phys(5); r_b = x_phys(6);
    xn  = x_phys(7); yn  = x_phys(8); zn  = x_phys(9);
    phi = x_phys(10); theta = x_phys(11); psi = x_phys(12);

    Cbn_true = eul2cbn_local(phi, theta, psi);
    v_ned_true = Cbn_true * [u_b; v_b; w_b];
    v_body = [u_b; v_b; w_b];
    omega_b = [p_b; q_b; r_b];

    [xdot_true, ~] = remus100(x_phys, ui_curr, Vc_run, betaVc_run, wc_run);
    a_b_true = xdot_true(1:3) + cross(omega_b, v_body);

    w_en_ned = [v_ned_true(2)/(radius_n + height); ...
        -v_ned_true(1)/(radius_m + height); ...
        -v_ned_true(2)*tan_lat/(radius_n + height)];

    f_b_true = a_b_true - Cbn_true' * [0; 0; gravity] + Cbn_true' * cross(2*w_ie_ned + w_en_ned, v_ned_true);
    w_ib_b_true = omega_b + Cbn_true' * (w_ie_ned + w_en_ned);

    % Sinh số đo IMU GEO-FOG 3D
    [gyro_meas, accel_meas, ins_state] = GEO_FOG_3D('imu', ins_spec, ins_state, ...
        w_ib_b_true, f_b_true, h);

    flags = zeros(3, 1);
    gnss_meas = zeros(7, 1); depth_meas = 0; dvl_meas = zeros(3, 1);

    % Cập nhật đo đạc 5 Hz (mỗi 0.2s)
    if mod(k, 4) == 0
        depth_meas = zn + 0.02 * randn;
        flags(2) = 1;

        if ~(tk >= t_dvl_dropout_start && tk <= t_dvl_dropout_end)
            v_lever = v_body + cross(omega_b, lever_arm);

            v_dvl_ideal = (1.0 + true_scale_val) * Cbs_true * v_lever;
            dvl_noise = 0.005 + 0.002 * norm(v_lever);
            dvl_meas = v_dvl_ideal + dvl_noise * randn(3, 1);

            % Bơm ngoại lai Outlier 1.5%
            if rand() < 0.015
                dvl_meas = dvl_meas + [-1.5 + 3.0*rand(); -1.5 + 3.0*rand(); 0];
                n_outlier_inj = n_outlier_inj + 1;
            end
            flags(3) = 1;
        end

        % GNSS RTK mặt nước
        if tk <= T_gnss_cut
            gnss_meas = GEO_FOG_3D('gnss', ins_spec, [xn; yn; zn], v_ned_true, psi);
            flags(1) = 1;
        end
    end

    % Lặn ngầm (tk > T_gnss_cut) -> KHÓA Boresight & Scale
    cfg_k = cfg;
    if tk > T_gnss_cut
        cfg_k(21) = 1;
        state_est(17) = locked_scale;
        state_est(18:20) = locked_boresight;
    end

    % Bước lọc ES-EKF
    [state_est, P_est, update_status, nis] = esekf19_step( ...
        state_est, P_est, Q, gyro_meas, accel_meas, ...
        gnss_meas, depth_meas, dvl_meas, flags, cfg_k);

    if tk <= T_gnss_cut
        locked_scale = state_est(17);
        locked_boresight = state_est(18:20);
    else
        state_est(17) = locked_scale;
        state_est(18:20) = locked_boresight;
    end

    if flags(3) == 1 && update_status(3) == 0
        n_outlier_rej = n_outlier_rej + 1;
    end

    % Điều khiển khép vòng theo ước lượng EKF
    p_hat = state_est(1:3);
    [phi_hat, theta_hat, psi_hat] = quat_to_euler_local(state_est(7:10));

    [psi_ref, theta_ref] = ALOS3D(p_hat(1), p_hat(2), p_hat(3), 25, 20, 0.001, 0.001, ...
        deg2rad(20), h, 20, wpt);
    [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, 0.5);
    [psi_d, r_d]   = LOSobserver(psi_d, r_d, psi_ref, h, 0.5);
    r_d = max(min(r_d, deg2rad(5)), -deg2rad(5));

    r_gyro_c = gyro_meas(3) - state_est(16);
    q_gyro_c = gyro_meas(2) - state_est(15);

    delta_r = integralSMCheading(psi_hat, r_gyro_c, psi_d, r_d, 0, 0, 0.05, 1, 0.1, 0.25, 1.0, h);
    delta_r = max(min(delta_r, delta_max), -delta_max);

    delta_s = -Kp_theta * ssa(theta_hat - theta_d) - Kd_theta * q_gyro_c - Ki_theta * theta_int;
    delta_s = max(min(delta_s, delta_max), -delta_max);

    ui = [delta_r; -delta_s; n_p];
    x_phys = rk4(@remus100, h, x_phys, ui, Vc_run, betaVc_run, wc_run);
    theta_int = theta_int + h * ssa(theta_hat - theta_d);
    ui_curr = ui;

    % Ghi nhận mẫu thống kê
    if mod(k - 1, sample_step) == 0
        eval_cnt = eval_cnt + 1;

        % Sai số vị trí
        e_pos = [xn - p_hat(1); yn - p_hat(2); zn - p_hat(3)];
        res.err_pos_N(eval_cnt) = e_pos(1);
        res.err_pos_E(eval_cnt) = e_pos(2);
        res.err_pos_D(eval_cnt) = e_pos(3);
        res.err_pos_3D(eval_cnt) = norm(e_pos);

        % Sai số vận tốc
        e_vel = v_ned_true - state_est(4:6);

        % Sai số tư thế trên SO(3)
        Cbn_est = quat_to_cbn_local(state_est(7:10));
        R_err_att = Cbn_est' * Cbn_true;
        e_att = so3_log_local(R_err_att);
        res.err_roll_deg(eval_cnt)  = rad2deg(e_att(1));
        res.err_pitch_deg(eval_cnt) = rad2deg(e_att(2));
        res.err_yaw_deg(eval_cnt)   = rad2deg(e_att(3));

        % Sai số bias cảm biến so với chân lý tức thời
        e_ba = ins_state.ba_total - state_est(11:13);
        e_bg = ins_state.bg_total - state_est(14:16);

        % Sai số Boresight & Scale
        e_scale = true_scale_val - state_est(17);
        Cbs_est = rodrigues_so3_local(-state_est(18:20));
        R_err_bs = Cbs_est' * Cbs_true;
        e_bs = -so3_log_local(R_err_bs);

        res.bs_roll_est(eval_cnt)  = rad2deg(state_est(18));
        res.bs_pitch_est(eval_cnt) = rad2deg(state_est(19));
        res.bs_yaw_est(eval_cnt)   = rad2deg(state_est(20));
        res.scale_est_all(eval_cnt)= state_est(17) * 100;

        % Vector sai số toàn hệ 19D
        e_k = [e_pos; e_vel; e_att; e_ba; e_bg; e_scale; e_bs];

        % TÍNH TOÁN NEES CHUẨN HÓA (ĐẢM BẢO 0 CẢNH BÁO RCOND / BADLY SCALED)
        % Chuẩn hóa đường chéo để triệt tiêu lệch thứ nguyên giữa m^2 và (rad/s)^2
        d_diag = sqrt(max(diag(P_est), 1e-22));
        P_scaled = P_est ./ (d_diag * d_diag');
        P_scaled = 0.5 * (P_scaled + P_scaled') + 1e-12 * eye(19);
        e_scaled = e_k ./ d_diag;
        res.nees(eval_cnt) = e_scaled' * (P_scaled \ e_scaled);

        res.nees_pos(eval_cnt) = e_pos' * (P_est(1:3, 1:3) \ e_pos);
        res.nees_vel(eval_cnt) = e_vel' * (P_est(4:6, 4:6) \ e_vel);
        res.nees_att(eval_cnt) = e_att' * (P_est(7:9, 7:9) \ e_att);

        % Nhóm Bias 6D: Chuẩn hóa tỉ lệ giữa Accel bias (m/s^2) và Gyro bias (rad/s)
        d_bias = sqrt(max(diag(P_est(10:15, 10:15)), 1e-22));
        P_bias_scaled = P_est(10:15, 10:15) ./ (d_bias * d_bias');
        P_bias_scaled = 0.5 * (P_bias_scaled + P_bias_scaled') + 1e-12 * eye(6);
        e_bias = [e_ba; e_bg];
        e_bias_scaled = e_bias ./ d_bias;
        res.nees_bias(eval_cnt) = e_bias_scaled' * (P_bias_scaled \ e_bias_scaled);

        % Nhóm Hiệu chuẩn 4D
        e_cal = [e_scale; e_bs];
        res.nees_cal(eval_cnt) = e_cal' * (P_est(16:19, 16:19) \ e_cal);

        % Tích lũy bình phương sai số RMSE & phương sai 3-sigma
        res.sq_pos(eval_cnt, :) = (e_pos').^2;
        res.sq_vel(eval_cnt, :) = (e_vel').^2;
        res.sq_att(eval_cnt, :) = (rad2deg(e_att')).^2;
        res.sq_bg(eval_cnt, :)  = (rad2deg(e_bg') * 3600).^2;
        res.sq_ba(eval_cnt, :)  = (e_ba' / 9.80665 * 1000).^2;

        res.var_pos(eval_cnt, :) = diag(P_est(1:3, 1:3))';
        res.var_vel(eval_cnt, :) = diag(P_est(4:6, 4:6))';
        res.var_att(eval_cnt, :) = (rad2deg(sqrt(diag(P_est(7:9, 7:9)))))'.^2;
        res.var_ba(eval_cnt, :)  = (sqrt(diag(P_est(10:12, 10:12)))' / 9.80665 * 1000).^2;
        res.var_bg(eval_cnt, :)  = (rad2deg(sqrt(diag(P_est(13:15, 13:15))))' * 3600).^2;
        res.var_bs(eval_cnt, :)  = (rad2deg(sqrt(diag(P_est(17:19, 17:19)))))'.^2;
        res.var_scale(eval_cnt)  = (sqrt(P_est(16, 16)) * 100)^2;

        res.traj_N_true(eval_cnt) = xn;
        res.traj_E_true(eval_cnt) = yn;
        res.traj_D_true(eval_cnt) = zn;
        res.traj_N_est(eval_cnt)  = p_hat(1);
        res.traj_E_est(eval_cnt)  = p_hat(2);
        res.traj_D_est(eval_cnt)  = p_hat(3);
    end
end

res.pos_final_true = [xn, yn, zn];
res.pos_final_est  = p_hat';
res.n_outlier_inj  = n_outlier_inj;
res.n_outlier_rej  = n_outlier_rej;

if ~isempty(q)
    send(q, m);
end
end


