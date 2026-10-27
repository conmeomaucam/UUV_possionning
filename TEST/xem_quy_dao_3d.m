function xem_quy_dao_3d()
%% XEM_QUY_DAO_3D - Tab rieng toan man hinh xem quy dao 3D REMUS 100
% KHONG cat ghep subplot nho. 1 cua so duy nhat chiem toan bo man hinh.
% Ho tro:
% - Xoay 3D tu do bang chuot (rotate3d on).
% - Cuon chuot (Scroll wheel) de phong to / thu nho bat ky vi tri nao.
% - Cac nut bam nhanh chuyen goc nhin (Toan canh, Top-down 2D, Trac doc, Zoom dich, Zoom lan).

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

integrated_mat = fullfile(script_dir, 'ket_qua_fossen_esekf_integrated.mat');
standalone_mat = fullfile(script_dir, 'ket_qua_fossen_standalone.mat');

if ~exist(integrated_mat, 'file')
    error('Chua tim thay file ket qua: %s. Hay chay run_fossen_esekf_integrated truoc!', integrated_mat);
end

e_data = load(integrated_mat);
t = e_data.t;
pos_true = e_data.pos_true_hist;
pos_est  = e_data.pos_est_hist;
wpt = e_data.wpt;
T_gnss_cut = e_data.T_gnss_cut;

has_fossen = false;
if exist(standalone_mat, 'file')
    try
        f_data = load(standalone_mat);
        pos_fossen = f_data.pos_history;
        has_fossen = true;
    catch
        has_fossen = false;
    end
end

%% TAO CUA SO FIGURE TOAN MAN HINH (1 TAB DUY NHAT, KHONG SUBPLOT)
fig = figure('Name', 'QUY DAO 3D REMUS 100 - TOAN MAN HINH (Click chuot XOAY 3D & Cuon chuot ZOOM)', ...
    'Color', 'w', 'Position', [40, 40, 1500, 880], 'WindowState', 'maximized', ...
    'Toolbar', 'figure', 'MenuBar', 'figure', 'Visible', 'on');

% Truc toa do chiem 92% dien tich cua so
ax = axes('Position', [0.06, 0.10, 0.90, 0.83]);

% Pham vi mat nuoc bien (z = 0)
e_all = [pos_true(:, 2); pos_est(:, 2)];
n_all = [pos_true(:, 1); pos_est(:, 1)];
if has_fossen
    e_all = [e_all; pos_fossen(:, 2)];
    n_all = [n_all; pos_fossen(:, 1)];
end
e_min = min(e_all) - 100;  e_max = max(e_all) + 100;
n_min = min(n_all) - 100;  n_max = max(n_all) + 100;

% Mat nuoc bien phang z = 0 (mau xanh lam nhat ban trong suot)
patch([e_min e_max e_max e_min], [n_min n_min n_max n_max], [0 0 0 0], ...
    [0.35, 0.75, 0.95], 'FaceAlpha', 0.15, 'EdgeColor', [0.2, 0.6, 0.85], ...
    'LineStyle', ':', 'LineWidth', 1.0, 'DisplayName', 'Mat nuoc bien (z = 0m)');
hold on;

% Luoi day bien tham chieu z = 105m
patch([e_min e_max e_max e_min], [n_min n_min n_max n_max], [105 105 105 105], ...
    [0.75, 0.75, 0.75], 'FaceAlpha', 0.06, 'EdgeColor', [0.6 0.6 0.6], ...
    'LineStyle', '--', 'LineWidth', 0.8, 'DisplayName', 'Day bien tham chieu (105m)');

leg_handles = [];
leg_labels  = {};

% 1. Duong Fossen Standalone ly tuong
if has_fossen
    h_fossen = plot3(pos_fossen(:, 2), pos_fossen(:, 1), pos_fossen(:, 3), ...
        'Color', [0, 0.55, 0], 'LineStyle', '--', 'LineWidth', 2.2);
    leg_handles(end+1) = h_fossen;
    leg_labels{end+1}  = '1. Fossen Standalone (Ly tuong)';
end

% 2. Duong Tau chay thuc te (Fossen 6-DOF + Nhieu + Dong chay)
h_true = plot3(pos_true(:, 2), pos_true(:, 1), pos_true(:, 3), ...
    'Color', [0, 0.25, 0.85], 'LineStyle', '-', 'LineWidth', 2.4);
leg_handles(end+1) = h_true;
leg_labels{end+1}  = '2. Tau that (Fossen 6-DOF + Nhieu)';

% 3. Duong ES-EKF uoc luong truc tuyen
h_est = plot3(pos_est(:, 2), pos_est(:, 1), pos_est(:, 3), ...
    'Color', [0.85, 0, 0.65], 'LineStyle', '-.', 'LineWidth', 2.0);
leg_handles(end+1) = h_est;
leg_labels{end+1}  = '3. Uoc luong ES-EKF';

% 4. Tuyen duong Waypoints
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'k:', 'LineWidth', 1.2);
h_wpt = plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'kd', 'MarkerSize', 6, ...
    'MarkerFaceColor', [1, 0.8, 0.2], 'LineWidth', 1.0);
leg_handles(end+1) = h_wpt;
leg_labels{end+1}  = 'Waypoints (WP1 - WP12)';

% Nhan danh dau cac moc Waypoint
for w = 1:numel(wpt.pos.x)
    text(wpt.pos.y(w) + 20, wpt.pos.x(w), wpt.pos.z(w) - 3, ...
        sprintf('WP%d', w), 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);
end

% 5. Cac moc su kien quan trong
% Xuat phat
plot3(pos_true(1, 2), pos_true(1, 1), pos_true(1, 3), 'gp', ...
    'MarkerSize', 13, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
text(pos_true(1, 2) + 30, pos_true(1, 1) + 20, pos_true(1, 3) - 6, ...
    'XUAT PHAT (t=0s)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0]);

% Cat GNSS / Lan ngam thuc te (thoi diem cuoi cung con nhan GNSS truoc khi anten chim duoi nuoc zn > 0.5m)
if isfield(e_data, 'acc_hist') && any(~isnan(e_data.acc_hist(:, 1)) & e_data.acc_hist(:, 1) == 1)
    idx_cut = find(~isnan(e_data.acc_hist(:, 1)) & e_data.acc_hist(:, 1) == 1, 1, 'last');
else
    idx_cut = find(t >= T_gnss_cut | pos_true(:, 3) > 0.5, 1, 'first');
end
if ~isempty(idx_cut)
    t_cut_actual = t(idx_cut);
    plot3(pos_true(idx_cut, 2), pos_true(idx_cut, 1), pos_true(idx_cut, 3), 'yd', ...
        'MarkerSize', 12, 'MarkerFaceColor', [1 0.6 0], 'MarkerEdgeColor', 'k');
    text(pos_true(idx_cut, 2) + 30, pos_true(idx_cut, 1) - 40, pos_true(idx_cut, 3) - 6, ...
        sprintf('CAT GNSS (t = %.1fs, zn = %.2fm)', t_cut_actual, pos_true(idx_cut, 3)), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8 0.4 0]);
end

% Dich den
plot3(pos_true(end, 2), pos_true(end, 1), pos_true(end, 3), 'rs', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
text(pos_true(end, 2) - 150, pos_true(end, 1) + 25, pos_true(end, 3) - 6, ...
    sprintf('DICH (t=%.0fs)', t(end)), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

% Doan thang do sai so vi tri cuoi cung giua Tau that va ES-EKF
pt_end = pos_true(end, :);
pe_end = pos_est(end, :);
plot3([pt_end(2), pe_end(2)], [pt_end(1), pe_end(1)], [pt_end(3), pe_end(3)], 'r-', 'LineWidth', 2.5);
err_final = norm(pt_end - pe_end);
text((pt_end(2)+pe_end(2))/2 - 30, (pt_end(1)+pe_end(1))/2 + 15, (pt_end(3)+pe_end(3))/2 - 3, ...
    sprintf('Sai so EKF: %.2f m', err_final), ...
    'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold', 'BackgroundColor', [1 1 0.85]);

grid on; box on;
set(gca, 'ZDir', 'reverse');
xlabel('East (m)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('North (m)', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('Down - Do sau (m)', 'FontSize', 11, 'FontWeight', 'bold');
title({'TOAN CANH QUY DAO 3D REMUS 100 (FOSSEN 6-DOF vs TAU THAT vs ES-EKF)', ...
       '\bf\color[rgb]{0,0.3,0.8}Click & Keo chuot de XOAY 3D tu do  |  Cuon con lan chuot de ZOOM TO bat ky cho nao'}, ...
       'FontSize', 13);
legend(leg_handles, leg_labels, 'Location', 'northwest', 'FontSize', 9);

% Ti le mac dinh: do sau dốc tự nhiên (~12 độ)
daspect([1 1 0.25]);
view(42, 22);

%% CAC NUT BAM NHANH DIEU KHIEN GOC NHIN (UI BUTTONS)
% 1. Nut Toan canh
uicontrol('Style', 'pushbutton', 'String', 'Toan Canh (Reset)', ...
    'Position', [40, 15, 130, 32], 'FontWeight', 'bold', ...
    'Callback', @(~,~) btn_reset(ax));

% 2. Nut Nhin 3D Phoi Canh
uicontrol('Style', 'pushbutton', 'String', 'Goc 3D Phoi Canh', ...
    'Position', [180, 15, 130, 32], ...
    'Callback', @(~,~) btn_3d(ax));

% 3. Nut Mat bang 2D (North - East)
uicontrol('Style', 'pushbutton', 'String', 'Mat Bang 2D (Top)', ...
    'Position', [320, 15, 130, 32], ...
    'Callback', @(~,~) btn_top(ax));

% 4. Nut Trac doc do sau (East - Down)
uicontrol('Style', 'pushbutton', 'String', 'Trac Doc Do Sau (Side)', ...
    'Position', [460, 15, 140, 32], ...
    'Callback', @(~,~) btn_side(ax));

% 5. Nut Zoom Ve Dich
uicontrol('Style', 'pushbutton', 'String', 'Zoom Ve Dich (Xem Sai So)', ...
    'Position', [610, 15, 160, 32], 'ForegroundColor', [0.8 0 0], 'FontWeight', 'bold', ...
    'Callback', @(~,~) btn_zoom_end(ax));

% 6. Nut Zoom Khuc Lan
uicontrol('Style', 'pushbutton', 'String', 'Zoom Khuc Lan Dau', ...
    'Position', [780, 15, 140, 32], ...
    'Callback', @(~,~) btn_zoom_dive(ax));

% 7. Nut Ti Le Thuc 1:1:1
uicontrol('Style', 'pushbutton', 'String', 'Ti Le Thuc 1:1:1', ...
    'Position', [930, 15, 120, 32], ...
    'Callback', @(~,~) daspect(ax, [1 1 1]));

% 8. Nut Ti Le Do Sau Chuan
uicontrol('Style', 'pushbutton', 'String', 'Ti Le Do Sau Chuan', ...
    'Position', [1060, 15, 130, 32], ...
    'Callback', @(~,~) daspect(ax, [1 1 0.25]));

%% KICH HOAT SAN CHE DO XOAY 3D TRUC TIEP
rotate3d(fig, 'on');

% Luu ca 2 dinh dang FIG va PNG
fig_path = fullfile(script_dir, 'fig_quy_dao_3d_chi_tiet.png');
fig_matlab = fullfile(script_dir, 'fig_quy_dao_3d_chi_tiet.fig');
exportgraphics(ax, fig_path);
savefig(fig, fig_matlab);
drawnow;

fprintf('\n=== DA MO TAB QUY DAO 3D TOAN MAN HINH (KHONG SUBPLOT) ===\n');
fprintf('  + Ban co the click va keo chuot truc tiep tren Figure de xoay 3D.\n');
fprintf('  + Ban co the cuon con lan chuot de zoom to bat ky vi tri nao.\n');
fprintf('  + Cac nut bam duoi day cua so giup chuyen nhanh giua cac goc nhin.\n');

end

%% CAC HAM CALLBACK CHO NUT BAM
function btn_reset(ax)
    xlim(ax, 'auto');
    ylim(ax, 'auto');
    zlim(ax, 'auto');
    daspect(ax, [1 1 0.25]);
    view(ax, 42, 22);
end

function btn_3d(ax)
    view(ax, 42, 22);
end

function btn_top(ax)
    view(ax, 0, 90);
end

function btn_side(ax)
    view(ax, 0, 0);
end

function btn_zoom_end(ax)
    xlim(ax, [3600, 3850]);
    ylim(ax, [800, 920]);
    zlim(ax, [46, 54]);
    daspect(ax, [1 1 0.08]);
    view(ax, 30, 22);
end

function btn_zoom_dive(ax)
    xlim(ax, [500, 1600]);
    ylim(ax, [-120, 40]);
    zlim(ax, [0, 105]);
    daspect(ax, [1 1 0.25]);
    view(ax, 38, 22);
end
