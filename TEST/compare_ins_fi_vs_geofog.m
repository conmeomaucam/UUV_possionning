function compare_ins_fi_vs_geofog()
%% COMPARE_INS_FI_VS_GEOFOG - Đối soát ES-EKF khi dùng INS-DM-FI và GEO-FOG 3D Dual (cả hai KHÔNG từ kế)
% Đầu vào : ket_qua_esekf_INS_FI.mat, ket_qua_esekf_GEO_FOG.mat (từ run_fossen_esekf_integrated)
% Đầu ra  : fig_so_sanh_INS_FI_vs_GEO_FOG.png, bao_cao_INS_FI_vs_GEO_FOG.txt

clc; close all;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

models = {'INS_FI', 'GEO_FOG'};
colors = {[0.85 0.33 0.10], [0.00 0.45 0.74]};
R = struct([]);
for k = 1:2
    f = fullfile(script_dir, sprintf('ket_qua_esekf_%s.mat', models{k}));
    if ~exist(f, 'file')
        error('Chua co %s. Hay chay run_fossen_esekf_integrated(''%s'', ...) truoc.', f, models{k});
    end
    d = load(f);
    e = d.pos_true_hist - d.pos_est_hist;
    R(k).name = d.sensor_name;
    R(k).t = d.t(:);
    R(k).Tcut = d.T_gnss_cut;
    R(k).err = e;
    R(k).err3 = sqrt(sum(e.^2, 2));
    R(k).yaw_err = rad2deg(atan2(sin(d.euler_true_hist(:,3) - d.euler_est_hist(:,3)), ...
                                 cos(d.euler_true_hist(:,3) - d.euler_est_hist(:,3))));
    R(k).bgz_true = rad2deg(d.bg_true_hist(:,3)) * 3600;
    R(k).bgz_est  = rad2deg(d.bg_est_hist(:,3)) * 3600;
    R(k).bs_err = abs(rad2deg(d.boresight_est_hist(end,:)) - rad2deg(d.true_boresight(:)'));
    R(k).sc_err = abs(d.scale_est_hist(end) - d.true_scale) * 100;
    R(k).acc = d.acc_hist;
    dive = R(k).t > R(k).Tcut;
    seg = diff(d.pos_true_hist(dive, :));
    R(k).dist_dive = sum(sqrt(sum(seg.^2, 2)));
    R(k).dive = dive;
end

%% ĐỒ THỊ
fig = figure('Position', [60 60 1500 850], 'Visible', 'off');
ttl = {'Sai so North (m)', 'Sai so East (m)', 'Sai so Down (m)'};
for j = 1:3
    subplot(2, 3, j); hold on; grid on;
    for k = 1:2, plot(R(k).t, R(k).err(:, j), 'Color', colors{k}, 'LineWidth', 1.6); end
    xline(R(1).Tcut, 'm--', 'Cut GNSS');
    xlabel('Thoi gian (s)'); ylabel(ttl{j});
    title(['ES-EKF: ' ttl{j}], 'FontWeight', 'bold');
    legend(R(1).name, R(2).name, 'Location', 'best');
end
subplot(2, 3, 4); hold on; grid on;
for k = 1:2, plot(R(k).t, R(k).err3, 'Color', colors{k}, 'LineWidth', 2); end
xline(R(1).Tcut, 'm--', 'Cut GNSS');
xlabel('Thoi gian (s)'); ylabel('Sai so vi tri 3D (m)');
title('Tong sai so vi tri 3D (tau that - ES-EKF)', 'FontWeight', 'bold');
legend(R(1).name, R(2).name, 'Location', 'best');

subplot(2, 3, 5); hold on; grid on;
for k = 1:2, plot(R(k).t, R(k).yaw_err, 'Color', colors{k}, 'LineWidth', 1.6); end
xline(R(1).Tcut, 'm--', 'Cut GNSS');
xlabel('Thoi gian (s)'); ylabel('Sai so Yaw (deg)');
title('Sai so goc huong Yaw', 'FontWeight', 'bold');
legend(R(1).name, R(2).name, 'Location', 'best');

subplot(2, 3, 6); hold on; grid on;
for k = 1:2
    plot(R(k).t, R(k).bgz_true, '--', 'Color', colors{k}, 'LineWidth', 1.2);
    plot(R(k).t, R(k).bgz_est,  '-',  'Color', colors{k}, 'LineWidth', 1.6);
end
xline(R(1).Tcut, 'm--', 'Cut GNSS');
xlabel('Thoi gian (s)'); ylabel('Bias gyro truc z (deg/h)');
title('Bias con quay truc z: that (--) va uoc luong (-)', 'FontWeight', 'bold');
legend([R(1).name ' that'], [R(1).name ' uoc'], [R(2).name ' that'], [R(2).name ' uoc'], 'Location', 'best');

fig_path = fullfile(script_dir, 'fig_so_sanh_INS_FI_vs_GEO_FOG.png');
saveas(fig, fig_path); close(fig);

%% BẢNG SỐ
lines = {};
lines{end+1} = '==========================================================================================';
lines{end+1} = '  DOI SOAT ES-EKF: INS-DM-FI vs GEO-FOG 3D DUAL (ca hai KHONG tu ke, cung seed, cung quy dao)';
lines{end+1} = '==========================================================================================';
lines{end+1} = sprintf('| %-40s | %20s | %20s |', 'Chi tieu', 'INS-DM-FI', 'GEO-FOG 3D Dual');
lines{end+1} = '|------------------------------------------|----------------------|----------------------|';
row = @(name, a, b, fmt) sprintf(['| %-40s | %20' fmt ' | %20' fmt ' |'], name, a, b);
sel = @(k, tt) R(k).t >= tt - 1e-9;
lines{end+1} = row('Sai so 3D luc cat GNSS (m)', R(1).err3(find(R(1).dive,1)-1), R(2).err3(find(R(2).dive,1)-1), '.3f');
lines{end+1} = row('Sai so 3D cuoi hanh trinh (m)', R(1).err3(end), R(2).err3(end), '.2f');
lines{end+1} = row('Sai so 3D cuc dai (m)', max(R(1).err3), max(R(2).err3), '.2f');
lines{end+1} = row('Quang duong lan (m)', R(1).dist_dive, R(2).dist_dive, '.1f');
lines{end+1} = row('Do troi / quang duong lan (%)', 100*R(1).err3(end)/R(1).dist_dive, 100*R(2).err3(end)/R(2).dist_dive, '.3f');
lines{end+1} = row('Sai so North cuoi (m)', R(1).err(end,1), R(2).err(end,1), '.2f');
lines{end+1} = row('Sai so East cuoi (m)', R(1).err(end,2), R(2).err(end,2), '.2f');
lines{end+1} = row('Sai so Down cuoi (m)', R(1).err(end,3), R(2).err(end,3), '.3f');
lines{end+1} = row('Sai so Yaw luc cat GNSS (deg)', abs(R(1).yaw_err(find(R(1).dive,1)-1)), abs(R(2).yaw_err(find(R(2).dive,1)-1)), '.4f');
lines{end+1} = row('Sai so Yaw cuoi (deg)', abs(R(1).yaw_err(end)), abs(R(2).yaw_err(end)), '.4f');
lines{end+1} = row('|Sai so Yaw| cuc dai khi lan (deg)', max(abs(R(1).yaw_err(R(1).dive))), max(abs(R(2).yaw_err(R(2).dive))), '.4f');
lines{end+1} = row('Sai so bias gyro z luc cat GNSS (deg/h)', ...
    abs(R(1).bgz_true(find(R(1).dive,1)-1) - R(1).bgz_est(find(R(1).dive,1)-1)), ...
    abs(R(2).bgz_true(find(R(2).dive,1)-1) - R(2).bgz_est(find(R(2).dive,1)-1)), '.4f');
lines{end+1} = row('Sai so Boresight Roll (deg)', R(1).bs_err(1), R(2).bs_err(1), '.3f');
lines{end+1} = row('Sai so Boresight Pitch (deg)', R(1).bs_err(2), R(2).bs_err(2), '.3f');
lines{end+1} = row('Sai so Boresight Yaw (deg)', R(1).bs_err(3), R(2).bs_err(3), '.3f');
lines{end+1} = row('Sai so Scale DVL (%)', R(1).sc_err, R(2).sc_err, '.3f');
nm = {'GNSS', 'Depth', 'DVL'};
for j = 1:3
    a1 = R(1).acc(:, j); a2 = R(2).acc(:, j);
    lines{end+1} = row(sprintf('Ty le chap nhan %s (%%)', nm{j}), ...
        100*mean(a1(~isnan(a1))), 100*mean(a2(~isnan(a2))), '.2f'); %#ok<AGROW>
end
lines{end+1} = '==========================================================================================';
lines{end+1} = 'Tham chieu cau hinh cu (tham so khong theo datasheet: ARW 0.001 rad/sqrt(s), gyro RW 1e-5):';
lines{end+1} = '  sai so 3D cuoi 859.22 m, sai so Yaw cuoi 43.84 deg.';

for j = 1:numel(lines), fprintf('%s\n', lines{j}); end
fid = fopen(fullfile(script_dir, 'bao_cao_INS_FI_vs_GEO_FOG.txt'), 'w', 'n', 'UTF-8');
for j = 1:numel(lines), fprintf(fid, '%s\r\n', lines{j}); end
fclose(fid);
fprintf('Da luu: %s\n', fig_path);
end
