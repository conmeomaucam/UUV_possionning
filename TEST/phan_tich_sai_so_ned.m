function phan_tich_sai_so_ned(mat_filename)
%PHAN_TICH_SAI_SO_NED Phân tích sai số quỹ đạo (m) theo hệ tọa độ NED từ file .mat
%
% Cú pháp:
%   phan_tich_sai_so_ned()                     % Tự động tìm ket_qua_esekf_VN_210.mat hoặc integrated
%   phan_tich_sai_so_ned('ket_qua_esekf_VN_210.mat')

if nargin < 1 || isempty(mat_filename)
    mat_filename = 'ket_qua_esekf_VN_210.mat';
end

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

mat_path = fullfile(script_dir, mat_filename);
if ~exist(mat_path, 'file')
    % Thử tìm trong thư mục hiện tại
    mat_path = mat_filename;
    if ~exist(mat_path, 'file')
        mat_path = fullfile(script_dir, 'ket_qua_fossen_esekf_integrated.mat');
    end
end

if ~exist(mat_path, 'file')
    error('Khong tim thay file .mat tai: %s', mat_path);
end

fprintf('Dang doc du lieu tu: %s\n', mat_path);
d = load(mat_path);

t = d.t(:);
pos_true = d.pos_true_hist;  % [N, E, D] Thật
pos_est  = d.pos_est_hist;   % [N, E, D] ES-EKF ước lượng

if isfield(d, 'T_gnss_cut')
    T_cut = d.T_gnss_cut;
else
    T_cut = 480.0;
end

if isfield(d, 'sensor_name')
    s_name = d.sensor_name;
elseif isfield(d, 'sensor_model')
    s_name = d.sensor_model;
else
    s_name = 'Khong xac dinh';
end

% 1. Tính sai số theo từng trục NED
err_N = pos_true(:, 1) - pos_est(:, 1);
err_E = pos_true(:, 2) - pos_est(:, 2);
err_D = pos_true(:, 3) - pos_est(:, 3);

err_2D = sqrt(err_N.^2 + err_E.^2);
err_3D = sqrt(err_N.^2 + err_E.^2 + err_D.^2);

% Phân tách 2 giai đoạn: Nổi (có GNSS) vs Lặn (Mất GNSS)
idx_gnss = (t <= T_cut);
idx_dive = (t > T_cut);

% 2. Thống kê RMS, Mean, Max, Final
rms_err = @(x) sqrt(mean(x.^2));
max_abs = @(x) max(abs(x));

% Toàn bộ hành trình
rms_N = rms_err(err_N);
rms_E = rms_err(err_E);
rms_D = rms_err(err_D);
rms_3D = rms_err(err_3D);

max_N = max_abs(err_N);
max_E = max_abs(err_E);
max_D = max_abs(err_D);
max_3D = max(err_3D);

final_N = err_N(end);
final_E = err_E(end);
final_D = err_D(end);
final_3D = err_3D(end);

% Giai đoạn nổi (t <= T_cut)
if any(idx_gnss)
    rms_N_gnss = rms_err(err_N(idx_gnss));
    rms_E_gnss = rms_err(err_E(idx_gnss));
    rms_D_gnss = rms_err(err_D(idx_gnss));
    rms_3D_gnss = rms_err(err_3D(idx_gnss));
    max_3D_gnss = max(err_3D(idx_gnss));
else
    rms_N_gnss = 0; rms_E_gnss = 0; rms_D_gnss = 0; rms_3D_gnss = 0; max_3D_gnss = 0;
end

% Giai đoạn lặn sâu (t > T_cut)
if any(idx_dive)
    rms_N_dive = rms_err(err_N(idx_dive));
    rms_E_dive = rms_err(err_E(idx_dive));
    rms_D_dive = rms_err(err_D(idx_dive));
    rms_3D_dive = rms_err(err_3D(idx_dive));
    max_3D_dive = max(err_3D(idx_dive));
else
    rms_N_dive = 0; rms_E_dive = 0; rms_D_dive = 0; rms_3D_dive = 0; max_3D_dive = 0;
end

% Đọc thêm so sánh với Fossen Standalone nếu có
standalone_mat = fullfile(script_dir, 'ket_qua_fossen_standalone.mat');
has_standalone = exist(standalone_mat, 'file');
if has_standalone
    f_data = load(standalone_mat);
    pos_fossen = f_data.pos_history;
    diff_N = pos_true(:, 1) - pos_fossen(:, 1);
    diff_E = pos_true(:, 2) - pos_fossen(:, 2);
    diff_D = pos_true(:, 3) - pos_fossen(:, 3);
    diff_3D = sqrt(diff_N.^2 + diff_E.^2 + diff_D.^2);
end

% 3. Xuất báo cáo ra Command Window và File text
report_file = fullfile(script_dir, 'bao_cao_sai_so_ned.txt');
fid = fopen(report_file, 'w', 'n', 'UTF-8');

out_str = sprintf([...
    '========================================================================================\n', ...
    '  KET QUA PHAN TICH SAI SO QUY DAO THEO HE TRUC TOA DO NED\n', ...
    '========================================================================================\n', ...
    'File du lieu : %s\n', ...
    'Cam bien     : %s\n', ...
    'Tong thoi gian: %.1f giay (Tong so buoc: %d)\n', ...
    'Thoi diem cat GNSS (Bat dau lan): %.1f giay\n\n', ...
    '--- 1. SAI SO CUOI CUNG TAI DIEM KET THUC HANH TRINH (t = %.1f s) ---\n', ...
    '  * Truc North (Bac)   : %+8.4f m\n', ...
    '  * Truc East (Dong)   : %+8.4f m\n', ...
    '  * Truc Down (Do sau) : %+8.4f m\n', ...
    '  * Tong hop sai so 3D : %8.4f m\n', ...
    '  * Sai so mat phang 2D: %8.4f m\n\n', ...
    '--- 2. SAI SO CAN TRUNG BINH BINH PHUONG (RMS ERROR) TOAN HANH TRINH ---\n', ...
    '  * RMS North (Bac)    : %8.4f m  | Max tuyet doi: %8.4f m\n', ...
    '  * RMS East (Dong)    : %8.4f m  | Max tuyet doi: %8.4f m\n', ...
    '  * RMS Down (Do sau)  : %8.4f m  | Max tuyet doi: %8.4f m\n', ...
    '  * RMS Tong hop 3D    : %8.4f m  | Max tuyet doi: %8.4f m\n\n', ...
    '--- 3. PHAN TICH THEO 2 GIAI DOAN HOAT DONG ---\n', ...
    'A. Giai doan noi tren mat nuoc (co GNSS, t <= %.0f s):\n', ...
    '   - RMS North / East / Down : %6.4f m / %6.4f m / %6.4f m\n', ...
    '   - RMS 3D                  : %6.4f m  (Max 3D: %6.4f m)\n', ...
    'B. Giai doan lan ngam duoi bien (mat GNSS hoan toan, t > %.0f s):\n', ...
    '   - RMS North / East / Down : %6.4f m / %6.4f m / %6.4f m\n', ...
    '   - RMS 3D                  : %6.4f m  (Max 3D: %6.4f m)\n\n'], ...
    mat_filename, s_name, t(end), length(t), T_cut, ...
    t(end), final_N, final_E, final_D, final_3D, sqrt(final_N^2 + final_E^2), ...
    rms_N, max_N, rms_E, max_E, rms_D, max_D, rms_3D, max_3D, ...
    T_cut, rms_N_gnss, rms_E_gnss, rms_D_gnss, rms_3D_gnss, max_3D_gnss, ...
    T_cut, rms_N_dive, rms_E_dive, rms_D_dive, rms_3D_dive, max_3D_dive);

if has_standalone
    out_str = [out_str, sprintf([...
        '--- 4. SO SANH VOI QUY DAO LY TUONG (FOSSEN STANDALONE) ---\n', ...
        '  * Do lech vi tri cuoi North / East / Down: %+8.4f m / %+8.4f m / %+8.4f m\n', ...
        '  * Do lech 3D cuoi cung                  : %8.4f m\n', ...
        '  * Do lech 3D lon nhat tren hanh trinh   : %8.4f m\n', ...
        '========================================================================================\n'], ...
        diff_N(end), diff_E(end), diff_D(end), diff_3D(end), max(diff_3D))];
else
    out_str = [out_str, sprintf('========================================================================================\n')];
end

fprintf('%s', out_str);
if fid ~= -1
    fwrite(fid, out_str);
    fclose(fid);
    fprintf('Da luu bao cao chi tiet vao: %s\n', report_file);
end

end
