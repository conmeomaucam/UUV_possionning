function p = thong_so_he_thong_chuan()
%THONG_SO_HE_THONG_CHUAN  Thông số DVL A125, INS-DM-FI, Trái Đất và kịch bản mô phỏng.
%
% 1. DVL A125 (Water Linked): datasheet WL-21037-2, DVL A125 Datasheet v7.
% 2. INS-DM-FI (Inertial Labs): datasheet INS-DM-FI rev 1.8 (01/2025), IMU-FI-200T.
% 3. Trái Đất: WGS-84.
% Số nào không có trong datasheet thì ghi GIẢ ĐỊNH.
%
% Gọi:  p = thong_so_he_thong_chuan();

%% ==================== 1. DVL A125 (Water Linked) ====================
p.dvl.ten = 'DVL A125';
p.dvl.hang = 'Water Linked';
p.dvl.datasheet = 'WL-21037-2 v7';

p.dvl.tan_so_phat_Hz = 420e3;        % Tần số sóng siêu âm 420 kHz
p.dvl.so_tia = 4;                    % 4 tia Janus
p.dvl.goc_beam_deg = 22.5;           % Góc tia so với trục đứng đầu đo
p.dvl.do_rong_bup_deg = 4.1;         % Độ rộng búp sóng

p.dvl.ping_Hz = 5;                   % GIẢ ĐỊNH 5 Hz, datasheet cho dải 2-15 Hz
p.dvl.van_toc_toi_da_mps = 9.0;      % Trần vận tốc bottom-lock
p.dvl.do_cao_min_m = 0.05;           % Độ cao cách đáy tối thiểu
p.dvl.do_cao_max_m = 125.0;          % Độ cao cách đáy tối đa

p.dvl.phan_giai_mps = 0.1e-3;        % Độ phân giải vận tốc 0.1 mm/s
p.dvl.do_chinh_xac_tieu_chuan = 0.001; % ±0.1% dài hạn, bản Performance
p.dvl.san_nhieu_mps = 0.1e-3;        % GIẢ ĐỊNH sàn nhiễu = phân giải
% Lưu ý: ±0.1% là độ chính xác dài hạn, KHÔNG phải nhiễu từng ping.
% Dùng làm nhiễu từng ping là GIẢ ĐỊNH.

% Đáp án ẩn cho mô phỏng. GIẢ ĐỊNH, không có trong datasheet.
p.dvl.k_that = 0.01;                          % 1%
p.dvl.boresight_that_deg = [0.2; -0.3; 1.0];  % [Alpha roll; Beta pitch; Gamma yaw], độ

%% ==================== 2. INS-DM-FI (Inertial Labs) ====================
p.ins.ten = 'INS-DM-FI';
p.ins.hang = 'Inertial Labs';
p.ins.imu = 'IMU-FI-200T (FOG vòng kín + gia tốc kế MEMS)';
p.ins.datasheet = 'INS-DM-FI rev 1.8 (01/2025)';

% 2.1 Gyro FOG
p.ins.gyro.dai_do_deg_s = 490.0;
p.ins.gyro.tan_so_Hz = 400.0;
p.ins.gyro.bang_thong_Hz = 200.0;
p.ins.gyro.arw_deg_sqrt_h = 0.025;            % ARW, typical
p.ins.gyro.bias_in_run_deg_h = 0.025;         % KHÔNG DÙNG: datasheet IMU không công bố, chỉ trang bán hàng ghi
p.ins.gyro.bias_repeatability_deg_h = 0.5;    % Bias lặp lại trên dải nhiệt
p.ins.gyro.scale_factor_ppm = 100.0;
p.ins.gyro.cross_axis_mrad = 0.1;

% 2.2 Gia tốc kế MEMS
p.ins.acc.dai_do_g = 8.0;
p.ins.acc.tan_so_Hz = 400.0;
p.ins.acc.bang_thong_Hz = 260.0;
p.ins.acc.vrw_mps_sqrt_h = 0.015;             % VRW, typical, dải ±8 g
p.ins.acc.bias_in_run_mg = 0.005;             % Allan, RMS, dải ±8 g
p.ins.acc.tau_in_run_s = 100;                 % GIẢ ĐỊNH thời hằng trôi chậm
p.ins.acc.bias_repeatability_mg = 0.5;        % Bias lặp lại trên dải nhiệt, ±8 g
p.ins.acc.scale_factor_ppm = 150.0;
p.ins.acc.cross_axis_mrad = 0.2;

%% ==================== 3. TRÁI ĐẤT VÀ KỊCH BẢN ====================
p.dat.omega_ie = 7.292115e-5;     % Tốc độ quay Trái Đất, rad/s
p.dat.a = 6378137.0;              % Bán trục lớn WGS-84, m
p.dat.e2 = 6.69437999014e-3;      % Bình phương tâm sai WGS-84
p.dat.vi_do_deg = 21.0;           % GIẢ ĐỊNH
p.dat.kinh_do_deg = 105.8;        % GIẢ ĐỊNH
p.dat.do_cao_m = -10;             % GIẢ ĐỊNH, âm = dưới mặt nước
p.dat.g = 9.80665;                % Hằng số, mô phỏng và INS dùng chung

p.kich_ban.t_can_chinh_s = 120;   % GIẢ ĐỊNH: đứng yên căn chỉnh thô
p.kich_ban.t_tang_toc_s = 10;     % 0 -> van_toc_mps
p.kich_ban.t_thang_s = 1800;      % 30 phút chạy thẳng (1800 giây)
p.kich_ban.t_quay_s = 20;         % quay vòng
p.kich_ban.van_toc_mps = 1.5;
p.kich_ban.toc_do_quay_deg_s = 6;
p.kich_ban.huong_dau_deg = 30;    % GIẢ ĐỊNH hướng ban đầu

%% ==================== 4. ĐỔI SANG SI ====================
g = p.dat.g;
p.si.dt_ins = 1.0 / p.ins.gyro.tan_so_Hz;
p.si.dt_dvl = 1.0 / p.dvl.ping_Hz;

% Gyro
p.si.gyro_arw = p.ins.gyro.arw_deg_sqrt_h * (pi/180) / sqrt(3600);          % rad/sqrt(s)
p.si.gyro_bias_repeat = p.ins.gyro.bias_repeatability_deg_h * (pi/180) / 3600; % rad/s
p.si.gyro_scale_err = p.ins.gyro.scale_factor_ppm * 1e-6;
p.si.gyro_cross_axis = p.ins.gyro.cross_axis_mrad * 1e-3;                   % rad

% Gia tốc
p.si.acc_vrw = p.ins.acc.vrw_mps_sqrt_h / sqrt(3600);                      % m/s/sqrt(s)
p.si.acc_bias_in_run = p.ins.acc.bias_in_run_mg * 1e-3 * g;                % m/s^2
p.si.acc_bias_repeat = p.ins.acc.bias_repeatability_mg * 1e-3 * g;         % m/s^2
p.si.acc_scale_err = p.ins.acc.scale_factor_ppm * 1e-6;
p.si.acc_cross_axis = p.ins.acc.cross_axis_mrad * 1e-3;                    % rad

%  GIẢ ĐỊNH: coi số datasheet là 1 sigma.
p.si.gyro_bias_sigma = p.si.gyro_bias_repeat;      % bias bật máy, hằng trong 1 lần chạy
p.si.acc_bias_sigma = p.si.acc_bias_repeat;        % bias bật máy, hằng trong 1 lần chạy
p.si.acc_bias_inrun_sigma = p.si.acc_bias_in_run;  % trôi chậm Gauss-Markov
p.si.acc_bias_tau = p.ins.acc.tau_in_run_s;

% DVL
p.si.k_that = p.dvl.k_that;
p.si.boresight_that_rad = p.dvl.boresight_that_deg * (pi/180);
p.si.sigma_dvl_fn = @(v_mps) p.dvl.do_chinh_xac_tieu_chuan * abs(v_mps) + p.dvl.san_nhieu_mps;

end
