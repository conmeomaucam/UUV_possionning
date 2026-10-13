function p = thong_so_he_thong_chuan()
%THONG_SO_HE_THONG_CHUAN  Thông số DVL A125, INS-DM-FI, Trái Đất và kịch bản mô phỏng.
%
% 1. DVL A125 (Water Linked): datasheet WL-21037-2, DVL A125 Datasheet v7.
% 2. INS-DM-FI (Inertial Labs): datasheet INS-DM-FI rev 1.8 (01/2025), IMU-FI-200T.
% 3. Trái Đất: WGS-84.
% 4. Chiết áp xoay CTS Series 282 (mã 282TAAR103A25B1): datasheet
%    282TAAR103A25B1.PDF (CTS Electrocomponents, 2013).
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

% Mô hình nhiễu DVL mức thực tế (dùng trong dithang.m)
% Datasheet A125 KHÔNG công bố độ lệch chuẩn từng ping. Các số dưới đây
% là GIẢ ĐỊNH cỡ DVL 400-600 kHz khi bám đáy tốt; có thể đối chiếu bằng
% trường "fom" (m/s) mà A125 trả về theo từng ping khi chạy thật.
% (a) Nhiễu trắng từng ping, mỗi trục: sigma = san + ti_le * |v|
p.dvl.ping_noise_san_mps = 5e-3;     % GIẢ ĐỊNH 5 mm/s
p.dvl.ping_noise_ti_le   = 2e-3;     % GIẢ ĐỊNH 0.2 % |v|  (=> ~8 mm/s ở 1.5 m/s)
% (b) Sai số tương quan chậm (Gauss-Markov) - đại diện cho ±0.1 % dài hạn:
%     thay đổi theo loại đáy, nhiệt độ/độ mặn (tốc độ âm)... Bộ lọc KHÔNG mô hình hóa.
p.dvl.gm_ti_le_sigma = p.dvl.do_chinh_xac_tieu_chuan; % 0.1 % |v|, coi là 1-sigma (GIẢ ĐỊNH)
p.dvl.gm_tau_s       = 300;          % GIẢ ĐỊNH thời hằng tương quan 300 s

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

%% ==================== 4. CHIẾT ÁP XOAY CTS SERIES 282 ====================
% Cảm biến GÓC XOAY (chiết áp nhựa dẫn điện), KHÔNG phải cảm biến độ sâu.
% Trên UUV dùng hợp lý nhất làm phản hồi góc bánh lái / cánh lặn (GIẢ ĐỊNH ứng dụng).
p.pot.ten = 'Series 282 Precision Potentiometer';
p.pot.hang = 'CTS Electrocomponents';
p.pot.datasheet = '282TAAR103A25B1.PDF (2013)';
p.pot.ma_dat_hang = '282TAAR103A25B1';

% 4.1 Giải mã mã đặt hàng (trang Ordering Information)
%   282 | T | A | A | R | 103 | A | 2 | 5 | B | 1
p.pot.ma.series        = '282';          % 16 mm, nhựa dẫn điện
p.pot.ma.chan          = 'T';            % Solder Lug (Faston)
p.pot.ma.bac_ren       = 'A';            % Bushing length 5.0 mm
p.pot.ma.chieu_dai_truc = 'A';           % Shaft length L = 10 mm
p.pot.ma.dang_truc     = 'R';            % Trục tròn, đường kính 6 mm
p.pot.ma.dien_tro      = '103';          % 10 kOhm
p.pot.ma.dung_sai_R    = 'A';            % +-10 %
p.pot.ma.tuyen_tinh    = '2';            % +-2 %
p.pot.ma.ip            = '5';            % IP54
p.pot.ma.goc_xoay      = 'B';            % Tổng 300 độ / hiệu dụng 280 độ
p.pot.ma.vau_dinh_vi   = '1';            % Vấu định vị trái, 10 mm

% 4.2 Thông số của đúng mã 282TAAR103A25B1
p.pot.dien_tro_ohm        = 10e3;        % 10 kOhm
p.pot.dung_sai_dien_tro   = 0.10;        % +-10 %
p.pot.tuyen_tinh          = 0.02;        % +-2 % (Linearity)
p.pot.goc_tong_deg        = 300;         % Total rotation angle
p.pot.goc_hieu_dung_deg   = 280;         % Effective electrical angle
p.pot.ip_rating           = 'IP54';
p.pot.duong_kinh_truc_mm  = 6.0;
p.pot.chieu_dai_truc_mm   = 10.0;
p.pot.chieu_dai_bac_mm    = 5.0;
p.pot.ren_bac             = 'M9 x 0.75P (metric) / 3/8"-32 UNEF (imperial)';
p.pot.vau_dinh_vi         = 'Trái, 10 mm';

% 4.3 Thông số chung của Series 282 (Electrical and Mechanical Specifications)
p.pot.dai_dien_tro_ohm    = [1e3, 500e3]; % 1 K - 500 K, linear taper
p.pot.cong_suat_W         = 0.25;         % @ 55 độ C
p.pot.dien_ap_cach_dien_VDC = 750;        % Dielectric strength, 1 phút
p.pot.dien_ap_max_VDC     = 500;          % Maximum operating voltage
p.pot.dien_tro_cach_dien_ohm = 1000e6;    % >= 1000 MOhm @ 500 VDC
p.pot.cac_goc_xoay_deg    = [250, 300, 360]; % Các tuỳ chọn tổng góc
p.pot.cac_goc_hieu_dung_deg = [240, 280, 340]; % Tương ứng mã A, B, C
p.pot.mo_men_xoay_gfcm    = [3, 50];      % Rotational torque
p.pot.do_ben_chan_kgfcm   = 9;            % Stop strength, tối thiểu
p.pot.luc_keo_day_truc_kg = 13.6;         % Push-pull, 10 s
p.pot.lap_dat             = 'Bushing Mount';
p.pot.tuoi_tho_vong       = 2e6;          % 2 triệu vòng xoay
p.pot.nhiet_do_hoat_dong_C = [-40, 125];
p.pot.nhiet_do_luu_kho_C  = [-55, 150];
p.pot.chiu_nhiet_han      = '350 độ C trong 5 s';
p.pot.rung_dong           = '15 G, 10 - 2000 Hz';
p.pot.cac_cap_ip          = {'IP54', 'IP67'};

% Lưu ý: IP54 chỉ chống bụi và nước bắn, KHÔNG chịu ngâm nước.
% Trên UUV phải đặt trong vỏ kín áp hoặc chọn bản IP67 (mã '6') kèm vỏ bảo vệ.

% 4.4 Mô hình đo cho mô phỏng (GIẢ ĐỊNH, datasheet không cho)
p.pot.adc_bit = 12;               % GIẢ ĐỊNH: ADC 12 bit đọc điện áp con trượt

%% ==================== 5. ĐỔI SANG SI ====================
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
% Nhiễu trắng từng ping mức thực tế (m/s, mỗi trục)
p.si.sigma_dvl_ping_fn = @(v_mps) p.dvl.ping_noise_san_mps + p.dvl.ping_noise_ti_le * abs(v_mps);

% Chiết áp CTS 282
p.si.pot_goc_hieu_dung = p.pot.goc_hieu_dung_deg * (pi/180);              % rad
% Sai số tuyến tính +-2 % tính trên góc hiệu dụng: 0.02 * 280 = +-5.6 độ (giới hạn)
p.si.pot_loi_tuyen_tinh_max = p.pot.tuyen_tinh * p.si.pot_goc_hieu_dung;  % rad
p.si.pot_luong_tu = p.si.pot_goc_hieu_dung / 2^p.pot.adc_bit;              % rad / LSB
% GIẢ ĐỊNH: lỗi tuyến tính có dạng cong tĩnh hình sin, biên độ bằng giới hạn datasheet
%   goc_do = goc_that + loi_max * sin(pi * goc_that / goc_hieu_dung), rồi lượng tử ADC
p.si.pot_do_fn = @(goc_rad) round((goc_rad + p.si.pot_loi_tuyen_tinh_max * ...
    sin(pi * goc_rad / p.si.pot_goc_hieu_dung)) / p.si.pot_luong_tu) * p.si.pot_luong_tu;

end
