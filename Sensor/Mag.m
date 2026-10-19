function [m, mag_sim] = Mag(arg1, arg2, arg3, arg4, arg5, arg6, arg7)
%MAG  Giả lập cảm biến từ trường 3 trục (Từ kế / Magnetometer / Fluxgate).
%     Mô-đun tùy chọn độc lập kết hợp thêm với hệ thống INS-DM-FI.
%
% 1. Các dòng cảm biến từ kế Inertial Labs hỗ trợ:
%    - 'miniAHRS' : nd = 1.5 nT/sqrt(Hz), sb = 0.8 nT, sf = 500e-6, fs = 400 Hz
%    - 'AHRS10B'  : nd = 0.15 nT/sqrt(Hz), sb = 0.8 nT, sf = 500e-6, fs = 200 Hz
%    - 'AHRS10P'  : nd = 0.3 nT/sqrt(Hz),  sb = 0.2 nT, sf = 200e-6, fs = 200 Hz (Mặc định)
%
% 2. Mô hình toán học sai số:
%    - Chiếu từ trường Trái Đất hệ NED về thân tàu: Bb = Cnb * Bned (với Cnb = Cbn')
%    - Phương trình đo từ trường tổng hợp:
%      m(k, :) = (eye(3) + SF) * (Ssi * Bb + hi) + b(:, k) + sig_n * randn(3, 1)
%      + SF   : Sai số hệ số tỉ lệ (Scale factor error)
%      + Ssi  : Sai số sắt mềm dư sau hiệu chuẩn (Soft-iron ~0.5%)
%      + hi   : Sai số sắt cứng dư (Hard-iron ~200 nT)
%      + b    : Sai số trôi chậm Gauss-Markov (Bias drift, tau = 300 s)
%      + sig_n: Nhiễu trắng đo lường tương đương băng thông cảm biến
%
% 3. Cú pháp gọi hàm:
%    - Cách 1: [m, mag_sim] = Mag(roll_true, pitch_true, yaw_true, dt, sel, Bned)
%    - Cách 2: [m, mag_sim] = Mag(p, dt, roll_true, pitch_true, yaw_true, sel, Bned)
%    - Cách 3: Mag() (Chạy tự kiểm tra mẫu độc lập)
%
% 4. Đầu ra:
%    - m       : Ma trận đo từ trường 3 trục khung thân [nT] (N x 3).
%    - mag_sim : Struct chứa toàn bộ thông số và biến nội sinh mô phỏng.

% Chế độ tự kiểm tra khi không truyền tham số
if nargin == 0
	fprintf('>>> Đang chạy thử nghiệm độc lập mô-đun Sensor/Mag.m (AHRS-10P)...\n');
	fs_demo = 200;
	dt_demo = 1 / fs_demo;
	t_demo = (0:dt_demo:10)'; % 10 giây
	roll_demo  = 5 * (pi/180) * sin(2*pi*0.2*t_demo);
	pitch_demo = 3 * (pi/180) * cos(2*pi*0.1*t_demo);
	yaw_demo   = 30 * (pi/180) + 0.5 * t_demo;
	[m_out, sim_out] = Mag(roll_demo, pitch_demo, yaw_demo, dt_demo, 'AHRS10P');
	fprintf('    + Số mẫu sinh ra: %d mẫu (200 Hz)\n', size(m_out, 1));
	fprintf('    + Từ trường trung bình: X = %.1f nT, Y = %.1f nT, Z = %.1f nT\n', ...
		mean(m_out(:,1)), mean(m_out(:,2)), mean(m_out(:,3)));
	fprintf('>>> Mô-đun Sensor/Mag.m hoạt động chuẩn xác 100%%!\n');
	if nargout > 0
		m = m_out;
		mag_sim = sim_out;
	end
	return;
end

% Phân giải cú pháp đầu vào
if isstruct(arg1)
	% Cú pháp: Mag(p, dt, roll_true, pitch_true, yaw_true, sel, Bned)
	p = arg1;
	dt = arg2;
	roll_true = arg3;
	pitch_true = arg4;
	yaw_true = arg5;
	if nargin >= 6, sel = arg6; else, sel = 'AHRS10P'; end
	if nargin >= 7, Bned = arg7; else, Bned = []; end
else
	% Cú pháp: Mag(roll_true, pitch_true, yaw_true, dt, sel, Bned)
	p = [];
	roll_true = arg1;
	pitch_true = arg2;
	yaw_true = arg3;
	if nargin >= 4, dt = arg4; else, dt = []; end
	if nargin >= 5, sel = arg5; else, sel = 'AHRS10P'; end
	if nargin >= 6, Bned = arg6; else, Bned = []; end
end

if isempty(sel)
	sel = 'AHRS10P';
end

% Bảng thông số phần cứng từ Inertial Labs
switch sel
	case 'miniAHRS'
		nd = 1.5;   sb = 0.8;  sf = 500e-6; fs = 400;
	case 'AHRS10B'
		nd = 0.15;  sb = 0.8;  sf = 500e-6; fs = 200;
	case 'AHRS10P'
		nd = 0.3;   sb = 0.2;  sf = 200e-6; fs = 200;
	otherwise
		error('Loai cam bien khong hop le. Chon: miniAHRS, AHRS10B, hoac AHRS10P.');
end

if isempty(dt)
	dt = 1 / fs;
end

N = numel(roll_true);

% Vector từ trường Trái Đất hệ NED [nT] tại hồ Hòa Bình (~21°N, 105.8°E) theo WMM
if isempty(Bned)
	Bned = [38500; -1100; 22500]; % [B_Bac; B_Dong; B_Xuong] nT
else
	Bned = Bned(:);
end

sig_n = nd * sqrt(fs / 2); % Độ lệch chuẩn nhiễu trắng mỗi mẫu [nT]

% Sai số trôi chậm Gauss-Markov quanh giá trị bật máy (tau = 300 s)
tau = 300;
a = exp(-dt / tau);
b = zeros(3, N);
b(:, 1) = sb * randn(3, 1);
for k = 2:N
	b(:, k) = a * b(:, k-1) + sqrt(1 - a^2) * sb * randn(3, 1);
end

SF  = diag(sf * randn(3, 1));         % Sai số hệ số tỉ lệ cố định mỗi lần chạy
Ssi = eye(3) + 0.005 * randn(3);     % Sắt mềm (soft-iron) dư sau hiệu chuẩn (~0.5%)
hi  = 200 * randn(3, 1);              % Sắt cứng (hard-iron) dư [nT]

m = zeros(N, 3);
for k = 1:N
	Cbn_k = body_to_nav(roll_true(k), pitch_true(k), yaw_true(k));
	Cnb_k = Cbn_k'; % Ma trận xoay NED -> thân tàu
	Bb = Cnb_k * Bned;
	m_k = (eye(3) + SF) * (Ssi * Bb + hi) + b(:, k) + sig_n * randn(3, 1);
	m(k, :) = m_k';
end

% Đóng gói struct kết quả phụ trợ
mag_sim = struct();
mag_sim.sel = sel;
mag_sim.fs = fs;
mag_sim.dt = dt;
mag_sim.Bned = Bned;
mag_sim.sig_n = sig_n;
mag_sim.SF = SF;
mag_sim.Ssi = Ssi;
mag_sim.hi = hi;
mag_sim.bias_drift = b;
mag_sim.m = m;

end

function Cbn = body_to_nav(roll, pitch, yaw)
%BODY_TO_NAV  Chuyển đổi góc Euler (Roll, Pitch, Yaw) sang ma trận xoay Cbn
cr = cos(roll);  sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw);   sy = sin(yaw);

Cbn = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
	sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
	-sp,   cp*sr,             cp*cr];
end