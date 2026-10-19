function [gyro_meas, accel_meas, varargout] = INS_FI(p, dt, velocity_true_ned, position_true, ...
	roll_true, pitch_true, yaw_true, measurement_indices, ...
	sigma_gps_pos, sigma_gps_vel, sigma_gps_yaw)
%INS_FI  Giả lập hệ thống dẫn đường quán tính tích hợp vệ tinh INS-DM-FI (Inertial Labs).
%
% 1. Thành phần phần cứng INS-DM-FI:
%    - Cụm cảm biến quán tính IMU-FI-200T (400 Hz):
%      + Con quay sợi quang vòng kín (Closed-loop FOG): ARW, bias tĩnh, scale error, cross-axis.
%      + Cụm 3 gia tốc kế vi cơ MEMS: VRW, trôi dạt in-run Gauss-Markov, bias tĩnh, scale error.
%      + Giới hạn bão hòa cảm biến (±490 deg/s và ±8 g).
%    - Bộ thu vệ tinh GNSS 2 ăng-ten tích hợp sẵn trong thiết bị (5 Hz):
%      + Định vị vị trí 3 chiều hệ NED (độ chính xác RTK 2 cm).
%      + Đo vận tốc chuyển động 3 chiều hệ NED (độ chính xác 2 cm/s).
%      + Đo góc hướng la bàn vệ tinh 2 ăng-ten Dual-antenna GNSS Heading (độ chính xác 0.05°).
%    - Tùy chọn mở rộng: Cảm biến từ trường Magnetometer được tách thành mô-đun tùy chọn độc lập tại Sensor/Mag.m để kết hợp thêm với INS-DM-FI khi cần.
%
% 2. Cú pháp sử dụng:
%    - Đầy đủ (xuất riêng rẽ cả IMU và GNSS):
%      [gyro_meas, accel_meas, Z_gps_pos, Z_gps_vel, Z_gps_yaw, ins_sim] = INS_FI(p, dt, ...
%          velocity_true_ned, position_true, roll_true, pitch_true, yaw_true, measurement_indices, ...
%          sigma_gps_pos, sigma_gps_vel, sigma_gps_yaw);
%
%    - Rút gọn (GNSS được đóng gói trong ins_sim):
%      [gyro_meas, accel_meas, ins_sim] = INS_FI(p, dt, velocity_true_ned, position_true, ...
%          roll_true, pitch_true, yaw_true, measurement_indices);
%
% 3. Đầu ra:
%    - gyro_meas         : Vận tốc góc đo được từ con quay FOG (N x 3, rad/s).
%    - accel_meas        : Lực riêng đo được từ gia tốc kế MEMS (N x 3, m/s^2).
%    - Z_gps_pos         : Tọa độ vị trí đo được từ GNSS (N_meas x 3, m).
%    - Z_gps_vel         : Vận tốc đo được từ GNSS (N_meas x 3, m/s).
%    - Z_gps_yaw         : Góc hướng la bàn đo được từ GNSS 2 ăng-ten (N_meas x 1, rad).
%    - ins_sim           : Struct chứa toàn bộ thông số đánh giá và phụ trợ.

% -------------------------------------------------------------------------
% 0. XỬ LÝ ĐỐI SỐ ĐẦU VÀO TƯƠNG THÍCH ĐA DẠNG CÚ PHÁP
% -------------------------------------------------------------------------
has_gnss = false;
if nargin >= 4 && size(position_true, 2) == 3
	% Cú pháp đầy đủ: tham số thứ 4 là position_true
	if nargin >= 8 && ~isempty(measurement_indices)
		has_gnss = true;
	end
elseif nargin >= 4 && size(position_true, 2) == 1
	% Cú pháp chỉ IMU: tham số 4 là roll, 5 là pitch, 6 là yaw
	yaw_true = pitch_true;
	pitch_true = roll_true;
	roll_true = position_true;
	position_true = [];
	has_gnss = false;
end

N = size(velocity_true_ned, 1);
g = p.dat.g;
latitude = p.dat.vi_do_deg * pi/180;
height = p.dat.do_cao_m;
sin_lat_sq = sin(latitude)^2;
RN = p.dat.a / sqrt(1 - p.dat.e2 * sin_lat_sq);
RM = p.dat.a * (1 - p.dat.e2) / (1 - p.dat.e2 * sin_lat_sq)^1.5;
w_ie = p.dat.omega_ie * [cos(latitude); 0; -sin(latitude)];
w_en = zeros(N, 3);
w_ib_true = zeros(N, 3);
specific_force_true = zeros(N, 3);

roll_rate = gradient(roll_true, dt);
pitch_rate = gradient(pitch_true, dt);
yaw_rate_profile = gradient(unwrap(yaw_true), dt);

% -------------------------------------------------------------------------
% 1. TÍNH LỰC RIÊNG VÀ VẬN TỐC GÓC THỰC TẾ (GROUND TRUTH MECHANICS)
% -------------------------------------------------------------------------
for i = 1:N
	prev = max(i - 1, 1);
	vn_prev = velocity_true_ned(prev, :)';

	w_en_i = [vn_prev(2) / (RN + height); ...
		-vn_prev(1) / (RM + height); ...
		-vn_prev(2) * tan(latitude) / (RN + height)];
	w_en(i, :) = w_en_i';

	phi_dot = roll_rate(i);
	theta_dot = pitch_rate(i);
	psi_dot = yaw_rate_profile(i);
	body_rate_relative = [phi_dot - psi_dot*sin(pitch_true(i)); ...
		theta_dot*cos(roll_true(i)) + psi_dot*sin(roll_true(i))*cos(pitch_true(i)); ...
		-theta_dot*sin(roll_true(i)) + psi_dot*cos(roll_true(i))*cos(pitch_true(i))];

	Cbn = body_to_nav(roll_true(i), pitch_true(i), yaw_true(i));
	Cnb = Cbn';
	w_ib_true(i, :) = (body_rate_relative + Cnb * (w_ie + w_en_i))';

	acceleration_n = (velocity_true_ned(i, :)' - vn_prev) / dt;
	coriolis = cross(2*w_ie + w_en_i, vn_prev);
	specific_force_true(i, :) = (Cnb * (acceleration_n + coriolis - [0; 0; g]))';
end

% -------------------------------------------------------------------------
% 2. CỘNG MÔ HÌNH NHIỄU PHẦN CỨNG IMU-FI-200T (FOG + MEMS)
% -------------------------------------------------------------------------
s = p.si;
bg = s.gyro_bias_sigma * randn(3, 1);
ba = s.acc_bias_sigma * randn(3, 1);
Kg = eye(3) + diag(s.gyro_scale_err * randn(3, 1)) + ...
	s.gyro_cross_axis * (randn(3) .* ~eye(3));
Ka = eye(3) + diag(s.acc_scale_err * randn(3, 1)) + ...
	s.acc_cross_axis * (randn(3) .* ~eye(3));

phi_acc = exp(-dt / s.acc_bias_tau);
q_acc = s.acc_bias_inrun_sigma * sqrt(1 - phi_acc^2);
b_acc = s.acc_bias_inrun_sigma * randn(3, 1);

gyro_white = s.gyro_arw / sqrt(dt);
accel_white = s.acc_vrw / sqrt(dt);

gyro_meas = zeros(N, 3);
accel_meas = zeros(N, 3);

for i = 1:N
	b_acc = phi_acc * b_acc + q_acc * randn(3, 1);
	gyro_meas(i, :) = (Kg * w_ib_true(i, :)' + bg + gyro_white * randn(3, 1))';
	accel_meas(i, :) = (Ka * specific_force_true(i, :)' + ba + b_acc + ...
		accel_white * randn(3, 1))';
end

% Giới hạn dải đo và kiểm tra bão hòa
gyro_limit = p.ins.gyro.dai_do_deg_s * pi/180;
accel_limit = p.ins.acc.dai_do_g * g;

gyro_saturation_count = nnz(abs(gyro_meas) > gyro_limit);
accel_saturation_count = nnz(abs(accel_meas) > accel_limit);

gyro_meas = min(max(gyro_meas, -gyro_limit), gyro_limit);
accel_meas = min(max(accel_meas, -accel_limit), accel_limit);

% -------------------------------------------------------------------------
% 3. GIẢ LẬP BỘ THU GNSS 2 ĂNG-TEN TÍCH HỢP TRONG INS-DM-FI
% -------------------------------------------------------------------------
Z_gps_pos = [];
Z_gps_vel = [];
Z_gps_yaw = [];

if has_gnss
	if nargin < 9 || isempty(sigma_gps_pos)
		if isfield(p, 'ins') && isfield(p.ins, 'gnss') && isfield(p.ins.gnss, 'sigma_pos_m')
			sigma_gps_pos = p.ins.gnss.sigma_pos_m;
		else
			sigma_gps_pos = 0.02; % RTK 2 cm
		end
	end
	if nargin < 10 || isempty(sigma_gps_vel)
		if isfield(p, 'ins') && isfield(p.ins, 'gnss') && isfield(p.ins.gnss, 'sigma_vel_mps')
			sigma_gps_vel = p.ins.gnss.sigma_vel_mps;
		else
			sigma_gps_vel = 0.02; % 2 cm/s
		end
	end
	if nargin < 11 || isempty(sigma_gps_yaw)
		if isfield(p, 'ins') && isfield(p.ins, 'gnss') && isfield(p.ins.gnss, 'sigma_heading_deg')
			sigma_gps_yaw = p.ins.gnss.sigma_heading_deg * pi / 180;
		else
			sigma_gps_yaw = 0.05 * pi / 180; % 0.05° la bàn 2 ăng-ten
		end
	end

	n_meas = numel(measurement_indices);
	Z_gps_pos = position_true(measurement_indices, :) + sigma_gps_pos * randn(n_meas, 3);
	Z_gps_vel = velocity_true_ned(measurement_indices, :) + sigma_gps_vel * randn(n_meas, 3);
	Z_gps_yaw = yaw_true(measurement_indices) + sigma_gps_yaw * randn(n_meas, 1);
end

% Đóng gói struct kết quả phụ trợ
ins_sim = struct();
ins_sim.w_ib_true = w_ib_true;
ins_sim.specific_force_true = specific_force_true;
ins_sim.w_ie = w_ie;
ins_sim.w_en = w_en;
ins_sim.RN = RN;
ins_sim.RM = RM;
ins_sim.latitude = latitude;
ins_sim.height = height;
ins_sim.gyro_saturation_count = gyro_saturation_count;
ins_sim.accel_saturation_count = accel_saturation_count;
ins_sim.Z_gps_pos = Z_gps_pos;
ins_sim.Z_gps_vel = Z_gps_vel;
ins_sim.Z_gps_yaw = Z_gps_yaw;
ins_sim.has_gnss = has_gnss;

% -------------------------------------------------------------------------
% 4. ĐÓNG GÓI OUTPUT THEO SỐ LƯỢNG ĐỐI SỐ TRẢ VỀ
% -------------------------------------------------------------------------
if nargout <= 3
	varargout{1} = ins_sim;
else
	varargout{1} = Z_gps_pos;
	varargout{2} = Z_gps_vel;
	varargout{3} = Z_gps_yaw;
	varargout{4} = ins_sim;
end

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



