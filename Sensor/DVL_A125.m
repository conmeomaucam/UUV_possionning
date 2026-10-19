function [Z_dvl, dvl_valid, dvl_sim] = DVL_A125(p, dt, measurement_indices, ...
	velocity_true_ned, position_true, roll_true, pitch_true, yaw_true, ...
	ins_sim, lever_arm, seabed_depth)
%DVL_A125  Giả lập cảm biến đo vận tốc Doppler DVL A125 (Water Linked).
%
% 1. Nguyên lý mô phỏng:
%    - Chế độ hoạt động: Bottom-Track (Bám đáy lòng hồ).
%    - Đầu ra: Vận tốc 3 trục [vx, vy, vz] trong hệ tọa độ đầu đo DVL (Sensor Frame).
%    - Bù hiệu ứng cánh tay đòn (Lever-Arm) do khoảng cách lệch tâm giữa IMU và DVL:
%      v_lever = v_body + cross(body_rate, lever_arm)
%    - Kiểm tra điều kiện bắt đáy (Bottom-lock):
%      + Độ cao cách đáy: 0.05 m <= altitude <= 125.0 m
%      + Vận tốc tối đa: norm(v_lever) <= 9.0 m/s
%    - Áp sai số lệch trục lắp đặt cơ khí (Boresight) và sai số hệ số tỉ lệ (Scale factor):
%      ideal = (1 + s_true) * C_b^s * v_lever
%    - Mô hình nhiễu đo:
%      + Nhiễu trắng từng ping (mô phỏng chỉ số FOM): sigma = san + ti_le * |v|
%      + Sai số tương quan chậm Gauss-Markov (thời hằng tau = 300s)
%
% 2. Đầu vào:
%    - p                   : Struct thông số hệ thống chuẩn (từ thong_so_he_thong_chuan).
%    - dt                  : Chu kỳ lấy mẫu cơ bản (giây).
%    - measurement_indices : Mảng chỉ số các thời điểm phát xung của DVL.
%    - velocity_true_ned   : Vận tốc chuẩn hệ NED (N x 3, m/s).
%    - position_true       : Tọa độ vị trí chuẩn hệ NED (N x 3, m).
%    - roll_true, pitch_true, yaw_true: Góc tư thế chuẩn của UUV (N x 1, rad).
%    - ins_sim             : Struct kết quả từ INS_FI (chứa w_ib_true, w_ie, w_en).
%    - lever_arm           : Vector cánh tay đòn [lx; ly; lz] (m, khung thân).
%    - seabed_depth        : Độ sâu đáy hồ (m, tùy chọn, mặc định 50m).
%
% 3. Đầu ra:
%    - Z_dvl               : Ma trận vận tốc đo (N_meas x 3, m/s) trong hệ đầu đo DVL.
%    - dvl_valid           : Vector logic báo trạng thái bám đáy (N_meas x 1).
%    - dvl_sim             : Struct chứa các biến phục vụ đánh giá và nghiệm thu:
%                            .velocity_dvl_lever_true : Vận tốc thật tại tâm DVL (N_meas x 3)
%                            .s_true                  : Hệ số scale thật đưa vào
%                            .boresight_true          : Góc boresight thật đưa vào (rad)
%                            .Cbs_true                : Ma trận xoay lệch trục thật
%                            .sigma_dvl               : Function handle tính độ lệch chuẩn nhiễu
%                            .altitude                : Mảng độ cao cách đáy (N_meas x 1)

if nargin < 10 || isempty(lever_arm)
	lever_arm = [0.2; 0; -0.1];
end
if nargin < 11 || isempty(seabed_depth)
	seabed_depth = 50;
end

n_measurements = numel(measurement_indices);
measurement_step = measurement_indices(2) - measurement_indices(1);

s_true = p.si.k_that;
boresight_true = p.si.boresight_that_rad;
Cbs_true = eye(3) - skew_mat(boresight_true);
sigma_dvl = p.si.sigma_dvl_ping_fn;

dt_dvl = measurement_step * dt;
phi_dvl = exp(-dt_dvl / p.dvl.gm_tau_s);
dvl_gm = p.dvl.gm_ti_le_sigma * randn(3, 1);

Z_dvl = nan(n_measurements, 3);
velocity_dvl_lever_true = zeros(n_measurements, 3);
dvl_valid = false(n_measurements, 1);
altitude_history = zeros(n_measurements, 1);

w_ib_true = ins_sim.w_ib_true;
w_ie = ins_sim.w_ie;
w_en = ins_sim.w_en;

for k = 1:n_measurements
	i = measurement_indices(k);
	Cnb_true = body_to_nav(roll_true(i), pitch_true(i), yaw_true(i))';
	velocity_body = Cnb_true * velocity_true_ned(i, :)';
	body_rate = w_ib_true(i, :)' - Cnb_true * (w_ie + w_en(i, :)');
	velocity_lever = velocity_body + cross(body_rate, lever_arm);
	velocity_dvl_lever_true(k, :) = velocity_lever';

	% Tính độ cao cách đáy (hỗ trợ hàm nội suy hồ Hòa Bình nếu có)
	if isfield(p, 'env') && isfield(p.env, 'ham_noi_suy_do_sau') && ~isempty(p.env.ham_noi_suy_do_sau)
		d_bed = p.env.ham_noi_suy_do_sau(position_true(i, 1), position_true(i, 2));
		if isnan(d_bed)
			d_bed = seabed_depth;
		end
	else
		d_bed = seabed_depth;
	end
	altitude = d_bed - position_true(i, 3);
	altitude_history(k) = altitude;

	% Kiểm tra ngưỡng bám đáy của DVL A125
	dvl_valid(k) = norm(velocity_lever) <= p.dvl.van_toc_toi_da_mps && ...
		altitude >= p.dvl.do_cao_min_m && altitude <= p.dvl.do_cao_max_m;
	if ~dvl_valid(k)
		continue;
	end

	% Mô hình sai số: trôi chậm Gauss-Markov + lệch Boresight + Scale + nhiễu từng ping
	dvl_gm = phi_dvl * dvl_gm + p.dvl.gm_ti_le_sigma * sqrt(1 - phi_dvl^2) * randn(3, 1);
	ideal = (1 + s_true) * Cbs_true * velocity_lever;
	white = sigma_dvl(norm(velocity_lever)) * randn(3, 1);
	Z_dvl(k, :) = (ideal + dvl_gm .* ideal + white)';
end

% Đóng gói struct phục vụ đánh giá nghiệm thu
dvl_sim = struct();
dvl_sim.velocity_dvl_lever_true = velocity_dvl_lever_true;
dvl_sim.s_true = s_true;
dvl_sim.boresight_true = boresight_true;
dvl_sim.Cbs_true = Cbs_true;
dvl_sim.sigma_dvl = sigma_dvl;
dvl_sim.altitude = altitude_history;
dvl_sim.dt_dvl = dt_dvl;

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

function S = skew_mat(v)
%SKEW_MAT  Tạo ma trận phản đối xứng 3x3
v = v(:);
S = [0, -v(3), v(2); ...
	v(3), 0, -v(1); ...
	-v(2), v(1), 0];
end
