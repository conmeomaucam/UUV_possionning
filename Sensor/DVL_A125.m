function varargout = DVL_A125(mode, varargin)
%DVL_A125  Giả lập cảm biến đo vận tốc Doppler DVL A125 (Water Linked).
%
% 1. NGUYÊN LÝ MÔ PHỎNG:
%    - Chế độ hoạt động: Bottom-Track (Bám đáy) trong hệ tọa độ đầu đo DVL (Sensor Frame).
%    - Bù hiệu ứng cánh tay đòn (Lever-Arm) do vị trí lắp lệch tâm giữa IMU và DVL:
%        v_lever = v_body + cross(omega_b, lever_arm)
%    - Áp sai số lệch trục lắp đặt cơ khí (Boresight) và sai số hệ số tỉ lệ (Scale Factor):
%        v_dvl_ideal = (1.0 + true_scale) * Cbs_true * v_lever
%    - Mô hình nhiễu đo theo datasheet Water Linked DVL A125:
%        sigma = noise_floor + noise_slope * norm(v_lever)
%        dvl_meas = v_dvl_ideal + sigma * randn(3, 1)
%
% 2. CÁCH GỌI MÔ-ĐUN:
%    spec = DVL_A125('spec');                                  % Lấy thông số kỹ thuật chuẩn DVL A125
%    [dvl_meas, v_lever, sigma] = DVL_A125('step', ...         % Giả lập 1 bước đo DVL (chu kỳ 5 Hz)
%        v_body, omega_b, lever_arm, true_scale, Cbs_true, spec);
%
%    (Hỗ trợ truyền trực tiếp noise_floor và noise_slope):
%    dvl_meas = DVL_A125('step', v_body, omega_b, lever_arm, true_scale, Cbs_true, noise_floor, noise_slope);
%
% 3. CHẾ ĐỘ BATCH CŨ (Tương thích ngược):
%    [Z_dvl, dvl_valid, dvl_sim] = DVL_A125(p, dt, measurement_indices, ...);

if nargin == 0
    mode = 'spec';
end

% Kiểm tra tương thích nếu gọi theo kiểu struct batch cũ
if ~ischar(mode) && ~isstring(mode)
    [varargout{1}, varargout{2}, varargout{3}] = dvl_batch_legacy(mode, varargin{:});
    return;
end

switch lower(char(mode))
    case 'spec'
        varargout{1} = build_spec();
    case {'step', 'measure'}
        [varargout{1}, varargout{2}, varargout{3}] = dvl_step_measurement(varargin{:});
    otherwise
        error('DVL_A125: mode khong hop le: %s. Chon ''spec'' hoac ''step''.', mode);
end

end

%% ===================== THÔNG SỐ KỸ THUẬT DVL A125 =====================
function s = build_spec()
s.ten = 'Water Linked DVL A125';
s.tan_so_hz = 5;                       % Tần số cập nhật danh định (Hz)
s.noise_floor = 0.01;                  % Nhiễu sàn vận tốc danh định (m/s)
s.noise_slope = 0.005;                 % Nhiễu tỉ lệ vận tốc (0.5% vận tốc)
s.van_toc_toi_da = 9.0;                % Vận tốc đo tối đa (m/s, ~17.5 knots)
s.do_cao_min = 0.05;                   % Tầm bám đáy tối thiểu (m)
s.do_cao_max = 125.0;                  % Tầm bám đáy tối đa (m)
s.do_phan_giai = 0.001;                % Độ phân giải vận tốc (1 mm/s)
s.gm_tau_s = 300.0;                    % Thời hằng trôi chậm Gauss-Markov (s)
s.gm_ti_le_sigma = 0.001;              % Tỉ lệ nhiễu Gauss-Markov
end

%% ===================== GIẢ LẬP BƯỚC ĐO ONLINE (STEP) =====================
function [dvl_meas, v_lever, sigma_dvl] = dvl_step_measurement( ...
    v_body, omega_b, lever_arm, true_scale, Cbs_true, spec_or_floor, slope_in)

v_body = v_body(:);
omega_b = omega_b(:);
lever_arm = lever_arm(:);

% 1. Vận tốc tại vị trí lắp đặt đầu đo DVL (bù vận tốc góc và cánh tay đòn)
v_lever = v_body + cross(omega_b, lever_arm);

% 2. Vận tốc lý tưởng sau khi chiếu vào hệ tọa độ DVL (kèm sai số Boresight và Scale factor thật)
v_dvl_ideal = (1.0 + true_scale) * (Cbs_true * v_lever);

% 3. Độ lệch chuẩn nhiễu đo theo datasheet DVL A125
if nargin >= 6 && isstruct(spec_or_floor)
    noise_floor = spec_or_floor.noise_floor;
    noise_slope = spec_or_floor.noise_slope;
elseif nargin >= 6 && isnumeric(spec_or_floor)
    noise_floor = spec_or_floor;
    if nargin >= 7 && ~isempty(slope_in)
        noise_slope = slope_in;
    else
        noise_slope = 0.005;
    end
else
    noise_floor = 0.01;
    noise_slope = 0.005;
end

speed_lever = norm(v_lever);
sigma_dvl = noise_floor + noise_slope * speed_lever;

% 4. Sinh số đo vận tốc 3 trục trong hệ DVL có nhiễu Gaussian
dvl_meas = v_dvl_ideal + sigma_dvl * randn(3, 1);

end

%% ===================== HÀM BATCH TƯƠNG THÍCH CŨ =====================
function [Z_dvl, dvl_valid, dvl_sim] = dvl_batch_legacy(p, dt, measurement_indices, ...
	velocity_true_ned, position_true, roll_true, pitch_true, yaw_true, ...
	ins_sim, lever_arm, seabed_depth)

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
Cbs_true = eye(3) - skew_mat_local(boresight_true);
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
	Cnb_true = body_to_nav_local(roll_true(i), pitch_true(i), yaw_true(i))';
	velocity_body = Cnb_true * velocity_true_ned(i, :)';
	body_rate = w_ib_true(i, :)' - Cnb_true * (w_ie + w_en(i, :)');
	velocity_lever = velocity_body + cross(body_rate, lever_arm);
	velocity_dvl_lever_true(k, :) = velocity_lever';

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

	dvl_valid(k) = norm(velocity_lever) <= p.dvl.van_toc_toi_da_mps && ...
		altitude >= p.dvl.do_cao_min_m && altitude <= p.dvl.do_cao_max_m;
	if ~dvl_valid(k)
		continue;
	end

	dvl_gm = phi_dvl * dvl_gm + p.dvl.gm_ti_le_sigma * sqrt(1 - phi_dvl^2) * randn(3, 1);
	ideal = (1 + s_true) * Cbs_true * velocity_lever;
	white = sigma_dvl(norm(velocity_lever)) * randn(3, 1);
	Z_dvl(k, :) = (ideal + dvl_gm .* ideal + white)';
end

dvl_sim = struct();
dvl_sim.velocity_dvl_lever_true = velocity_dvl_lever_true;
dvl_sim.s_true = s_true;
dvl_sim.boresight_true = boresight_true;
dvl_sim.Cbs_true = Cbs_true;
dvl_sim.sigma_dvl = sigma_dvl;
dvl_sim.altitude = altitude_history;
dvl_sim.dt_dvl = dt_dvl;

end

function Cbn = body_to_nav_local(roll, pitch, yaw)
cr = cos(roll);  sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw);   sy = sin(yaw);

Cbn = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
	sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
	-sp,   cp*sr,             cp*cr];
end

function S = skew_mat_local(v)
v = v(:);
S = [0, -v(3), v(2); ...
	v(3), 0, -v(1); ...
	-v(2), v(1), 0];
end
