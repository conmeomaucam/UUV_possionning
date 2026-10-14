function result = roll_boresight(scenario_id, make_plots, force_run)
%ROLL_BORESIGHT Chay mo phong danh gia boresight DVL-IMU va ES-EKF.
%   result = roll_boresight(1)            % Chay case 1 co ban
%   result = roll_boresight(6, true, true)% Chay kịch bản vòng số 8 Nổi-Lặn-Nổi

if nargin < 1 || isempty(scenario_id)
	scenario_id = 1;
end
if nargin < 2 || isempty(make_plots)
	make_plots = true;
end
if nargin < 3 || isempty(force_run)
	force_run = false;
end

if isstring(scenario_id) || ischar(scenario_id)
	scenario_id = str2double(scenario_id);
end
if ~isscalar(scenario_id) || ~isfinite(scenario_id) || scenario_id < 1 || ...
		scenario_id ~= floor(scenario_id)
	error('scenario_id phai la so nguyen duong.');
end

p = thong_so_he_thong_chuan();
scenario = get_roll_boresight_scenario(scenario_id);

rng(scenario.rng_seed, 'twister');

results_dir = fullfile(fileparts(mfilename('fullpath')), 'roll_boresight_results');
if ~exist(results_dir, 'dir')
	mkdir(results_dir);
end
save_file = fullfile(results_dir, sprintf('roll_boresight_case%02d.mat', scenario_id));

if ~force_run && exist(save_file, 'file')
	S = load(save_file, 'result');
	if isfield(S, 'result')
		result = S.result;

		if ~isfield(result, 'calibration_end_s')
			result.calibration_end_s = scenario.T_cal;
		end
		if ~isfield(result, 'underwater_mask')
			result.underwater_mask = result.time > result.calibration_end_s;
		end
		result = add_compat_fields(result, scenario);

		if make_plots && exist('roll_boresight_plot', 'file') == 2
			roll_boresight_plot(result);
		end
		return;
	end
end

result = roll_boresight_run(p, scenario, scenario_id);
result.scenario_id = scenario_id;
result.scenario = scenario;
result.rng_seed = scenario.rng_seed;
result.calibration_end_s = scenario.T_cal;
result.saved_file = save_file;
result = add_compat_fields(result, scenario);

save(save_file, 'result', '-v7.3');

if make_plots && exist('roll_boresight_plot', 'file') == 2
	roll_boresight_plot(result);
end

end

function result = add_compat_fields(result, scenario)
%ADD_COMPAT_FIELDS Tao cac truong tuong thich cho file plot va du lieu cu.
alias = { ...
	'position_true_ned', 'position_true'; ...
	'position_est_ned',  'position_est_history'; ...
	'velocity_est_ned',  'velocity_est_history'; ...
	'attitude_est_rpy',  'attitude_est_history'};
for k = 1:size(alias, 1)
	new_name = alias{k, 1};
	old_name = alias{k, 2};
	if ~isfield(result, new_name) && isfield(result, old_name)
		result.(new_name) = result.(old_name);
	end
end

if ~isfield(result, 'attitude_true_rpy') && ...
		all(isfield(result, {'roll_true', 'pitch_true', 'yaw_true'}))
	result.attitude_true_rpy = [result.roll_true(:), result.pitch_true(:), result.yaw_true(:)];
end

if ~isfield(result, 'destination_ned')
	result.destination_ned = scenario.destination_ned(:);
end

if ~isfield(result, 'dive_index')
	result.dive_index = find(result.time >= result.calibration_end_s, 1);
end
end

function scenario = get_roll_boresight_scenario(scenario_id)
scenario = struct();
scenario.rng_seed = 100 + scenario_id;

switch scenario_id
	case 1
		scenario.name = 'Baseline turn and dive';
		scenario.T_cal = 250;
		scenario.T_max = 1050;
		scenario.V_cruise = 1.2;
		scenario.destination_ned = [450; 250; 15];
		scenario.excitation_start_s = 170;
		scenario.sway_amp_mps = 0.18;
		scenario.heave_amp_mps = 0.08;
		scenario.roll_amp_deg = 10;
		scenario.sway_period_s = 40;
		scenario.heave_period_s = 24;
		scenario.roll_period_s = 32;

	case 2
		scenario.name = 'More aggressive maneuver';
		scenario.T_cal = 220;
		scenario.T_max = 980;
		scenario.V_cruise = 1.5;
		scenario.destination_ned = [520; -140; 20];
		scenario.excitation_start_s = 150;
		scenario.sway_amp_mps = 0.24;
		scenario.heave_amp_mps = 0.12;
		scenario.roll_amp_deg = 12;
		scenario.sway_period_s = 30;
		scenario.heave_period_s = 18;
		scenario.roll_period_s = 26;

	case 3
		scenario.name = 'Mild excitation';
		scenario.T_cal = 280;
		scenario.T_max = 1100;
		scenario.V_cruise = 1.0;
		scenario.destination_ned = [350; 420; 10];
		scenario.excitation_start_s = 180;
		scenario.sway_amp_mps = 0.12;
		scenario.heave_amp_mps = 0.05;
		scenario.roll_amp_deg = 6;
		scenario.sway_period_s = 44;
		scenario.heave_period_s = 28;
		scenario.roll_period_s = 36;

	case 4
		scenario.name = 'Longer run with stronger excitation';
		scenario.T_cal = 200;
		scenario.T_max = 1200;
		scenario.V_cruise = 1.7;
		scenario.destination_ned = [650; 150; 25];
		scenario.excitation_start_s = 140;
		scenario.sway_amp_mps = 0.28;
		scenario.heave_amp_mps = 0.14;
		scenario.roll_amp_deg = 15;
		scenario.sway_period_s = 28;
		scenario.heave_period_s = 16;
		scenario.roll_period_s = 24;

	case 5
		scenario.name = 'Low-excitation stress test';
		scenario.T_cal = 260;
		scenario.T_max = 1000;
		scenario.V_cruise = 1.2;
		scenario.destination_ned = [400; 200; 12];
		scenario.excitation_start_s = 170;
		scenario.sway_amp_mps = 0.05;
		scenario.heave_amp_mps = 0.02;
		scenario.roll_amp_deg = 3;
		scenario.sway_period_s = 42;
		scenario.heave_period_s = 26;
		scenario.roll_period_s = 34;

	case 6
		scenario.name = 'Lemniscate: Surface Calib -> Dive -> Resurface benchmark';
		scenario.T_cal = 250;           % Tat GNSS bat dau lan (giay 250)
		scenario.T_resurface = 700;     % Thoi diem ngoi len mat nuoc va bat lai GNSS
		scenario.T_max = 850;           % Ket thuc bai chay
		scenario.V_cruise = 1.3;
		scenario.destination_ned = [0; 0; 0];
		scenario.fig8_A = 70;           % Ban truc North (m)
		scenario.fig8_B = 45;           % Ban truc East (m)
		scenario.fig8_T = 250;          % Chu ky 1 vong so 8 la 250 giay
		scenario.target_depth = 15;     % Do sau lan on dinh 15m

	otherwise
		error('Unknown scenario_id %d. Use 1..6.', scenario_id);
end
end

function result = roll_boresight_run(p, scenario, scenario_id)
dt = p.si.dt_ins;
fs = p.ins.gyro.tan_so_Hz;
g = p.dat.g;
T_cal = scenario.T_cal;
T_max = scenario.T_max;
V_cruise = scenario.V_cruise;
destination_ned = scenario.destination_ned(:);
psi_goal = atan2(destination_ned(2), destination_ned(1));
psi_reverse = psi_goal + pi;
seabed_depth = 50;

gnss_rate_Hz = 5;
sigma_gps_pos = 0.02;
sigma_gps_vel = 0.02;
sigma_gps_yaw = 0.05 * pi / 180;
depth_rate_Hz = 5;
sigma_depth = 0.05;
lever_arm = [0.2; 0; -0.1];
max_yaw_rate = 3 * pi / 180;
max_pitch = 8 * pi / 180;
max_accel = 0.10;
arrival_radius = 2.0;

N_max = floor(T_max / dt) + 1;
time = (0:N_max-1)' * dt;

position_true = zeros(N_max, 3);
velocity_true_ned = zeros(N_max, 3);
velocity_true_body = zeros(N_max, 3);
roll_true = zeros(N_max, 1);
pitch_true = zeros(N_max, 1);
yaw_true = zeros(N_max, 1);
speed_true = zeros(N_max, 1);

if scenario_id == 6
	% =========================================================================
	% QUY DAO VONG SO 8: NOI (0-250s) -> LAN (250-660s) -> NOI LAI (700-850s)
	% =========================================================================
	w8 = 2 * pi / scenario.fig8_T;
	A = scenario.fig8_A;
	B = scenario.fig8_B;
	target_depth = scenario.target_depth;
	T_resurface = scenario.T_resurface;
	t_transit = 40; % Thoi gian chuyen tiep do sau (giay)

	t_ramp = 15;
	theta = zeros(N_max, 1);
	theta_dot = zeros(N_max, 1);
	for k = 1:N_max
		ti = time(k);
		if ti < t_ramp
			theta(k) = w8 * (0.5 * ti - 0.5 * (t_ramp / pi) * sin(pi * ti / t_ramp));
			theta_dot(k) = w8 * (0.5 - 0.5 * cos(pi * ti / t_ramp));
		else
			theta(k) = w8 * (ti - 0.5 * t_ramp);
			theta_dot(k) = w8;
		end
	end

	pos_N = A * sin(theta);
	pos_E = B * sin(2 * theta);
	vel_N = A * cos(theta) .* theta_dot;
	vel_E = 2 * B * cos(2 * theta) .* theta_dot;

	pos_D = zeros(N_max, 1);
	vel_D = zeros(N_max, 1);
	for k = 1:N_max
		ti = time(k);
		if ti < T_cal
			pos_D(k) = 0;
			vel_D(k) = 0;
		elseif ti >= T_cal && ti < (T_cal + t_transit)
			tau = (ti - T_cal) / t_transit;
			pos_D(k) = target_depth * (0.5 - 0.5 * cos(pi * tau));
			vel_D(k) = target_depth * 0.5 * (pi / t_transit) * sin(pi * tau);
		elseif ti >= (T_cal + t_transit) && ti < (T_resurface - t_transit)
			pos_D(k) = target_depth;
			vel_D(k) = 0;
		elseif ti >= (T_resurface - t_transit) && ti < T_resurface
			tau = (ti - (T_resurface - t_transit)) / t_transit;
			pos_D(k) = target_depth * (0.5 + 0.5 * cos(pi * tau));
			vel_D(k) = -target_depth * 0.5 * (pi / t_transit) * sin(pi * tau);
		else
			pos_D(k) = 0;
			vel_D(k) = 0;
		end
	end

	yaw_prof = atan2(vel_E, vel_N);
	yaw_prof(1) = atan2(2*B, A);

	speed_horiz = sqrt(vel_N.^2 + vel_E.^2);
	speed_3d = sqrt(speed_horiz.^2 + vel_D.^2);

	pitch_prof = -atan2(vel_D, max(speed_horiz, 1e-4));

	yaw_rate_prof = gradient(unwrap(yaw_prof), dt);
	roll_prof = atan(speed_horiz .* yaw_rate_prof / g);
	roll_prof = max(min(roll_prof, 12 * pi/180), -12 * pi/180);

	position_true = [pos_N, pos_E, pos_D];
	velocity_true_ned = [vel_N, vel_E, vel_D];
	roll_true = roll_prof;
	pitch_true = pitch_prof;
	yaw_true = yaw_prof;
	speed_true = speed_3d;
	idx_dive = find(time >= T_cal, 1);
	N = N_max;

	for k = 1:N
		Cbn_k = body_to_nav(roll_true(k), pitch_true(k), yaw_true(k));
		velocity_true_body(k, :) = (Cbn_k' * velocity_true_ned(k, :)')';
	end
else
	% =========================================================================
	% QUY DAO CASES 1 DEN 5 (CHAY THANG, BE LAI, KICH THICH VA LAN DEN DICH)
	% =========================================================================
	position = zeros(3, 1);
	speed = 0;
	yaw = psi_goal;
	yaw_rate = 0;
	pitch = 0;
	pitch_rate = 0;
	destination = [];
	idx_dive = NaN;
	N = N_max;

	for i = 1:N_max
		ti = time(i);

		if ti < T_cal
			roll = 0;
			pitch = 0;

			if ti < 30
				speed = 0;
				yaw = psi_reverse;
			elseif ti < 40
				ramp = (ti - 30) / 10;
				speed = V_cruise * (0.5 - 0.5 * cos(pi * ramp));
				yaw = psi_reverse;
			elseif ti < 110
				speed = V_cruise;
				yaw = psi_reverse;
			elseif ti < 170
				speed = V_cruise;
				turn_fraction = (ti - 110) / 60;
				smooth_turn = turn_fraction^2 * (3 - 2 * turn_fraction);
				yaw = psi_reverse - pi * smooth_turn;
				yaw_rate = -pi * (6 * turn_fraction * (1 - turn_fraction)) / 60;
			else
				speed = V_cruise;
				yaw = psi_goal;
			end

			body_sway = 0;
			body_heave = 0;
			if ti >= scenario.excitation_start_s
				tau = ti - scenario.excitation_start_s;
				excitation_phase = tau / max(T_cal - scenario.excitation_start_s, dt);
				excitation_phase = min(max(excitation_phase, 0), 1);
				envelope = sin(pi * excitation_phase)^2;

				body_sway = scenario.sway_amp_mps * envelope * sin(2*pi*tau / scenario.sway_period_s);
				body_heave = scenario.heave_amp_mps * envelope * sin(2*pi*tau / scenario.heave_period_s);
				roll = scenario.roll_amp_deg * pi/180 * envelope * sin(2*pi*tau / scenario.roll_period_s);
			end

			body_velocity = [speed; body_sway; body_heave];
		else
			if isempty(destination)
				destination = destination_ned;
				idx_dive = i;
			end

			delta_destination = destination - position;
			horizontal_range = norm(delta_destination(1:2));

			if horizontal_range < arrival_radius && abs(delta_destination(3)) < 0.5
				N = i - 1;
				break;
			end

			commanded_yaw = atan2(delta_destination(2), delta_destination(1));
			yaw_error = atan2(sin(commanded_yaw - yaw), cos(commanded_yaw - yaw));
			commanded_yaw_rate = max(min(0.10 * yaw_error, max_yaw_rate), -max_yaw_rate);
			yaw_rate = yaw_rate + (commanded_yaw_rate - yaw_rate) * dt / 2.0;
			yaw = yaw + yaw_rate * dt;

			% SUA CHUAN NED: delta_destination(3) > 0 tuc can lan sau -> pitch phai am (chui mui)
			commanded_pitch = max(min(-atan(0.05 * delta_destination(3)), max_pitch), -max_pitch);
			pitch_rate = max(min((commanded_pitch - pitch) / 3.0, 1.5*pi/180), -1.5*pi/180);
			pitch = pitch + pitch_rate * dt;

			commanded_speed = max(min(0.08 * horizontal_range, V_cruise), 0.3);
			speed = speed + max(min((commanded_speed - speed) / 5.0, max_accel), -max_accel) * dt;

			roll = max(min(atan(speed * yaw_rate / g), 10*pi/180), -10*pi/180);
			body_velocity = [speed; 0; 0];
		end

		Cbn_true = body_to_nav(roll, pitch, yaw);
		velocity_n = Cbn_true * body_velocity;

		roll_true(i) = roll;
		pitch_true(i) = pitch;
		yaw_true(i) = yaw;
		speed_true(i) = speed;
		velocity_true_body(i, :) = body_velocity';
		velocity_true_ned(i, :) = velocity_n';
		position = position + velocity_n * dt;
		position_true(i, :) = position';
	end
end

time = time(1:N);
position_true = position_true(1:N, :);
velocity_true_ned = velocity_true_ned(1:N, :);
velocity_true_body = velocity_true_body(1:N, :);
roll_true = roll_true(1:N);
pitch_true = pitch_true(1:N);
yaw_true = yaw_true(1:N);
speed_true = speed_true(1:N);

if isnan(idx_dive)
	error('Quy dao chua vao duoc pha lan. Tang T_max hoac kiem tra autopilot.');
end

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

gyro_limit = p.ins.gyro.dai_do_deg_s * pi/180;
accel_limit = p.ins.acc.dai_do_g * g;
gyro_saturation_count = nnz(abs(gyro_meas) > gyro_limit);
accel_saturation_count = nnz(abs(accel_meas) > accel_limit);
gyro_meas = min(max(gyro_meas, -gyro_limit), gyro_limit);
accel_meas = min(max(accel_meas, -accel_limit), accel_limit);

if abs(gnss_rate_Hz - p.dvl.ping_Hz) > 1e-9 || abs(depth_rate_Hz - p.dvl.ping_Hz) > 1e-9
	error('Kich ban nay gia dinh GNSS, depth va DVL cung tan so lay mau.');
end

measurement_step = round((1 / p.dvl.ping_Hz) / dt);
measurement_indices = (1:measurement_step:N)';
n_measurements = numel(measurement_indices);
measurement_time = time(measurement_indices);

Z_gps_pos = position_true(measurement_indices, :) + sigma_gps_pos * randn(n_measurements, 3);
Z_gps_vel = velocity_true_ned(measurement_indices, :) + sigma_gps_vel * randn(n_measurements, 3);
Z_gps_yaw = yaw_true(measurement_indices) + sigma_gps_yaw * randn(n_measurements, 1);
Z_depth = position_true(measurement_indices, 3) + sigma_depth * randn(n_measurements, 1);

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

for k = 1:n_measurements
	i = measurement_indices(k);
	Cnb_true = body_to_nav(roll_true(i), pitch_true(i), yaw_true(i))';
	velocity_body = Cnb_true * velocity_true_ned(i, :)';
	body_rate = w_ib_true(i, :)' - Cnb_true * (w_ie + w_en(i, :)');
	velocity_lever = velocity_body + cross(body_rate, lever_arm);
	velocity_dvl_lever_true(k, :) = velocity_lever';

	altitude = seabed_depth - position_true(i, 3);
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

noise_cfg = [s.acc_vrw; s.gyro_arw; s.acc_bias_inrun_sigma; ...
	s.acc_bias_tau; s.gyro_bias_sigma*1e-4; 5e-7; 5e-5*pi/180; dt];
[estimator_state, P, Q] = esekf19_init( ...
	[roll_true(1); pitch_true(1); yaw_true(1)], g, noise_cfg);

filter_cfg = zeros(19, 1);
filter_cfg(1) = dt;
filter_cfg(2) = g;
filter_cfg(3:5) = w_ie;
filter_cfg(6) = RN;
filter_cfg(7) = RM;
filter_cfg(8) = tan(latitude);
filter_cfg(9) = height;
filter_cfg(10:12) = lever_arm;
filter_cfg(13) = p.dvl.ping_noise_san_mps;
filter_cfg(14) = p.dvl.ping_noise_ti_le;
filter_cfg(15) = sigma_gps_pos;
filter_cfg(16) = sigma_gps_vel;
filter_cfg(17) = sigma_gps_yaw;
filter_cfg(18) = sigma_depth;
filter_cfg(19) = p.si.acc_bias_tau;

position_est_history = zeros(N, 3);
velocity_est_history = zeros(N, 3);
attitude_est_history = zeros(N, 3);

parameter_history = zeros(N, 4);
parameter_sigma_history = zeros(N, 4);
measurement_ptr = 1;
n_dvl_rejected = 0;
n_dvl_used = 0;

for i = 1:N
	measurement_flags = zeros(3, 1);
	gnss_measurement = zeros(7, 1);
	depth_measurement = 0;
	dvl_measurement = zeros(3, 1);

	if measurement_ptr <= n_measurements && measurement_indices(measurement_ptr) == i
		measurement_idx = measurement_ptr;
		
		% LOGIC GNSS: Hoat dong o mat nuoc (truoc T_cal) VA khi da noi len lai (sau T_resurface)
		has_gnss = (measurement_time(measurement_idx) <= T_cal);
		if isfield(scenario, 'T_resurface') && measurement_time(measurement_idx) >= scenario.T_resurface
			has_gnss = true;
		end
		
		if has_gnss
			measurement_flags(1) = 1;
			gnss_measurement = [Z_gps_pos(measurement_idx, :)'; ...
				Z_gps_vel(measurement_idx, :)'; Z_gps_yaw(measurement_idx)];
		end
		
		measurement_flags(2) = 1;
		depth_measurement = Z_depth(measurement_idx);
		
		if dvl_valid(measurement_idx)
			measurement_flags(3) = 1;
			dvl_measurement = Z_dvl(measurement_idx, :)';
		end
		measurement_ptr = measurement_ptr + 1;
	end

	[estimator_state, P, update_status, ~] = esekf19_step( ...
		estimator_state, P, Q, gyro_meas(i, :)', accel_meas(i, :)', ...
		gnss_measurement, depth_measurement, dvl_measurement, measurement_flags, filter_cfg);

	if measurement_flags(3) ~= 0
		if update_status(3) ~= 0
			n_dvl_used = n_dvl_used + 1;
		else
			n_dvl_rejected = n_dvl_rejected + 1;
		end
	end

	Cbn_est = quat_to_Cbn(estimator_state(7:10));
	position_est_history(i, :) = estimator_state(1:3)';
	velocity_est_history(i, :) = estimator_state(4:6)';
	attitude_est_history(i, :) = [atan2(Cbn_est(3, 2), Cbn_est(3, 3)), ...
		-asin(max(min(Cbn_est(3, 1), 1), -1)), atan2(Cbn_est(2, 1), Cbn_est(1, 1))];
	parameter_history(i, :) = [estimator_state(17)*100, estimator_state(18:20)'*180/pi];
	parameter_sigma_history(i, :) = [sqrt(P(16, 16))*100, ...
		sqrt(diag(P(17:19, 17:19)))'*180/pi];
end

alpha_observability = alpha_information(velocity_dvl_lever_true, measurement_indices, ...
	measurement_time, T_cal, sigma_dvl, s_true, boresight_true, dvl_valid);

cal_idx = find(time <= T_cal, 1, 'last');

% Tinh sai so rieng trong pha lan ngam
if isfield(scenario, 'T_resurface')
	underwater = (time > T_cal) & (time <= scenario.T_resurface);
else
	underwater = time > T_cal;
end

velocity_error = velocity_est_history - velocity_true_ned;
velocity_rmse_ned = sqrt(mean(velocity_error(underwater, :).^2, 1));
position_error = position_est_history - position_true;
position_error_norm = sqrt(sum(position_error(underwater, :).^2, 2));
position_error_horizontal = sqrt(sum(position_error(underwater, 1:2).^2, 2));
parameter_truth = [s_true*100, boresight_true'*180/pi];

fprintf('\nROLL-BORESIGHT / ES-EKF: NED, IMU %.0f Hz, DVL %.1f Hz\n', fs, p.dvl.ping_Hz);
fprintf('Calib cut %.1f s; DVL valid/used/rejected: %d/%d/%d\n', T_cal, ...
	nnz(dvl_valid), n_dvl_used, n_dvl_rejected);
fprintf(['Parameter at GNSS cut: s %.4f%% (true %.4f%%), ' ...
	'alpha %.4f deg (true %.4f), beta %.4f (true %.4f), gamma %.4f (true %.4f)\n'], ...
	parameter_history(cal_idx, 1), parameter_truth(1), ...
	parameter_history(cal_idx, 2), parameter_truth(2), ...
	parameter_history(cal_idx, 3), parameter_truth(3), ...
	parameter_history(cal_idx, 4), parameter_truth(4));

final_position_error = position_est_history(end, :) - position_true(end, :);
final_velocity_error = velocity_est_history(end, :) - velocity_true_ned(end, :);
position_rmse_m = sqrt(mean(sum(position_error(underwater, :).^2, 2)));
velocity_rmse_mps = sqrt(mean(sum(velocity_error(underwater, :).^2, 2)));

% =========================================================================
% IN BANG DANH GIA NGHIEM THU KHI NOI LEN MAT NUOC (RESURFACE BENCHMARK)
% =========================================================================
resurface_data = struct();
if isfield(scenario, 'T_resurface')
	idx_resurface = find(time >= scenario.T_resurface, 1);
	err_surface_vec = position_est_history(idx_resurface, :) - position_true(idx_resurface, :);
	err_surface_horiz = norm(err_surface_vec(1:2));
	err_surface_3d = norm(err_surface_vec);
	
	idx_dive_start = find(time >= T_cal, 1);
	dt_swim = diff(time(idx_dive_start:idx_resurface));
	vel_mag = sqrt(sum(velocity_true_ned(idx_dive_start:idx_resurface-1, :).^2, 2));
	distance_underwater = sum(vel_mag .* dt_swim);
	
	dt_underwater_hours = (scenario.T_resurface - T_cal) / 3600;
	pct_dt = (err_surface_horiz / max(distance_underwater, 1e-3)) * 100;
	drift_rate_mph = err_surface_horiz / max(dt_underwater_hours, 1e-6);

	fprintf('\n================== KET QUA NGHIEM THU NOI LEN (RESURFACE BENCHMARK) ==================\n');
	fprintf('Kich ban                           : Case %d - %s\n', scenario_id, scenario.name);
	fprintf('Thoi gian lan ngam (khong co GNSS) : %.1f giay (%.2f phut)\n', ...
		scenario.T_resurface - T_cal, (scenario.T_resurface - T_cal)/60);
	fprintf('Tong quang duong di ngam           : %.2f met\n', distance_underwater);
	fprintf('Sai so vi tri mat ngang luc noi    : %.3f met\n', err_surface_horiz);
	fprintf('Sai so vi tri 3D luc noi           : %.3f met\n', err_surface_3d);
	fprintf('Ty le sai so theo quang duong      : %.3f %% DT\n', pct_dt);
	fprintf('Toc do troi dat (Drift rate)       : %.2f m/gio\n', drift_rate_mph);
	fprintf('=======================================================================================\n\n');
	
	resurface_data = struct( ...
		'time_s', scenario.T_resurface, ...
		'underwater_duration_s', scenario.T_resurface - T_cal, ...
		'distance_underwater_m', distance_underwater, ...
		'error_horiz_m', err_surface_horiz, ...
		'error_3d_m', err_surface_3d, ...
		'pct_dt', pct_dt, ...
		'drift_rate_mph', drift_rate_mph);
end

result = struct();
result.underwater_mask = underwater;
result.calibration_end_s = T_cal;
result.scenario = scenario;
result.time = time;
result.dive_index = idx_dive;
result.destination_ned = destination_ned;
result.attitude_true_rpy = [roll_true, pitch_true, yaw_true];
result.attitude_est_rpy = attitude_est_history;
result.position_true_ned = position_true;
result.position_true = position_true;
result.velocity_true_ned = velocity_true_ned;
result.velocity_true_body = velocity_true_body;
result.roll_true = roll_true;
result.pitch_true = pitch_true;
result.yaw_true = yaw_true;
result.speed_true = speed_true;
result.gyro_meas = gyro_meas;
result.accel_meas = accel_meas;
result.Z_gps_pos = Z_gps_pos;
result.Z_gps_vel = Z_gps_vel;
result.Z_gps_yaw = Z_gps_yaw;
result.Z_depth = Z_depth;
result.Z_dvl = Z_dvl;
result.dvl_valid = dvl_valid;
result.position_est_history = position_est_history;
result.velocity_est_history = velocity_est_history;
result.attitude_est_history = attitude_est_history;
result.parameter_history = parameter_history;
result.parameter_sigma_history = parameter_sigma_history;
result.position_error = position_error;
result.velocity_error = velocity_error;
result.position_error_norm = position_error_norm;
result.position_error_horizontal = position_error_horizontal;
result.velocity_rmse_ned = velocity_rmse_ned;
result.position_rmse_m = position_rmse_m;
result.velocity_rmse_mps = velocity_rmse_mps;
result.final_position_error = final_position_error;
result.final_velocity_error = final_velocity_error;
result.parameter_truth = parameter_truth;
result.alpha_observability = alpha_observability;
result.gyro_saturation_count = gyro_saturation_count;
result.accel_saturation_count = accel_saturation_count;
result.n_dvl_used = n_dvl_used;
result.n_dvl_rejected = n_dvl_rejected;
result.resurface_benchmark = resurface_data;
result.save_file = '';
result.p = p;
result.Q = Q;
result.P_final = P;
result.measurement_time = measurement_time;
result.measurement_indices = measurement_indices;
result.velocity_dvl_lever_true = velocity_dvl_lever_true;
result.w_ib_true = w_ib_true;
result.specific_force_true = specific_force_true;
result.position_est_ned = position_est_history;
result.velocity_est_ned = velocity_est_history;

end

function Cbn = body_to_nav(roll, pitch, yaw)
cr = cos(roll);
sr = sin(roll);
cp = cos(pitch);
sp = sin(pitch);
cy = cos(yaw);
sy = sin(yaw);

Cbn = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
	sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
	-sp,   cp*sr,             cp*cr];
end

function Cbn = quat_to_Cbn(q)
q = q(:);
q = q / norm(q);
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

Cbn = [qw^2 + qx^2 - qy^2 - qz^2, 2*(qx*qy - qw*qz),       2*(qx*qz + qw*qy); ...
	2*(qx*qy + qw*qz),       qw^2 - qx^2 + qy^2 - qz^2, 2*(qy*qz - qw*qx); ...
	2*(qx*qz - qw*qy),       2*(qy*qz + qw*qx),       qw^2 - qx^2 - qy^2 + qz^2];
end

function S = skew_mat(v)
v = v(:);
S = [0, -v(3), v(2); ...
	v(3), 0, -v(1); ...
	-v(2), v(1), 0];
end

function info = alpha_information(velocity_dvl_lever_true, measurement_indices, ...
	measurement_time, T_cal, sigma_dvl, s_true, boresight_true, dvl_valid)

n = numel(measurement_indices);
info = zeros(n, 1);
Cbs0 = eye(3) - skew_mat(boresight_true);

for k = 1:n
	if ~dvl_valid(k) || measurement_time(k) < T_cal
		continue;
	end

	v = velocity_dvl_lever_true(k, :)';
	sig = sigma_dvl(norm(v));
	base = (1 + s_true) * Cbs0 * v;

	eps_angle = 1e-6;
	sens = zeros(3, 1);
	for j = 1:3
		d = zeros(3, 1);
		d(j) = eps_angle;
		Cbs1 = eye(3) - skew_mat(boresight_true + d);
		pert = (1 + s_true) * Cbs1 * v;
		sens(j) = norm(pert - base) / eps_angle;
	end

	info(k) = sum(sens.^2) / max(sig^2, eps);
end
end