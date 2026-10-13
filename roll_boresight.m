function result = roll_boresight(make_plots)
%ROLL_BORESIGHT Kich ban 3D kich thich boresight DVL-IMU bang ES-EKF.
% Chay: result = roll_boresight();
% Nguon thong so sensor: thong_so_he_thong_chuan.m.

if nargin < 1
	make_plots = true;
end

p = thong_so_he_thong_chuan();
rng(17, 'twister');

dt = p.si.dt_ins;
fs = p.ins.gyro.tan_so_Hz;
g = p.dat.g;
gn = [0; 0; g];
T_cal = 250;
T_max = T_cal + 800;
N_max = round(T_max * fs);
V_cruise = 1.2;
psi_goal = atan2(250, 450);
psi_reverse = psi_goal + pi;
seabed_depth = 50;

% GNSS, pressure-depth noise, lever arm and control limits are assumptions.
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

time = (0:N_max-1)' * dt;
position_true = zeros(N_max, 3);
velocity_true_ned = zeros(N_max, 3);
velocity_true_body = zeros(N_max, 3);
roll_true = zeros(N_max, 1);
pitch_true = zeros(N_max, 1);
yaw_true = zeros(N_max, 1);
speed_true = zeros(N_max, 1);
position = zeros(3, 1);
speed = 0;
yaw = psi_reverse;
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
		if ti >= 170
			% Prescribed kinematic excitation; not a hydrodynamic/actuator model.
			excitation_phase = (ti - 170) / (T_cal - 170);
			envelope = sin(pi * excitation_phase)^2;
			body_sway = 0.18 * envelope * sin(2*pi*(ti - 170) / 40);
			body_heave = 0.08 * envelope * sin(2*pi*(ti - 170) / 24);
			roll = 10 * pi/180 * envelope * sin(2*pi*(ti - 170) / 32);
		end
		body_velocity = [speed; body_sway; body_heave];
	else
		if isempty(destination)
			destination = position + [350*cos(psi_goal); 350*sin(psi_goal); 15];
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

		commanded_pitch = -max(min(atan(0.05 * delta_destination(3)), max_pitch), -max_pitch);
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
T_total = time(end);

% IMU truth includes Earth rotation and transport rate, as in dithang.m.
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
	w_en_i = [vn_prev(2) / (RN + height); -vn_prev(1) / (RM + height); ...
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

% Datasheet-derived sensor settings; repeatability/VRW are treated as 1-sigma.
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

% Assumption: GNSS, depth and DVL are synchronized at 5 Hz in this scenario.
if abs(gnss_rate_Hz - p.dvl.ping_Hz) > 1e-9 || ...
		abs(depth_rate_Hz - p.dvl.ping_Hz) > 1e-9
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

% DVL is modeled as a 3-axis output, not as four individual Janus beams.
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

% Target core: fixed-size nominal state, error covariance and process noise.
noise_cfg = [s.acc_vrw; s.gyro_arw; s.acc_bias_inrun_sigma; ...
	s.acc_bias_tau; s.gyro_bias_sigma*1e-4; 5e-7; 5e-5*pi/180; dt];
[estimator_state, P, Q] = esekf19_init( ...
	[roll_true(1); pitch_true(1); yaw_true(1)], g, noise_cfg);

% cfg indices are documented in esekf19_step.m and remain fixed for codegen.
filter_cfg = zeros(18, 1);
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
		if measurement_time(measurement_idx) <= T_cal
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

% Parameter-only observability diagnostic; reference velocity/attitude are assumed known.
alpha_observability = alpha_information(velocity_dvl_lever_true, measurement_indices, ...
	measurement_time, T_cal, sigma_dvl, s_true, boresight_true, dvl_valid);
cal_idx = find(time <= T_cal, 1, 'last');
underwater = time > T_cal;
velocity_error = velocity_est_history - velocity_true_ned;
velocity_rmse_ned = sqrt(mean(velocity_error(underwater, :).^2, 1));
position_error = position_est_history - position_true;
position_error_norm = sqrt(sum(position_error(underwater, :).^2, 2));
position_error_horizontal = sqrt(sum(position_error(underwater, 1:2).^2, 2));
parameter_truth = [s_true*100, boresight_true'*180/pi];

fprintf('\nROLL-BORESIGHT / ES-EKF: NED, IMU %.0f Hz, DVL %.1f Hz\n', fs, p.dvl.ping_Hz);
fprintf('Calib cut %.1f s; DVL valid/used/rejected: %d/%d/%d\n', T_cal, ...
	nnz(dvl_valid), n_dvl_used, n_dvl_rejected);
fprintf('Parameter at GNSS cut: s %.4f%% (true %.4f%%), alpha %.4f deg (true %.4f), beta %.4f (true %.4f), gamma %.4f (true %.4f)\n', ...
	parameter_history(cal_idx, 1), parameter_truth(1), parameter_history(cal_idx, 2), ...
	parameter_truth(2), parameter_history(cal_idx, 3), parameter_truth(3), ...
	parameter_history(cal_idx, 4), parameter_truth(4));
fprintf('Alpha parameter-only ideal bound: %.4g deg; independent sensitivity %.2f%%\n', ...
	alpha_observability.sigma_bound_deg, 100*alpha_observability.independent_fraction);
fprintf('Underwater position error final/RMS: %.3f/%.3f m; max horizontal %.3f m\n', ...
	position_error_norm(end), sqrt(mean(position_error_norm.^2)), max(position_error_horizontal));
fprintf('Underwater velocity RMSE [N E D]: [%.4f %.4f %.4f] m/s\n', velocity_rmse_ned);
fprintf('IMU full-scale clipping samples gyro/accel: %d/%d\n', ...
	gyro_saturation_count, accel_saturation_count);

result = struct('time', time, 'position_true_ned', position_true, ...
	'position_est_ned', position_est_history, 'velocity_true_ned', velocity_true_ned, ...
	'velocity_est_ned', velocity_est_history, 'velocity_rmse_ned', velocity_rmse_ned, ...
	'velocity_true_body', velocity_true_body, ...
	'attitude_true_rpy', [roll_true, pitch_true, yaw_true], ...
	'attitude_est_rpy', attitude_est_history, 'parameter_history', parameter_history, ...
	'parameter_sigma_history', parameter_sigma_history, 'parameter_truth', parameter_truth, ...
	'alpha_observability', alpha_observability, 'dvl_valid', dvl_valid, ...
	'dvl_used', n_dvl_used, 'dvl_rejected', n_dvl_rejected, ...
	'position_error_ned', position_error, ...
	'position_error_horizontal', position_error_horizontal, ...
	'position_error_norm', position_error_norm, ...
	'underwater_mask', underwater, 'calibration_end_s', T_cal, ...
	'dive_index', idx_dive, 'destination_ned', destination);

if make_plots
	roll_boresight_plot(result);
end
end

function Cbn = body_to_nav(roll, pitch, yaw)
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
Cbn = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
	sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
	-sp, cp*sr, cp*cr];
end

function obs = alpha_information(v_l, indices, measurement_time, calibration_end, sigma_fn, scale, boresight, valid)
J = zeros(3*numel(indices), 4);
count = 0;
Cbs = eye(3) - skew_mat(boresight);
for k = 1:numel(indices)
	if measurement_time(k) > calibration_end || ~valid(k)
		continue;
	end
	velocity = v_l(k, :)';
	H_scale = Cbs * velocity;
	H_angles = (1 + scale) * skew_mat(velocity);
	sigma = max(sigma_fn(norm(velocity)), 1e-6);
	rows = 3*count + (1:3);
	J(rows, :) = [H_scale, H_angles] / sigma;
	count = count + 1;
end
J = J(1:3*count, :);
obs.samples = count;
if count == 0
	obs.sigma_bound_deg = Inf;
	obs.independent_fraction = 0;
	return;
end
alpha_column = J(:, 2);
nuisance = J(:, [1, 3, 4]);
alpha_independent = alpha_column - nuisance * (pinv(nuisance) * alpha_column);
information = alpha_independent' * alpha_independent;
obs.sigma_bound_deg = (180/pi) / sqrt(max(information, eps));
obs.independent_fraction = norm(alpha_independent) / max(norm(alpha_column), eps);
end

function S = skew_mat(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function Cbn = quat_to_Cbn(q)
q = q(:) / norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
	2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
	2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end
