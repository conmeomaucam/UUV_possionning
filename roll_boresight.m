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
	gyro_limit = p.ins.gyro.dai_do_deg_s * pi/180;
	accel_limit = p.ins.acc.dai_do_g * g;
	gyro_saturation_count = nnz(abs(gyro_meas) > gyro_limit);
	accel_saturation_count = nnz(abs(accel_meas) > accel_limit);
	gyro_meas = min(max(gyro_meas, -gyro_limit), gyro_limit);
	accel_meas = min(max(accel_meas, -accel_limit), accel_limit);
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
			speed = V_cruise * (0.5 - 0.5 * cos(pi * ramp));
gyro_limit = p.ins.gyro.dai_do_deg_s * pi/180;
accel_limit = p.ins.acc.dai_do_g * g;
gyro_saturation_count = nnz(abs(gyro_meas) > gyro_limit);
accel_saturation_count = nnz(abs(accel_meas) > accel_limit);
gyro_meas = min(max(gyro_meas, -gyro_limit), gyro_limit);
accel_meas = min(max(accel_meas, -accel_limit), accel_limit);
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

% ES-EKF error state: dp, dv, dtheta, dba, dbg, ds, d(alpha,beta,gamma).
pos_est = [0; 0; 0];
vel_est = [0; 0; 0];
q_est = quat_from_euler(roll_true(1), pitch_true(1), yaw_true(1));
ba_est = zeros(3, 1);
bg_est = zeros(3, 1);
s_est = 0;
boresight_est = zeros(3, 1);

P = diag([(0.02)^2 * ones(1, 3), (0.05)^2 * ones(1, 3), ...
	(0.5*pi/180)^2 * ones(1, 3), (0.5e-3*g)^2 * ones(1, 3), ...
	(0.5*pi/180/3600)^2 * ones(1, 3), (0.02)^2, ...
	(1.5*pi/180)^2 * ones(1, 3)]);
Q = zeros(19, 19);
Q(4:6, 4:6) = s.acc_vrw^2 * dt * eye(3);
Q(7:9, 7:9) = s.gyro_arw^2 * dt * eye(3);
Q(10:12, 10:12) = s.acc_bias_inrun_sigma^2 * (2*dt/s.acc_bias_tau) * eye(3);
Q(13:15, 13:15) = (s.gyro_bias_sigma*1e-4)^2 * dt * eye(3);
Q(16, 16) = (5e-7)^2 * dt;
Q(17:19, 17:19) = (5e-5*pi/180)^2 * dt * eye(3);

position_est_history = zeros(N, 3);
velocity_est_history = zeros(N, 3);
attitude_est_history = zeros(N, 3);
parameter_history = zeros(N, 4);
parameter_sigma_history = zeros(N, 4);
measurement_ptr = 1;
n_dvl_rejected = 0;
n_dvl_used = 0;
CHI2_3 = 11.34;
CHI2_7 = 18.48;

for i = 1:N
	Cbn_est = quat_to_Cbn(q_est);
	Cnb_est = Cbn_est';
	f_clean = accel_meas(i, :)' - ba_est;
	w_clean = gyro_meas(i, :)' - bg_est;
	w_ie_n = w_ie;
	w_en_n = [vel_est(2) / (RN + height); -vel_est(1) / (RM + height); ...
		-vel_est(2) * tan(latitude) / (RN + height)];
	w_nb_b = w_clean - Cnb_est * (w_ie_n + w_en_n);
	accel_n = Cbn_est * f_clean + [0; 0; g] - cross(2*w_ie_n + w_en_n, vel_est);
	vel_est = vel_est + accel_n * dt;
	pos_est = pos_est + vel_est * dt;

	delta_angle = w_nb_b * dt;
	angle_norm = norm(delta_angle);
	dq = [cos(angle_norm/2); sin(angle_norm/2) * delta_angle / max(angle_norm, 1e-12)];
	q_est = quat_multiply(q_est, dq);
	q_est = q_est / norm(q_est);

	F = eye(19);
	F(1:3, 4:6) = eye(3) * dt;
	F(4:6, 7:9) = -Cbn_est * skew_mat(f_clean) * dt;
	F(4:6, 10:12) = -Cbn_est * dt;
	F(7:9, 7:9) = eye(3) - skew_mat(w_nb_b) * dt;
	F(7:9, 13:15) = -eye(3) * dt;
	P = F * P * F' + Q;

	if measurement_ptr <= n_measurements && measurement_indices(measurement_ptr) == i
		k = measurement_ptr;
		ti = measurement_time(k);
		if ti <= T_cal
			Cbn_est = quat_to_Cbn(q_est);
			yaw_est = atan2(Cbn_est(2, 1), Cbn_est(1, 1));
			yaw_residual = atan2(sin(Z_gps_yaw(k) - yaw_est), cos(Z_gps_yaw(k) - yaw_est));
			residual = [Z_gps_pos(k, :)' - pos_est; Z_gps_vel(k, :)' - vel_est; yaw_residual];
			H = zeros(7, 19);
			H(1:3, 1:3) = eye(3);
			H(4:6, 4:6) = eye(3);
			H(7, 9) = 1;
			R = diag([sigma_gps_pos^2*ones(1, 3), sigma_gps_vel^2*ones(1, 3), sigma_gps_yaw^2]);
			[dx, P, accepted] = kf_update(P, H, residual, R, CHI2_7);
			if accepted
				[pos_est, vel_est, q_est, ba_est, bg_est, s_est, boresight_est, P] = ...
					inject_error(dx, P, pos_est, vel_est, q_est, ba_est, bg_est, s_est, boresight_est);
			end
		end

		H_depth = zeros(1, 19);
		H_depth(3) = 1;
		[dx, P, accepted] = kf_update(P, H_depth, Z_depth(k) - pos_est(3), sigma_depth^2, 6.63);
		if accepted
			[pos_est, vel_est, q_est, ba_est, bg_est, s_est, boresight_est, P] = ...
				inject_error(dx, P, pos_est, vel_est, q_est, ba_est, bg_est, s_est, boresight_est);
		end

		if dvl_valid(k)
			Cbn_est = quat_to_Cbn(q_est);
			Cnb_est = Cbn_est';
			velocity_body_est = Cnb_est * vel_est;
			velocity_lever_est = velocity_body_est + cross(w_clean, lever_arm);
			Cbs_est = eye(3) - skew_mat(boresight_est);
			dvl_residual = Z_dvl(k, :)' - (1 + s_est) * Cbs_est * velocity_lever_est;

			H_dvl = zeros(3, 19);
			H_dvl(:, 4:6) = (1 + s_est) * Cbs_est * Cnb_est;
			H_dvl(:, 7:9) = (1 + s_est) * Cbs_est * skew_mat(velocity_body_est);
			H_dvl(:, 13:15) = (1 + s_est) * Cbs_est * skew_mat(lever_arm);
			H_dvl(:, 16) = Cbs_est * velocity_lever_est;
			H_dvl(:, 17:19) = (1 + s_est) * skew_mat(velocity_lever_est);
			sigma_ping = sigma_dvl(norm(velocity_lever_est));
			[dx, P, accepted] = kf_update(P, H_dvl, dvl_residual, sigma_ping^2 * eye(3), CHI2_3);
			if accepted
				[pos_est, vel_est, q_est, ba_est, bg_est, s_est, boresight_est, P] = ...
					inject_error(dx, P, pos_est, vel_est, q_est, ba_est, bg_est, s_est, boresight_est);
				n_dvl_used = n_dvl_used + 1;
			else
				n_dvl_rejected = n_dvl_rejected + 1;
			end
		end
		measurement_ptr = measurement_ptr + 1;
	end

	Cbn_est = quat_to_Cbn(q_est);
	position_est_history(i, :) = pos_est';
	velocity_est_history(i, :) = vel_est';
	attitude_est_history(i, :) = [atan2(Cbn_est(3, 2), Cbn_est(3, 3)), ...
		-asin(max(min(Cbn_est(3, 1), 1), -1)), atan2(Cbn_est(2, 1), Cbn_est(1, 1))];
	parameter_history(i, :) = [s_est*100, boresight_est'*180/pi];
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
	'attitude_true_rpy', [roll_true, pitch_true, yaw_true], ...
	'attitude_est_rpy', attitude_est_history, 'parameter_history', parameter_history, ...
	'parameter_sigma_history', parameter_sigma_history, 'parameter_truth', parameter_truth, ...
	'alpha_observability', alpha_observability, 'dvl_valid', dvl_valid, ...
	'dvl_used', n_dvl_used, 'dvl_rejected', n_dvl_rejected, ...
	'position_error_ned', position_error);

if make_plots
	figure('Name', 'Roll-boresight: quy dao', 'Color', 'w', 'Position', [40 60 1350 620]);
	subplot(1, 2, 1);
	plot3(position_true(:, 2), position_true(:, 1), -position_true(:, 3), 'r-', 'LineWidth', 1.8); hold on; grid on;
	plot3(position_est_history(:, 2), position_est_history(:, 1), -position_est_history(:, 3), 'k--', 'LineWidth', 1.3);
	plot3(0, 0, 0, 'go', 'MarkerFaceColor', 'g');
	plot3(position_true(idx_dive, 2), position_true(idx_dive, 1), -position_true(idx_dive, 3), ...
		'm^', 'MarkerFaceColor', 'm');
	plot3(destination(2), destination(1), -destination(3), 'bp', 'MarkerFaceColor', 'b');
	xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
	title('Quy dao 3D'); legend('Truth', 'ES-EKF', 'Start', 'Dive', 'Goal', 'Location', 'best');
	axis equal; view([-35 25]);

	subplot(1, 2, 2);
	plot(position_true(:, 2), position_true(:, 1), 'r-', 'LineWidth', 1.8); hold on; grid on;
	plot(position_est_history(:, 2), position_est_history(:, 1), 'k--', 'LineWidth', 1.3);
	xline(position_true(idx_dive, 2), 'm:', 'GNSS off');
	xlabel('East (m)'); ylabel('North (m)'); title('Mặt bằng N-E');
	legend('Truth', 'ES-EKF', 'Location', 'best'); axis equal;

	figure('Name', 'Roll-boresight: thai do va kich thich', 'Color', 'w', 'Position', [80 40 1300 760]);
	angles_true = [roll_true, pitch_true, unwrap(yaw_true)] * 180/pi;
	angles_est = attitude_est_history;
	angles_est(:, 3) = unwrap(angles_est(:, 3));
	angle_names = {'Roll phi (deg)', 'Pitch theta (deg)', 'Yaw psi (deg)'};
	for axis_idx = 1:3
		subplot(3, 2, 2*axis_idx-1);
		plot(time, angles_true(:, axis_idx), 'r-', 'LineWidth', 1.4); hold on; grid on;
		plot(time, angles_est(:, axis_idx), 'k--', 'LineWidth', 1.1);
		xline(T_cal, 'm--', 'GNSS off'); ylabel(angle_names{axis_idx});
		legend('Truth', 'ES-EKF', 'Location', 'best');
		subplot(3, 2, 2*axis_idx);
		if axis_idx == 1
			plot(time, velocity_true_body(:, 2), 'b-', 'LineWidth', 1.2); hold on;
			plot(time, velocity_true_body(:, 3), 'Color', [0 0.6 0], 'LineWidth', 1.2);
			ylabel('Body sway/heave (m/s)'); legend('v sway', 'w heave', 'Location', 'best');
		elseif axis_idx == 2
			plot(time, position_true(:, 3), 'r-', 'LineWidth', 1.3); hold on;
			plot(time, position_est_history(:, 3), 'k--', 'LineWidth', 1.1);
			ylabel('Depth down (m)'); legend('Truth', 'ES-EKF', 'Location', 'best');
		else
			plot(time(underwater), position_error_horizontal, 'b-', 'LineWidth', 1.2); hold on;
			plot(time(underwater), position_error_norm, 'k--', 'LineWidth', 1.2);
			ylabel('Position error (m)'); legend('Horizontal', '3D', 'Location', 'best');
		end
		grid on; xlabel('Time (s)'); xline(T_cal, 'm--');
	end

	figure('Name', 'Roll-boresight: hoi tu tham so', 'Color', 'w', 'Position', [120 60 1200 820]);
	parameter_names = {'Scale s (%)', 'Alpha roll (deg)', 'Beta pitch (deg)', 'Gamma yaw (deg)'};
	for parameter_idx = 1:4
		subplot(4, 1, parameter_idx);
		plot(time, parameter_history(:, parameter_idx), 'b-', 'LineWidth', 1.4); hold on; grid on;
		plot(time, parameter_truth(parameter_idx) + 3*parameter_sigma_history(:, parameter_idx), 'k:');
		plot(time, parameter_truth(parameter_idx) - 3*parameter_sigma_history(:, parameter_idx), 'k:');
		yline(parameter_truth(parameter_idx), 'r--', 'Truth');
		xline(T_cal, 'm--', 'GNSS off');
		ylabel(parameter_names{parameter_idx});
		if parameter_idx == 1, legend('ES-EKF', 'Truth +/- 3 sigma', 'Location', 'best'); end
	end
	xlabel('Time (s)');

	figure('Name', 'Roll-boresight: van toc NED', 'Color', 'w', 'Position', [160 80 1100 720]);
	ned_names = {'North v_N (m/s)', 'East v_E (m/s)', 'Down v_D (m/s)'};
	for axis_idx = 1:3
		subplot(3, 1, axis_idx);
		plot(time, velocity_true_ned(:, axis_idx), 'r-', 'LineWidth', 1.4); hold on; grid on;
		plot(time, velocity_est_history(:, axis_idx), 'k--', 'LineWidth', 1.1);
		xline(T_cal, 'm--', 'GNSS off'); ylabel(ned_names{axis_idx});
		legend('Truth', 'ES-EKF', 'Location', 'best');
	end
	xlabel('Time (s)');
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

function q = quat_from_euler(roll, pitch, yaw)
cr = cos(roll/2); sr = sin(roll/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cy = cos(yaw/2); sy = sin(yaw/2);
q = [cy*cp*cr + sy*sp*sr; cy*cp*sr - sy*sp*cr; ...
	cy*sp*cr + sy*cp*sr; sy*cp*cr - cy*sp*sr];
q = q / norm(q);
end

function [dx, P, accepted] = kf_update(P, H, residual, R, gate)
S = H * P * H' + R;
nis = residual' * (S \ residual);
accepted = isfinite(nis) && nis <= gate;
dx = zeros(size(P, 1), 1);
if ~accepted
	return;
end
K = (P * H') / S;
dx = K * residual;
I_KH = eye(size(P)) - K * H;
P = I_KH * P * I_KH' + K * R * K';
P = 0.5 * (P + P');
end

function [pos, vel, q, ba, bg, scale, boresight, P] = inject_error(dx, P, pos, vel, q, ba, bg, scale, boresight)
pos = pos + dx(1:3);
vel = vel + dx(4:6);
dtheta = dx(7:9);
dq = [1; 0.5*dtheta];
dq = dq / norm(dq);
q = quat_multiply(q, dq);
q = q / norm(q);
ba = ba + dx(10:12);
bg = bg + dx(13:15);
scale = scale + dx(16);
boresight = boresight + dx(17:19);
G = eye(19);
G(7:9, 7:9) = eye(3) - 0.5*skew_mat(dtheta);
P = G * P * G';
P = 0.5 * (P + P');
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

function r = quat_multiply(a, b)
r = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4); ...
	a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3); ...
	a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2); ...
	a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
end
