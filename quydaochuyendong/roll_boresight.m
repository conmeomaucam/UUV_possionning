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

% 4. Chay mo phong
result = roll_boresight_run(p, scenario, scenario_id);
result.scenario_id = scenario_id;
result.scenario = scenario;
result.rng_seed = scenario.rng_seed;
result.calibration_end_s = scenario.T_cal;
result.saved_file = save_file;
result = add_compat_fields(result, scenario);

% 5. Luu ket qua va ve do thi
save(save_file, 'result', '-v7.3');

if make_plots && exist('roll_boresight_plot', 'file') == 2
	roll_boresight_plot(result);
end

end

function result = roll_boresight_run(p, scenario, scenario_id)
%ROLL_BORESIGHT_RUN Khoi chay quy trinh tich hop cam bien va loc ES-EKF.

dt = p.si.dt_ins;
g = p.dat.g;
T_cal = scenario.T_cal;

% A. Sinh quy dao thuc cua UUV va che do lai tu dong
traj = tao_quy_dao_uuv(p, scenario, scenario_id);
time = traj.time;
N = traj.N;
position_true = traj.position_true;
velocity_true_ned = traj.velocity_true_ned;
roll_true = traj.roll_true;
pitch_true = traj.pitch_true;
yaw_true = traj.yaw_true;

% B. Thiet lap tan so va do lech chuan cam bien ngoai
gnss_rate_Hz = 5;
sigma_gps_pos = 0.02;
sigma_gps_vel = 0.02;
sigma_gps_yaw = 0.05 * pi / 180;
depth_rate_Hz = 5;
sigma_depth = 0.05;
lever_arm = [0.2; 0; -0.1];
seabed_depth = 50;

if abs(gnss_rate_Hz - p.dvl.ping_Hz) > 1e-9 || abs(depth_rate_Hz - p.dvl.ping_Hz) > 1e-9
	error('Kich ban nay yeu cau GNSS, depth va DVL cung tan so lay mau (5 Hz).');
end

measurement_step = round((1 / p.dvl.ping_Hz) / dt);
measurement_indices = (1:measurement_step:N)';
n_measurements = numel(measurement_indices);
measurement_time = time(measurement_indices);

% C. Gia lap cum cam bien INS-DM-FI (IMU FOG 400 Hz + GNSS 5 Hz)
[gyro_meas, accel_meas, Z_gps_pos, Z_gps_vel, Z_gps_yaw, ins_sim] = INS_FI( ...
	p, dt, velocity_true_ned, position_true, roll_true, pitch_true, yaw_true, ...
	measurement_indices, sigma_gps_pos, sigma_gps_vel, sigma_gps_yaw);

% D. Gia lap cam bien do sau ap suat thuy tinh (5 Hz)
[Z_depth, ~] = Apsuat(p, measurement_indices, position_true, sigma_depth);

% E. Gia lap dau do Water Linked DVL A125 (5 Hz)
[Z_dvl, dvl_valid, dvl_sim] = DVL_A125(p, dt, measurement_indices, ...
	velocity_true_ned, position_true, roll_true, pitch_true, yaw_true, ...
	ins_sim, lever_arm, seabed_depth);

% F. Khoi tao bo loc ES-EKF 19 trang thai
s = p.si;
noise_cfg = [s.acc_vrw; s.gyro_arw; s.acc_bias_inrun_sigma; ...
	s.acc_bias_tau; s.gyro_bias_sigma*1e-4; 5e-7; 5e-5*pi/180; dt];
[estimator_state, P, Q] = esekf19_init( ...
	[roll_true(1); pitch_true(1); yaw_true(1)], g, noise_cfg);

filter_cfg = zeros(20, 1);
filter_cfg(1) = dt;
filter_cfg(2) = g;
filter_cfg(3:5) = ins_sim.w_ie;
filter_cfg(6) = ins_sim.RN;
filter_cfg(7) = ins_sim.RM;
filter_cfg(8) = tan(ins_sim.latitude);
filter_cfg(9) = ins_sim.height;
filter_cfg(10:12) = lever_arm;
filter_cfg(13) = p.dvl.ping_noise_san_mps;
filter_cfg(14) = p.dvl.ping_noise_ti_le;
filter_cfg(15) = sigma_gps_pos;
filter_cfg(16) = sigma_gps_vel;
filter_cfg(17) = sigma_gps_yaw;
filter_cfg(18) = sigma_depth;
filter_cfg(19) = p.si.acc_bias_tau;
filter_cfg(20) = s.gyro_arw;

position_est_history = zeros(N, 3);
velocity_est_history = zeros(N, 3);
attitude_est_history = zeros(N, 3);
parameter_history = zeros(N, 4);
parameter_sigma_history = zeros(N, 4);
measurement_ptr = 1;
n_dvl_rejected = 0;
n_dvl_used = 0;

% G. Vong lap loc uoc luong ES-EKF tai 400 Hz
for i = 1:N
	measurement_flags = zeros(3, 1);
	gnss_measurement = zeros(7, 1);
	depth_measurement = 0;
	dvl_measurement = zeros(3, 1);

	if measurement_ptr <= n_measurements && measurement_indices(measurement_ptr) == i
		measurement_idx = measurement_ptr;
		
		% Logic GNSS: Hoat dong o mat nuoc (truoc T_cal) VA khi da noi len lai (sau T_resurface)
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

% H. Tinh toan sai so va bao cao nghiem thu (Resurface Benchmark)
eval_res = danh_gia_ket_qua(p, scenario, scenario_id, traj, ...
	position_est_history, velocity_est_history, parameter_history, ...
	parameter_sigma_history, dvl_sim, dvl_valid, n_dvl_used, n_dvl_rejected, ...
	measurement_indices, measurement_time);

% I. Dong goi toan bo ket qua tra ve
result = struct();
result.underwater_mask = eval_res.underwater_mask;
result.calibration_end_s = T_cal;
result.scenario = scenario;
result.time = time;
result.dive_index = traj.idx_dive;
result.destination_ned = traj.destination_ned;
result.attitude_true_rpy = [roll_true, pitch_true, yaw_true];
result.attitude_est_rpy = attitude_est_history;
result.position_true_ned = position_true;
result.position_true = position_true;
result.velocity_true_ned = velocity_true_ned;
result.velocity_true_body = traj.velocity_true_body;
result.roll_true = roll_true;
result.pitch_true = pitch_true;
result.yaw_true = yaw_true;
result.speed_true = traj.speed_true;
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
result.position_error = eval_res.position_error;
result.velocity_error = eval_res.velocity_error;
result.position_error_norm = eval_res.position_error_norm;
result.position_error_horizontal = eval_res.position_error_horizontal;
result.velocity_rmse_ned = eval_res.velocity_rmse_ned;
result.position_rmse_m = eval_res.position_rmse_m;
result.velocity_rmse_mps = eval_res.velocity_rmse_mps;
result.final_position_error = eval_res.final_position_error;
result.final_velocity_error = eval_res.final_velocity_error;
result.parameter_truth = eval_res.parameter_truth;
result.alpha_observability = eval_res.alpha_observability;
result.gyro_saturation_count = ins_sim.gyro_saturation_count;
result.accel_saturation_count = ins_sim.accel_saturation_count;
result.n_dvl_used = n_dvl_used;
result.n_dvl_rejected = n_dvl_rejected;
result.resurface_benchmark = eval_res.resurface_benchmark;
result.save_file = '';
result.p = p;
result.Q = Q;
result.P_final = P;
result.measurement_time = measurement_time;
result.measurement_indices = measurement_indices;
result.velocity_dvl_lever_true = dvl_sim.velocity_dvl_lever_true;
result.w_ib_true = ins_sim.w_ib_true;
result.specific_force_true = ins_sim.specific_force_true;
result.position_est_ned = position_est_history;
result.velocity_est_ned = velocity_est_history;

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

function Cbn = quat_to_Cbn(q)
%QUAT_TO_CBN Chuyen quaternion sang ma tran xoay Cbn (Body sang NED).
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