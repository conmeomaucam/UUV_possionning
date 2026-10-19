function eval_res = danh_gia_ket_qua(p, scenario, scenario_id, traj, ...
	position_est_history, velocity_est_history, parameter_history, ...
	parameter_sigma_history, dvl_sim, dvl_valid, n_dvl_used, n_dvl_rejected, ...
	measurement_indices, measurement_time)
%DANH_GIA_KET_QUA Tinh toan sai so, do quan sat va in bao cao nghiem thu (Benchmark).
%   eval_res = danh_gia_ket_qua(...)
%
%   Dau ra:
%       eval_res.underwater_mask           - Mask thoi gian khi UUV lan ngam
%       eval_res.velocity_error            - Sai so van toc theo thoi gian (m/s)
%       eval_res.velocity_rmse_ned         - RMSE van toc theo tung truc N-E-D (m/s)
%       eval_res.velocity_rmse_mps         - RMSE van toc 3D (m/s)
%       eval_res.position_error            - Sai so vi tri theo thoi gian (m)
%       eval_res.position_error_norm       - Do lon sai so vi tri 3D (m)
%       eval_res.position_error_horizontal - Sai so mat phang ngang (m)
%       eval_res.position_rmse_m           - RMSE vi tri 3D khi lan ngam (m)
%       eval_res.final_position_error      - Sai so vi tri tai thoi diem cuoi (m)
%       eval_res.final_velocity_error      - Sai so van toc tai thoi diem cuoi (m/s)
%       eval_res.parameter_truth           - Gia tri chuan [s(%), alpha(deg), beta(deg), gamma(deg)]
%       eval_res.alpha_observability       - Thong tin Fisher / do quan sat goc boresight
%       eval_res.resurface_benchmark       - Struct ket qua nghiem thu khi noi len mat nuoc

s_true = dvl_sim.s_true;
boresight_true = dvl_sim.boresight_true;
sigma_dvl = dvl_sim.sigma_dvl;
velocity_dvl_lever_true = dvl_sim.velocity_dvl_lever_true;
T_cal = scenario.T_cal;
time = traj.time;
position_true = traj.position_true;
velocity_true_ned = traj.velocity_true_ned;
fs = p.ins.gyro.tan_so_Hz;

% 1. Tinh do quan sat cua goc boresight (Fisher Information Proxy)
alpha_observability = alpha_information(velocity_dvl_lever_true, measurement_indices, ...
	measurement_time, T_cal, sigma_dvl, s_true, boresight_true, dvl_valid);

cal_idx = find(time <= T_cal, 1, 'last');

% 2. Tinh sai so rieng trong pha lan ngam
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

final_position_error = position_est_history(end, :) - position_true(end, :);
final_velocity_error = velocity_est_history(end, :) - velocity_true_ned(end, :);
position_rmse_m = sqrt(mean(sum(position_error(underwater, :).^2, 2)));
velocity_rmse_mps = sqrt(mean(sum(velocity_error(underwater, :).^2, 2)));

% 3. In bao cao tham so tai thoi diem cat GNSS
fprintf('\nROLL-BORESIGHT / ES-EKF: NED, IMU %.0f Hz, DVL %.1f Hz\n', fs, p.dvl.ping_Hz);
fprintf('Calib cut %.1f s; DVL valid/used/rejected: %d/%d/%d\n', T_cal, ...
	nnz(dvl_valid), n_dvl_used, n_dvl_rejected);
fprintf(['Parameter at GNSS cut: s %.4f%% (true %.4f%%), ' ...
	'alpha %.4f deg (true %.4f), beta %.4f (true %.4f), gamma %.4f (true %.4f)\n'], ...
	parameter_history(cal_idx, 1), parameter_truth(1), ...
	parameter_history(cal_idx, 2), parameter_truth(2), ...
	parameter_history(cal_idx, 3), parameter_truth(3), ...
	parameter_history(cal_idx, 4), parameter_truth(4));

% 4. In bang danh gia nghiem thu khi noi len mat nuoc (Resurface Benchmark)
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

% 5. Dong goi ket qua
eval_res = struct();
eval_res.underwater_mask = underwater;
eval_res.velocity_error = velocity_error;
eval_res.velocity_rmse_ned = velocity_rmse_ned;
eval_res.velocity_rmse_mps = velocity_rmse_mps;
eval_res.position_error = position_error;
eval_res.position_error_norm = position_error_norm;
eval_res.position_error_horizontal = position_error_horizontal;
eval_res.position_rmse_m = position_rmse_m;
eval_res.final_position_error = final_position_error;
eval_res.final_velocity_error = final_velocity_error;
eval_res.parameter_truth = parameter_truth;
eval_res.alpha_observability = alpha_observability;
eval_res.resurface_benchmark = resurface_data;

end

function S = skew_mat(v)
%SKEW_MAT Ma tran phan doi xung 3x3 tu vector 3x1.
v = v(:);
S = [0, -v(3), v(2); ...
	v(3), 0, -v(1); ...
	-v(2), v(1), 0];
end

function info = alpha_information(velocity_dvl_lever_true, measurement_indices, ...
	measurement_time, T_cal, sigma_dvl, s_true, boresight_true, dvl_valid)
%ALPHA_INFORMATION Uoc luong thong tin quan sat goc boresight theo phuong phap Fisher proxy.

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
