function traj = tao_quy_dao_uuv(p, scenario, scenario_id)
%TAO_QUY_DAO_UUV Tao quy dao chuyen dong 3D chuan va bo dieu khien lai (Autopilot) cho UUV.
%   traj = tao_quy_dao_uuv(p, scenario, scenario_id)
%
%   Dau vao:
%       p           - Struct thong so he thong chuan (thong_so_he_thong_chuan)
%       scenario    - Struct thong so kich ban (get_roll_boresight_scenario)
%       scenario_id - Ma kich ban mo phong (1..6)
%
%   Dau ra:
%       traj.time               - Vector thoi gian (s) [N x 1]
%       traj.N                  - Tong so diem mau mo phong
%       traj.position_true      - Vi tri thuc NED [North, East, Down] (m) [N x 3]
%       traj.velocity_true_ned  - Van toc thuc khung NED (m/s) [N x 3]
%       traj.velocity_true_body - Van toc thuc khung than Body (m/s) [N x 3]
%       traj.roll_true          - Goc roll thuc (rad) [N x 1]
%       traj.pitch_true         - Goc pitch thuc (rad) [N x 1]
%       traj.yaw_true           - Goc yaw thuc (rad) [N x 1]
%       traj.speed_true         - Toc do thuc 3D (m/s) [N x 1]
%       traj.idx_dive           - Chi so thoi diem bat dau lan
%       traj.destination_ned    - Toa do diem dich NED (m) [3 x 1]

dt = p.si.dt_ins;
g = p.dat.g;
T_cal = scenario.T_cal;
T_max = scenario.T_max;
V_cruise = scenario.V_cruise;
destination_ned = scenario.destination_ned(:);
psi_goal = atan2(destination_ned(2), destination_ned(1));
psi_reverse = psi_goal + pi;

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

			% NED chieu Down huong xuong: delta_destination(3) > 0 nghia la lan sau -> pitch am (chui mui)
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

traj = struct();
traj.time = time;
traj.N = N;
traj.position_true = position_true;
traj.velocity_true_ned = velocity_true_ned;
traj.velocity_true_body = velocity_true_body;
traj.roll_true = roll_true;
traj.pitch_true = pitch_true;
traj.yaw_true = yaw_true;
traj.speed_true = speed_true;
traj.idx_dive = idx_dive;
traj.destination_ned = destination_ned;

end

function Cbn = body_to_nav(roll, pitch, yaw)
%BODY_TO_NAV Ma tran chuyen toa do tu khung than (Body) sang khung NED theo Euler Z-Y-X.
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
