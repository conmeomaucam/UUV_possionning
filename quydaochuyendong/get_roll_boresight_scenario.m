function scenario = get_roll_boresight_scenario(scenario_id)
%GET_ROLL_BORESIGHT_SCENARIO Thiet lap cac kich ban mo phong hieu chuan boresight DVL-IMU.
%   scenario = get_roll_boresight_scenario(scenario_id)
%
%   Danh sach kich ban:
%       Case 1: Baseline turn and dive (co ban: chay thang, quay dau, kich thich va lan)
%       Case 2: More aggressive maneuver (van dong manh, van toc 1.5 m/s)
%       Case 3: Mild excitation (kich thich nhe, van toc 1.0 m/s)
%       Case 4: Longer run with stronger excitation (chay dai, kich thich manh)
%       Case 5: Low-excitation stress test (kiem thu do nhay kich thich thap)
%       Case 6: Lemniscate: Surface Calib -> Dive -> Resurface benchmark (Vong so 8 Noi-Lan-Noi)

if nargin < 1 || isempty(scenario_id)
	scenario_id = 1;
end

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
