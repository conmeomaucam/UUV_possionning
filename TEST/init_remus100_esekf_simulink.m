function init_remus100_esekf_simulink(injected_yaw_deg)
%INIT_REMUS100_ESEKF_SIMULINK Khởi tạo tất cả biến không gian làm việc cho mô hình Simulink
%   Nạp thông số cảm biến (VN-210-S, DVL A125 0.15%, Bar10XT), thông số bộ lọc ES-EKF 19 trạng thái,
%   thông số bộ điều khiển ALOS3D & Autopilot và 12 Waypoints nhiệm vụ.
%   Tham số:
%       injected_yaw_deg - Góc lệch hướng bơm vào EKF lúc bắt đầu lặn (độ, mặc định: 1.0 deg).

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);

addpath(fullfile(root_dir, 'algo'));
addpath(fullfile(root_dir, 'Sensor'));
addpath(fullfile(root_dir, 'MISSION_PLANNER'));
addpath(fullfile(root_dir, 'TEST'));
addpath(genpath(fullfile(root_dir, 'AUV_simulation', 'MSS')));

% Reset biến persistent của thuật toán ALOS3D và xóa cache S-Function
clear ALOS3D;
clear sfun_remus100_sensors sfun_esekf19 sfun_alos3d_control;

% 1. Kịch bản 12 Waypoints (kèm chặng chạy ngược hiệu chuẩn Boresight)
run(fullfile(root_dir, 'MISSION_PLANNER', 'kich_ban_mission.m'));
assignin('base', 'wpt', wpt);

% 2. Cảm biến theo đúng Datasheet
ins_spec = VN_210_S('spec');
dvl_spec = DVL_A125('spec');
bar_spec = Bar_10XT('spec', 'nominal');

true_scale     = 0.015;                          % Scale factor thật = +1.5%
true_boresight = deg2rad([1.5; -2.0; 3.0]);      % Boresight thật [Roll; Pitch; Yaw] (rad)
lever_arm      = [0.5; -0.1; 0.2];               % Cánh tay đòn IMU -> DVL (m)
lat_rad        = deg2rad(20.72);                 % Vĩ độ Hải Phòng (rad)

sensor_params = struct( ...
    'ins_spec', ins_spec, ...
    'dvl_spec', dvl_spec, ...
    'bar_spec', bar_spec, ...
    'true_scale', true_scale, ...
    'true_boresight', true_boresight, ...
    'lever_arm', lever_arm, ...
    'lat_rad', lat_rad);
assignin('base', 'sensor_params', sensor_params);

% 3. Cấu hình bộ lọc ES-EKF 19 trạng thái (100 Hz)
h = 0.01; % 100 Hz
gravity = gravity_fossen(lat_rad, 0);

sigma_vrw = ins_spec.acc.vrw;
sigma_arw = ins_spec.gyro.arw;
acc_bias_sigma = ins_spec.acc.bias_instab;
acc_bias_tau = ins_spec.acc.bias_tau;
gyro_bias_rw = ins_spec.gyro.bias_rw;
scale_rw = 1e-5;
boresight_rw = 1e-5;

noise_cfg = [sigma_vrw; sigma_arw; acc_bias_sigma; acc_bias_tau; ...
             gyro_bias_rw; scale_rw; boresight_rw; h];
assignin('base', 'noise_cfg', noise_cfg);

cfg = zeros(23, 1);
cfg(1) = h;
cfg(2) = gravity;
cfg(3:5) = [0; 0; 7.292115e-5];
cfg(6) = 6378137.0;
cfg(7) = 6356752.3;
cfg(8) = tan(lat_rad);
cfg(9) = 0;
cfg(10:12) = lever_arm;
cfg(13) = dvl_spec.noise_floor;
cfg(14) = dvl_spec.noise_slope;
cfg(15) = ins_spec.gnss.pos_v;
cfg(16) = ins_spec.gnss.vel;
cfg(17) = ins_spec.gnss.heading;
cfg(18) = bar_spec.sigma_depth_nominal;
cfg(19) = acc_bias_tau;
cfg(20) = sigma_arw;
cfg(21) = 0;
cfg(22) = ins_spec.mag.declination;
cfg(23) = deg2rad(ins_spec.heading_accuracy_mag_deg);
assignin('base', 'cfg', cfg);

% 4. Thông số bộ điều khiển ALOS3D và Autopilot
[~,~,Mauv] = remus100();
ctrl_prm = struct();
ctrl_prm.Delta_h  = 20.0;
ctrl_prm.Delta_v  = 20.0;
ctrl_prm.gamma_h  = 0.001;
ctrl_prm.gamma_v  = 0.001;
ctrl_prm.M_theta  = deg2rad(15);
ctrl_prm.R_switch = 15.0;
ctrl_prm.K_f      = 0.5;
ctrl_prm.r_max    = deg2rad(15);
ctrl_prm.a_d      = 0.0;
ctrl_prm.K_d      = 0.5;
ctrl_prm.K_sigma  = 0.1;
ctrl_prm.phi_b    = 0.1;
ctrl_prm.K_yaw    = 5 / 20;
ctrl_prm.T_yaw    = 1.0;
ctrl_prm.Kp_theta = Mauv(5,5) * 0.8^2;
ctrl_prm.Kd_theta = Mauv(5,5) * 2 * 0.8 * 0.8;
ctrl_prm.Ki_theta = ctrl_prm.Kp_theta * 0.8 / 10;
ctrl_prm.delta_max = deg2rad(20);
ctrl_prm.n_max     = 1525;
ctrl_prm.n_rate    = 0.1;
assignin('base', 'ctrl_prm', ctrl_prm);

% 5. Điều kiện ban đầu Fossen và dòng chảy
psi0 = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));
x0 = [1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, psi0];
assignin('base', 'x0', x0);

Vc = 0.5;
betaVc = deg2rad(150);
assignin('base', 'Vc', Vc);
assignin('base', 'betaVc', betaVc);
assignin('base', 'true_boresight', true_boresight);
assignin('base', 'true_scale', true_scale);

% 6. Góc lệch hướng bơm vào EKF lúc bắt đầu lặn ngầm (mặc định 1.0 độ theo yêu cầu test)
if nargin < 1 || isempty(injected_yaw_deg)
    injected_yaw_deg = 1.0;
end
assignin('base', 'injected_yaw_submerged_deg', injected_yaw_deg);

fprintf('[INIT] Đã nạp thành công toàn bộ biến Workspace (Góc lệch hướng lặn ngầm = %+.2f deg)!\n', injected_yaw_deg);
end
