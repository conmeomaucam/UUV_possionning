% TEST_GNSS_REACQUISITION
clc;
script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
addpath(fullfile(root_dir, 'algo'));

% Giả lập trạng thái sau khi lặn ngầm
pos_true = [500; 12000; 0]; % Mặt nước
vel_true = [2.5; 0.1; 0];
phi = 0; theta = 0; psi = 0.5;
q_true = [cos(psi/2); 0; 0; sin(psi/2)];

% Bộ lọc bị trôi 35 mét vị trí do tích lũy dead reckoning
pos_est = pos_true + [25; -20; 5];
vel_est = vel_true + [0.05; -0.02; 0.01];
q_est = q_true;

% Covariance P_pos nhỏ do dead reckoning với DVL
P = eye(19) * 0.01;
P(1:3, 1:3) = eye(3) * 4.0; % std = 2m, P = 4 m^2
P(4:6, 4:6) = eye(3) * 0.005;

% GNSS đo được tọa độ thật (với nhiễu nhỏ 0.03m)
gnss_pos = pos_true + [0.02; -0.01; 0.01];
gnss_vel = vel_true + [0.01; 0.01; 0];
gnss_yaw = psi + 0.005;

gnss_meas = [gnss_pos; gnss_vel; gnss_yaw];
flags = [1; 0; 0; 0]; % Chỉ có GNSS tại thời điểm ngoi lên
cfg = zeros(30, 1);
cfg(1) = 0.05; % dt = 0.05s
cfg(2) = 9.81; % gravity
cfg(3:5) = [0; 0; 7.292115e-5];
cfg(6) = 6378137.0; % radius_n
cfg(7) = 6356752.3; % radius_m
cfg(8) = tan(deg2rad(20.8));
cfg(9) = 0;
cfg(10:12) = [0.5; -0.1; 0.2];
cfg(15) = 0.05; % GNSS pos std = 0.05m
cfg(16) = 0.02; % GNSS vel std = 0.02m
cfg(17) = deg2rad(0.1); % GNSS yaw std = 0.1 deg
cfg(18) = 0.05; % depth std
cfg(19) = 3600; % accel_bias_tau
cfg(21) = 1; % Khóa calib

state = zeros(20, 1);
state(1:3) = pos_est;
state(4:6) = vel_est;
state(7:10) = q_est;
state(11:13) = 0;
state(14:16) = 0;
state(17) = 0.015;
state(18:20) = deg2rad([1.5; -2.0; 3.0]);

Q = zeros(19, 19);
gyro_meas = [0; 0; 0];
accel_meas = [0; 0; -9.81];

disp('--- TRƯỚC CẬP NHẬT GNSS TÁI HÒA NHẬP ---');
fprintf('Sai số vị trí ban đầu = %.2f m\n', norm(state(1:3) - pos_true));

[state_new, P_new, update_status, nis] = esekf19_step( ...
    state, P, Q, gyro_meas, accel_meas, ...
    gnss_meas, 0, [0;0;0], flags, cfg, [0;0;0]);

disp('--- SAU CẬP NHẬT GNSS TÁI HÒA NHẬP ---');
fprintf('update_status(1) = %d (1 = accepted, 0 = rejected)\n', update_status(1));
fprintf('NIS = %.2f\n', nis(1));
fprintf('Sai số vị trí sau cập nhật = %.4f m\n', norm(state_new(1:3) - pos_true));
fprintf('Độ lệch chuẩn P_pos sau cập nhật = %.4f m\n', sqrt(mean(diag(P_new(1:3, 1:3)))));

assert(update_status(1) == 1, 'GNSS phải được chấp nhận!');
assert(norm(state_new(1:3) - pos_true) < 0.2, 'Sai số vị trí sau GNSS phải < 0.2m!');
disp('>>> XÁC THỰC THÀNH CÔNG: GNSS RE-ACQUISITION HOẠT ĐỘNG HOÀN HẢO! <<<');
