% BUILD_REMUS100_ESEKF_MODEL
% Tự động tạo mới và đấu nối mô hình Simulink tích hợp REMUS 100 + ES-EKF 19 trạng thái (100 Hz).
% Mô hình: TEST/remus100_esekf_integrated.slx

mdl = 'remus100_esekf_integrated';
script_dir = fileparts(mfilename('fullpath'));
slx_path = fullfile(script_dir, [mdl '.slx']);

% 0. Nạp biến vào không gian làm việc
init_remus100_esekf_simulink;

% Đóng mô hình nếu đang mở
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

% Tạo hệ thống Simulink mới hoàn toàn sạch
new_system(mdl);
load_system(mdl);

% 1. Thiết lập cấu hình Solver: Fixed-step Runge-Kutta 4 (ode4) ở 100 Hz (h = 0.01s)
set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver', 'ode4');
set_param(mdl, 'FixedStep', '0.01');
set_param(mdl, 'StopTime', '2668');
set_param(mdl, 'InitFcn', 'init_remus100_esekf_simulink;');
set_param(mdl, 'ReturnWorkspaceOutputs', 'on');

% 2. THÊM CÁC KHỐI CHÍNH
% (a) Dòng chảy đại dương
add_block('simulink/Sources/Constant', [mdl '/Ocean Current Speed Vc'], ...
    'Value', 'Vc', 'Position', [300, 160, 370, 190]);
add_block('simulink/Sources/Constant', [mdl '/Ocean Current Dir betaVc'], ...
    'Value', 'betaVc', 'Position', [300, 210, 370, 240]);

% (b) Khối mô hình động lực học 6-DOF Fossen REMUS 100
% Inports: ui (3x1), Vc (1x1), betaVc (1x1)
% Outports: x_phys (12x1), xdot (12x1), U (1x1)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/REMUS 100 Plant (6-DOF)'], ...
    'Position', [420, 110, 620, 250], ...
    'FunctionName', 'sfun_remus100_plant', ...
    'Parameters', 'x0, 0.1');

% (c) Cụm cảm biến đo đạc (Sensor Suite)
% Inports: x_phys (12x1), ui (3x1), xdot (12x1)
% Outports: imu_meas (6x1), dvl_meas (3x1), depth_meas (1x1), gnss_meas (7x1), mag_meas (3x1), flags (4x1)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/Sensor Suite'], ...
    'Position', [690, 110, 850, 250], ...
    'FunctionName', 'sfun_remus100_sensors', ...
    'Parameters', 'sensor_params');

% (d) Bộ lọc định vị và hiệu chuẩn ES-EKF 19 trạng thái (100 Hz)
% Inports: imu_meas, dvl_meas, depth_meas, gnss_meas, mag_meas, flags
% Outports: pos_est (3x1), vel_est (3x1), euler_est (3x1), bs_est (3x1), scale_est (1x1), cov_diag (19x1), status (4x1)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/ES-EKF 19 Navigation'], ...
    'Position', [920, 110, 1100, 250], ...
    'FunctionName', 'sfun_esekf19', ...
    'Parameters', 'cfg, noise_cfg');

% (e) Khối dẫn đường ALOS3D và điều khiển khép vòng (Guidance & Control)
% Inports: pos_est (3x1), vel_est (3x1), euler_est (3x1), gyro_meas (3x1)
% Outports: ui (3x1: [delta_r; delta_s; n_p]), guidance_info (4x1)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/ALOS3D Guidance & Control'], ...
    'Position', [60, 110, 240, 250], ...
    'FunctionName', 'sfun_alos3d_control', ...
    'Parameters', 'wpt, ctrl_prm');

% (f) Demux IMU tách gyro_meas cho bộ điều khiển
add_block('simulink/Signal Routing/Demux', [mdl '/Demux IMU'], ...
    'Outputs', '[3 3]', ...
    'Position', [875, 75, 880, 125]);

% (g) Demux trạng thái thật Fossen để ghi nhận kiểm chứng
add_block('simulink/Signal Routing/Demux', [mdl '/Demux Phys State'], ...
    'Outputs', '[6 3 3]', ...
    'Position', [650, 280, 655, 360]);

% (h) Các khối To Workspace xuất dữ liệu mô phỏng
logs = {
    'pos_est_sim',     [1160, 95, 1240, 115],  1;
    'vel_est_sim',     [1160, 120, 1240, 140], 2;
    'euler_est_sim',   [1160, 145, 1240, 165], 3;
    'boresight_sim',   [1160, 170, 1240, 190], 4;
    'scale_sim',       [1160, 195, 1240, 215], 5;
    'cov_diag_sim',    [1160, 220, 1240, 240], 6
};

for i = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{i, 1}], ...
        'VariableName', logs{i, 1}, ...
        'SaveFormat', 'Array', ...
        'SampleTime', '0.01', ...
        'Position', logs{i, 2});
end

add_block('simulink/Sinks/To Workspace', [mdl '/pos_true_sim'], ...
    'VariableName', 'pos_true_sim', ...
    'SaveFormat', 'Array', ...
    'SampleTime', '0.01', ...
    'Position', [700, 305, 780, 325]);

add_block('simulink/Sinks/To Workspace', [mdl '/euler_true_sim'], ...
    'VariableName', 'euler_true_sim', ...
    'SaveFormat', 'Array', ...
    'SampleTime', '0.01', ...
    'Position', [700, 335, 780, 355]);

add_block('simulink/Sinks/To Workspace', [mdl '/ui_sim'], ...
    'VariableName', 'ui_sim', ...
    'SaveFormat', 'Array', ...
    'SampleTime', '0.01', ...
    'Position', [280, 65, 340, 85]);

% Thêm Terminator cho các cổng chẩn đoán phụ
add_block('simulink/Sinks/Terminator', [mdl '/Terminator_Status'], ...
    'Position', [1130, 240, 1150, 260]);
add_block('simulink/Sinks/Terminator', [mdl '/Terminator_Guidance'], ...
    'Position', [265, 205, 285, 225]);
add_block('simulink/Sinks/Terminator', [mdl '/Terminator_U'], ...
    'Position', [645, 220, 665, 240]);
add_block('simulink/Sinks/Terminator', [mdl '/Terminator_VelBody'], ...
    'Position', [685, 280, 705, 300]);

% 3. ĐẤU NỐI ĐƯỜNG DÂY TÍN HIỆU
% (a) Tín hiệu điều khiển ui từ Guidance & Control
add_line(mdl, 'ALOS3D Guidance & Control/1', 'REMUS 100 Plant (6-DOF)/1', 'autorouting', 'on');
add_line(mdl, 'ALOS3D Guidance & Control/1', 'Sensor Suite/2', 'autorouting', 'on');
add_line(mdl, 'ALOS3D Guidance & Control/1', 'ui_sim/1', 'autorouting', 'on');
add_line(mdl, 'ALOS3D Guidance & Control/2', 'Terminator_Guidance/1', 'autorouting', 'on');

% Dòng chảy tới Plant
add_line(mdl, 'Ocean Current Speed Vc/1', 'REMUS 100 Plant (6-DOF)/2', 'autorouting', 'on');
add_line(mdl, 'Ocean Current Dir betaVc/1', 'REMUS 100 Plant (6-DOF)/3', 'autorouting', 'on');

% (b) Trạng thái từ Plant tới Sensor Suite & To Workspace
add_line(mdl, 'REMUS 100 Plant (6-DOF)/1', 'Sensor Suite/1', 'autorouting', 'on');
add_line(mdl, 'REMUS 100 Plant (6-DOF)/1', 'Demux Phys State/1', 'autorouting', 'on');
add_line(mdl, 'REMUS 100 Plant (6-DOF)/2', 'Sensor Suite/3', 'autorouting', 'on');
add_line(mdl, 'REMUS 100 Plant (6-DOF)/3', 'Terminator_U/1', 'autorouting', 'on');

add_line(mdl, 'Demux Phys State/1', 'Terminator_VelBody/1', 'autorouting', 'on');
add_line(mdl, 'Demux Phys State/2', 'pos_true_sim/1', 'autorouting', 'on');
add_line(mdl, 'Demux Phys State/3', 'euler_true_sim/1', 'autorouting', 'on');

% (c) Sensor Suite -> ES-EKF Navigation (6 ports)
add_line(mdl, 'Sensor Suite/1', 'ES-EKF 19 Navigation/1', 'autorouting', 'on');
add_line(mdl, 'Sensor Suite/1', 'Demux IMU/1', 'autorouting', 'on');
add_line(mdl, 'Sensor Suite/2', 'ES-EKF 19 Navigation/2', 'autorouting', 'on');
add_line(mdl, 'Sensor Suite/3', 'ES-EKF 19 Navigation/3', 'autorouting', 'on');
add_line(mdl, 'Sensor Suite/4', 'ES-EKF 19 Navigation/4', 'autorouting', 'on');
add_line(mdl, 'Sensor Suite/5', 'ES-EKF 19 Navigation/5', 'autorouting', 'on');
add_line(mdl, 'Sensor Suite/6', 'ES-EKF 19 Navigation/6', 'autorouting', 'on');

% Gyro meas từ IMU tới bộ điều khiển
add_line(mdl, 'Demux IMU/2', 'ALOS3D Guidance & Control/4', 'autorouting', 'on');

% (d) Ước lượng từ ES-EKF -> Guidance & Control (khép vòng phản hồi) & To Workspaces
add_line(mdl, 'ES-EKF 19 Navigation/1', 'ALOS3D Guidance & Control/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/2', 'ALOS3D Guidance & Control/2', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/3', 'ALOS3D Guidance & Control/3', 'autorouting', 'on');

add_line(mdl, 'ES-EKF 19 Navigation/1', 'pos_est_sim/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/2', 'vel_est_sim/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/3', 'euler_est_sim/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/4', 'boresight_sim/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/5', 'scale_sim/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/6', 'cov_diag_sim/1', 'autorouting', 'on');
add_line(mdl, 'ES-EKF 19 Navigation/7', 'Terminator_Status/1', 'autorouting', 'on');

% 4. Lưu và đóng mô hình
save_system(mdl, slx_path);
close_system(mdl);
fprintf('Đã tạo mới và đấu nối thành công 100%% mô hình tích hợp: %s\n', slx_path);
