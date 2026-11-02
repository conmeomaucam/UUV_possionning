function sfun_remus100_sensors(block)
%SFUN_REMUS100_SENSORS Level-2 MATLAB S-Function mô phỏng cụm cảm biến đo đạc REMUS 100
%   Cảm biến tích hợp theo đúng Datasheet:
%     1. IMU Tactical MEMS VectorNav VN-210-S (100 Hz): ARW, VRW, In-run bias Gauss-Markov
%     2. DVL Water Linked A125 (5 Hz): Vận tốc bám đáy, Boresight lệch [1.5; -2.0; 3.0] deg,
%        Scale factor +1.5%, Lever arm [0.5; -0.1; 0.2] m, độ chính xác 0.15% (noise 2 mm/s, slope 0.0015)
%     3. Áp kế đo sâu BlueRobotics Bar10XT (5 Hz): Sai số danh định 0.08 m
%     4. GNSS RTK (5 Hz): Hoạt động khi nổi (z <= 0.5m), cắt vĩnh viễn khi lặn
%     5. Từ kế 3 trục VN-210-S (5 Hz): Sai số hướng 2.0 deg RMS, chịu ảnh hưởng từ trường thân tàu

setup(block);
end

function setup(block)
% Tham số cấu hình cảm biến (Dialog Parameters)
% Param 1: sensor_params (struct chứa ins_spec, dvl_spec, bar_spec, true_scale, true_boresight, lever_arm, lat_rad)
block.NumDialogPrms = 1;

% Khai báo số cổng vào (Inputs)
block.NumInputPorts = 3;
% Inport 1: x_phys (12x1: [u; v; w; p; q; r; x; y; z; phi; theta; psi])
block.InputPort(1).Dimensions        = 12;
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode      = 'Sample';

% Inport 2: ui (3x1: [delta_r; delta_s; n_p])
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode      = 'Sample';

% Inport 3: xdot (12x1: đạo hàm trạng thái vật lý để tính lực riêng f_b chính xác)
block.InputPort(3).Dimensions        = 12;
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode      = 'Sample';

% Khai báo số cổng ra (Outputs)
block.NumOutputPorts = 6;
% Outport 1: imu_meas [f_b (3x1); omega_b (3x1)] (6x1, 100 Hz)
block.OutputPort(1).Dimensions       = 6;
block.OutputPort(1).SamplingMode     = 'Sample';

% Outport 2: dvl_meas (3x1, cập nhật 5 Hz)
block.OutputPort(2).Dimensions       = 3;
block.OutputPort(2).SamplingMode     = 'Sample';

% Outport 3: depth_meas (1x1, cập nhật 5 Hz)
block.OutputPort(3).Dimensions       = 1;
block.OutputPort(3).SamplingMode     = 'Sample';

% Outport 4: gnss_meas (7x1, cập nhật 5 Hz)
block.OutputPort(4).Dimensions       = 7;
block.OutputPort(4).SamplingMode     = 'Sample';

% Outport 5: mag_meas (3x1, cập nhật 5 Hz)
block.OutputPort(5).Dimensions       = 3;
block.OutputPort(5).SamplingMode     = 'Sample';

% Outport 6: flags [co_GNSS; co_Depth; co_DVL; co_MAG] (4x1)
block.OutputPort(6).Dimensions       = 4;
block.OutputPort(6).SamplingMode     = 'Sample';

% Chu kỳ trích mẫu 100 Hz
block.SampleTimes = [0.01 0];

block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',  @InitializeConditions);
block.RegBlockMethod('Outputs',               @Outputs);
block.RegBlockMethod('Update',                @Update);
end

function DoPostPropSetup(block)
block.NumDworks = 6;

% Dwork 1: imu_meas lưu đệm (6x1)
block.Dwork(1).Name            = 'imu_buff';
block.Dwork(1).Dimensions      = 6;
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';

% Dwork 2: dvl_meas lưu đệm (3x1)
block.Dwork(2).Name            = 'dvl_buff';
block.Dwork(2).Dimensions      = 3;
block.Dwork(2).DatatypeID      = 0;
block.Dwork(2).Complexity      = 'Real';

% Dwork 3: depth_meas lưu đệm (1x1)
block.Dwork(3).Name            = 'depth_buff';
block.Dwork(3).Dimensions      = 1;
block.Dwork(3).DatatypeID      = 0;
block.Dwork(3).Complexity      = 'Real';

% Dwork 4: gnss_meas lưu đệm (7x1)
block.Dwork(4).Name            = 'gnss_buff';
block.Dwork(4).Dimensions      = 7;
block.Dwork(4).DatatypeID      = 0;
block.Dwork(4).Complexity      = 'Real';

% Dwork 5: mag_meas lưu đệm (3x1)
block.Dwork(5).Name            = 'mag_buff';
block.Dwork(5).Dimensions      = 3;
block.Dwork(5).DatatypeID      = 0;
block.Dwork(5).Complexity      = 'Real';

% Dwork 6: flags lưu đệm (4x1)
block.Dwork(6).Name            = 'flags_buff';
block.Dwork(6).Dimensions      = 4;
block.Dwork(6).DatatypeID      = 0;
block.Dwork(6).Complexity      = 'Real';
end

function InitializeConditions(block)
block.Dwork(1).Data = [0; 0; -9.81; 0; 0; 0];
block.Dwork(2).Data = zeros(3, 1);
block.Dwork(3).Data = 0.0;
block.Dwork(4).Data = zeros(7, 1);
block.Dwork(5).Data = zeros(3, 1);
block.Dwork(6).Data = zeros(4, 1);

% Khởi tạo trạng thái ngẫu nhiên và bias in-run trong workspace UserData của block
p = block.DialogPrm(1).Data;
st.ba_inrun = p.ins_spec.acc.bias_instab * randn(3, 1);
st.bg_inrun = p.ins_spec.gyro.bias_instab * randn(3, 1);
st.ba_turn_on = [0.008; -0.012; 0.006];
st.bg_turn_on = [deg2rad(0.2)/3600; -deg2rad(0.15)/3600; deg2rad(0.3)/3600];
st.gnss_cut_done = false;
st.step_count = 0;
set_param(block.BlockHandle, 'UserData', st);
end

function Outputs(block)
block.OutputPort(1).Data = block.Dwork(1).Data;
block.OutputPort(2).Data = block.Dwork(2).Data;
block.OutputPort(3).Data = block.Dwork(3).Data;
block.OutputPort(4).Data = block.Dwork(4).Data;
block.OutputPort(5).Data = block.Dwork(5).Data;
block.OutputPort(6).Data = block.Dwork(6).Data;
end

function Update(block)
x_phys = block.InputPort(1).Data;
ui     = block.InputPort(2).Data;
xdot   = block.InputPort(3).Data;

p  = block.DialogPrm(1).Data;
st = get_param(block.BlockHandle, 'UserData');
st.step_count = st.step_count + 1;
dt = 0.01; % 100 Hz

% Tách biến trạng thái Fossen
v_body  = x_phys(1:3); % [u; v; w]
omega_b = x_phys(4:6); % [p; q; r]
p_ned   = x_phys(7:9); % [x; y; z]
euler   = x_phys(10:12); % [phi; theta; psi]
phi = euler(1); theta = euler(2); psi = euler(3);
n_p = ui(3);

% Ma trận xoay Cbn (Body sang NED)
cr = cos(phi); sr = sin(phi);
cp = cos(theta); sp = sin(theta);
cy = cos(psi); sy = sin(psi);
Cbn = [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy; ...
       cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy; ...
       -sp,   sr*cp,            cr*cp];

% Vận tốc trong hệ NED
v_ned = Cbn * v_body;

% Trọng lực chuẩn
g0 = gravity_fossen(p.lat_rad, p_ned(3));
g_ned = [0; 0; g0];
g_b = Cbn' * g_ned;

% Gia tốc chuyển động thật trong hệ thân: a_b = vdot_b + omega_b x v_body
vdot_b = xdot(1:3);
a_b = vdot_b + cross(omega_b, v_body);
f_b_true = a_b - g_b; % Lực riêng đo bởi gia tốc kế

% 1. IMU 100 Hz (Cập nhật liên tục mỗi chu kỳ 0.01 s)
pa = exp(-dt / p.ins_spec.acc.bias_tau);
pg = exp(-dt / p.ins_spec.gyro.bias_tau);
st.ba_inrun = pa * st.ba_inrun + p.ins_spec.acc.bias_instab * sqrt(max(1 - pa^2, 0)) * randn(3, 1);
st.bg_inrun = pg * st.bg_inrun + p.ins_spec.gyro.bias_instab * sqrt(max(1 - pg^2, 0)) * randn(3, 1);

ba_total = st.ba_turn_on + st.ba_inrun;
bg_total = st.bg_turn_on + st.bg_inrun;

w_accel = (p.ins_spec.acc.vrw / sqrt(dt)) * randn(3, 1);
w_gyro  = (p.ins_spec.gyro.arw / sqrt(dt)) * randn(3, 1);

accel_meas = f_b_true + ba_total + w_accel;
gyro_meas  = omega_b  + bg_total + w_gyro;

block.Dwork(1).Data = [accel_meas; gyro_meas];

% 2. Các cảm biến 5 Hz (Cập nhật sau mỗi 20 chu kỳ 100 Hz)
dvl_step = 20;
flags = [0; 0; 0; 0];

if mod(st.step_count, dvl_step) == 0
    dt_5hz = 0.20;

    % --- (a) Áp kế Bar10XT (5 Hz) ---
    sigma_depth = p.bar_spec.sigma_depth_nominal;
    depth_meas  = p_ned(3) + sigma_depth * randn;
    block.Dwork(3).Data = depth_meas;
    flags(2) = 1;

    % --- (b) DVL A125 (5 Hz) ---
    Cbs_true = rodrigues_local(-p.true_boresight);
    % Vận tốc tại vị trí lắp đặt DVL: v_dvl_body = v_body + omega_b x lever_arm
    v_at_dvl = v_body + cross(omega_b, p.lever_arm);
    % Chiếu sang khung cảm biến DVL có Boresight và Scale factor
    v_dvl_clean = (1 + p.true_scale) * (Cbs_true * v_at_dvl);
    % Nhiễu đo theo chuẩn Datasheet 0.15% (floor = 0.002 m/s, slope = 0.0015)
    sigma_dvl = p.dvl_spec.noise_floor + p.dvl_spec.noise_slope * norm(v_dvl_clean);
    dvl_meas  = v_dvl_clean + sigma_dvl * randn(3, 1);
    block.Dwork(2).Data = dvl_meas;
    flags(3) = 1;

    % --- (c) Từ kế VN-210-S (5 Hz) ---
    % Vector từ trường NED
    F = p.ins_spec.mag.field_total;
    I = p.ins_spec.mag.inclination;
    D = p.ins_spec.mag.declination;
    H_horiz = F * cos(I);
    m_ned = [H_horiz * cos(D); H_horiz * sin(D); F * sin(I)];
    
    is_submerged = (p_ned(3) > 0.5);
    if is_submerged
        yaw_err_deg = 1.0;
        if evalin('base', 'exist(''injected_yaw_submerged_deg'', ''var'')')
            yaw_err_deg = evalin('base', 'injected_yaw_submerged_deg');
        end
        if yaw_err_deg ~= 0
            psi_bias_rad = deg2rad(yaw_err_deg);
            R_z_bias = [cos(psi_bias_rad), -sin(psi_bias_rad), 0; ...
                        sin(psi_bias_rad),  cos(psi_bias_rad), 0; ...
                        0,                  0,                 1];
            m_true_body = Cbn' * (R_z_bias * m_ned);
        else
            m_true_body = Cbn' * m_ned;
        end
    else
        m_true_body = Cbn' * m_ned;
    end
    % Số đo tuân thủ đúng nhiễu ngẫu nhiên Gauss theo Datasheet VN-210-S (2.0 deg RMS)
    sigma_mag = p.ins_spec.mag.noise_density / sqrt(dt_5hz);
    mag_meas = m_true_body + sigma_mag * randn(3, 1);
    block.Dwork(5).Data = mag_meas;
    flags(4) = 1;

    % --- (d) GNSS RTK (5 Hz) ---
    % Nước biển hấp thụ hoàn toàn sóng vệ tinh khi ăng-ten chìm sâu > 0.5m
    if p_ned(3) > 0.5
        flags(1) = 0; % Mất tín hiệu GNSS hoàn toàn khi lặn ngầm
    else
        pos_noise = [p.ins_spec.gnss.pos_h * randn; p.ins_spec.gnss.pos_h * randn; p.ins_spec.gnss.pos_v * randn];
        vel_noise = p.ins_spec.gnss.vel * randn(3, 1);
        gnss_meas = [p_ned + pos_noise; v_ned + vel_noise; NaN]; % Đơn ăng-ten (không đo yaw)
        block.Dwork(4).Data = gnss_meas;
        flags(1) = 1;
    end

    block.Dwork(6).Data = flags;
else
    % Giữa các chu kỳ 5 Hz: cờ đo đạc đặt về 0
    block.Dwork(6).Data = [0; 0; 0; 0];
end

set_param(block.BlockHandle, 'UserData', st);
end

function R = rodrigues_local(phi)
theta = norm(phi);
K = [0, -phi(3), phi(2); phi(3), 0, -phi(1); -phi(2), phi(1), 0];
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
end
end
