function sfun_alos3d_control(block)
%SFUN_ALOS3D_CONTROL Level-2 MATLAB S-Function thực hiện dẫn đường 3D ALOS và điều khiển khép vòng
%   Nhận tín hiệu phản hồi từ ước lượng của ES-EKF:
%     - Dẫn đường ALOS3D bám Waypoints nhiệm vụ
%     - Autopilot góc hướng (Integral Sliding Mode Control - ISMC) điều khiển bánh lái delta_r
%     - Autopilot góc chúi / độ sâu điều khiển cánh lặn delta_s
%     - Điều khiển vòng tua chân vịt n_p theo tốc độ đặt từng chặng

setup(block);
end

function setup(block)
% Param 1: wpt (struct waypoints nhiệm vụ)
% Param 2: ctrl_prm (struct thông số bộ điều khiển)
block.NumDialogPrms = 2;

% Khai báo số cổng vào (Inputs)
block.NumInputPorts = 4;
% Inport 1: pos_est (3x1: [p_N; p_E; p_D])
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode      = 'Sample';

% Inport 2: vel_est (3x1: [v_N; v_E; v_D])
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode      = 'Sample';

% Inport 3: euler_est (3x1: [phi; theta; psi])
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode      = 'Sample';

% Inport 4: gyro_meas (3x1: [p; q; r])
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DirectFeedthrough = false;
block.InputPort(4).SamplingMode      = 'Sample';

% Khai báo số cổng ra (Outputs)
block.NumOutputPorts = 2;
% Outport 1: ui [delta_r; delta_s; n_p] (3x1)
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(1).SamplingMode     = 'Sample';

% Outport 2: guidance_info [curr_wp; y_e; z_e; psi_ref] (4x1)
block.OutputPort(2).Dimensions       = 4;
block.OutputPort(2).SamplingMode     = 'Sample';

% Chu kỳ trích mẫu 100 Hz
block.SampleTimes = [0.01 0];

block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',  @InitializeConditions);
block.RegBlockMethod('Outputs',               @Outputs);
block.RegBlockMethod('Update',                @Update);
end

function DoPostPropSetup(block)
block.NumDworks = 4;

% Dwork 1: ui_out [delta_r; delta_s; n_p] (3x1)
block.Dwork(1).Name            = 'ui_buff';
block.Dwork(1).Dimensions      = 3;
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';

% Dwork 2: guidance_buff [curr_wp; y_e; z_e; psi_ref] (4x1)
block.Dwork(2).Name            = 'guidance_buff';
block.Dwork(2).Dimensions      = 4;
block.Dwork(2).DatatypeID      = 0;
block.Dwork(2).Complexity      = 'Real';

% Dwork 3: Bộ quan sát và tích phân điều khiển (6x1)
% [psi_d; r_d; theta_d; q_d; theta_int; n_p]
block.Dwork(3).Name            = 'ctrl_states';
block.Dwork(3).Dimensions      = 6;
block.Dwork(3).DatatypeID      = 0;
block.Dwork(3).Complexity      = 'Real';

% Dwork 4: Chỉ số mốc hiện tại curr_wp_idx (1x1)
block.Dwork(4).Name            = 'wp_idx';
block.Dwork(4).Dimensions      = 1;
block.Dwork(4).DatatypeID      = 0;
block.Dwork(4).Complexity      = 'Real';
end

function InitializeConditions(block)
wpt = block.DialogPrm(1).Data;
psi0 = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));

block.Dwork(1).Data = [0; 0; 1300]; % [delta_r; delta_s; n_p ban đầu]
block.Dwork(2).Data = [1; 0; 0; psi0];
block.Dwork(3).Data = [psi0; 0; 0; 0; 0; 1300]; % [psi_d; r_d; theta_d; q_d; theta_int; n_p]
block.Dwork(4).Data = 1; % mốc 1
end

function Outputs(block)
block.OutputPort(1).Data = block.Dwork(1).Data;
block.OutputPort(2).Data = block.Dwork(2).Data;
end

function Update(block)
pos_est   = block.InputPort(1).Data;
euler_est = block.InputPort(3).Data;
gyro_meas = block.InputPort(4).Data;

wpt       = block.DialogPrm(1).Data;
cp        = block.DialogPrm(2).Data;

h = 0.01; % 100 Hz
curr_wp_idx = block.Dwork(4).Data;
c_states    = block.Dwork(3).Data;
psi_d       = c_states(1);
r_d         = c_states(2);
theta_d     = c_states(3);
q_d         = c_states(4);
theta_int   = c_states(5);
n_p         = c_states(6);

phi_hat   = euler_est(1);
theta_hat = euler_est(2);
psi_hat   = euler_est(3);

% Nếu đang trong pha căn chỉnh tĩnh ban đầu (chưa có vị trí ước lượng)
if all(pos_est == 0) && block.CurrentTime < 3.0
    delta_r = 0;
    delta_s = 0;
    n_p = 1300;
    block.Dwork(1).Data = [delta_r; -delta_s; n_p];
    return;
end

% 1. Thuật toán dẫn đường ALOS3D
Delta_h = cp.Delta_h;
Delta_v = cp.Delta_v;
gamma_h = cp.gamma_h;
gamma_v = cp.gamma_v;
M_theta = cp.M_theta;
R_switch = cp.R_switch;

[psi_ref, theta_ref, y_e, z_e, ~, ~] = ALOS3D( ...
    pos_est(1), pos_est(2), pos_est(3), ...
    Delta_h, Delta_v, gamma_h, gamma_v, M_theta, h, R_switch, wpt);

% 2. Bộ quan sát lọc tín hiệu đặt LOSObserver
K_f = cp.K_f;
r_max = cp.r_max;
[theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
[psi_d, r_d]   = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
r_d = sat(r_d, r_max);

% 3. Autopilot góc hướng (Integral Sliding Mode Control)
r_gyro = gyro_meas(3);
delta_r = integralSMCheading(psi_hat, r_gyro, psi_d, r_d, cp.a_d, ...
    cp.K_d, cp.K_sigma, 1, cp.phi_b, cp.K_yaw, cp.T_yaw, h);
delta_r = sat(delta_r, cp.delta_max);

% 4. Autopilot góc chúi / độ sâu
q_gyro = gyro_meas(2);
theta_int = theta_int + h * ssa(theta_hat - theta_d);
delta_s = -cp.Kp_theta * ssa(theta_hat - theta_d) - cp.Kd_theta * q_gyro - cp.Ki_theta * theta_int;
delta_s = sat(delta_s, cp.delta_max);

% 5. Điều khiển chân vịt theo tốc độ mong muốn của mốc
if isfield(wpt, 'speed') && numel(wpt.speed) >= curr_wp_idx
    u_target = wpt.speed(curr_wp_idx);
    n_d = sat(round(u_target * 870), cp.n_max);
else
    n_d = 1300;
end
if n_p < n_d
    n_p = n_p + cp.n_rate;
elseif n_p > n_d
    n_p = n_p - cp.n_rate;
end
n_p = sat(n_p, cp.n_max);

% Kiểm tra chuyển mốc (Half-plane boundary & R_switch)
if curr_wp_idx < length(wpt.pos.x)
    dist_curr = norm([pos_est(1) - wpt.pos.x(curr_wp_idx); ...
                      pos_est(2) - wpt.pos.y(curr_wp_idx); ...
                      pos_est(3) - wpt.pos.z(curr_wp_idx)]);
    if curr_wp_idx > 1
        v_leg = [wpt.pos.x(curr_wp_idx) - wpt.pos.x(curr_wp_idx-1); ...
                 wpt.pos.y(curr_wp_idx) - wpt.pos.y(curr_wp_idx-1); ...
                 wpt.pos.z(curr_wp_idx) - wpt.pos.z(curr_wp_idx-1)];
    else
        v_leg = [wpt.pos.x(2) - wpt.pos.x(1); ...
                 wpt.pos.y(2) - wpt.pos.y(1); ...
                 wpt.pos.z(2) - wpt.pos.z(1)];
    end
    v_to_curr = [pos_est(1) - wpt.pos.x(curr_wp_idx); ...
                 pos_est(2) - wpt.pos.y(curr_wp_idx); ...
                 pos_est(3) - wpt.pos.z(curr_wp_idx)];
    passed_plane = (dot(v_to_curr, v_leg) >= 0 && dist_curr < R_switch * 3);
    if (dist_curr < R_switch) || passed_plane
        curr_wp_idx = curr_wp_idx + 1;
    end
end

% Đóng gói đầu ra và lưu trạng thái
ui_out = [delta_r; -delta_s; n_p];
block.Dwork(1).Data = ui_out;
block.Dwork(2).Data = [curr_wp_idx; y_e; z_e; psi_ref];
block.Dwork(3).Data = [psi_d; r_d; theta_d; q_d; theta_int; n_p];
block.Dwork(4).Data = curr_wp_idx;
end

function z = sat(x, x_max)
z = min(max(x, -x_max), x_max);
end

function angle = ssa(angle)
angle = mod(angle + pi, 2*pi) - pi;
end
