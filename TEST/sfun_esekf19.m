function sfun_esekf19(block)
%SFUN_ESEKF19 Level-2 MATLAB S-Function thực thi bộ lọc sai số ES-EKF 19 trạng thái (100 Hz)
%   Đóng gói thuật toán esekf19_step và esekf19_init trong môi trường Simulink.
%   Hỗ trợ đầy đủ:
%     - Căn chỉnh tĩnh (Leveling 2s gia tốc kế + Heading từ kế/GNSS)
%     - Cập nhật quán tính INS 100 Hz
%     - Hiệu chuẩn trực tuyến Boresight [Roll; Pitch; Yaw] và Scale Factor DVL khi nổi
%     - Khóa hiệu chuẩn và duy trì định vị ngầm khi lặn
%
% Tác giả: Nhóm phát triển thuật toán ES-EKF REMUS 100

setup(block);
end

function setup(block)
% Số tham số đầu vào của block (Dialog Parameters)
% Param 1: cfg (23x1) - vector cấu hình bộ lọc
% Param 2: noise_cfg (8x1) - vector thông số nhiễu quá trình IMU
block.NumDialogPrms = 2;

% Khai báo số cổng vào (Inputs)
block.NumInputPorts = 6;
% 1. imu_meas: [f_bx; f_by; f_bz; w_bx; w_by; w_bz] (6x1)
block.InputPort(1).Dimensions        = 6;
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode      = 'Sample';

% 2. dvl_meas: [v_dx; v_dy; v_dz] (3x1)
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode      = 'Sample';

% 3. depth_meas: độ sâu thủy tĩnh (1x1)
block.InputPort(3).Dimensions        = 1;
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode      = 'Sample';

% 4. gnss_meas: [p_N; p_E; p_D; v_N; v_E; v_D; yaw_gnss] (7x1)
block.InputPort(4).Dimensions        = 7;
block.InputPort(4).DirectFeedthrough = false;
block.InputPort(4).SamplingMode      = 'Sample';

% 5. mag_meas: từ trường 3 trục hệ thân [m_x; m_y; m_z] (3x1)
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DirectFeedthrough = false;
block.InputPort(5).SamplingMode      = 'Sample';

% 6. flags: cờ cảm biến [co_GNSS; co_Depth; co_DVL; co_MAG] (4x1)
block.InputPort(6).Dimensions        = 4;
block.InputPort(6).DirectFeedthrough = false;
block.InputPort(6).SamplingMode      = 'Sample';

% Khai báo số cổng ra (Outputs)
block.NumOutputPorts = 7;
% 1. pos_est: [p_N; p_E; p_D] (3x1)
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(1).SamplingMode     = 'Sample';

% 2. vel_est: [v_N; v_E; v_D] (3x1)
block.OutputPort(2).Dimensions       = 3;
block.OutputPort(2).SamplingMode     = 'Sample';

% 3. euler_est: [phi; theta; psi] (3x1, rad)
block.OutputPort(3).Dimensions       = 3;
block.OutputPort(3).SamplingMode     = 'Sample';

% 4. boresight_est: [Roll; Pitch; Yaw] (3x1, deg)
block.OutputPort(4).Dimensions       = 3;
block.OutputPort(4).SamplingMode     = 'Sample';

% 5. scale_est: Scale Factor DVL (1x1, %)
block.OutputPort(5).Dimensions       = 1;
block.OutputPort(5).SamplingMode     = 'Sample';

% 6. cov_diag: 1-sigma độ lệch chuẩn 19 biến trạng thái (19x1)
block.OutputPort(6).Dimensions       = 19;
block.OutputPort(6).SamplingMode     = 'Sample';

% 7. status: [upd_gnss; upd_depth; upd_dvl; upd_mag] (4x1)
block.OutputPort(7).Dimensions       = 4;
block.OutputPort(7).SamplingMode     = 'Sample';

% Chu kỳ trích mẫu rời rạc 100 Hz (h = 0.01 s)
block.SampleTimes = [0.01 0];

% Đăng ký các phương thức vòng đời
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',  @InitializeConditions);
block.RegBlockMethod('Outputs',               @Outputs);
block.RegBlockMethod('Update',                @Update);
end

function DoPostPropSetup(block)
% Cấu hình bộ nhớ trạng thái nội bộ Dwork
block.NumDworks = 6;

% Dwork 1: State vector danh định (20x1)
block.Dwork(1).Name            = 'state_est';
block.Dwork(1).Dimensions      = 20;
block.Dwork(1).DatatypeID      = 0; % double
block.Dwork(1).Complexity      = 'Real';

% Dwork 2: Ma trận hiệp phương sai P (19x19 = 361 phần tử)
block.Dwork(2).Name            = 'P_est';
block.Dwork(2).Dimensions      = 361;
block.Dwork(2).DatatypeID      = 0;
block.Dwork(2).Complexity      = 'Real';

% Dwork 3: Ma trận hiệp phương sai nhiễu quá trình Q (19x19 = 361 phần tử)
block.Dwork(3).Name            = 'Q_mat';
block.Dwork(3).Dimensions      = 361;
block.Dwork(3).DatatypeID      = 0;
block.Dwork(3).Complexity      = 'Real';

% Dwork 4: Bộ đếm căn chỉnh và góc ban đầu (11x1)
% [align_phase; level_count; head_count; roll0; pitch0; psi0; head_sin; head_cos; gnss_cut_done; t_align; step_count]
block.Dwork(4).Name            = 'align_vars';
block.Dwork(4).Dimensions      = 11;
block.Dwork(4).DatatypeID      = 0;
block.Dwork(4).Complexity      = 'Real';

% Dwork 5: Tích lũy gia tốc kế cho leveling (3x1)
block.Dwork(5).Name            = 'accel_sum';
block.Dwork(5).Dimensions      = 3;
block.Dwork(5).DatatypeID      = 0;
block.Dwork(5).Complexity      = 'Real';

% Dwork 6: Lưu vết trạng thái cập nhật (4x1)
block.Dwork(6).Name            = 'upd_status';
block.Dwork(6).Dimensions      = 4;
block.Dwork(6).DatatypeID      = 0;
block.Dwork(6).Complexity      = 'Real';
end

function InitializeConditions(block)
% Khởi tạo các trạng thái ban đầu khi bắt đầu mô phỏng
block.Dwork(1).Data = zeros(20, 1);
P0 = eye(19);
block.Dwork(2).Data = P0(:);
Q0 = zeros(19, 19);
block.Dwork(3).Data = Q0(:);

% align_phase = 1 (bắt đầu leveling), level_count = 0, head_count = 0, step_count = 0
block.Dwork(4).Data = [1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
block.Dwork(5).Data = zeros(3, 1);
block.Dwork(6).Data = zeros(4, 1);
end

function Outputs(block)
% Trích xuất các ước lượng hiện tại ra cổng Output
state_est = block.Dwork(1).Data;
P_flat    = block.Dwork(2).Data;
P_est     = reshape(P_flat, 19, 19);
align_vars = block.Dwork(4).Data;
align_phase = align_vars(1);

if align_phase == 3
    % Đã căn chỉnh xong: xuất trạng thái thực tế từ bộ lọc
    p_hat = state_est(1:3);
    v_hat = state_est(4:6);
    q_hat = state_est(7:10);
    [phi_hat, theta_hat, psi_hat] = quat2euler_local(q_hat);
    scale_hat_pct = state_est(17) * 100; % đổi ra %
    bs_hat_deg    = rad2deg(state_est(18:20)); % đổi ra deg
    cov_diag      = sqrt(max(diag(P_est), 0));
else
    % Đang trong pha căn chỉnh ban đầu (2-3 giây đầu)
    p_hat = zeros(3, 1);
    v_hat = zeros(3, 1);
    phi_hat   = align_vars(4);
    theta_hat = align_vars(5);
    psi_hat   = align_vars(6);
    scale_hat_pct = 0.0;
    bs_hat_deg    = zeros(3, 1);
    cov_diag      = sqrt(max(diag(P_est), 0));
end

block.OutputPort(1).Data = p_hat;
block.OutputPort(2).Data = v_hat;
block.OutputPort(3).Data = [phi_hat; theta_hat; psi_hat];
block.OutputPort(4).Data = bs_hat_deg;
block.OutputPort(5).Data = scale_hat_pct;
block.OutputPort(6).Data = cov_diag;
block.OutputPort(7).Data = block.Dwork(6).Data;
end

function Update(block)
% Chu kỳ cập nhật trạng thái thuật toán ES-EKF (100 Hz)
imu_meas   = block.InputPort(1).Data;
dvl_meas   = block.InputPort(2).Data;
depth_meas = block.InputPort(3).Data;
gnss_meas  = block.InputPort(4).Data;
mag_meas   = block.InputPort(5).Data;
flags      = block.InputPort(6).Data;

cfg       = block.DialogPrm(1).Data;
noise_cfg = block.DialogPrm(2).Data;
h         = cfg(1);
gravity   = cfg(2);

accel_meas = imu_meas(1:3);
gyro_meas  = imu_meas(4:6);

state_est  = block.Dwork(1).Data;
P_flat     = block.Dwork(2).Data;
P_est      = reshape(P_flat, 19, 19);
Q_flat     = block.Dwork(3).Data;
Q          = reshape(Q_flat, 19, 19);
align_vars = block.Dwork(4).Data;
accel_sum  = block.Dwork(5).Data;

align_phase = align_vars(1);
level_count = align_vars(2);
head_count  = align_vars(3);
roll0       = align_vars(4);
pitch0      = align_vars(5);
psi0        = align_vars(6);
head_sin    = align_vars(7);
head_cos    = align_vars(8);
gnss_cut    = align_vars(9);
t_align     = align_vars(10);
step_count  = align_vars(11) + 1;
t_curr      = step_count * h;

N_level = round(2.0 / h); % 2 giây leveling = 200 mẫu ở 100 Hz
N_head  = 5;              % 5 mẫu đo hướng (1 giây ở 5 Hz)
upd_stat = zeros(4, 1);

% Quản lý tự động khóa GNSS khi lặn ngầm theo độ sâu đo được từ cảm biến áp suất (> 0.5m)
% sau khi hoàn thành chặng nổi hiệu chuẩn Boresight (t >= 410s)
if t_curr >= 410.0 && flags(2) ~= 0 && depth_meas > 0.5 && gnss_cut == 0
    gnss_cut = 1;
    % Bơm bước nhảy sai số góc hướng vào EKF tại thời điểm bắt đầu lặn ngầm
    yaw_bias_deg = 0.5;
    if evalin('base', 'exist(''injected_yaw_submerged_deg'', ''var'')')
        yaw_bias_deg = evalin('base', 'injected_yaw_submerged_deg');
    end
    if yaw_bias_deg ~= 0
        d_psi = deg2rad(yaw_bias_deg);
        dq = [cos(d_psi/2); 0; 0; sin(d_psi/2)];
        qc = state_est(7:10);
        q_new = [qc(1)*dq(1) - qc(4)*dq(4); ...
                 qc(2)*dq(1) + qc(3)*dq(4); ...
                 qc(3)*dq(1) - qc(2)*dq(4); ...
                 qc(1)*dq(4) + qc(4)*dq(1)];
        state_est(7:10) = q_new / norm(q_new);
        fprintf('\n[EKF FAULT INJECTION] Tàu lặn độ sâu = %.2f m (> 0.5m tại t = %.1f s): Đã bơm sai số hướng %+5.2f độ vào EKF!\n', ...
            depth_meas, t_curr, yaw_bias_deg);
    end
end
if gnss_cut == 1
    flags(1) = 0;   % Tự động vô hiệu hóa GNSS khi ngập nước
    cfg(21)  = 1;   % Tự động khóa hiệu chuẩn Boresight và Scale Factor
else
    cfg(21)  = 0;   % Cho phép hiệu chuẩn trực tuyến khi nổi
end

if align_phase == 1
    % --- Pha 1: Căn chỉnh thô độ nghiêng (Leveling) bằng gia tốc kế ---
    accel_sum   = accel_sum + accel_meas;
    level_count = level_count + 1;
    if level_count >= N_level
        f_avg  = accel_sum / level_count;
        roll0  = atan2(-f_avg(2), -f_avg(3));
        pitch0 = atan2(f_avg(1), sqrt(f_avg(2)^2 + f_avg(3)^2));
        align_phase = 2;
    end

elseif align_phase == 2
    % --- Pha 2: Căn chỉnh thô góc hướng (Heading Alignment) từ Từ kế/GNSS ---
    if flags(4) ~= 0
        % Bù nghiêng cho từ kế bằng roll0, pitch0
        cr0 = cos(roll0); sr0 = sin(roll0);
        cp0 = cos(pitch0); sp0 = sin(pitch0);
        Rx0 = [1, 0, 0; 0, cr0, -sr0; 0, sr0, cr0];
        Ry0 = [cp0, 0, sp0; 0, 1, 0; -sp0, 0, cp0];
        m_level = Ry0 * Rx0 * mag_meas;
        dec = cfg(22);
        psi_sample = atan2(-m_level(2), m_level(1)) + dec;

        head_sin = head_sin + sin(psi_sample);
        head_cos = head_cos + cos(psi_sample);
        head_count = head_count + 1;

        if head_count >= N_head
            psi0 = atan2(head_sin, head_cos);

            % Khởi tạo bộ lọc ES-EKF 19 trạng thái
            [state_est, P_est, Q] = esekf19_init([roll0; pitch0; psi0], gravity, noise_cfg);
            if flags(1) ~= 0 || any(gnss_meas(1:3) ~= 0)
                state_est(1:3) = gnss_meas(1:3);
                state_est(4:6) = gnss_meas(4:6);
            end
            if flags(2) ~= 0
                state_est(3) = depth_meas;
            end
            align_phase = 3;
            t_align = block.CurrentTime;
        end
    end

else
    % --- Pha 3: Chế độ điều hướng định vị tích hợp ES-EKF (100 Hz) ---
    [state_est, P_est, upd_stat, ~] = esekf19_step( ...
        state_est, P_est, Q, gyro_meas, accel_meas, ...
        gnss_meas, depth_meas, dvl_meas, flags, cfg, mag_meas);
end

% Lưu lại Dwork
block.Dwork(1).Data = state_est;
block.Dwork(2).Data = P_est(:);
block.Dwork(3).Data = Q(:);
block.Dwork(4).Data = [align_phase; level_count; head_count; roll0; pitch0; psi0; head_sin; head_cos; gnss_cut; t_align; step_count];
block.Dwork(5).Data = accel_sum;
block.Dwork(6).Data = upd_stat;
end

function [phi, theta, psi] = quat2euler_local(q)
q = q / norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);
phi = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
sin_theta = max(min(2*(w*y - z*x), 1), -1);
theta = asin(sin_theta);
psi = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
end
