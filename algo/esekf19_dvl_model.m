function [h_dvl, H_dvl, R_dvl, velocity_at_dvl, body_rate_relative, Cbs] = ...
    esekf19_dvl_model(state, gyro_meas, cfg)
%ESEKF19_DVL_MODEL Hàm phụ trợ dùng chung cho production và test:
%   Tính toán dự đoán đo DVL, ma trận Jacobian H_dvl, và ma trận hiệp phương sai R_dvl.
%
%   Đầu vào:
%       state     - Vector trạng thái danh định 20 chiều:
%                   [pos(1:3); vel(4:6); quat(7:10); ba(11:13); bg(14:16); scale(17); boresight(18:20)]
%       gyro_meas - Mẫu đo con quay khung thân [wx; wy; wz] (rad/s)
%       cfg       - Vector tham số hệ thống (chứa dt, lever_arm, w_ie, noise params...)
%
%   Đầu ra:
%       h_dvl              - Vận tốc đo dự đoán tại DVL: (1 + s) * Cbs * v_dvl (3x1)
%       H_dvl              - Ma trận quan sát giải tích (3x19)
%       R_dvl              - Ma trận hiệp phương sai nhiễu đo (3x3, bao gồm cả số hạng bù E1)
%       velocity_at_dvl    - Vận tốc khung thân tại đầu đo DVL (3x1)
%       body_rate_relative - Vận tốc góc tương đối khung thân (3x1)
%       Cbs                - Ma trận quay góc lắp đặt Boresight trên SO(3) (3x3)

dt = cfg(1);
w_ie_ned = cfg(3:5);
radius_n = cfg(6);
radius_m = cfg(7);
tan_latitude = cfg(8);
height = cfg(9);
lever_arm = cfg(10:12);

velocity = state(4:6);
quaternion = state(7:10);
gyro_bias = state(14:16);
scale = state(17);
boresight = state(18:20);

% 1. Ma trận xoay và vận tốc khung thân
Cbn = quat_to_cbn_local(quaternion);
Cnb = Cbn';
velocity_body = Cnb * velocity;

% 2. Bù bias con quay và vận tốc góc chuyển dời
gyro_corrected = gyro_meas - gyro_bias;
w_en_ned = [velocity(2)/(radius_n + height); ...
           -velocity(1)/(radius_m + height); ...
           -velocity(2)*tan_latitude/(radius_n + height)];
w_in_body = Cnb * (w_ie_ned + w_en_ned);
body_rate_relative = gyro_corrected - w_in_body;

% 3. Bù vận tốc cảm ứng do cánh tay đòn
velocity_at_dvl = velocity_body + cross(body_rate_relative, lever_arm);

% 4. Ma trận Boresight chuẩn trên nhóm Lie SO(3)
Cbs = rodrigues_so3_local(-boresight);

% 5. Dự đoán phép đo DVL
h_dvl = (1 + scale) * Cbs * velocity_at_dvl;

% 6. Xây dựng ma trận Jacobi đo đạc H_dvl (3x19)
H_dvl = zeros(3, 19);
H_dvl(:, 4:6)   = (1 + scale) * Cbs * Cnb; % Đạo hàm theo sai số vận tốc NED
H_dvl(:, 7:9)   = (1 + scale) * Cbs * (skew_mat_local(velocity_body) + skew_mat_local(lever_arm) * skew_mat_local(w_in_body)); % Đạo hàm theo delta_theta
H_dvl(:, 13:15) = (1 + scale) * Cbs * skew_mat_local(lever_arm); % Đạo hàm theo sai số bias con quay
if norm(velocity_at_dvl) ~= 0
    H_dvl(:, 16)    = Cbs * velocity_at_dvl;                      % Đạo hàm theo scale factor s
    H_dvl(:, 17:19) = (1 + scale) * Cbs * skew_mat_local(velocity_at_dvl); % Đạo hàm theo 3 góc Boresight
end

% 7. Tính phương sai nhiễu đo thích nghi theo vận tốc
sigma_dvl = cfg(13) + cfg(14) * norm(velocity_at_dvl);
R_dvl = (sigma_dvl^2) * eye(3);

% Thí nghiệm E1: Bổ sung covariance do nhiễu trắng con quay truyền qua cánh tay đòn
if length(cfg) >= 20 && cfg(20) > 0
    sigma_arw = cfg(20);
    sigma_g = sigma_arw / sqrt(dt);
    skew_r = skew_mat_local(lever_arm);
    R_gyro_lever = (1 + scale)^2 * Cbs * skew_r * (sigma_g^2 * eye(3)) * skew_r' * Cbs';
    R_dvl = R_dvl + R_gyro_lever;
end

end

%% Các hàm phụ trợ cục bộ
function R = rodrigues_so3_local(phi)
theta = norm(phi);
K = skew_mat_local(phi);
if theta > 1e-8
    R = eye(3) + (sin(theta)/theta)*K + ((1 - cos(theta))/(theta^2))*(K*K);
else
    R = eye(3) + K + 0.5*(K*K);
end
end

function S = skew_mat_local(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function Cbn = quat_to_cbn_local(q)
q = q / norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
       2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
       2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end
