function [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance, G] = ...
    esekf19_inject_error(delta_state, covariance, position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight)
%ESEKF19_INJECT_ERROR Bơm vector sai số ước lượng vào trạng thái danh định và đặt lại covariance.
%   Hàm phụ trợ dùng chung cho cả bộ lọc production (esekf19_step) và các script kiểm thử.
%
%   Đầu vào:
%       delta_state - Vector sai số ước lượng Kalman 19 chiều (19x1)
%       covariance  - Ma trận hiệp phương sai sai số P (19x19)
%       position    - Vị trí danh định NED (3x1)
%       velocity    - Vận tốc danh định NED (3x1)
%       quaternion  - Quaternion tư thế danh định [w; x; y; z] (4x1)
%       accel_bias  - Bias gia tốc kế (3x1)
%       gyro_bias   - Bias con quay (3x1)
%       scale       - Hệ số tỉ lệ DVL s (1x1)
%       boresight   - 3 góc lệch Boresight DVL (3x1)
%
%   Đầu ra:
%       Các biến trạng thái và covariance sau khi đã được cập nhật, cùng ma trận reset G (19x19).

% 1. Cộng bù trực tiếp cho các đại lượng vector tuyến tính
position = position + delta_state(1:3);
velocity = velocity + delta_state(4:6);

% 2. Nhân quaternion danh định với quaternion vi phân delta_theta
delta_theta = delta_state(7:9);
delta_quaternion = [1; 0.5*delta_theta];
delta_quaternion = delta_quaternion / norm(delta_quaternion);
quaternion = quat_multiply_local(quaternion, delta_quaternion);
quaternion = quaternion / norm(quaternion);

% 3. Cộng bù bias cảm biến và scale factor
accel_bias = accel_bias + delta_state(10:12);
gyro_bias  = gyro_bias  + delta_state(13:15);
scale      = scale      + delta_state(16);

% 4. Cập nhật Boresight chuẩn trên nhóm Lie SO(3): Cbs_new = Cbs_old * exp(-[delta_bs]x)
delta_bs  = delta_state(17:19);
Cbs_new   = rodrigues_so3_local(-boresight) * rodrigues_so3_local(-delta_bs);
boresight = -so3_log_local(Cbs_new);

% 5. Ma trận Jacobian đặt lại hiệp phương sai sai số G (19x19)
G = eye(19);
G(7:9, 7:9)     = eye(3) - 0.5 * skew_mat_local(delta_theta); % Đặt lại tư thế UUV
G(17:19, 17:19) = eye(3) + 0.5 * skew_mat_local(delta_bs);    % Đặt lại Boresight DVL (DẤU CỘNG)

covariance = G * covariance * G';
covariance = 0.5 * (covariance + covariance'); % Giữ tính đối xứng
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

function phi = so3_log_local(R)
cos_theta = max(min(0.5*(trace(R) - 1), 1), -1);
theta = acos(cos_theta);
if theta > 1e-8
    phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
else
    phi = 0.5 * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
end
end

function S = skew_mat_local(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function q = quat_multiply_local(p, r)
q = [p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4); ...
     p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3); ...
     p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2); ...
     p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1)];
end
