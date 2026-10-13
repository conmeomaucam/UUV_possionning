function [state, covariance, update_status, nis] = esekf19_step( ...
    state, covariance, process_noise, gyro_meas, accel_meas, ...
    gnss_meas, depth_meas, dvl_meas, measurement_flags, cfg)
%ESEKF19_STEP One fixed-size INS prediction and optional sensor updates.
% Code-generation entry point: no plotting, RNG, variable-size data or I/O.
% state (20x1): p[1:3], v[4:6], quaternion wxyz[7:10], ba[11:13],
% bg[14:16], scale[17], boresight alpha/beta/gamma[18:20].
% covariance/process_noise: fixed 19x19 error-state matrices.
% measurement_flags: [GNSS_valid; depth_valid; DVL_valid], fixed 3x1.
% gnss_meas (7x1): position NED, velocity NED, yaw (rad).
% cfg (18x1): dt, gravity, w_ie_NED[3], RN, RM, tan(latitude), height,
% lever_arm_body[3], DVL noise floor/slope, GPS pos/vel/yaw sigma, depth sigma.




% viết code comment vào 
dt = cfg(1);
gravity = cfg(2);
w_ie_ned = cfg(3:5);
radius_n = cfg(6);
radius_m = cfg(7);
tan_latitude = cfg(8);
height = cfg(9);
lever_arm = cfg(10:12);

position = state(1:3);
velocity = state(4:6);
quaternion = state(7:10);
accel_bias = state(11:13);
gyro_bias = state(14:16);
scale = state(17);
boresight = state(18:20);

% INS mechanization prediction in the local NED frame.
Cbn = quat_to_cbn(quaternion);
Cnb = Cbn';
accel_corrected = accel_meas - accel_bias;
gyro_corrected = gyro_meas - gyro_bias;
w_en_ned = [velocity(2)/(radius_n + height); ...
    -velocity(1)/(radius_m + height); ...
    -velocity(2)*tan_latitude/(radius_n + height)];
w_nb_body = gyro_corrected - Cnb*(w_ie_ned + w_en_ned);
accel_ned = Cbn*accel_corrected + [0; 0; gravity] ...
    - cross(2*w_ie_ned + w_en_ned, velocity);
velocity = velocity + accel_ned*dt;
position = position + velocity*dt;

delta_angle = w_nb_body*dt;
angle_norm = norm(delta_angle);
delta_quaternion = [cos(angle_norm/2); ...
    sin(angle_norm/2)*delta_angle/max(angle_norm, 1e-12)];
quaternion = quat_multiply(quaternion, delta_quaternion);
quaternion = quaternion/norm(quaternion);

F = eye(19);
F(1:3, 4:6) = eye(3)*dt;
F(4:6, 7:9) = -Cbn*skew_mat(accel_corrected)*dt;
F(4:6, 10:12) = -Cbn*dt;
F(7:9, 7:9) = eye(3) - skew_mat(w_nb_body)*dt;
F(7:9, 13:15) = -eye(3)*dt;
covariance = F*covariance*F' + process_noise;

update_status = zeros(3, 1);
nis = zeros(3, 1);

if measurement_flags(1) ~= 0
    Cbn = quat_to_cbn(quaternion);
    yaw_estimate = atan2(Cbn(2, 1), Cbn(1, 1));
    yaw_residual = atan2(sin(gnss_meas(7) - yaw_estimate), ...
        cos(gnss_meas(7) - yaw_estimate));
    residual = [gnss_meas(1:3) - position; ...
        gnss_meas(4:6) - velocity; yaw_residual];
    H = zeros(7, 19);
    H(1:3, 1:3) = eye(3);
    H(4:6, 4:6) = eye(3);
    H(7, 9) = 1;
    R = diag([cfg(15)^2*ones(1, 3), cfg(16)^2*ones(1, 3), cfg(17)^2]);
    [delta_state, covariance, accepted, nis_value] = ...
        update_gnss(covariance, H, residual, R);
    update_status(1) = accepted;
    nis(1) = nis_value;
    if accepted ~= 0
        [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
            inject_error(delta_state, covariance, position, velocity, quaternion, ...
            accel_bias, gyro_bias, scale, boresight);
    end
end

if measurement_flags(2) ~= 0
    H = zeros(1, 19);
    H(1, 3) = 1;
    residual = depth_meas - position(3);
    [delta_state, covariance, accepted, nis_value] = ...
        update_depth(covariance, H, residual, cfg(18)^2);
    update_status(2) = accepted;
    nis(2) = nis_value;
    if accepted ~= 0
        [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
            inject_error(delta_state, covariance, position, velocity, quaternion, ...
            accel_bias, gyro_bias, scale, boresight);
    end
end

if measurement_flags(3) ~= 0
    Cbn = quat_to_cbn(quaternion);
    Cnb = Cbn';
    velocity_body = Cnb*velocity;
    w_en_ned = [velocity(2)/(radius_n + height); ...
        -velocity(1)/(radius_m + height); ...
        -velocity(2)*tan_latitude/(radius_n + height)];
    body_rate_relative = gyro_corrected - Cnb*(w_ie_ned + w_en_ned);
    velocity_at_dvl = velocity_body + cross(body_rate_relative, lever_arm);
    Cbs = eye(3) - skew_mat(boresight);
    residual = dvl_meas - (1 + scale)*Cbs*velocity_at_dvl;

    H = zeros(3, 19);
    H(:, 4:6) = (1 + scale)*Cbs*Cnb;
    H(:, 7:9) = (1 + scale)*Cbs*skew_mat(velocity_body);
    H(:, 13:15) = (1 + scale)*Cbs*skew_mat(lever_arm);
    H(:, 16) = Cbs*velocity_at_dvl;
    H(:, 17:19) = (1 + scale)*skew_mat(velocity_at_dvl);
    sigma_dvl = cfg(13) + cfg(14)*norm(velocity_at_dvl);
    [delta_state, covariance, accepted, nis_value] = ...
        update_dvl(covariance, H, residual, sigma_dvl^2*eye(3));
    update_status(3) = accepted;
    nis(3) = nis_value;
    if accepted ~= 0
        [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
            inject_error(delta_state, covariance, position, velocity, quaternion, ...
            accel_bias, gyro_bias, scale, boresight);
    end
end

state(1:3) = position;
state(4:6) = velocity;
state(7:10) = quaternion;
state(11:13) = accel_bias;
state(14:16) = gyro_bias;
state(17) = scale;
state(18:20) = boresight;
end

function [delta_state, covariance, accepted, nis] = update_gnss(covariance, H, residual, R)
S = H*covariance*H' + R;
nis = residual'*(S\residual);
accepted = double(isfinite(nis) && nis <= 18.48);
delta_state = zeros(19, 1);
if accepted == 0
    return;
end
K = (covariance*H')/S;
delta_state = K*residual;
A = eye(19) - K*H;
covariance = A*covariance*A' + K*R*K';
covariance = 0.5*(covariance + covariance');
end

function [delta_state, covariance, accepted, nis] = update_depth(covariance, H, residual, variance)
S = H*covariance*H' + variance;
nis = residual*(S\residual);
accepted = double(isfinite(nis) && nis <= 6.63);
delta_state = zeros(19, 1);
if accepted == 0
    return;
end
K = (covariance*H')/S;
delta_state = K*residual;
A = eye(19) - K*H;
covariance = A*covariance*A' + K*variance*K';
covariance = 0.5*(covariance + covariance');
end

function [delta_state, covariance, accepted, nis] = update_dvl(covariance, H, residual, R)
S = H*covariance*H' + R;
nis = residual'*(S\residual);
accepted = double(isfinite(nis) && nis <= 11.34);
delta_state = zeros(19, 1);
if accepted == 0
    return;
end
K = (covariance*H')/S;
delta_state = K*residual;
A = eye(19) - K*H;
covariance = A*covariance*A' + K*R*K';
covariance = 0.5*(covariance + covariance');
end

function [position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight, covariance] = ...
    inject_error(delta_state, covariance, position, velocity, quaternion, accel_bias, gyro_bias, scale, boresight)
position = position + delta_state(1:3);
velocity = velocity + delta_state(4:6);
delta_theta = delta_state(7:9);
delta_quaternion = [1; 0.5*delta_theta];
delta_quaternion = delta_quaternion/norm(delta_quaternion);
quaternion = quat_multiply(quaternion, delta_quaternion);
quaternion = quaternion/norm(quaternion);
accel_bias = accel_bias + delta_state(10:12);
gyro_bias = gyro_bias + delta_state(13:15);
scale = scale + delta_state(16);
boresight = boresight + delta_state(17:19);
G = eye(19);
G(7:9, 7:9) = eye(3) - 0.5*skew_mat(delta_theta);
covariance = G*covariance*G';
covariance = 0.5*(covariance + covariance');
end

function Cbn = quat_to_cbn(quaternion)
quaternion = quaternion/norm(quaternion);
w = quaternion(1); x = quaternion(2); y = quaternion(3); z = quaternion(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
    2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
    2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function S = skew_mat(vector)
S = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function quaternion = quat_multiply(first, second)
quaternion = [first(1)*second(1)-first(2)*second(2)-first(3)*second(3)-first(4)*second(4); ...
    first(1)*second(2)+first(2)*second(1)+first(3)*second(4)-first(4)*second(3); ...
    first(1)*second(3)-first(2)*second(4)+first(3)*second(1)+first(4)*second(2); ...
    first(1)*second(4)+first(2)*second(3)-first(3)*second(2)+first(4)*second(1)];
end