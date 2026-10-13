function [x, P, Q] = esekf19_init(initial_rpy, gravity, noise_cfg)
%ESEKF19_INIT Initialize fixed-size nominal state and error covariance.
% Nominal state x (20x1): p[1:3], v[4:6], q_wxyz[7:10], ba[11:13],
% bg[14:16], DVL scale[17], boresight alpha/beta/gamma[18:20].
% Error state and covariance are fixed at 19 states.
% noise_cfg (8x1): [acc_vrw; gyro_arw; acc_bias_sigma; acc_bias_tau;
%                   gyro_bias_rw; scale_rw; boresight_rw; dt].

dt = noise_cfg(8);
roll = initial_rpy(1);
pitch = initial_rpy(2);
yaw = initial_rpy(3);

cr = cos(roll / 2);
sr = sin(roll / 2);
cp = cos(pitch / 2);
sp = sin(pitch / 2);
cy = cos(yaw / 2);
sy = sin(yaw / 2);

x = zeros(20, 1);
x(7:10) = [cy*cp*cr + sy*sp*sr; ...
    cy*cp*sr - sy*sp*cr; ...
    cy*sp*cr + sy*cp*sr; ...
    sy*cp*cr - cy*sp*sr];

P = diag([(0.02)^2 * ones(1, 3), ...
    (0.05)^2 * ones(1, 3), ...
    (0.5*pi/180)^2 * ones(1, 3), ...
    (0.5e-3*gravity)^2 * ones(1, 3), ...
    (0.5*pi/180/3600)^2 * ones(1, 3), ...
    (0.02)^2, ...
    (1.5*pi/180)^2 * ones(1, 3)]);

Q = zeros(19, 19);
Q(4:6, 4:6) = noise_cfg(1)^2 * dt * eye(3);
Q(7:9, 7:9) = noise_cfg(2)^2 * dt * eye(3);
Q(10:12, 10:12) = noise_cfg(3)^2 * (2*dt/noise_cfg(4)) * eye(3);
Q(13:15, 13:15) = noise_cfg(5)^2 * dt * eye(3);
Q(16, 16) = noise_cfg(6)^2 * dt;
Q(17:19, 17:19) = noise_cfg(7)^2 * dt * eye(3);
end