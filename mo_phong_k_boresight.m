% mo_phong_k_boresight.m
% Tau di thang roi cua. DVL A125 co k that va boresight that.
% INS danh dinh khong biet hai so do. Script in uoc luong.
% Chay: mo_phong_k_boresight

g = 9.80665;
fs = 400;
dt = 1/fs;
T = 40;
N = T * fs;
t = (0:N-1)' * dt;

k_that = 0.01;                          % 1%
bs_that = [0; 0; 1*pi/180];             % heading 1 do

% Quy dao that: 20 s di Bac 1.5 m/s, 20 s cua 6 do/s
v_that = zeros(N,3);
yaw = zeros(N,1);
yaw_rate = zeros(N,1);
for i = 2:N
    if t(i) < 20
        yaw_rate(i) = 0;
        vb = [1.5; 0; 0];
    else
        yaw_rate(i) = 6*pi/180;
        vb = [1.5; 0; 0];
    end
    yaw(i) = yaw(i-1) + yaw_rate(i)*dt;
    cy = cos(yaw(i)); sy = sin(yaw(i));
    v_that(i,:) = ( [cy -sy 0; sy cy 0; 0 0 1] * vb )';
end

% INS: khong bias, de thay k va boresight. Bias lon se lap sau.
q = [1;0;0;0];
v_n = [0;0;0];
Vn = zeros(N,3);
Q = zeros(N,4);
for i = 1:N
    omega = [0; 0; yaw_rate(i)];
    % gia toc rieng: dao ham van toc + trong luc, xap xi tau nam ngang
    if i == 1
        a_n = [0;0;0];
    else
        a_n = (v_that(i,:)' - v_that(i-1,:)') / dt;
    end
    cy = cos(yaw(i)); sy = sin(yaw(i));
    Cbn = [cy -sy 0; sy cy 0; 0 0 1];
    f_b = Cbn' * (a_n - [0;0;g]);
    dq = [1; 0.5*omega*dt];
    q = [ q(1) -q(2) -q(3) -q(4); ...
          q(2)  q(1) -q(4)  q(3); ...
          q(3)  q(4)  q(1) -q(2); ...
          q(4) -q(3)  q(2)  q(1) ] * dq;
    q = q / norm(q);
    Cnb = quat_Cnb(q);
    v_n = v_n + (Cnb' * f_b + [0;0;g]) * dt;
    Vn(i,:) = v_n';
    Q(i,:) = q';
end

% DVL 5 Hz, ban tin that co k va boresight
idx = 1:fs/5:N;
nd = numel(idx);
Z = zeros(nd,3);
H0 = zeros(nd,3);
for j = 1:nd
    i = idx(j);
    Z(j,:) = dvl_vec(v_that(i,:)', yaw(i), k_that, bs_that)';
    H0(j,:) = dvl_vec(Vn(i,:)', Q(i,:), 0, [0;0;0])';
end
Y = Z - H0;

% Di thang: k = v_dvl / v_ins - 1, lay truc doc
thang = t(idx) < 20 & t(idx) > 2;
k_uoc = mean(Z(thang,1) ./ H0(thang,1)) - 1;

% Luc cua: lech ngang / van toc doc ~ goc heading boresight
cua = t(idx) > 22;
bs_uoc_deg = -mean(Y(cua,2) ./ Z(cua,1)) * 180/pi;

fprintf('k that = %.4f, k uoc = %.4f\n', k_that, k_uoc);
fprintf('boresight heading that = 1.00 do, uoc = %.2f do\n', bs_uoc_deg);
fprintf('phan du di thang, m/s: '); fprintf('%.4f ', mean(Y(thang,:))); fprintf('\n');

function z = dvl_vec(v_n, q_or_yaw, k, bs)
    if numel(q_or_yaw) == 1
        cy = cos(q_or_yaw); sy = sin(q_or_yaw);
        Cnb = [cy sy 0; -sy cy 0; 0 0 1];
    else
        Cnb = quat_Cnb(q_or_yaw);
    end
    v_b = Cnb * v_n(:);
    Cbs = eye(3) - [0 -bs(3) bs(2); bs(3) 0 -bs(1); -bs(2) bs(1) 0];
    z = (1+k) * Cbs * v_b;
end

function C = quat_Cnb(q)
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y); ...
         2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x); ...
         2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)];
end
