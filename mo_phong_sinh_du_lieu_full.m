% mo_phong_sinh_du_lieu_full.m
% Sinh dữ liệu IMU 400 Hz và DVL 5 Hz.
% Có sai số IMU theo datasheet và tốc độ quay Trái Đất. INS chạy cơ học khung NED.
%
% Kịch bản:
%   Pha 0: đứng yên, căn chỉnh thô bằng trọng lực + tốc độ quay Trái Đất
%   Pha 1: tăng tốc 0 -> 1.5 m/s
%   Pha 2: chạy thẳng đều  (dùng để tìm k, Beta, Gamma)
%   Pha 3: quay vòng 6 deg/s
%
% Sai số IMU, rút ngẫu nhiên từng trục (coi số datasheet là 1 sigma - GIẢ ĐỊNH):
%   gyro : bias bật máy, ARW, hệ số tỉ lệ, lệch trục
%   accel: bias bật máy, bias trôi chậm (Gauss-Markov), VRW, hệ số tỉ lệ, lệch trục
% CHƯA CÓ: lever arm, trễ DVL, phi tuyến, cảm biến sâu cho kênh đứng.

p = thong_so_he_thong_chuan();
rng(1);                                  % cố định nhiễu để chạy lại ra cùng kết quả

dt  = p.si.dt_ins;
fs  = p.ins.gyro.tan_so_Hz;
g   = p.dat.g;
gn  = [0; 0; g];                         % trọng lực khung NED, z xuống
W   = p.dat.omega_ie;
a_e = p.dat.a;
e2  = p.dat.e2;
kb  = p.kich_ban;
s   = p.si;

T0 = kb.t_can_chinh_s;                   % hết đứng yên
T1 = T0 + kb.t_tang_toc_s;               % hết tăng tốc
T2 = T1 + kb.t_thang_s;                  % hết chạy thẳng
T3 = T2 + kb.t_quay_s;                   % hết quay vòng
N  = round(T3 * fs);
t  = (0:N-1)' * dt;

pha = zeros(N, 1);
pha(t >= T0) = 1;
pha(t >= T1) = 2;
pha(t >= T2) = 3;
k0 = find(pha == 0, 1, 'last');          % mẫu cuối của pha căn chỉnh

%% 1. Quỹ đạo thật
V = zeros(N, 1);
yaw_rate = zeros(N, 1);
V(pha == 1) = kb.van_toc_mps * (t(pha == 1) - T0) / kb.t_tang_toc_s;
V(pha >= 2) = kb.van_toc_mps;
yaw_rate(pha == 3) = kb.toc_do_quay_deg_s * pi/180;

yaw = zeros(N, 1);
yaw(1) = kb.huong_dau_deg * pi/180;
for i = 2:N
    yaw(i) = yaw(i-1) + yaw_rate(i) * dt;
end

v_n = zeros(N, 3);                       % vận tốc thật so với đáy, khung NED
for i = 1:N
    v_n(i, :) = (C_nb_yaw(yaw(i))' * [V(i); 0; 0])';
end

lat = zeros(N, 1); lon = zeros(N, 1); h = zeros(N, 1);
lat(1) = p.dat.vi_do_deg * pi/180;
lon(1) = p.dat.kinh_do_deg * pi/180;
h(1)   = p.dat.do_cao_m;
for i = 2:N
    [RM, RN] = ban_kinh(lat(i-1), a_e, e2);
    lat(i) = lat(i-1) + v_n(i-1, 1) / (RM + h(i-1)) * dt;
    lon(i) = lon(i-1) + v_n(i-1, 2) / ((RN + h(i-1)) * cos(lat(i-1))) * dt;
    h(i)   = h(i-1)   - v_n(i-1, 3) * dt;
end

%% 2. IMU lý tưởng: tốc độ góc và gia tốc riêng thật
% Dùng trạng thái bước trước (j = i-1) giống hệt INS. Khi không có sai số IMU
% và căn chỉnh đúng, INS tái tạo đúng quỹ đạo thật. Sai số còn lại chỉ do IMU.
w_that = zeros(N, 3);
f_that = zeros(N, 3);
for i = 1:N
    j = max(i - 1, 1);
    [w_ie, w_en] = toc_do_nav(lat(j), h(j), v_n(j, :)', W, a_e, e2);
    w_that(i, :) = ([0; 0; yaw_rate(i)] + C_nb_yaw(yaw(j)) * (w_ie + w_en))';
    a_n = (v_n(i, :)' - v_n(j, :)') / dt;
    f_that(i, :) = (C_nb_yaw(yaw(i)) * (a_n + cross(2*w_ie + w_en, v_n(j, :)') - gn))';
end

%% 3. Sai số IMU
bg = s.gyro_bias_sigma * randn(3, 1);                            % rad/s
Kg = eye(3) + diag(s.gyro_scale_err * randn(3, 1)) ...
     + s.gyro_cross_axis * (randn(3) .* ~eye(3));                % tỉ lệ + lệch trục
ba = s.acc_bias_sigma * randn(3, 1);                             % m/s^2
Ka = eye(3) + diag(s.acc_scale_err * randn(3, 1)) ...
     + s.acc_cross_axis * (randn(3) .* ~eye(3));
phi_gm = exp(-dt / s.acc_bias_tau);
q_gm   = s.acc_bias_inrun_sigma * sqrt(1 - phi_gm^2);
b_gm   = s.acc_bias_inrun_sigma * randn(3, 1);
sd_g   = s.gyro_arw / sqrt(dt);                                  % nhiễu 1 mẫu gyro, rad/s
sd_a   = s.acc_vrw  / sqrt(dt);                                  % nhiễu 1 mẫu gia tốc, m/s^2

w_do = zeros(N, 3);
f_do = zeros(N, 3);
for i = 1:N
    b_gm = phi_gm * b_gm + q_gm * randn(3, 1);
    w_do(i, :) = (Kg * w_that(i, :)' + bg + sd_g * randn(3, 1))';
    f_do(i, :) = (Ka * f_that(i, :)' + ba + b_gm + sd_a * randn(3, 1))';
end

%% 4. Căn chỉnh thô lúc đứng yên (TRIAD: trọng lực + trục quay Trái Đất)
f_tb = mean(f_do(1:k0, :), 1)';
w_tb = mean(w_do(1:k0, :), 1)';
Tn = triad([0; 0; 1], [cos(lat(1)); 0; -sin(lat(1))]);
Tb = triad(-f_tb / norm(f_tb), w_tb / norm(w_tb));
Cbn0 = Tn * Tb';                         % C_b^n ước lượng
q0 = dcm_sang_quat(Cbn0);

[r_e, p_e, y_e] = euler_tu_Cbn(Cbn0);
loi_roll  = r_e * 180/pi;
loi_pitch = p_e * 180/pi;
loi_yaw   = goc_wrap(y_e - yaw(k0)) * 180/pi;

% Dự đoán lý thuyết từ sai số đo trung bình
df   = f_tb - f_that(k0, :)';            % sai số gia tốc trung bình, khung thân
dw   = w_tb - w_that(k0, :)';            % sai số gyro trung bình, khung thân
dw_n = C_nb_yaw(yaw(k0))' * dw;          % chiếu sang NED
du_pitch = df(1) / g * 180/pi;
du_roll  = -df(2) / g * 180/pi;
du_yaw   = -dw_n(2) / (W * cos(lat(1))) * 180/pi;
sd_yaw_arw = s.gyro_arw / sqrt(T0) / (W * cos(lat(1))) * 180/pi;

%% 5. INS cơ học khung NED, chạy từ cuối pha căn chỉnh
Q_ins  = repmat(q0', N, 1);
Vn_ins = zeros(N, 3);
q  = q0;
v  = [0; 0; 0];
la = lat(k0); lo = lon(k0); hh = h(k0);
for i = k0+1:N
    [w_ie, w_en] = toc_do_nav(la, hh, v, W, a_e, e2);
    Cnb  = quat_sang_Cnb(q);
    w_nb = w_do(i, :)' - Cnb * (w_ie + w_en);
    q = quat_nhan(q, quat_quay(w_nb * dt));
    q = q / norm(q);
    Cbn = quat_sang_Cnb(q)';
    v_moi = v + (Cbn * f_do(i, :)' - cross(2*w_ie + w_en, v) + gn) * dt;
    [RM, RN] = ban_kinh(la, a_e, e2);
    la_moi = la + v(1) / (RM + hh) * dt;
    lo = lo + v(2) / ((RN + hh) * cos(la)) * dt;
    hh = hh - v(3) * dt;
    la = la_moi;
    v  = v_moi;
    Q_ins(i, :)  = q';
    Vn_ins(i, :) = v';
end

%% 6. DVL 5 Hz: k thật, boresight thật, nhiễu theo vận tốc
buoc    = round(fs / p.dvl.ping_Hz);
idx_dvl = (1:buoc:N)';
ndvl    = numel(idx_dvl);
t_dvl   = t(idx_dvl);
pha_dvl = pha(idx_dvl);
bs = s.boresight_that_rad;               % [Alpha; Beta; Gamma]
Cbs_that = eye(3) - [0 -bs(3) bs(2); bs(3) 0 -bs(1); -bs(2) bs(1) 0];
Z_dvl = zeros(ndvl, 3);
v_b_that_dvl = zeros(ndvl, 3);
for j = 1:ndvl
    i  = idx_dvl(j);
    vb = C_nb_yaw(yaw(i)) * v_n(i, :)';
    v_b_that_dvl(j, :) = vb';
    Z_dvl(j, :) = ((1 + s.k_that) * Cbs_that * vb + s.sigma_dvl_fn(norm(vb)) * randn(3, 1))';
end

%% 7. In tóm tắt
fprintf('\n--- SAI SỐ IMU RÚT ĐƯỢC (lần chạy này) ---\n');
fprintf('Bias gyro    [x y z] = [%7.3f %7.3f %7.3f] deg/h\n', bg * 180/pi * 3600);
fprintf('Bias gia tốc [x y z] = [%7.3f %7.3f %7.3f] mg\n', ba / g * 1e3);
fprintf('\n--- CĂN CHỈNH THÔ %g s ---\n', T0);
fprintf('             mô phỏng   dự đoán lý thuyết\n');
fprintf('Roll  (deg): %9.4f   %9.4f\n', loi_roll,  du_roll);
fprintf('Pitch (deg): %9.4f   %9.4f\n', loi_pitch, du_pitch);
fprintf('Yaw   (deg): %9.4f   %9.4f\n', loi_yaw,   du_yaw);
fprintf('Yaw lệch = -(bias gyro hướng Đông) / (Omega*cos(vĩ độ)). Riêng ARW đã góp ~%.2f deg (1 sigma).\n', sd_yaw_arw);
fprintf('\nSai số vận tốc INS lúc kết thúc (NED): [%.4f %.4f %.4f] m/s\n', Vn_ins(N, :) - v_n(N, :));

save('du_lieu_cam_bien_full.mat', 't', 'pha', 'Vn_ins', 'Q_ins', 't_dvl', 'Z_dvl', ...
     'idx_dvl', 'pha_dvl', 'v_b_that_dvl', 'v_n', 'yaw', 'p');

%% ==================== Hàm cục bộ ====================
function C = C_nb_yaw(psi)
% C_n^b khi tàu nằm ngang, chỉ có hướng psi.
    c = cos(psi); sn = sin(psi);
    C = [c sn 0; -sn c 0; 0 0 1];
end

function [RM, RN] = ban_kinh(lat, a, e2)
% Bán kính kinh tuyến RM và vòng thẳng đứng RN, WGS-84.
    sl2 = sin(lat)^2;
    RN = a / sqrt(1 - e2 * sl2);
    RM = a * (1 - e2) / (1 - e2 * sl2)^1.5;
end

function [w_ie, w_en] = toc_do_nav(lat, h, v, W, a, e2)
% w_ie: quay Trái Đất. w_en: transport rate do tàu di chuyển trên mặt cong.
    [RM, RN] = ban_kinh(lat, a, e2);
    w_ie = W * [cos(lat); 0; -sin(lat)];
    w_en = [v(2) / (RN + h); -v(1) / (RM + h); -v(2) * tan(lat) / (RN + h)];
end

function C = quat_sang_Cnb(q)
% C_n^b từ q = [w x y z] (q quay thân -> navigation).
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y); ...
         2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x); ...
         2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)];
end

function r = quat_nhan(a, b)
% Tích Hamilton a (x) b.
    r = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
         a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
         a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
         a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

function dq = quat_quay(phi)
% Quaternion của vectơ quay phi (rad), công thức chính xác.
    goc = norm(phi);
    if goc < 1e-12
        dq = [1; 0.5 * phi];
    else
        dq = [cos(goc/2); sin(goc/2) * phi / goc];
    end
end

function T = triad(u1, u2)
% Bộ ba trực chuẩn từ vectơ chính u1 (đơn vị) và vectơ phụ u2.
    t2 = cross(u1, u2); t2 = t2 / norm(t2);
    T = [u1, t2, cross(u1, t2)];
end

function q = dcm_sang_quat(R)
% R = C_b^n. Trả q sao cho quat_sang_Cnb(q) = R'.
    tr = trace(R);
    if tr > 0
        S = 2 * sqrt(tr + 1);
        q = [0.25*S; (R(3,2)-R(2,3))/S; (R(1,3)-R(3,1))/S; (R(2,1)-R(1,2))/S];
    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        S = 2 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
        q = [(R(3,2)-R(2,3))/S; 0.25*S; (R(1,2)+R(2,1))/S; (R(1,3)+R(3,1))/S];
    elseif R(2,2) > R(3,3)
        S = 2 * sqrt(1 + R(2,2) - R(1,1) - R(3,3));
        q = [(R(1,3)-R(3,1))/S; (R(1,2)+R(2,1))/S; 0.25*S; (R(2,3)+R(3,2))/S];
    else
        S = 2 * sqrt(1 + R(3,3) - R(1,1) - R(2,2));
        q = [(R(2,1)-R(1,2))/S; (R(1,3)+R(3,1))/S; (R(2,3)+R(3,2))/S; 0.25*S];
    end
    q = q / norm(q);
    if q(1) < 0, q = -q; end
end

function [roll, pitch, yaw_] = euler_tu_Cbn(C)
% Góc Euler ZYX từ C_b^n.
    pitch = -asin(C(3,1));
    roll  = atan2(C(3,2), C(3,3));
    yaw_  = atan2(C(2,1), C(1,1));
end

function a = goc_wrap(a)
    a = mod(a + pi, 2*pi) - pi;
end
