% dithang.m
% =========================================================================
% DẪN ĐƯỜNG UUV: HIỆU CHUẨN DVL BẰNG RECIPROCAL TRACKS (A -> B, B -> A)
% RỒI LẶN NGẦM ĐI THẲNG TỚI ĐIỂM ĐÍCH CỐ ĐỊNH
%
% Cơ sở: Kinsey & Whitcomb (IEEE JOE 2007) - hiệu chuẩn DVL bằng các lượt
%        chạy thẳng ngược chiều nhau; ES-EKF theo Joan Sola (2017).
%
% THIẾT KẾ KỊCH BẢN (bản sửa):
%   Hai lượt hiệu chuẩn được đặt TRÙNG phương với hướng tới đích:
%     - Lượt 1 (A -> B): chạy NGƯỢC hướng đích (psi_T + 180 độ).
%     - Quay đầu 180 độ (vận tốc giữ nguyên 1.5 m/s).
%     - Lượt 2 (B -> A): chạy XUÔI hướng đích (psi_T), đi qua lại vùng A.
%   Hết lượt 2, tàu đã sẵn mũi hướng về đích -> cắt GNSS, lặn và đi tiếp
%   gần như thẳng tới đích, không phải vòng chữ U như bản trước.
%
%   Phase 2 dùng vòng điều khiển kín (mô phỏng autopilot thật):
%     - Dẫn hướng LOS: lệnh hướng = phương vị từ vị trí hiện tại tới đích.
%     - Giữ độ sâu: lệnh pitch tỉ lệ sai số độ sâu, giới hạn +-8 độ.
%     - Giảm tốc khi gần đích, dừng mô phỏng khi cách đích < 2 m.
%
% BỘ LỌC (bản sửa):
%   - Ma trận H của DVL đầy đủ ở CẢ HAI phase (dv, dtheta, dbg, s, boresight).
%   - Mọi phép cập nhật (GNSS, độ sâu, DVL) đều tiêm TOÀN BỘ dx vào trạng thái.
%   - Loại ngoại lai bằng khoảng cách Mahalanobis (Chi-bình phương 99%).
%
% Quy ước màu: ĐỎ = Đi thật (Ground Truth), ĐEN = Ước lượng ES-EKF.
% =========================================================================

clear; close all; clc;

disp('===================================================================');
disp('  UUV: HIỆU CHUẨN DVL RECIPROCAL TRACKS + LẶN THẲNG TỚI ĐÍCH');
disp('===================================================================');

%% -------------------------------------------------------------------------
% BƯỚC 1: THÔNG SỐ
% -------------------------------------------------------------------------
p = thong_so_he_thong_chuan();
rng(1);

dt  = p.si.dt_ins;
fs  = p.ins.gyro.tan_so_Hz;
g   = p.dat.g;
gn  = [0; 0; g];
W   = p.dat.omega_ie;
a_e = p.dat.a;
e2  = p.dat.e2;
s_cfg = p.si;

% Điểm đích (NED, gốc tại điểm xuất phát A)
target = [450; 250; 15];                 % [Bắc; Đông; Sâu] (m)
psi_T  = atan2(target(2), target(1));    % phương vị A -> đích

% Mốc thời gian Phase 1 (trên mặt nước, có GNSS)
V_cr         = 1.5;    % vận tốc hành trình (m/s)
T_static     = 60;     % đứng yên căn chỉnh
T_acc        = 10;     % tăng tốc 0 -> 1.5 m/s
T_turn_start = 210;    % hết lượt 1
T_turn       = 30;     % quay đầu 180 độ
T_turn_end   = T_turn_start + T_turn;
T_phase1     = 390;    % hết lượt 2, cắt GNSS

psi_leg1 = psi_T + pi; % lượt 1: ngược hướng đích
psi_leg2 = psi_T;      % lượt 2: xuôi hướng đích

% Thông số autopilot Phase 2 (GIẢ ĐỊNH, cỡ AUV hạng nhẹ)
K_psi  = 0.10;              % hệ số lệnh tốc độ quay hướng (1/s)
r_max  = 3.0 * pi/180;      % giới hạn tốc độ quay hướng (rad/s)
tau_r  = 2.0;               % hằng số thời gian bánh lái hướng (s)
K_z    = 0.05;              % hệ số lệnh pitch theo sai số độ sâu (1/m)
th_max = 8.0 * pi/180;      % giới hạn góc pitch khi lặn (rad)
tau_th = 3.0;               % hằng số thời gian bánh lái độ sâu (s)
q_max  = 1.5 * pi/180;      % giới hạn tốc độ thay đổi pitch (rad/s)
tau_V  = 5.0;               % hằng số thời gian tốc độ (s)
a_max  = 0.10;              % giới hạn gia tốc dọc (m/s^2)
R_arrive = 2.0;             % bán kính coi như đã tới đích (m)

% Cảm biến độ sâu (áp suất)
% GIẢ ĐỊNH - cần thay bằng số liệu datasheet cảm biến thực tế
depth_rate_Hz = 5;          % tần số cập nhật
sig_depth     = 0.05;       % nhiễu đo độ sâu 1-sigma (m)

%% -------------------------------------------------------------------------
% BƯỚC 2: SINH QUỸ ĐẠO THẬT (Phase 1 theo lịch, Phase 2 vòng kín)
% -------------------------------------------------------------------------
disp('1. Sinh quỹ đạo thật...');
T_max = T_phase1 + 900;
N_max = round(T_max * fs);

V        = zeros(N_max, 1);
yaw      = zeros(N_max, 1);
pitch    = zeros(N_max, 1);
v_n      = zeros(N_max, 3);
POS_TRUE = zeros(N_max, 3);

pos = [0; 0; 0];
Vc = 0; psi = psi_leg1; th = 0; r = 0;
N = N_max;

for i = 1:N_max
    ti = (i - 1) * dt;

    if ti < T_phase1
        th = 0; r = 0;
        if ti < T_static
            Vc = 0;    psi = psi_leg1;
        elseif ti < T_static + T_acc
            tau = (ti - T_static) / T_acc;
            Vc = V_cr * (0.5 - 0.5 * cos(pi * tau));
            psi = psi_leg1;
        elseif ti < T_turn_start
            Vc = V_cr; psi = psi_leg1;
        elseif ti < T_turn_end
            tau = (ti - T_turn_start) / T_turn;
            Vc = V_cr;                                    % giữ nguyên tốc độ
            psi = psi_leg1 - pi * (0.5 - 0.5 * cos(pi * tau));
        else
            Vc = V_cr; psi = psi_leg2;
        end
    else
        % ---- Phase 2: autopilot vòng kín ----
        d   = target - pos;
        d_h = norm(d(1:2));
        if d_h < R_arrive
            N = i - 1;
            break;
        end
        % Dẫn hướng LOS
        psi_cmd = atan2(d(2), d(1));
        e_psi   = atan2(sin(psi_cmd - psi), cos(psi_cmd - psi));
        r_cmd   = max(min(K_psi * e_psi, r_max), -r_max);
        r       = r + (r_cmd - r) * dt / tau_r;
        psi     = psi + r * dt;
        % Giữ độ sâu (pitch âm = chúi mũi xuống)
        th_cmd  = -max(min(atan(K_z * (target(3) - pos(3))), th_max), -th_max);
        th_rate = max(min((th_cmd - th) / tau_th, q_max), -q_max);
        th      = th + th_rate * dt;
        % Giảm tốc khi gần đích
        V_cmd   = max(min(0.08 * d_h, V_cr), 0.3);
        a_cmd   = max(min((V_cmd - Vc) / tau_V, a_max), -a_max);
        Vc      = Vc + a_cmd * dt;
    end

    V(i) = Vc; yaw(i) = psi; pitch(i) = th;
    v_n(i, :) = [Vc*cos(th)*cos(psi), Vc*cos(th)*sin(psi), -Vc*sin(th)];
    pos = pos + v_n(i, :)' * dt;
    POS_TRUE(i, :) = pos';
end

V = V(1:N); yaw = yaw(1:N); pitch = pitch(1:N);
v_n = v_n(1:N, :); POS_TRUE = POS_TRUE(1:N, :);
t = (0:N-1)' * dt;
T_total = t(end);
roll = zeros(N, 1);

idx_B    = round(T_turn_start * fs);
idx_dive = round(T_phase1 * fs) + 1;
fprintf('   --> Phương vị A -> đích       : %.2f độ\n', psi_T * 180/pi);
fprintf('   --> Điểm B (hết lượt 1)       : [%.1f, %.1f] m\n', POS_TRUE(idx_B,1), POS_TRUE(idx_B,2));
fprintf('   --> Điểm bắt đầu lặn          : [%.1f, %.1f] m\n', POS_TRUE(idx_dive,1), POS_TRUE(idx_dive,2));
fprintf('   --> Tới đích tại t            : %.1f s\n', T_total);

%% -------------------------------------------------------------------------
% BƯỚC 3: SINH DỮ LIỆU CẢM BIẾN
% -------------------------------------------------------------------------
disp('2. Sinh dữ liệu cảm biến IMU / GNSS / độ sâu / DVL...');

% Tọa độ địa lý WGS-84
lat = zeros(N, 1); lon = zeros(N, 1); h = zeros(N, 1);
lat(1) = p.dat.vi_do_deg * pi/180;
lon(1) = p.dat.kinh_do_deg * pi/180;
h(1)   = p.dat.do_cao_m;
for i = 2:N
    sl2 = sin(lat(i-1))^2;
    RN = a_e / sqrt(1 - e2 * sl2);
    RM = a_e * (1 - e2) / (1 - e2 * sl2)^1.5;
    lat(i) = lat(i-1) + v_n(i-1, 1) / (RM + h(i-1)) * dt;
    lon(i) = lon(i-1) + v_n(i-1, 2) / ((RN + h(i-1)) * cos(lat(i-1))) * dt;
    h(i)   = h(i-1)   - v_n(i-1, 3) * dt;
end

% Tốc độ góc và lực riêng thật (có quay Trái Đất, Coriolis)
w_that = zeros(N, 3);
f_that = zeros(N, 3);
for i = 1:N
    j = max(i - 1, 1);
    sl2 = sin(lat(j))^2;
    RN = a_e / sqrt(1 - e2 * sl2);
    RM = a_e * (1 - e2) / (1 - e2 * sl2)^1.5;
    w_ie = W * [cos(lat(j)); 0; -sin(lat(j))];
    w_en = [v_n(j,2) / (RN + h(j)); -v_n(j,1) / (RM + h(j)); -v_n(j,2) * tan(lat(j)) / (RN + h(j))];

    dp_dt = (pitch(i) - pitch(j)) / dt;
    dy_dt = (yaw(i) - yaw(j)) / dt;

    cy = cos(yaw(j));   sy = sin(yaw(j));
    cp = cos(pitch(j)); sp = sin(pitch(j));
    R_b2n = [cy*cp, -sy, cy*sp;
             sy*cp,  cy, sy*sp;
             -sp,    0,   cp  ];
    R_n2b = R_b2n';

    w_body_rel = [-dy_dt * sp; dp_dt; dy_dt * cp];
    w_that(i, :) = (w_body_rel + R_n2b * (w_ie + w_en))';

    a_n = (v_n(i, :)' - v_n(j, :)') / dt;
    f_that(i, :) = (R_n2b * (a_n + cross(2*w_ie + w_en, v_n(j, :)') - gn))';
end

% Sai số IMU theo datasheet INS-DM-FI
bg = s_cfg.gyro_bias_sigma * randn(3, 1);
Kg = eye(3) + diag(s_cfg.gyro_scale_err * randn(3, 1)) + s_cfg.gyro_cross_axis * (randn(3) .* ~eye(3));
ba = s_cfg.acc_bias_sigma * randn(3, 1);
Ka = eye(3) + diag(s_cfg.acc_scale_err * randn(3, 1)) + s_cfg.acc_cross_axis * (randn(3) .* ~eye(3));
phi_gm = exp(-dt / s_cfg.acc_bias_tau);
q_gm   = s_cfg.acc_bias_inrun_sigma * sqrt(1 - phi_gm^2);
b_gm   = s_cfg.acc_bias_inrun_sigma * randn(3, 1);
sd_g   = s_cfg.gyro_arw / sqrt(dt);
sd_a   = s_cfg.acc_vrw  / sqrt(dt);

w_do = zeros(N, 3);
f_do = zeros(N, 3);
for i = 1:N
    b_gm = phi_gm * b_gm + q_gm * randn(3, 1);
    w_do(i, :) = (Kg * w_that(i, :)' + bg + sd_g * randn(3, 1))';
    f_do(i, :) = (Ka * f_that(i, :)' + ba + b_gm + sd_a * randn(3, 1))';
end
fprintf('   --> Lực riêng lớn nhất |f|    : %.3f m/s^2 (dải đo +-8 g)\n', max(sqrt(sum(f_that.^2, 2))));

% Lưới thời điểm đo 5 Hz (GNSS, độ sâu, DVL dùng chung)
idx_meas = 1:round((1/depth_rate_Hz) / dt):N;
n_meas   = length(idx_meas);

% GNSS RTK + 2 anten (chỉ Phase 1)
sig_gps_pos = 0.02;
sig_gps_vel = 0.02;
sig_gps_yaw = 0.05 * pi/180;
Z_gps_pos = POS_TRUE(idx_meas, :) + sig_gps_pos * randn(n_meas, 3);
Z_gps_vel = v_n(idx_meas, :)      + sig_gps_vel * randn(n_meas, 3);
Z_gps_yaw = yaw(idx_meas)         + sig_gps_yaw * randn(n_meas, 1);

% Cảm biến độ sâu
Z_depth = POS_TRUE(idx_meas, 3) + sig_depth * randn(n_meas, 1);

% DVL: (1 + s) * C_bs * (v_b + w x l) + sai số tương quan chậm + nhiễu trắng
s_that   = p.si.k_that;               % +1.0 %
bs_that  = p.si.boresight_that_rad;   % [alpha 0.2; beta -0.3; gamma 1.0] độ
Cbs_that = eye(3) - skew_mat(bs_that);
lever_arm = [0.2; 0.0; -0.1];

% Công tắc mô hình nhiễu DVL
DVL_NHIEU_THUC_TE = true;   % false = nhiễu lý tưởng cũ (0.1 % |v| + 0.1 mm/s)
DVL_SAI_SO_GM     = true;   % sai số tỷ lệ tương quan chậm, bộ lọc KHÔNG biết

if DVL_NHIEU_THUC_TE
    sigma_dvl = s_cfg.sigma_dvl_ping_fn;
else
    sigma_dvl = s_cfg.sigma_dvl_fn;
end
dt_dvl_meas = dt * (idx_meas(2) - idx_meas(1));
phi_dvl_gm  = exp(-dt_dvl_meas / p.dvl.gm_tau_s);
g_dvl       = DVL_SAI_SO_GM * p.dvl.gm_ti_le_sigma * randn(3, 1);   % sai số tỷ lệ theo trục
G_DVL_REC   = zeros(n_meas, 3);

Z_dvl = zeros(n_meas, 3);
for j = 1:n_meas
    idx = idx_meas(j);
    cy = cos(yaw(idx));   sy = sin(yaw(idx));
    cp = cos(pitch(idx)); sp = sin(pitch(idx));
    R_n2b = [cy*cp,  sy*cp, -sp;
            -sy,     cy,     0;
             cy*sp,  sy*sp,  cp ];
    vb  = R_n2b * v_n(idx, :)';
    v_L = vb + cross(w_that(idx, :)', lever_arm);
    if DVL_SAI_SO_GM
        g_dvl = phi_dvl_gm * g_dvl + p.dvl.gm_ti_le_sigma * sqrt(1 - phi_dvl_gm^2) * randn(3, 1);
    end
    G_DVL_REC(j, :) = g_dvl';
    v_dvl = (1 + s_that) * Cbs_that * v_L;
    Z_dvl(j, :) = (v_dvl + g_dvl .* v_dvl + sigma_dvl(norm(v_L)) * randn(3, 1))';
end
fprintf('   --> Nhiễu DVL từng ping ở 1.5 m/s : %.1f mm/s mỗi trục (1-sigma)\n', sigma_dvl(1.5) * 1e3);
if DVL_SAI_SO_GM
    fprintf('   --> Sai số tỷ lệ GM            : sigma %.2f %%, tau %.0f s (bộ lọc không mô hình hóa)\n', ...
        p.dvl.gm_ti_le_sigma * 100, p.dvl.gm_tau_s);
end

%% -------------------------------------------------------------------------
% BƯỚC 4: KHỞI TẠO ES-EKF 19 TRẠNG THÁI
% -------------------------------------------------------------------------
% x = [dp(3) dv(3) dtheta(3) dba(3) dbg(3) ds(1) dphi_bs(3)]
disp('3. Khởi tạo ES-EKF 19 trạng thái...');

pos_est = [0; 0; 0];
v_est   = [0; 0; 0];
q_est   = [cos(yaw(1)/2); 0; 0; sin(yaw(1)/2)];
ba_est  = [0; 0; 0];
bg_est  = [0; 0; 0];
s_est   = 0.0;
phi_bs_est = [0; 0; 0];
lever_arm_est = lever_arm;

P = diag([
    (0.01)^2 * ones(1, 3), ...
    (0.01)^2 * ones(1, 3), ...
    (0.2 * pi/180)^2 * ones(1, 2), ...
    (0.05 * pi/180)^2, ...
    (0.5e-3 * 9.81)^2 * ones(1, 3), ...
    (0.5 * pi/180 / 3600)^2 * ones(1, 3), ...
    (0.02)^2, ...
    (0.01 * pi/180)^2, ...      % alpha: gần như cố định (không quan sát được khi không lắc ngang)
    (1.5 * pi/180)^2, ...
    (1.5 * pi/180)^2 ]);

Q_d = zeros(19, 19);
Q_d(4:6, 4:6)     = (s_cfg.acc_vrw)^2 * dt * eye(3);
Q_d(7:9, 7:9)     = (s_cfg.gyro_arw)^2 * dt * eye(3);
Q_d(10:12, 10:12) = (s_cfg.acc_bias_inrun_sigma)^2 * (2*dt/s_cfg.acc_bias_tau) * eye(3);
Q_d(13:15, 13:15) = (s_cfg.gyro_bias_sigma * 1e-4)^2 * dt * eye(3);
Q_d(16, 16)       = (5e-7)^2 * dt;
Q_d(18:19, 18:19) = (5e-5 * pi/180)^2 * dt * eye(2);

CHI2_3 = 11.34;   % 3 bậc tự do, 99 %
CHI2_7 = 18.48;   % 7 bậc tự do, 99 %

POS_REC   = zeros(N, 3);
YAW_REC   = zeros(N, 1);
PITCH_REC = zeros(N, 1);
S_REC     = zeros(N, 1);
BETA_REC  = zeros(N, 1);
GAMMA_REC = zeros(N, 1);
SIG_REC   = zeros(N, 3);   % 1-sigma của s, beta, gamma
n_rej_dvl = 0;

meas_ptr = 1;

%% -------------------------------------------------------------------------
% BƯỚC 5: VÒNG LẶP ES-EKF (INS 400 Hz, cảm biến 5 Hz)
% -------------------------------------------------------------------------
disp('4. Chạy ES-EKF...');

for i = 1:N
    ti = t(i);

    if mod(i, round(50 / dt)) == 0
        fprintf('   t = %5.0f s | Z = %5.1f m | s = %+.3f %% | beta = %+.3f deg | gamma = %+.3f deg\n', ...
            ti, pos_est(3), s_est*100, phi_bs_est(2)*180/pi, phi_bs_est(3)*180/pi);
    end

    % --- [A] Dự báo INS ---
    f_clean = f_do(i, :)' - ba_est;
    w_clean = w_do(i, :)' - bg_est;

    Cbn = quat_to_Cbn(q_est);
    Cnb = Cbn';

    lat_i = lat(i);
    w_ie_nav = W * [cos(lat_i); 0; -sin(lat_i)];
    sl2_i = sin(lat_i)^2;
    RN_i = a_e / sqrt(1 - e2 * sl2_i);
    RM_i = a_e * (1 - e2) / (1 - e2 * sl2_i)^1.5;
    w_en_nav = [v_est(2)/(RN_i + h(i)); -v_est(1)/(RM_i + h(i)); -v_est(2)*tan(lat_i)/(RN_i + h(i))];
    w_nb_body = w_clean - Cnb * (w_ie_nav + w_en_nav);

    a_nav = Cbn * f_clean + gn - cross(2*w_ie_nav + w_en_nav, v_est);
    v_est   = v_est + a_nav * dt;
    pos_est = pos_est + v_est * dt;

    dth_step = w_nb_body * dt;
    ang = norm(dth_step);
    dq_step = [cos(ang/2); sin(ang/2) * dth_step / max(ang, 1e-12)];
    q_est = quat_nhan(q_est, dq_step);
    q_est = q_est / norm(q_est);

    F_x = eye(19);
    F_x(1:3, 4:6)   = eye(3) * dt;
    F_x(4:6, 7:9)   = -Cbn * skew_mat(f_clean) * dt;
    F_x(4:6, 10:12) = -Cbn * dt;
    F_x(7:9, 7:9)   = eye(3) - skew_mat(w_nb_body) * dt;
    F_x(7:9, 13:15) = -eye(3) * dt;
    P = F_x * P * F_x' + Q_d;

    % --- [B] Cập nhật 5 Hz ---
    if meas_ptr <= n_meas && i == idx_meas(meas_ptr)
        k = meas_ptr;

        % (1) GNSS vị trí + vận tốc + hướng 2 anten (chỉ Phase 1)
        if ti <= T_phase1
            Cbn_c = quat_to_Cbn(q_est);
            yaw_c = atan2(Cbn_c(2, 1), Cbn_c(1, 1));
            d_yaw = atan2(sin(Z_gps_yaw(k) - yaw_c), cos(Z_gps_yaw(k) - yaw_c));
            res = [Z_gps_pos(k, :)' - pos_est; Z_gps_vel(k, :)' - v_est; d_yaw];
            H = zeros(7, 19);
            H(1:3, 1:3) = eye(3);
            H(4:6, 4:6) = eye(3);
            H(7, 9)     = 1.0;
            R = diag([sig_gps_pos^2*ones(1,3), sig_gps_vel^2*ones(1,3), sig_gps_yaw^2]);
            [dx, P, ok] = kf_update(P, H, res, R, CHI2_7);
            if ok
                [pos_est, v_est, q_est, ba_est, bg_est, s_est, phi_bs_est, P] = ...
                    tiem_sai_so(dx, P, pos_est, v_est, q_est, ba_est, bg_est, s_est, phi_bs_est);
            end
        end

        % (2) Cảm biến độ sâu (cả hai phase)
        H = zeros(1, 19); H(1, 3) = 1.0;
        [dx, P, ok] = kf_update(P, H, Z_depth(k) - pos_est(3), sig_depth^2, 6.63);
        if ok
            [pos_est, v_est, q_est, ba_est, bg_est, s_est, phi_bs_est, P] = ...
                tiem_sai_so(dx, P, pos_est, v_est, q_est, ba_est, bg_est, s_est, phi_bs_est);
        end

        % (3) DVL - mô hình đo và H đầy đủ, giống nhau ở cả hai phase
        Cnb_c = quat_to_Cbn(q_est)';
        v_b   = Cnb_c * v_est;
        v_L   = v_b + cross(w_clean, lever_arm_est);
        if norm(v_L) > 0.3
            C_bs = eye(3) - skew_mat(phi_bs_est);
            res  = Z_dvl(k, :)' - (1 + s_est) * C_bs * v_L;

            H = zeros(3, 19);
            H(:, 4:6)   = (1 + s_est) * C_bs * Cnb_c;              % dv
            H(:, 7:9)   = (1 + s_est) * C_bs * skew_mat(v_b);       % dtheta
            H(:, 13:15) = (1 + s_est) * C_bs * skew_mat(lever_arm_est); % dbg qua w x l
            H(:, 16)    = C_bs * v_L;                               % ds
            H(:, 17:19) = (1 + s_est) * skew_mat(v_L);              % dphi_bs

            % R chỉ gồm phần nhiễu trắng; sai số GM không được khai báo cho bộ lọc
            sig_d = sigma_dvl(norm(v_L));
            [dx, P, ok] = kf_update(P, H, res, sig_d^2 * eye(3), CHI2_3);
            if ok
                [pos_est, v_est, q_est, ba_est, bg_est, s_est, phi_bs_est, P] = ...
                    tiem_sai_so(dx, P, pos_est, v_est, q_est, ba_est, bg_est, s_est, phi_bs_est);
            else
                n_rej_dvl = n_rej_dvl + 1;
            end
        end

        meas_ptr = meas_ptr + 1;
    end

    % --- [C] Lưu kết quả ---
    Cbn_now = quat_to_Cbn(q_est);
    POS_REC(i, :) = pos_est';
    YAW_REC(i)    = atan2(Cbn_now(2, 1), Cbn_now(1, 1));
    PITCH_REC(i)  = -asin(max(min(Cbn_now(3, 1), 1), -1));
    S_REC(i)      = s_est * 100;
    BETA_REC(i)   = phi_bs_est(2) * 180/pi;
    GAMMA_REC(i)  = phi_bs_est(3) * 180/pi;
    SIG_REC(i, :) = [sqrt(P(16,16)) * 100, sqrt(P(18,18)) * 180/pi, sqrt(P(19,19)) * 180/pi];
end

%% -------------------------------------------------------------------------
% BƯỚC 6: KẾT QUẢ
% -------------------------------------------------------------------------
mask_p2 = (t > T_phase1);
ERR_POS = sqrt(sum((POS_REC(mask_p2, :) - POS_TRUE(mask_p2, :)).^2, 2));
ERR_H   = sqrt(sum((POS_REC(mask_p2, 1:2) - POS_TRUE(mask_p2, 1:2)).^2, 2));
dist_p2 = sum(sqrt(sum(diff(POS_TRUE(mask_p2, :), 1, 1).^2, 2)));

% Hướng EKF: bỏ bước nhảy +-180 độ của atan2 rồi khớp nhánh 2*pi với giá trị thật
yaw_ekf = unwrap(YAW_REC);
yaw_ekf = yaw_ekf + 2*pi * round((yaw(1) - yaw_ekf(1)) / (2*pi));
ERR_YAW   = (yaw_ekf - yaw) * 180/pi;
ERR_PITCH = (PITCH_REC - pitch) * 180/pi;

idx_end_p1 = round(T_phase1 * fs);

fprintf('\n===================================================================\n');
fprintf('                         KẾT QUẢ\n');
fprintf('===================================================================\n');
fprintf('Tham số DVL tại thời điểm cắt GNSS (t = %.0f s):\n', T_phase1);
fprintf('  s     = %+.3f %%   (+-%.3f, 1-sigma) | thật +1.000 %%\n', S_REC(idx_end_p1), SIG_REC(idx_end_p1, 1));
fprintf('  beta  = %+.3f deg (+-%.3f, 1-sigma) | thật -0.300 deg\n', BETA_REC(idx_end_p1), SIG_REC(idx_end_p1, 2));
fprintf('  gamma = %+.3f deg (+-%.3f, 1-sigma) | thật +1.000 deg\n', GAMMA_REC(idx_end_p1), SIG_REC(idx_end_p1, 3));
fprintf('-------------------------------------------------------------------\n');
fprintf('Phase 2 (không GNSS):\n');
fprintf('  Quãng đường lặn               : %.1f m trong %.0f s\n', dist_p2, T_total - T_phase1);
fprintf('  Vị trí thật khi tới đích      : [%.1f, %.1f, %.1f] m\n', POS_TRUE(end,1), POS_TRUE(end,2), POS_TRUE(end,3));
fprintf('  Vị trí ES-EKF khi tới đích    : [%.1f, %.1f, %.1f] m\n', POS_REC(end,1), POS_REC(end,2), POS_REC(end,3));
fprintf('  Sai số vị trí cuối (3D / ngang): %.2f m / %.2f m\n', ERR_POS(end), ERR_H(end));
fprintf('  Tỉ lệ dạt ngang                : %.3f %% quãng đường\n', ERR_H(end) / dist_p2 * 100);
fprintf('  Sai số độ sâu cuối            : %.3f m\n', abs(POS_REC(end,3) - POS_TRUE(end,3)));
fprintf('  Bản tin DVL bị loại (Chi2)    : %d\n', n_rej_dvl);
fprintf('===================================================================\n\n');

%% -------------------------------------------------------------------------
% HÌNH 1: QUỸ ĐẠO
% -------------------------------------------------------------------------
figure('Name', 'Quỹ đạo (ĐỎ: thật | ĐEN: ES-EKF)', 'Color', 'w', 'Position', [40 60 1350 650]);

subplot(1, 2, 1);
plot3(POS_TRUE(:,2), POS_TRUE(:,1), -POS_TRUE(:,3), 'r-', 'LineWidth', 2.2); hold on; grid on;
plot3(POS_REC(:,2),  POS_REC(:,1),  -POS_REC(:,3),  'k--', 'LineWidth', 1.6);
plot3(0, 0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot3(POS_TRUE(idx_B,2), POS_TRUE(idx_B,1), 0, 'cs', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
plot3(POS_TRUE(idx_dive,2), POS_TRUE(idx_dive,1), 0, 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
plot3(target(2), target(1), -target(3), 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 13);
xlabel('Đông (m)'); ylabel('Bắc (m)'); zlabel('Cao độ (m)');
title('Quỹ đạo 3D');
legend('Thật', 'ES-EKF', 'A (xuất phát)', 'B (quay đầu)', 'Bắt đầu lặn', 'Đích', 'Location', 'best');
view([-35, 25]); axis equal;

subplot(1, 2, 2);
plot(POS_TRUE(:,2), POS_TRUE(:,1), 'r-', 'LineWidth', 2.2); hold on; grid on;
plot(POS_REC(:,2),  POS_REC(:,1),  'k--', 'LineWidth', 1.6);
plot(0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(POS_TRUE(idx_B,2), POS_TRUE(idx_B,1), 'cs', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
plot(POS_TRUE(idx_dive,2), POS_TRUE(idx_dive,1), 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
plot(target(2), target(1), 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 13);
xlabel('Đông (m)'); ylabel('Bắc (m)');
title('Mặt bằng 2D: A \rightarrow B, B \rightarrow A, lặn tới đích');
legend('Thật', 'ES-EKF', 'A', 'B', 'Bắt đầu lặn', 'Đích', 'Location', 'best');
axis equal;

%% -------------------------------------------------------------------------
% HÌNH 2: TƯ THẾ, ĐỘ SÂU, SAI SỐ
% -------------------------------------------------------------------------
figure('Name', 'Tư thế và độ sâu theo thời gian', 'Color', 'w', 'Position', [80 40 1350 820]);

subplot(3, 2, 1);
plot(t, yaw*180/pi, 'r-', 'LineWidth', 2.0); hold on; grid on;
plot(t, yaw_ekf*180/pi, 'k--', 'LineWidth', 1.4);
xline(T_phase1, 'm--', 'Cắt GNSS');
xlabel('t (s)'); ylabel('\psi (độ)'); title('Góc hướng \psi(t)');
legend('Thật', 'ES-EKF', 'Location', 'best');
xlim([0, T_total]);

subplot(3, 2, 2);
plot(t, ERR_YAW, 'k-', 'LineWidth', 1.2); grid on;
xline(T_phase1, 'm--', 'Cắt GNSS');
xlabel('t (s)'); ylabel('độ'); title('Sai số hướng \psi_{EKF} - \psi_{thật}');
xlim([0, T_total]);

subplot(3, 2, 3);
plot(t, pitch*180/pi, 'r-', 'LineWidth', 2.0); hold on; grid on;
plot(t, PITCH_REC*180/pi, 'k--', 'LineWidth', 1.4);
xline(T_phase1, 'm--', 'Cắt GNSS');
xlabel('t (s)'); ylabel('\theta (độ)'); title('Góc pitch \theta(t)');
legend('Thật', 'ES-EKF', 'Location', 'best');
xlim([0, T_total]);

subplot(3, 2, 4);
plot(t, ERR_PITCH, 'k-', 'LineWidth', 1.2); grid on;
xline(T_phase1, 'm--', 'Cắt GNSS');
xlabel('t (s)'); ylabel('độ'); title('Sai số pitch \theta_{EKF} - \theta_{thật}');
xlim([0, T_total]);

subplot(3, 2, 5);
plot(t, POS_TRUE(:,3), 'r-', 'LineWidth', 2.0); hold on; grid on;
plot(t, POS_REC(:,3), 'k--', 'LineWidth', 1.4);
yline(target(3), 'b:', sprintf('Độ sâu đặt %.0f m', target(3)));
xline(T_phase1, 'm--', 'Cắt GNSS');
set(gca, 'YDir', 'reverse');
xlabel('t (s)'); ylabel('Z (m)'); title('Độ sâu Z(t)');
legend('Thật', 'ES-EKF', 'Location', 'best');
xlim([0, T_total]);

subplot(3, 2, 6);
plot(t(mask_p2) - T_phase1, ERR_H, 'k-', 'LineWidth', 1.6); hold on; grid on;
plot(t(mask_p2) - T_phase1, ERR_POS, 'b--', 'LineWidth', 1.2);
xlabel('Thời gian lặn (s)'); ylabel('m'); title('Sai số vị trí khi lặn');
legend('Ngang', '3D', 'Location', 'best');

%% -------------------------------------------------------------------------
% HÌNH 3: HỘI TỤ THAM SỐ DVL (kèm biên +-3 sigma)
% -------------------------------------------------------------------------
figure('Name', 'Hội tụ tham số DVL', 'Color', 'w', 'Position', [120 80 1200 720]);
names = {'Hệ số tỉ lệ s (%)', 'Pitch boresight \beta (độ)', 'Yaw boresight \gamma (độ)'};
truth = [1.0, -0.3, 1.0];
REC   = [S_REC, BETA_REC, GAMMA_REC];
cols  = {'b', [0 0.6 0], 'm'};
for k = 1:3
    subplot(3, 1, k);
    h1 = plot(t, REC(:,k), '-', 'Color', cols{k}, 'LineWidth', 1.8); hold on; grid on;
    h2 = plot(t, truth(k) + 3*SIG_REC(:,k), ':', 'Color', [0.4 0.4 0.4]);
    plot(t, truth(k) - 3*SIG_REC(:,k), ':', 'Color', [0.4 0.4 0.4]);
    h3 = yline(truth(k), 'r--', 'Giá trị thật', 'LineWidth', 1.3);
    xline(T_turn_start, 'k:', 'Quay đầu');
    xline(T_phase1, 'm--', 'Cắt GNSS');
    xlabel('t (s)'); title(names{k});
    ylim(truth(k) + [-1.5, 1.5]);
    legend([h1 h2 h3], {'ES-EKF', 'thật \pm 3\sigma (từ P)', 'Giá trị thật'}, 'Location', 'best');
end

% Đồng bộ sang workspace cho mo_phong_3d_matlab.m
assignin('base', 'POS_TRUE', POS_TRUE);
assignin('base', 'POS_REC', POS_REC);
assignin('base', 'yaw', yaw);
assignin('base', 'V', V);
assignin('base', 't', t);
assignin('base', 'S_REC', S_REC);
assignin('base', 'BETA_REC', BETA_REC);
assignin('base', 'GAMMA_REC', GAMMA_REC);
assignin('base', 'dt', dt);
assignin('base', 'T_phase1', T_phase1);
assignin('base', 'T_can_chinh', T_phase1);

%% -------------------------------------------------------------------------
% HÀM PHỤ TRỢ
% -------------------------------------------------------------------------
function [dx, P, ok] = kf_update(P, H, res, R, chi2_gate)
    % Cập nhật Kalman dạng Joseph, có cổng Mahalanobis
    S  = H * P * H' + R;
    d2 = res' / S * res;
    ok = (d2 < chi2_gate);
    dx = zeros(size(P, 1), 1);
    if ~ok, return; end
    K   = (P * H') / S;
    dx  = K * res;
    IKH = eye(size(P)) - K * H;
    P   = IKH * P * IKH' + K * R * K';
end

function [pos, v, q, ba, bg, s, phi, P] = tiem_sai_so(dx, P, pos, v, q, ba, bg, s, phi)
    % Tiêm toàn bộ error-state vào trạng thái danh định rồi reset
    pos = pos + dx(1:3);
    v   = v   + dx(4:6);
    dth = dx(7:9);
    dq  = [1; 0.5 * dth]; dq = dq / norm(dq);
    q   = quat_nhan(q, dq); q = q / norm(q);
    ba  = ba + dx(10:12);
    bg  = bg + dx(13:15);
    s   = s  + dx(16);
    phi = phi + dx(17:19);
    G = eye(19);
    G(7:9, 7:9) = eye(3) - 0.5 * skew_mat(dth);
    P = G * P * G';
    P = 0.5 * (P + P');
end

function S = skew_mat(v)
    S = [  0,    -v(3),  v(2);
          v(3),    0,   -v(1);
         -v(2),   v(1),   0   ];
end

function C = quat_to_Cbn(q)
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [ 1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y);
          2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x);
          2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y) ];
end

function r = quat_nhan(p, q)
    r = [ p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4);
          p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3);
          p(1)*q(3) - p(2)*q(4) + p(3)*q(1) + p(4)*q(2);
          p(1)*q(4) + p(2)*q(3) - p(3)*q(2) + p(4)*q(1) ];
end
