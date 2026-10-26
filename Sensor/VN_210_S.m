function varargout = VN_210_S(mode, varargin)
%VN_210_S  Mô phỏng cảm biến dẫn đường GNSS/INS VectorNav VN-210-S (Tactical Grade).
%
% NGUỒN GỐC VÀ THÔNG SỐ KỸ THUẬT:
%   Dựa trên VN-210-S Product Brief, Version 12-0038-R1, VectorNav Technologies 2024.
%
% KIẾN TRÚC ĐA CHẾ ĐỘ (DUAL-CAPABILITY):
%   1. Drop-in Replacement cho GEO-FOG 3D trong hệ thống ES-EKF khép vòng (Tightly-coupled):
%      - mode 'spec': Lấy bảng thông số SI chuẩn khớp hoàn toàn cấu trúc bộ lọc ES-EKF.
%      - mode 'init': Khởi tạo trạng thái bias thật (nhận bg0, ba0 hoặc seed ngẫu nhiên).
%      - mode 'imu' : Sinh số đo thô con quay và gia tốc kế (400 Hz / 20 Hz) từ động lực học Fossen.
%      - mode 'gnss': Sinh vector đo GNSS chuẩn 7x1 khi tàu nổi (5 Hz), mô phỏng đặc tính Single-Antenna.
%   2. Loosely-coupled mode (cho nghiên cứu so sánh giải pháp EKF nội của VectorNav):
%      - mode 'step': Xuất [position, velocity, attitude] đã lọc qua EKF nội chip VN-210-S.
%
% CÁC CÁCH GỌI CHÍNH:
%   spec  = VN_210_S('spec');
%   state = VN_210_S('init', spec, bg0, ba0, seed);
%   [gyro_meas, accel_meas, state] = VN_210_S('imu', spec, state, w_ib_true, f_b_true, dt);
%   [gnss_meas, has_yaw] = VN_210_S('gnss', spec, pos_ned, vel_ned, yaw, state);
%   [meas, is_valid, state, info] = VN_210_S('step', true_pos, true_vel, true_att, dt, state, spec, ...);

if ~exist('gravity_fossen', 'file')
    root_dir = fileparts(fileparts(mfilename('fullpath')));
    if exist(fullfile(root_dir, 'algo'), 'dir')
        addpath(fullfile(root_dir, 'algo'));
    end
end

if nargin == 0
    mode = 'spec';
end

% Dispatcher tương thích ngược khi gọi kiểu số: VN_210_S(true_pos, true_vel, true_att, ...)
if isnumeric(mode)
    [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = ...
        vn210s_step(mode, varargin{:});
    return;
end

switch lower(char(mode))
    case 'spec'
        varargout{1} = build_spec();

    case 'init'
        varargout{1} = init_state(varargin{:});

    case 'imu'
        % Cú pháp: [gyro_meas, accel_meas, state] = VN_210_S('imu', spec, state, w_ib_true, f_b_true, dt)
        [varargout{1}, varargout{2}, varargout{3}] = imu_step(varargin{:});

    case 'gnss'
        % Cú pháp: [gnss_meas, has_valid_yaw] = VN_210_S('gnss', spec, pos_ned, vel_ned, yaw, state)
        [varargout{1}, varargout{2}] = gnss_step(varargin{:});

    case {'step', 'measure'}
        % Cú pháp loosely-coupled EKF nội
        [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = ...
            vn210s_step(varargin{:});

    otherwise
        error('VN_210_S: mode khong hop le: ''%s''. Chon ''spec'', ''init'', ''imu'', ''gnss'', ''step''.', mode);
end

end

%% ===================================================================
%  1. THÔNG SỐ KỸ THUẬT (BUILD_SPEC) - NGUỒN: VN-210-S Product Brief R1
%% ===================================================================
function s = build_spec()
d2r = pi / 180;
g0 = 9.80665;

s.ten = 'VectorNav VN-210-S Tactical-Grade GNSS/INS';
s.version = 'Product Brief v12-0038-R1, 2024';
s.co_tu_ke = false;                    % Mặc định tắt từ kế trên UUV do nhiễu động cơ và ắc quy

% ── A. CẤU TRÚC PHÂN CẤP TƯƠNG THÍCH CHUẨN ES-EKF & GEO-FOG 3D ──
% Con quay hồi chuyển (Tactical MEMS Gyroscope)
s.gyro.arw         = deg2rad(5.0 / 3600);        % Noise Density: 5 deg/hr/sqrt(Hz) -> rad/sqrt(s)
s.gyro.bias_instab = deg2rad(0.6 / 3600);        % In-run bias stability: 0.6 deg/hr typ. -> rad/s
s.gyro.bias_tau    = 3600.0;                     % Thời hằng Gauss-Markov: 1 giờ (s)
s.gyro.bias_rw     = s.gyro.bias_instab * sqrt(2 / s.gyro.bias_tau); % Random walk tương đương
s.gyro.dai_do      = 490 * d2r;                  % Dải đo tiêu chuẩn: +-490 deg/s (8.55 rad/s)

% Gia tốc kế (Tactical MEMS Accelerometer)
s.acc.vrw          = 0.04e-3 * g0;               % Noise Density: 0.04 mg/sqrt(Hz) -> m/s/sqrt(s)
s.acc.bias_instab  = 10e-6 * g0;                 % In-run bias stability: < 10 ug -> m/s^2
s.acc.bias_tau     = 1800.0;                     % Thời hằng Gauss-Markov: 30 phút (s)
s.acc.bias_rw      = s.acc.bias_instab * sqrt(2 / s.acc.bias_tau);
s.acc.dai_do       = 15 * g0;                    % Dải đo tiêu chuẩn: +-15 g (147.15 m/s^2)

% Bộ thu GNSS Đa băng tần (Single-Antenna GNSS Receiver)
s.gnss.pos_h       = 0.6;                        % Horizontal Accuracy: 0.6 m RMS (SBAS standard)
s.gnss.pos_v       = 0.8;                        % Vertical Accuracy: 0.8 m RMS
s.gnss.vel         = 0.02;                       % Velocity Accuracy: 0.02 m/s
s.gnss.heading     = 0.075 * d2r;                % Dynamic Heading: 0.075 deg khi có chuyển động
s.gnss.rate_hz     = 5;                          % Tần số cập nhật đo đạc vệ tinh thực tế: 5 Hz
s.gnss.single_antenna_mode = true;               % Cờ mô phỏng đặc tính đơn ăng-ten trên UUV

% ── B. THÔNG SỐ CHI TIẾT THEO DATASHEET PHẦN CỨNG ───────────────────
% Kích thước, khối lượng, điện năng (SWaP)
s.size_mm       = [56, 56, 31];                  % Kích thước vỏ nhôm phay: 56 x 56 x 31 mm
s.weight_g      = 170;                           % Khối lượng: 170 g
s.vin_min_v     = 12.0;                          % Điện áp vào tối thiểu: 12 V
s.vin_max_v     = 34.0;                          % Điện áp vào tối đa: 34 V
s.current_ma    = 135;                           % Dòng tiêu thụ danh định tại 24 V: 135 mA
s.power_max_w   = 3.3;                           % Công suất tiêu thụ tối đa: < 3.3 W

% Nhiệt độ hoạt động và môi trường
s.temp_op_min_c = -40.0;
s.temp_op_max_c = +85.0;
s.temp_storage_min_c = -40.0;
s.temp_storage_max_c = +85.0;

% Tốc độ xuất dữ liệu phần cứng (Output Data Rate)
s.odr_imu_max_hz = 800;                          % Tốc độ xuất IMU thô tối đa: 800 Hz
s.odr_pva_max_hz = 400;                          % Tốc độ xuất PVA qua EKF nội tối đa: 400 Hz

% Gia tốc kế
s.accel_range_g         = 15.0;                  % Dải đo: +-15 g
s.accel_bias_stability_ug = 10.0;                % < 10 ug
s.accel_noise_density_mg_sqrthz = 0.04;          % < 0.04 mg/sqrt(Hz)
s.accel_bandwidth_hz    = 200;                   % Bandwidth: 200 Hz
s.accel_cross_axis_deg  = 0.05;                  % Cross-Axis: +-0.05 deg

% Con quay
s.gyro_range_dps        = 490;                   % Dải đo: +-490 deg/s
s.gyro_range_ext_dps    = 2000;                  % Dải đo tùy chọn mở rộng: +-2000 deg/s
s.gyro_bias_stability_dph_typ = 0.6;             % Bias stability typ: 0.6 deg/hr
s.gyro_bias_stability_dph_max = 1.0;             % Bias stability max: < 1 deg/hr
s.gyro_noise_density_dph_sqrthz = 5.0;           % Noise density: 5 deg/hr/sqrt(Hz)
s.gyro_bandwidth_hz     = 210;                   % Bandwidth: 210 Hz
s.gyro_cross_axis_deg   = 0.05;                  % Cross-Axis: < 0.05 deg

% Từ kế (Magnetometer - 3 trục)
s.mag_range_gauss       = 2.5;                   % Dải đo từ trường: +-2.5 Gauss
s.mag_noise_density_ugauss_sqrthz = 140;         % Mật độ nhiễu: 140 uGauss/sqrt(Hz)
s.mag_bandwidth_hz      = 200;                   % Bandwidth: 200 Hz
s.heading_accuracy_mag_deg = 2.0;                % Sai số hướng đo thuần từ kế khi đứng yên: 2.0 deg RMS

% Độ chính xác Attitude (xuất từ EKF nội)
s.heading_accuracy_ins_deg  = 0.075;             % Dynamic Heading: 0.05 đến 0.1 deg (1-sigma)
s.pitch_roll_static_deg     = 0.05;              % Pitch/Roll tĩnh: 0.05 deg RMS
s.pitch_roll_ins_deg        = 0.015;             % Pitch/Roll động: 0.015 deg (1-sigma)
s.heading_misalign_deg      = 0.05;              % Misalignment Heading: < 0.05 deg
s.pitch_roll_misalign_deg   = 0.05;              % Misalignment Pitch/Roll: < 0.05 deg
s.angular_resolution_deg    = 0.001;             % Độ phân giải góc: 0.001 deg

% Độ chính xác Vị trí & Vận tốc
s.pos_horizontal_rms_m  = 0.6;                   % Ngang tiêu chuẩn: 0.6 m RMS
s.pos_vertical_rms_m    = 0.8;                   % Đứng tiêu chuẩn: 0.8 m RMS
s.pos_rtk_cep_m         = 0.01;                  % RTK: 1 cm + 1 ppm CEP
s.pos_rtk_ppm           = 1e-6;
s.free_inertial_drift_rate = 0.005;              % Độ trôi tự do khi mất GNSS: 0.5 cm/s^2 (0.005 m/s^2)
s.vel_accuracy_ms       = 0.02;                  % Độ chính xác vận tốc: 0.02 m/s

% Bộ thu GNSS
s.gnss_channels         = 448;
s.gnss_bands            = 'L1/L2/L5/E1/E5a/E5b/B1/B2/B3/L5/SBAS';
s.ttff_cold_s           = 45;
s.ttff_hot_s            = 20;
s.alt_limit_m           = 50000;
s.vel_limit_ms          = 500;
s.gnss_pps_rms_ns       = 10;
s.gnss_pps_p99_ns       = 25;
s.gnss_mode             = 'standard';

% Cờ mô phỏng bổ trợ
s.use_imu_bias          = true;
s.use_imu_noise         = true;
s.use_attitude_noise    = true;
s.use_position_noise    = true;
s.use_free_inertial_drift = true;
end

%% ===================================================================
%  2. KHỞI TẠO TRẠNG THÁI (INIT_STATE)
%% ===================================================================
function st = init_state(spec, varargin)
% CÚ PHÁP HỖ TRỢ:
%   state = VN_210_S('init', spec, seed);
%   state = VN_210_S('init', spec, bg0, ba0);
%   state = VN_210_S('init', spec, bg0, ba0, seed);

if nargin < 1 || isempty(spec)
    spec = build_spec();
end

bg0 = [];
ba0 = [];
seed = [];

if nargin >= 2
    arg1 = varargin{1};
    if isscalar(arg1) && isnumeric(arg1) && numel(varargin) == 1
        % Gọi kiểu init(spec, seed)
        seed = arg1;
    elseif isvector(arg1) && numel(arg1) == 3
        % Gọi kiểu init(spec, bg0, ba0, ...)
        bg0 = arg1(:);
        if numel(varargin) >= 2
            ba0 = varargin{2}(:);
        end
        if numel(varargin) >= 3
            seed = varargin{3};
        end
    end
end

if ~isempty(seed)
    st.rng_stream = RandStream('mt19937ar', 'Seed', seed);
else
    st.rng_stream = RandStream.getGlobalStream();
end

% Turn-on bias ban đầu
if ~isempty(bg0) && ~isempty(ba0)
    st.bg_turn_on = bg0;
    st.ba_turn_on = ba0;
else
    st.bg_turn_on = spec.gyro.bias_instab * randn(st.rng_stream, 3, 1);
    st.ba_turn_on = spec.acc.bias_instab  * randn(st.rng_stream, 3, 1);
end

% In-run bias ban đầu
st.bg_inrun = spec.gyro.bias_instab * randn(st.rng_stream, 3, 1);
st.ba_inrun = spec.acc.bias_instab  * randn(st.rng_stream, 3, 1);

st.b_gyro_rads = st.bg_turn_on + st.bg_inrun;
st.b_accel_ms2 = st.ba_turn_on + st.ba_inrun;
st.bg_total = st.b_gyro_rads;
st.ba_total = st.b_accel_ms2;

% Trạng thái mất GNSS và thời gian
st.gnss_lost    = false;
st.t_gnss_loss  = 0.0;
st.time_accum_s = 0.0;
end

%% ===================================================================
%  3. SINH SỐ ĐO IMU THÔ 400 HZ / 20 HZ (IMU_STEP)
%% ===================================================================
function [gyro_meas, accel_meas, st] = imu_step(spec, st, w_ib_true, f_b_true, dt)
% CÚ PHÁP:
%   [gyro_meas, accel_meas, st] = imu_step(spec, st, w_ib_true, f_b_true, dt)
%
% ĐẦU VÀO:
%   spec       - Struct thông số từ build_spec()
%   st         - Struct trạng thái cảm biến
%   w_ib_true  - Vector tốc độ góc thật trong hệ thân (rad/s) [3x1]
%   f_b_true   - Vector lực riêng thật trong hệ thân (m/s^2) [3x1]
%   dt         - Chu kỳ lấy mẫu (s)

if nargin < 5 || isempty(dt), dt = 0.0025; end % 400 Hz mặc định

% Cập nhật bias In-Run Gauss-Markov bậc 1 (AR1)
pg = exp(-dt / spec.gyro.bias_tau);
pa = exp(-dt / spec.acc.bias_tau);

st.bg_inrun = pg * st.bg_inrun + spec.gyro.bias_instab * sqrt(max(1 - pg^2, 0)) * randn(st.rng_stream, 3, 1);
st.ba_inrun = pa * st.ba_inrun + spec.acc.bias_instab  * sqrt(max(1 - pa^2, 0)) * randn(st.rng_stream, 3, 1);

bg = st.bg_turn_on + st.bg_inrun;
ba = st.ba_turn_on + st.ba_inrun;

% Nhiễu trắng rời rạc chuẩn Wiener: ARW / sqrt(dt) và VRW / sqrt(dt)
w_gyro  = (spec.gyro.arw / sqrt(dt)) * randn(st.rng_stream, 3, 1);
w_accel = (spec.acc.vrw  / sqrt(dt)) * randn(st.rng_stream, 3, 1);

% Tổng hợp số đo thô có động lực học thật từ Fossen
gyro_meas  = w_ib_true(:) + bg + w_gyro;
accel_meas = f_b_true(:)  + ba + w_accel;

% Bão hòa theo dải đo phần cứng VN-210-S
gyro_meas  = min(max(gyro_meas,  -spec.gyro.dai_do), spec.gyro.dai_do);
accel_meas = min(max(accel_meas, -spec.acc.dai_do),  spec.acc.dai_do);

st.bg_total = bg;
st.ba_total = ba;
st.b_gyro_rads = bg;
st.b_accel_ms2 = ba;
st.time_accum_s = st.time_accum_s + dt;
end

%% ===================================================================
%  4. SINH SỐ ĐO GNSS 5 HZ KHI NỔI (GNSS_STEP)
%% ===================================================================
function [z, has_valid_yaw] = gnss_step(spec, pos_ned, vel_ned, yaw, state)
% CÚ PHÁP:
%   [z, has_valid_yaw] = gnss_step(spec, pos_ned, vel_ned, yaw, state)
%
% ĐẦU RA:
%   z             - Vector đo GNSS cột 7x1: [p_N; p_E; p_D; v_N; v_E; v_D; yaw_meas]
%   has_valid_yaw - Cờ báo góc hướng có hợp lệ hay không (true/false)

if nargin < 5 || isempty(state) || ~isfield(state, 'rng_stream') || isempty(state.rng_stream)
    stream = RandStream.getGlobalStream();
else
    stream = state.rng_stream;
end

pos_ned = pos_ned(:);
vel_ned = vel_ned(:);

% 1. Sinh nhiễu vị trí và vận tốc GNSS
pos_noise = [spec.gnss.pos_h * randn(stream, 1); ...
             spec.gnss.pos_h * randn(stream, 1); ...
             spec.gnss.pos_v * randn(stream, 1)];

vel_noise = spec.gnss.vel * randn(stream, 3, 1);

% 2. Góc hướng Yaw
% Bộ thu GNSS đơn ăng-ten CHỈ đo được vị trí và vận tốc (Doppler).
% Nó KHÔNG đo được góc hướng thân tàu. Từ kế tắt trên UUV (méo từ trường động cơ/pin).
% -> Không sinh số đo yaw: z(7) = NaN để bộ lọc chỉ cập nhật 6 hàng [vị trí; vận tốc].
% (Thông số 'Dynamic Heading 0.075 deg' trong datasheet là đầu ra EKF nội của chip
%  sau khi tự căn chỉnh, không phải số đo thô, nên không được đưa vào bộ lọc ngoài.)
if isfield(spec.gnss, 'single_antenna_mode') && spec.gnss.single_antenna_mode
    yaw_meas = NaN;
    has_valid_yaw = false;
else
    % Chỉ dùng cho cấu hình 2 ăng-ten (không phải phần cứng VN-210-S)
    yaw_meas = yaw + spec.gnss.heading * randn(stream, 1);
    yaw_meas = atan2(sin(yaw_meas), cos(yaw_meas)); % Cuộn tròn [-pi, pi]
    has_valid_yaw = true;
end

z = [pos_ned + pos_noise; ...
     vel_ned + vel_noise; ...
     yaw_meas];
end

%% ===================================================================
%  5. BƯỚC ĐO LOOSELY-COUPLED TỔNG HỢP (VN210S_STEP)
%% ===================================================================
function [meas, is_valid, state, info] = vn210s_step( ...
    true_pos_ned, true_vel_ned, true_att_rad, ...
    dt, state, spec, gnss_available, t_since_gnss_loss)

if nargin < 1 || isempty(true_pos_ned),  true_pos_ned = zeros(3,1); end
if nargin < 2 || isempty(true_vel_ned),  true_vel_ned = zeros(3,1); end
if nargin < 3 || isempty(true_att_rad),  true_att_rad = zeros(3,1); end
if nargin < 4 || isempty(dt),            dt = 0.02; end
if nargin < 6 || isempty(spec),          spec = build_spec(); end
if nargin < 5 || isempty(state),         state = init_state(spec); end
if nargin < 7 || isempty(gnss_available), gnss_available = true; end
if nargin < 8 || isempty(t_since_gnss_loss), t_since_gnss_loss = 0.0; end

true_pos_ned = true_pos_ned(:);
true_vel_ned = true_vel_ned(:);
true_att_rad = true_att_rad(:);

state.time_accum_s = state.time_accum_s + dt;

% Cập nhật bias In-run
pg = exp(-dt / spec.gyro.bias_tau);
pa = exp(-dt / spec.acc.bias_tau);
state.bg_inrun = pg * state.bg_inrun + spec.gyro.bias_instab * sqrt(max(1 - pg^2, 0)) * randn(state.rng_stream, 3, 1);
state.ba_inrun = pa * state.ba_inrun + spec.acc.bias_instab  * sqrt(max(1 - pa^2, 0)) * randn(state.rng_stream, 3, 1);

state.b_gyro_rads = state.bg_turn_on + state.bg_inrun;
state.b_accel_ms2 = state.ba_turn_on + state.ba_inrun;
state.bg_total = state.b_gyro_rads;
state.ba_total = state.b_accel_ms2;

% Xác định độ lệch chuẩn theo chế độ có / mất GNSS
if gnss_available
    sigma_pos = [spec.pos_horizontal_rms_m; spec.pos_horizontal_rms_m; spec.pos_vertical_rms_m];
    sigma_heading_rad = deg2rad(spec.heading_accuracy_ins_deg);
    sigma_pitch_roll_rad = deg2rad(spec.pitch_roll_ins_deg);
    state.gnss_lost = false;
else
    % Mất GNSS: Free inertial drift tăng theo bậc 2 thời gian
    t_drift = t_since_gnss_loss;
    drift_sigma = 0.5 * spec.free_inertial_drift_rate * t_drift^2;
    sigma_pos = [drift_sigma; drift_sigma; drift_sigma * 1.5];
    sigma_heading_rad = deg2rad(spec.heading_accuracy_ins_deg * (1 + t_drift / 600));
    sigma_pitch_roll_rad = deg2rad(spec.pitch_roll_static_deg);
    state.gnss_lost = true;
    state.t_gnss_loss = t_drift;
end

sigma_vel = spec.vel_accuracy_ms * ones(3, 1);
sigma_att = [sigma_pitch_roll_rad; sigma_pitch_roll_rad; sigma_heading_rad];

pos_noise = sigma_pos .* randn(state.rng_stream, 3, 1);
vel_noise = sigma_vel .* randn(state.rng_stream, 3, 1);
att_noise = sigma_att .* randn(state.rng_stream, 3, 1);

pos_meas = true_pos_ned + pos_noise;
vel_meas = true_vel_ned + vel_noise;
att_meas = true_att_rad + att_noise;

% Lượng tử hóa góc theo Angular Resolution
ang_res_rad = deg2rad(spec.angular_resolution_deg);
att_meas = round(att_meas / ang_res_rad) * ang_res_rad;

speed_ms = norm(true_vel_ned);
altitude_m = abs(true_pos_ned(3));
is_valid = (speed_ms <= spec.vel_limit_ms) && (altitude_m <= spec.alt_limit_m);

% Đóng gói đầu ra
meas = struct();
meas.pos_ned        = pos_meas;
meas.vel_ned        = vel_meas;
meas.att_rad        = att_meas;
meas.att_deg        = rad2deg(att_meas);
meas.sigma_pos      = sigma_pos;
meas.sigma_vel      = sigma_vel;
meas.sigma_att      = sigma_att;
meas.b_accel_ms2    = state.b_accel_ms2;
meas.b_gyro_rads    = state.b_gyro_rads;
meas.gnss_available = gnss_available;

info = struct();
info.true_pos_ned   = true_pos_ned;
info.true_vel_ned   = true_vel_ned;
info.true_att_rad   = true_att_rad;
info.sigma_pos      = sigma_pos;
info.sigma_vel      = sigma_vel;
info.sigma_att      = sigma_att;
info.is_valid       = is_valid;
end