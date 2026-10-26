function varargout = Bar_10XT(mode, varargin)
%BAR_10XT  Mô phỏng cảm biến đo áp suất thủy tĩnh và độ sâu BlueRobotics Bar10XT.
%
% ĐẶC ĐIỂM NỔI BẬT CỦA PHIÊN BẢN HOÀN THIỆN:
% 1. Tách bạch 100% giữa môi trường vật lý (rho_true, g_true) và driver trên tàu
%    (rho_assumed, g_assumed), tái hiện chân thực sai số Scale mismatch cho ES-EKF.
% 2. Hỗ trợ tương thích ngược 100%: Nhận diện cả chuỗi lệnh ('spec', 'init', 'step')
%    lẫn cách gọi batch truyền thống Bar_10XT(p, indices, pos, sigma) và gọi số trực tiếp.
% 3. Đầy đủ 100% các trường thông số kỹ thuật (fields) từ file warning.md và phiên bản cũ.
% 4. Mô hình hóa nhiễu đa thành phần: Nhiễu trắng danh định, Sai số nhiệt độ, Trôi bias Gauss-Markov.
% 5. Tách biệt hệ số quy đổi: sigma_factor_white (3-sigma) và sigma_factor_bias (1-sigma typ).
% 6. Quản lý luồng ngẫu nhiên RandStream độc lập phục vụ kiểm thử Monte Carlo lặp lại.
%
% CÁC CÁCH GỌI MÔ-ĐUN:
%   - Lấy thông số kỹ thuật:
%       spec = Bar_10XT('spec');
%   - Khởi tạo trạng thái và bộ sinh số ngẫu nhiên:
%       state = Bar_10XT('init', spec, seed);
%   - Bước đo trực tiếp (Online Step):
%       [depth_meas, P_meas_bar, is_valid, state, info] = Bar_10XT('step', ...
%           depth_true, dt, state, spec, rho_true, lat_rad, rho_assumed, g_assumed);
%   - Gọi nhanh 1 bước:
%       depth_meas = Bar_10XT('step', depth_true);
%       depth_meas = Bar_10XT(depth_true); % Gọi số trực tiếp
%   - Chế độ mảng Batch tương thích cũ (Apsuat.m):
%       [Z_depth, depth_sim] = Bar_10XT(p, measurement_indices, position_true, sigma_depth);

% Đảm bảo thư mục algo có trong path để gọi gravity_fossen
if ~exist('gravity_fossen', 'file')
    root_dir = fileparts(fileparts(mfilename('fullpath')));
    if exist(fullfile(root_dir, 'algo'), 'dir')
        addpath(fullfile(root_dir, 'algo'));
    end
end

if nargin == 0
    mode = 'spec';
end

% 1. HỖ TRỢ TƯƠNG THÍCH NGƯỢC: Nếu tham số đầu tiên là struct (gọi batch cũ)
if isstruct(mode)
    [varargout{1}, varargout{2}] = bar10xt_batch_wrapper(mode, varargin{:});
    return;
end

% 2. HỖ TRỢ GỌI SỐ TRỰC TIẾP: Bar_10XT(depth_true, ...)
if isnumeric(mode)
    [varargout{1}, varargout{2}, varargout{3}, varargout{4}, varargout{5}] = ...
        bar10xt_step(mode, varargin{:});
    return;
end

% 3. CHẾ ĐỘ MÔ-ĐUN CHUẨN
switch lower(char(mode))
    case 'spec'
        varargout{1} = build_spec();
    case 'init'
        varargout{1} = init_state(varargin{:});
    case {'step', 'measure'}
        [varargout{1}, varargout{2}, varargout{3}, varargout{4}, varargout{5}] = ...
            bar10xt_step(varargin{:});
    case 'batch'
        [varargout{1}, varargout{2}] = bar10xt_batch_wrapper(varargin{:});
    otherwise
        error('Bar_10XT: mode khong hop le: %s. Chon ''spec'', ''init'', ''step'' hoac ''batch''.', mode);
end

end

%% ===================== THÔNG SỐ KỸ THUẬT BAR10XT =====================
function s = build_spec()
s.ten = 'BlueRobotics Bar10XT High-Resolution Depth/Pressure Sensor';

% 1. Thông số điện năng và logic phần cứng (từ warning.md)
s.vin_typical_v = 3.3;                 % Điện áp danh định (V)
s.vin_min_v = 2.5;                     % Điện áp cấp nguồn tối thiểu (V)
s.vin_max_v = 5.5;                     % Điện áp cấp nguồn tối đa (V)
s.i2c_logic_min_v = 1.8;               % Mức logic I2C tối thiểu (V)
s.i2c_logic_max_v = 3.6;               % Mức logic I2C tối đa (V)
s.current_typical_a = 1.5e-3;          % Dòng tiêu thụ hoạt động (1.5 mA)
s.current_idle_a = 100e-9;             % Dòng tiêu thụ chế độ nghỉ (100 nA)
s.temp_operating_min_c = -10.0;        % Nhiệt độ hoạt động tối thiểu (-10 độ C)
s.temp_operating_max_c = 80.0;         % Nhiệt độ hoạt động tối đa (+80 độ C)

% 2. Thông số áp suất và độ sâu (từ warning.md)
s.p_atm_pa = 101325.0;                 % Áp suất khí quyển chuẩn tham chiếu PA (1.01325 bar)
s.p_max_mechanical_bar = 40.0;         % Ngưỡng phá hủy cơ khí tối đa (40 bar)
s.p_operating_min_bar = 0.0;           % Áp suất làm việc tối thiểu (0 bar)
s.p_operating_max_bar = 10.0;          % Áp suất làm việc danh định tối đa (10.0 bar)
s.depth_operating_max_m = 92.0;        % Độ sâu công bố danh định nước ngọt (92 m)
s.pressure_type = 'PA: Sealed pressure, 1 bar absolute reference';

% 3. Sai số và độ phân giải theo datasheet (từ warning.md)
s.accuracy_mbar = 15.0;                % Độ chính xác danh định 0.15% FS (+-15 mbar)
s.accuracy_freshwater_m = 0.153;       % Quy đổi nước ngọt (+-0.153 m)
s.total_error_band_mbar = 70.0;        % Tổng dải sai số biến thiên nhiệt 0-50 độ C (+-70 mbar)
s.total_error_band_freshwater_m = 0.72;% Quy đổi nước ngọt (+-0.72 m)
s.resolution_mbar = 0.31;              % Độ phân giải áp suất ADC (0.31 mbar = 31 Pa)
s.resolution_freshwater_m = 3.12e-3;   % Quy đổi nước ngọt (3.12 mm)
s.long_term_stability_mbar = 10.0;     % Độ ổn định dài hạn công bố (+-10 mbar/năm, typ)

% 4. Thông số nhiệt độ đo tích hợp
s.temp_accuracy_c = 2.0;               % Sai số đo nhiệt độ (+-2 độ C)
s.temp_resolution_c = 0.0031;          % Độ phân giải đo nhiệt độ (0.0031 độ C)

% 5. Cấu hình độ lệch chuẩn danh định cho bộ lọc ES-EKF (giữ tương thích)
s.sigma_depth_nominal = 0.08;          % Độ lệch chuẩn độ sâu điều kiện ổn định (m)
s.sigma_depth_total = 0.15;            % Độ lệch chuẩn độ sâu khi có biến thiên nhiệt (m)

% 6. Các hệ số quy đổi thống kê và mô hình hóa
s.sigma_factor_white = 3.0;            % Accuracy là cận sai số tối đa 3-sigma (k = 3.0)
s.sigma_factor_bias  = 1.0;            % Long term stability ghi chú 'typ.' tức 1-sigma chuẩn (k = 1.0)
s.sigma_factor_temp  = 2.0;            % Dải sai số nhiệt độ quy đổi 2-sigma
s.bias_tau_s = 3600.0;                 % Thời hằng tương quan trôi bias Gauss-Markov (1 giờ cho stress test)
s.use_realistic_annual_drift = false;  % true: tính drift theo năm; false: stress-test 1 giờ cho EKF
s.enable_thermal_error = false;        % true: kích hoạt sai số biến thiên nhiệt độ

% 7. Giá trị giả định cố định của firmware / driver trên tàu
s.rho_assumed_seawater = 1025.0;       % Mật độ nước biển mặc định driver sử dụng (kg/m^3)
s.rho_assumed_freshwater = 997.5;      % Mật độ nước ngọt mặc định driver sử dụng (kg/m^3)
s.g_assumed = 9.80665;                 % Gia tốc trọng trường chuẩn driver sử dụng (m/s^2)
end

%% ===================== KHỞI TẠO TRẠNG THÁI CẢM BIEN =====================
function st = init_state(spec, seed)
if nargin < 1 || isempty(spec), spec = build_spec(); end
if nargin >= 2 && ~isempty(seed)
    st.rng_stream = RandStream('mt19937ar', 'Seed', seed);
else
    st.rng_stream = RandStream.getGlobalStream();
end

% Khởi tạo bias ban đầu (Turn-on Bias) theo đặc tính ổn định dài hạn
sigma_bias0 = (spec.long_term_stability_mbar * 100) / spec.sigma_factor_bias;
st.b_P_pa = sigma_bias0 * randn(st.rng_stream, 1);
st.time_accum_s = 0.0;
end

%% ===================== BƯỚC ĐO TRỰC TIẾP ONLINE (STEP) =====================
function [depth_meas, P_meas_bar, is_valid, state, info] = bar10xt_step(varargin)

% 1. BỘ TỰ ĐỘNG NHẬN DIỆN CHỮ KÝ VÀ BẢO VỆ CHỐNG NHẦM THAM SỐ (SIGNATURE AUTO-DETECTION)
% Nhận diện 2 phong cách gọi:
%   Phong cách Cũ: bar10xt_step(depth_true, rho, lat_rad, spec, use_total_error)
%   Phong cách Mới: bar10xt_step(depth_true, dt, state, spec, rho_true, lat_rad, rho_assumed, g_assumed)

persistent default_state;

if nargin == 0
    depth_true = 0.0;
    dt = 0.2;
    spec = build_spec();
    if isempty(default_state), default_state = init_state(spec); end
    state = default_state;
    rho_true = 1025.0;
    lat_rad = deg2rad(20.8);
    rho_assumed = spec.rho_assumed_seawater;
    g_assumed = spec.g_assumed;
else
    depth_true = varargin{1};
    
    % Kiểm tra xem có phải gọi theo cú pháp cũ (tham số 2 là mật độ nước rho > 50 kg/m^3):
    is_legacy_step = false;
    if nargin >= 2 && ~isempty(varargin{2}) && isnumeric(varargin{2}) && varargin{2} > 50.0
        is_legacy_step = true;
    elseif nargin >= 3 && ~isempty(varargin{3}) && isnumeric(varargin{3}) && ~isstruct(varargin{3})
        if nargin >= 2 && varargin{2} > 10.0
            is_legacy_step = true;
        end
    end
    
    if is_legacy_step
        % Phát cảnh báo và tự động ánh xạ đúng vị trí vật lý
        warning('Bar_10XT:LegacyStepSignature', ...
            ['Phát hiện gọi Bar_10XT(''step'') theo cú pháp cũ: (depth, rho, lat_rad, spec). ' ...
             'Hệ thống tự động ánh xạ tham số 2 thành rho_true = %.1f kg/m^3 (thay vì hiểu nhầm là chu kỳ lấy mẫu dt).'], varargin{2});
        rho_true = varargin{2};
        if nargin >= 3 && ~isempty(varargin{3}), lat_rad = varargin{3}; else, lat_rad = deg2rad(20.8); end
        if nargin >= 4 && ~isempty(varargin{4}), spec = varargin{4}; else, spec = build_spec(); end
        if nargin >= 5 && ~isempty(varargin{5}) && islogical(varargin{5}), spec.enable_thermal_error = varargin{5}; end
        
        dt = 0.2; % Đặt chu kỳ lấy mẫu an toàn 5 Hz
        if isempty(default_state), default_state = init_state(spec); end
        state = default_state;
        rho_assumed = spec.rho_assumed_seawater;
        g_assumed = spec.g_assumed;
    else
        % Cú pháp mới có dt và state
        if nargin >= 2 && ~isempty(varargin{2}), dt = varargin{2}; else, dt = 0.2; end
        
        % Cảnh báo nếu dt lớn bất thường (> 10s)
        if dt > 10.0
            warning('Bar_10XT:LargeDtWarning', ...
                'Chu kỳ dt = %.2f s lớn bất thường đối với cảm biến đo sâu. Hãy kiểm tra xem có truyền nhầm rho vào vị trí dt hay không.', dt);
        end
        
        if nargin >= 4 && ~isempty(varargin{4}), spec = varargin{4}; else, spec = build_spec(); end
        if nargin >= 3 && ~isempty(varargin{3}) && isstruct(varargin{3})
            state = varargin{3};
        else
            if isempty(default_state), default_state = init_state(spec); end
            state = default_state;
        end
        if nargin >= 5 && ~isempty(varargin{5}), rho_true = varargin{5}; else, rho_true = 1025.0; end
        if nargin >= 6 && ~isempty(varargin{6}), lat_rad = varargin{6}; else, lat_rad = deg2rad(20.8); end
        if nargin >= 7 && ~isempty(varargin{7}), rho_assumed = varargin{7}; else, rho_assumed = spec.rho_assumed_seawater; end
        if nargin >= 8 && ~isempty(varargin{8}), g_assumed = varargin{8}; else, g_assumed = spec.g_assumed; end
    end
end

% 2. MÔ HÌNH VẬT LÝ THỰC TẾ (GROUND TRUTH ENVIRONMENT)
% Tính gia tốc trọng trường thật theo vĩ độ WGS-84 và độ sâu tức thời
if exist('gravity_fossen', 'file')
    g_true = gravity_fossen(lat_rad, max(depth_true, 0));
else
    g_true = 9.80665;
end

% Tính áp suất thủy tĩnh thật và áp suất tuyệt đối thật tác động lên màng cảm biến (Pa)
P_atm_Pa = spec.p_atm_pa;
P_hydro_true_Pa = rho_true * g_true * max(depth_true, 0);
P_total_true_Pa = P_atm_Pa + P_hydro_true_Pa;
P_total_true_bar = P_total_true_Pa * 1e-5;

% Kiểm tra ngưỡng bão hòa cơ học: Áp suất tác động vượt 10 bar tuyệt đối
is_valid = (P_total_true_bar <= spec.p_operating_max_bar);

% 3. MÔ HÌNH HÓA CÁC THÀNH PHẦN SAI SỐ PHẦN CỨNG
% 3.1. Sai số trôi dạt chậm Gauss-Markov bậc 1 (Long-Term Stability)
if spec.use_realistic_annual_drift
    % Mô phỏng theo tỷ lệ thời gian thực của 1 năm (8760 giờ)
    tau_eff = 8760 * 3600;
else
    % Giả định kỹ thuật cho bài test độ bền vững (Stress test 1 giờ)
    tau_eff = spec.bias_tau_s;
end
phi_bias = exp(-dt / tau_eff);
sigma_bias_steady = (spec.long_term_stability_mbar * 100) / spec.sigma_factor_bias;
sigma_gm_step = sigma_bias_steady * sqrt(max(1 - phi_bias^2, 0));
state.b_P_pa = phi_bias * state.b_P_pa + sigma_gm_step * randn(state.rng_stream, 1);
state.time_accum_s = state.time_accum_s + dt;

% 3.2. Nhiễu trắng danh định của mạch ADC (White Gaussian Noise)
sigma_white_pa = (spec.accuracy_mbar * 100) / spec.sigma_factor_white;
w_white_pa = sigma_white_pa * randn(state.rng_stream, 1);

% 3.3. Sai số do biến thiên nhiệt độ (Thermal Sensitivity Error)
w_temp_pa = 0.0;
if spec.enable_thermal_error
    % Chênh lệch giữa Total Error Band (70 mbar) và Accuracy (15 mbar)
    delta_thermal_mbar = sqrt(max(spec.total_error_band_mbar^2 - spec.accuracy_mbar^2, 0));
    sigma_temp_pa = (delta_thermal_mbar * 100) / spec.sigma_factor_temp;
    w_temp_pa = sigma_temp_pa * randn(state.rng_stream, 1);
end

% 3.4. Áp suất thô trước lượng tử hóa
P_raw_pa = P_total_true_Pa + state.b_P_pa + w_white_pa + w_temp_pa;

% 3.5. Lượng tử hóa theo độ phân giải 24-bit của ADC (0.31 mbar = 31 Pa)
res_pa = spec.resolution_mbar * 100;
P_quant_pa = round(P_raw_pa / res_pa) * res_pa;

% 3.6. Giới hạn bão hòa điện áp của mạch đo tại 10.0 bar
P_meas_pa = min(max(P_quant_pa, 0), spec.p_operating_max_bar * 1e5);
P_meas_bar = P_meas_pa * 1e-5;

% 4. FIRMWARE / DRIVER TRÊN TÀU QUY ĐỔI SANG ĐỘ SÂU (ONBOARD REVERSE MAPPING)
% Driver hoàn toàn KHÔNG biết rho_true hay g_true, chỉ dùng rho_assumed và g_assumed:
depth_meas = (P_meas_pa - P_atm_Pa) / (rho_assumed * g_assumed);

% 5. ĐÓNG GÓI THÔNG TIN CHẨN ĐOÁN
info = struct();
info.depth_true = depth_true;
info.depth_meas = depth_meas;
info.P_meas_bar = P_meas_bar;
info.P_total_true_bar = P_total_true_bar;
info.bias_P_pa = state.b_P_pa;
info.w_white_pa = w_white_pa;
info.w_temp_pa = w_temp_pa;
info.rho_true = rho_true;
info.rho_assumed = rho_assumed;
info.g_true = g_true;
info.g_assumed = g_assumed;
info.is_valid = is_valid;

% Sai số tỷ lệ do sai lệch mật độ và trọng trường (Scale mismatch error):
info.scale_mismatch = (rho_true * g_true) / (rho_assumed * g_assumed) - 1.0;

% Độ lệch chuẩn nhiễu trắng tương đương cấp cho ma trận R của ES-EKF:
% Chú ý: R chỉ nên đại diện cho nhiễu trắng tức thời, không gộp bias trôi chậm b_P
info.sigma_depth_ekf = sigma_white_pa / (rho_assumed * g_assumed);
end

%% ===================== HÀM BATCH WRAPPER TƯƠNG THÍCH =====================
function [Z_depth, depth_sim] = bar10xt_batch_wrapper(p, measurement_indices, position_true, sigma_depth, dt_in)
% Wrapper bảo đảm tương thích 100% với các script gọi kiểu cũ (như Apsuat.m)

spec = build_spec();
state = init_state(spec);

if nargin < 1 || isempty(p), p = struct(); end

% Xác định mật độ nước thật từ struct cấu hình
if isfield(p, 'env') && isfield(p.env, 'khoi_luong_rieng_kg_m3')
    rho_true = p.env.khoi_luong_rieng_kg_m3;
else
    rho_true = 1025.0; % Mặc định nước biển
end

% Vĩ độ
if isfield(p, 'lat_rad'), lat_rad = p.lat_rad; else, lat_rad = deg2rad(20.8); end

% Giả định firmware trên tàu
rho_assumed = 1025.0;
g_assumed = 9.80665;

n_meas = numel(measurement_indices);
depth_true_all = position_true(measurement_indices, 3);

% Xác định chu kỳ lấy mẫu dt_meas động từ measurement_indices hoặc tham số đầu vào
if nargin >= 5 && ~isempty(dt_in)
    dt_meas = dt_in;
elseif isfield(p, 'dt_dvl') && ~isempty(p.dt_dvl)
    dt_meas = p.dt_dvl;
elseif isfield(p, 'dt') && numel(measurement_indices) > 1
    step_idx = measurement_indices(2) - measurement_indices(1);
    dt_meas = step_idx * p.dt;
else
    dt_meas = 0.2; % Mặc định 5 Hz nếu không có thông tin bước thời gian
end

Z_depth = zeros(n_meas, 1);
P_meas_bar_all = zeros(n_meas, 1);
valid_all = true(n_meas, 1);

for k = 1:n_meas
    d_true = depth_true_all(k);
    if nargin >= 4 && ~isempty(sigma_depth)
        % Chế độ ép độ lệch chuẩn đo lường thủ công (dùng đúng state.rng_stream để tái lập Monte Carlo)
        g_k = gravity_fossen(lat_rad, max(d_true, 0));
        P_tot = spec.p_atm_pa + rho_true * g_k * max(d_true, 0);
        sig_P = rho_assumed * g_assumed * sigma_depth;
        P_m = P_tot + sig_P * randn(state.rng_stream, 1);
        res_pa = spec.resolution_mbar * 100;
        P_m = round(P_m / res_pa) * res_pa;
        P_m = min(max(P_m, 0), spec.p_operating_max_bar * 1e5);
        Z_depth(k) = (P_m - spec.p_atm_pa) / (rho_assumed * g_assumed);
        P_meas_bar_all(k) = P_m * 1e-5;
        valid_all(k) = (P_tot * 1e-5 <= spec.p_operating_max_bar);
    else
        % Chế độ mô phỏng Bar10XT vật lý hoàn chỉnh
        [Z_depth(k), P_meas_bar_all(k), valid_all(k), state] = ...
            bar10xt_step(d_true, dt_meas, state, spec, rho_true, lat_rad, rho_assumed, g_assumed);
    end
end

depth_sim = struct();
depth_sim.P_meas_bar = P_meas_bar_all;
depth_sim.P_hydro_bar = P_meas_bar_all - (spec.p_atm_pa * 1e-5);
depth_sim.rho_water = rho_true;
depth_sim.is_valid = valid_all;
depth_sim.spec = spec;
end
