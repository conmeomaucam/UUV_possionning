function varargout = GEO_FOG_3D(mode, varargin)
%GEO_FOG_3D  Giả lập hệ dẫn đường quán tính GEO-FOG 3D Dual INS (Emcore / KVH).
%
% CẤU HÌNH DÙNG CHO UUV: KHÔNG DÙNG TỪ KẾ (Magnetometer = OFF).
%   Lý do: từ trường bên trong vỏ UUV bị méo bởi động cơ, chân vịt và ắc quy.
%
% NGUỒN THÔNG SỐ:
%   [D] dieukiendauvao/SoSanh_INS-DM-FI_-_GEO-FOG-3D_final.docx (bảng so sánh)
%   [W] Datasheet GEO-FOG 3D Dual (Emcore, bản phân phối Canal Geomatics):
%       gyro bias instability <= 0.05 deg/h (typ, 1 sigma, 25 C)
%   GIẢ ĐỊNH: các số không có trong [D] và [W] (ghi rõ tại chỗ).
%
% CÁCH GỌI:
%   spec  = GEO_FOG_3D('spec');                 % Bộ thông số SI của thiết bị
%   state = GEO_FOG_3D('init', spec, bg0, ba0); % Khởi tạo trạng thái bias thật
%   [gyro, accel, state] = GEO_FOG_3D('imu', spec, state, w_ib_true, f_b_true, dt);
%   gnss  = GEO_FOG_3D('gnss', spec, pos_ned, vel_ned, yaw);  % 7x1, chỉ gọi khi nổi
%
% Ghi chú: thiết bị KHÔNG cấp góc hướng tham chiếu khi lặn ở mô phỏng này.
%   Tài liệu [D] ghi GEO-FOG có gyrocompass chạy liên tục nhưng "Không ghi số độ"
%   cho hướng khi mất GNSS -> không đưa phép đo hướng bịa đặt vào bộ lọc.

switch lower(mode)
    case 'spec'
        varargout{1} = build_spec();
    case 'init'
        varargout{1} = init_state(varargin{:});
    case 'imu'
        [varargout{1}, varargout{2}, varargout{3}] = imu_step(varargin{:});
    case 'gnss'
        varargout{1} = gnss_meas(varargin{:});
    otherwise
        error('GEO_FOG_3D: mode khong hop le: %s', mode);
end
end

%% ===================== THÔNG SỐ THIẾT BỊ (SI) =====================
function s = build_spec()
d2r = pi/180;
s.ten = 'GEO-FOG 3D Dual INS (Emcore)';
s.co_tu_ke = false;                         % Bỏ từ kế theo yêu cầu

% Con quay FOG có chip quang tử (PIC)
s.gyro.arw        = 0.012 * d2r / 60;       % [D] <= 0.012 deg/sqrt(h) -> rad/sqrt(s)
s.gyro.bias_instab = 0.05 * d2r / 3600;     % [W] <= 0.05 deg/h typ -> rad/s
s.gyro.bias_tau   = 300;                    % GIẢ ĐỊNH thời hằng Gauss-Markov (s)
s.gyro.bias_rw    = s.gyro.bias_instab * sqrt(2 / s.gyro.bias_tau); % tương đương cho bộ lọc
s.gyro.dai_do     = 490 * d2r;              % [D] +-490 deg/s

% Gia tốc kế MEMS
s.acc.vrw         = 0.07 / 60;              % [D] 0.07 m/s/sqrt(h) -> m/s/sqrt(s)
s.acc.bias_instab = 0.05e-3 * 9.80665;      % [D] < 0.05 mg (1 sigma) -> m/s^2
s.acc.bias_tau    = 300;                    % GIẢ ĐỊNH (s)
s.acc.dai_do      = 10 * 9.80665;           % [D] +-10 g

% GNSS 2 ăng-ten RTK (chỉ khi nổi)
s.gnss.pos_h      = 0.008;                  % [D] RTK ngang 0.008 m
s.gnss.pos_v      = 0.015;                  % [D] RTK đứng 0.015 m
s.gnss.vel        = 0.005;                  % [D] 0.005 m/s
s.gnss.heading    = 0.01 * d2r;             % [D] 0.01 deg (không ghi baseline)
end

%% ===================== TRẠNG THÁI BIAS THẬT =====================
function st = init_state(spec, bg0, ba0)
% bg0, ba0: bias bật máy (turn-on) dùng chung cho mọi thiết bị để so sánh công bằng
st.bg_turn_on = bg0(:);
st.ba_turn_on = ba0(:);
st.bg_inrun = spec.gyro.bias_instab * randn(3, 1);
st.ba_inrun = spec.acc.bias_instab  * randn(3, 1);
end

%% ===================== SINH SỐ ĐO IMU =====================
function [gyro, accel, st] = imu_step(spec, st, w_ib_true, f_b_true, dt)
% In-run bias: Gauss-Markov bậc 1
pg = exp(-dt / spec.gyro.bias_tau);
pa = exp(-dt / spec.acc.bias_tau);
st.bg_inrun = pg*st.bg_inrun + spec.gyro.bias_instab*sqrt(1 - pg^2)*randn(3, 1);
st.ba_inrun = pa*st.ba_inrun + spec.acc.bias_instab *sqrt(1 - pa^2)*randn(3, 1);

bg = st.bg_turn_on + st.bg_inrun;
ba = st.ba_turn_on + st.ba_inrun;

gyro  = w_ib_true(:) + bg + (spec.gyro.arw / sqrt(dt)) * randn(3, 1);
accel = f_b_true(:)  + ba + (spec.acc.vrw  / sqrt(dt)) * randn(3, 1);

gyro  = min(max(gyro,  -spec.gyro.dai_do), spec.gyro.dai_do);
accel = min(max(accel, -spec.acc.dai_do),  spec.acc.dai_do);
st.bg_total = bg;
st.ba_total = ba;
end

%% ===================== SINH SỐ ĐO GNSS =====================
function z = gnss_meas(spec, pos_ned, vel_ned, yaw)
z = [pos_ned(1) + spec.gnss.pos_h*randn; ...
     pos_ned(2) + spec.gnss.pos_h*randn; ...
     pos_ned(3) + spec.gnss.pos_v*randn; ...
     vel_ned(:) + spec.gnss.vel*randn(3, 1); ...
     yaw + spec.gnss.heading*randn];
end
