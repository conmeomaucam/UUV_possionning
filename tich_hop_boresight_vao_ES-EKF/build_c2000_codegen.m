% =========================================================================
% SCRIPT: build_c2000_codegen.m
% MUC DICH: Sinh ma C chuan static library cho TI TMS320F28379D tu 2 file:
%           - esekf19_init.m
%           - esekf19_step.m
% =========================================================================

clear; clc;
fprintf('>>> Dang khoi tao cau hinh MATLAB Coder cho TI C2000 F28379D...\n');

%% 1. THIET LAP CAU HINH CODER (CODER CONFIGURATION)
cfg = coder.config('lib');                      % Sinh static library (.c / .h)
cfg.TargetLang = 'C';
cfg.TargetLangStandard = 'C99 (ISO)';
cfg.GenCodeOnly = true;                         % Chi sinh source C, build bang Code Composer Studio (CCS)
cfg.GenerateReport = true;                       % Sinh HTML report de xem code va interface
cfg.GenerateComments = true;
cfg.EnableMemcpy = true;
cfg.DynamicMemoryAllocation = 'Threshold';           % Cam tuyet doi cap phat dong (malloc) tren vi dieu khien

% --- Cau hinh dac thu cho Texas Instruments C2000 (C28x DSP) ---
% Tren C28x: 1 byte (char) = 16-bit (CHAR_BIT = 16), int = 16-bit, long = 32-bit
cfg.HardwareImplementation.ProdHWDeviceType = 'Texas Instruments->C2000';

%% 2. DINH NGHIA KIEU DU LIEU DAU VAO (TYPE SPECIFICATION)

% --- Tham so cho ham: esekf19_init(initial_rpy, gravity, noise_cfg) ---
% initial_rpy: [roll; pitch; yaw] (3x1 double)
type_initial_rpy = coder.typeof(double(0), [3, 1]); 

% gravity: So thuc vo huong (scalar double chuan)
type_gravity     = coder.typeof(double(0)); 

% noise_cfg: Vector cau hinh nhieu (8x1 double)
type_noise_cfg   = coder.typeof(double(0), [8, 1]); 

% --- Tham so cho ham: esekf19_step(...) ---
% state: Vector trang thai danh dinh (20x1 double)
type_state             = coder.typeof(double(0), [20, 1]); 

% covariance: Ma tran hiep phuong sai P (19x19 double)
type_covariance        = coder.typeof(double(0), [19, 19]); 

% process_noise: Ma tran nhieu qua trinh Q (19x19 double)
type_process_noise     = coder.typeof(double(0), [19, 19]); 

% gyro_meas: Van toc goc IMU do duoc [rad/s] (3x1 double)
type_gyro_meas         = coder.typeof(double(0), [3, 1]); 

% accel_meas: Gia toc IMU do duoc [m/s^2] (3x1 double)
type_accel_meas        = coder.typeof(double(0), [3, 1]); 

% gnss_meas: [pos_ned(3); vel_ned(3); yaw] (7x1 double)
type_gnss_meas         = coder.typeof(double(0), [7, 1]); 

% depth_meas: Do sau vo huong [m] (scalar double chuan)
type_depth_meas        = coder.typeof(double(0)); 

% dvl_meas: Van toc DVL 3 truc (3x1 double)
type_dvl_meas          = coder.typeof(double(0), [3, 1]); 

% measurement_flags: [GNSS_valid; depth_valid; DVL_valid] (3x1 double)
type_measurement_flags = coder.typeof(double(0), [3, 1]); 

% cfg: Mang thong so he thong va do lech chuan cam bien (18x1 double)
type_step_cfg          = coder.typeof(double(0), [18, 1]); 

%% 3. THUC THI SINH MA (CODE GENERATION)
fprintf('>>> Bat dau chay codegen...\n');

% Dong goi danh sach tham so dau vao
args_init = {type_initial_rpy, type_gravity, type_noise_cfg};
args_step = {type_state, type_covariance, type_process_noise, ...
    type_gyro_meas, type_accel_meas, type_gnss_meas, ...
    type_depth_meas, type_dvl_meas, type_measurement_flags, type_step_cfg};

codegen -config cfg -o esekf19_c2000 ...
    esekf19_init -args args_init ...
    esekf19_step -args args_step

fprintf('\n>>> [THANH CONG] Toan bo ma C da duoc sinh trong thu muc: codegen/lib/esekf19_c2000/\n');
fprintf('>>> Ban co the mo file "codegen/lib/esekf19_c2000/html/index.html" de kiem tra giao dien C sinh ra.\n');