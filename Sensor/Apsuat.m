function [Z_depth, depth_sim] = Apsuat(p, measurement_indices, position_true, sigma_depth)
%APSUAT  Giả lập cảm biến áp suất thủy tĩnh đo độ sâu UUV.
%
% 1. Nguyên lý mô phỏng:
%    - Cảm biến áp suất đo áp suất thủy tĩnh tổng P_total (khí quyển + cột nước):
%      P_total = P_atm + rho * g * depth_true
%    - Trong đó:
%      + P_atm: Áp suất khí quyển mặt hồ (1.01325 bar = 101,325 Pa).
%      + rho  : Khối lượng riêng nước ngọt hồ Hòa Bình (~997.8 kg/m^3 ở 22°C).
%      + g    : Gia tốc trọng trường chuẩn (9.80665 m/s^2).
%    - Mô hình sai số:
%      + Nhiễu trắng ngẫu nhiên của bộ biến lực áp điện / màng áp suất (sigma_depth).
%    - Quy đổi từ áp suất đo sang độ sâu:
%      Z_depth = (P_meas - P_atm) / (rho * g) = depth_true + noise
%
% 2. Đầu vào:
%    - p                   : Struct thông số hệ thống chuẩn (từ thong_so_he_thong_chuan).
%    - measurement_indices : Mảng chỉ số các thời điểm lấy mẫu đo sâu (chu kỳ 5 Hz).
%    - position_true       : Tọa độ vị trí chuẩn hệ NED (N x 3, m), cột 3 là độ sâu Down.
%    - sigma_depth         : Độ lệch chuẩn sai số đo sâu (m, tùy chọn, mặc định 0.05 m).
%
% 3. Đầu ra:
%    - Z_depth             : Vector độ sâu đo được (N_meas x 1, mét, Z > 0 khi lặn sâu).
%    - depth_sim           : Struct thông tin chi tiết:
%                            .P_meas_bar   : Áp suất tổng đo được (bar)
%                            .P_hydro_bar  : Áp suất thủy tĩnh do cột nước (bar)
%                            .rho_water    : Mật độ nước (kg/m^3)
%                            .sigma_depth  : Độ lệch chuẩn sai số (m)

% Đảm bảo thư mục algo có trong path để gọi gravity_fossen
if ~exist('gravity_fossen', 'file')
	root_dir = fileparts(fileparts(mfilename('fullpath')));
	if exist(fullfile(root_dir, 'algo'), 'dir')
		addpath(fullfile(root_dir, 'algo'));
	end
end

% Giá trị mặc định nếu chạy thử hoặc thiếu tham số
if nargin < 1 || isempty(p)
	p = struct();
	p.lat_rad = deg2rad(20.8);
	p.dat.g = gravity_fossen(p.lat_rad, 0);
	p.env.khoi_luong_rieng_kg_m3 = 1025.0;
end
if nargin < 4 || isempty(sigma_depth)
	sigma_depth = 0.05; % Sai số mặc định 5 cm
end

% Lấy mật độ nước (mặc định nước biển 1025 kg/m^3)
if isfield(p, 'env') && isfield(p.env, 'khoi_luong_rieng_kg_m3')
	rho = p.env.khoi_luong_rieng_kg_m3;
else
	rho = 1025.0; % Mặc định nước biển
end

% Lấy vĩ độ địa lý
if isfield(p, 'lat_rad') && ~isempty(p.lat_rad)
	lat_rad = p.lat_rad;
else
	lat_rad = deg2rad(20.8);
end

% Tính gia tốc trọng trường chuẩn qua algo/gravity_fossen.m
if isfield(p, 'dat') && isfield(p.dat, 'g') && ~isempty(p.dat.g)
	g = p.dat.g;
else
	g = gravity_fossen(lat_rad, 0);
end
P_atm_Pa = 101325; % 1 atm chuẩn (Pa)

n_measurements = numel(measurement_indices);
depth_true = position_true(measurement_indices, 3);

% 1. Tính áp suất thủy tĩnh thực tế
P_hydro_true_Pa = rho * g * max(depth_true, 0); % Chỉ tính cột nước khi ở dưới mặt nước
P_total_true_Pa = P_atm_Pa + P_hydro_true_Pa;

% 2. Cộng nhiễu cảm biến áp suất (đổi tương đương từ sigma_depth sang Pa)
sigma_P_Pa = rho * g * sigma_depth;
P_meas_Pa = P_total_true_Pa + sigma_P_Pa * randn(n_measurements, 1);

% 3. Quy đổi ngược về độ sâu đo được (Z_depth)
Z_depth = (P_meas_Pa - P_atm_Pa) / (rho * g);

% Đóng gói dữ liệu phụ trợ
depth_sim = struct();
depth_sim.P_meas_bar = P_meas_Pa * 1e-5;
depth_sim.P_hydro_bar = (P_meas_Pa - P_atm_Pa) * 1e-5;
depth_sim.rho_water = rho;
depth_sim.sigma_depth = sigma_depth;
depth_sim.sigma_P_bar = sigma_P_Pa * 1e-5;

end
