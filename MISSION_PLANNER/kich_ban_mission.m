% =========================================================================
% KỊCH BẢN QUỸ ĐẠO NHIỆM VỤ REMUS 100 (TỰ ĐỘNG SINH TỪ MISSION PLANNER)
% Ngày tạo: 27/10/2026, 10:20:00
% Tọa độ gốc WGS-84 (WP1):
%   Vĩ độ (Lat0) = 20.800000 deg (Vịnh Hạ Long / Cát Bà)
%   Kinh độ (Lon0) = 107.050000 deg
% =========================================================================

% 1. Định nghĩa ma trận Waypoints chuẩn 3D cho bộ dẫn đường ALOS3D
wpt.pos.x = [    0.0,    0.0,  -30.0,   30.0,  -20.0,    0.0,  -20.0, -100.0,    0.0,  200.0,  200.0,  400.0 ]; % North (m)
wpt.pos.y = [    0.0,  120.0,  270.0,  430.0,  580.0,  720.0,  920.0, 1320.0, 1670.0, 2020.0, 2520.0, 2920.0 ]; % East (m)
wpt.pos.z = [    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,   10.0,  100.0,  100.0,   50.0,   50.0,   50.0 ]; % Down (m)
wpt.speed = [   1.50,   1.50,   1.50,   1.50,   1.50,   1.50,   1.50,   1.50,   1.50,   1.50,   1.50,   1.50 ]; % Tốc độ hành trình đặt (m/s)

% 2. Cấu hình thời gian chạy mô phỏng
T_final    = 2360;       % Tổng thời gian mô phỏng (giây) [bao gồm dự phòng quay đầu]
T_gnss_cut = 502.3;      % Thời điểm cắt GNSS (giây) [khi tàu lặn z > 0.5m]

% 3. Tọa độ địa lý gốc WGS-84
lat_origin_deg = 20.800000;
lon_origin_deg = 107.050000;
lat_rad        = deg2rad(lat_origin_deg);

fprintf('[KỊCH BẢN] Đã nạp thành công 12 Waypoints từ Mission Planner.\n');
fprintf('          - Tổng cự ly hành trình: 3.08 km\n');
fprintf('          - Thời gian ước tính    : 2053.0 s (~34.2 phút)\n');
fprintf('          - Thời điểm cắt GNSS    : %.1f s\n', T_gnss_cut);
