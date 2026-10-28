% =========================================================================
% KỊCH BẢN QUỸ ĐẠO NHIỆM VỤ REMUS 100 (TỰ ĐỘNG SINH TỪ MISSION PLANNER)
% Ngày tạo: 08:00:16 28/10/2026
% Tọa độ gốc WGS-84 (WP1):
%   Vĩ độ (Lat0) = 20.720000 deg
%   Kinh độ (Lon0) = 107.090000 deg
% =========================================================================

% 1. Định nghĩa ma trận Waypoints chuẩn 3D cho bộ dẫn đường ALOS3D
wpt.pos.x = [    0.0,    0.0,  -30.0,   30.0,  -30.0,    0.0,    3.6,   56.9,  -23.0,   -0.8,   14.7 ]; % North (m)
wpt.pos.y = [    0.0,  120.0,  270.0,  420.0,  570.0,  720.0, 1500.4, 2582.1, 2928.6, 4379.2, 6446.6 ]; % East (m)
wpt.pos.z = [    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,   50.0,   50.0,   30.0,   30.0,    0.0 ]; % Down (m)
wpt.speed = [   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   2.50,   2.50,   1.50,   3.00,   3.50 ]; % Tốc độ hành trình đặt (m/s)

% 2. Cấu hình thời gian chạy mô phỏng
T_final    = 3576;       % Tổng thời gian mô phỏng (giây) [bao gồm dự phòng quay đầu]
T_gnss_cut = 252.5;      % Thời điểm cắt GNSS (giây) [khi tàu lặn z > 0.5m]

% 3. Tọa độ địa lý gốc WGS-84
lat_origin_deg = 20.720000;
lon_origin_deg = 107.090000;
lat_rad        = deg2rad(lat_origin_deg);

fprintf('[KỊCH BẢN] Đã nạp thành công %d Waypoints.\n', length(wpt.pos.x));
fprintf('          - Tổng cự ly hành trình: %.2f km\n', 6.49);
fprintf('          - Thời gian ước tính    : %.1f s (~%.1f phút)\n', 2426.3, 40.4);
fprintf('          - Thời điểm cắt GNSS    : %.1f s\n', T_gnss_cut);
