% =========================================================================
% KỊCH BẢN QUỸ ĐẠO NHIỆM VỤ REMUS 100 (TỰ ĐỘNG SINH TỪ MISSION PLANNER)
% Ngày tạo: 14:33:19 27/10/2026
% Tọa độ gốc WGS-84 (WP1):
%   Vĩ độ (Lat0) = 20.720000 deg
%   Kinh độ (Lon0) = 107.090000 deg
% =========================================================================

% 1. Định nghĩa ma trận Waypoints chuẩn 3D cho bộ dẫn đường ALOS3D
wpt.pos.x = [    0.0,    0.0,  -30.0,   30.0,  -20.0,    0.0,  -20.0, -100.0,    0.0,  200.0,  200.0,  381.4,  210.3 ]; % North (m)
wpt.pos.y = [    0.0,  120.0,  270.0,  430.0,  580.0,  720.0,  920.0, 1320.0, 1670.0, 2020.0, 2520.0, 6607.6,12099.2 ]; % East (m)
wpt.pos.z = [    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,   50.0,    0.0,   50.0 ]; % Down (m)
wpt.speed = [   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   1.50 ]; % Tốc độ hành trình đặt (m/s)

% 2. Cấu hình thời gian chạy mô phỏng
T_final    = 6618;       % Tổng thời gian mô phỏng (giây) [bao gồm dự phòng quay đầu]
T_gnss_cut = 708.1;      % Thời điểm cắt GNSS (giây) [khi tàu lặn z > 0.5m]

% 3. Tọa độ địa lý gốc WGS-84
lat_origin_deg = 20.720000;
lon_origin_deg = 107.090000;
lat_rad        = deg2rad(lat_origin_deg);

fprintf('[KỊCH BẢN] Đã nạp thành công %d Waypoints.\n', length(wpt.pos.x));
fprintf('          - Tổng cự ly hành trình: %.2f km\n', 12.21);
fprintf('          - Thời gian ước tính    : %.1f s (~%.1f phút)\n', 4679.9, 78.0);
fprintf('          - Thời điểm cắt GNSS    : %.1f s\n', T_gnss_cut);
