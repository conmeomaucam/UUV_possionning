% =========================================================================
% KỊCH BẢN QUỸ ĐẠO NHIỆM VỤ REMUS 100 (TỰ ĐỘNG SINH TỪ MISSION PLANNER)
% Ngày tạo: 08:00:16 28/10/2026
% Tọa độ gốc WGS-84 (WP1):
%   Vĩ độ (Lat0) = 20.720000 deg
%   Kinh độ (Lon0) = 107.090000 deg
% =========================================================================

% 1. Định nghĩa ma trận Waypoints chuẩn 3D cho bộ dẫn đường ALOS3D
% Đã bổ sung 1 waypoint quay ngược (x=40.0, y=180.0, z=0.0) để tạo cơ động chạy 2 chiều ngược nhau (reciprocal track)
% giúp bộ lọc ES-EKF tách rời sai số Boresight DVL khỏi sai số từ kế trước khi lặn.
wpt.pos.x = [    0.0,    0.0,  -30.0,   30.0,   40.0,  -30.0,    0.0,    3.6,   56.9,  -23.0,   -0.8,   14.7 ]; % North (m)
wpt.pos.y = [    0.0,  120.0,  270.0,  420.0,  180.0,  570.0,  720.0, 1500.4, 2582.1, 2928.6, 4379.2, 6446.6 ]; % East (m)
wpt.pos.z = [    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,   50.0,   50.0,   30.0,   30.0,    0.0 ]; % Down (m)
wpt.speed = [   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   3.00,   2.50,   2.50,   1.50,   3.00,   3.50 ]; % Tốc độ hành trình đặt (m/s)

% 2. Cấu hình thời gian chạy mô phỏng
T_final    = 3800;       % Tổng thời gian mô phỏng (giây) [đã bù thêm thời gian chạy ngược 2 chiều]
T_gnss_cut = 410.0;      % Thời điểm ước tính bắt đầu lặn z > 0.5m (giây)

% 3. Tọa độ địa lý gốc WGS-84
lat_origin_deg = 20.720000;
lon_origin_deg = 107.090000;
lat_rad        = deg2rad(lat_origin_deg);

fprintf('[KỊCH BẢN] Đã nạp thành công %d Waypoints (bao gồm 1 chặng quay ngược hiệu chuẩn Boresight).\n', length(wpt.pos.x));
fprintf('          - Tổng cự ly hành trình: %.2f km\n', 6.96);
fprintf('          - Thời gian ước tính    : %.1f s (~%.1f phút)\n', 2600.0, 43.3);
fprintf('          - Thời điểm cắt GNSS ước tính: %.1f s\n', T_gnss_cut);
