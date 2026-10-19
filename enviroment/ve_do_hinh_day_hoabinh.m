% VE_DO_HINH_DAY_HOABINH.M
% =========================================================================
% BẢN ĐỒ ĐỊA HÌNH ĐÁY HỒ HÒA BÌNH CHUẨN HỆ TỌA ĐỘ NED (NORTH - EAST - DOWN)
% Xử lý trắc địa chuyên sâu:
% - Loại bỏ 100% vách dựng ảo ở mép biên (Lọc tam giác Delaunay theo luồng đo)
% - Hệ NED: X = North (Bắc), Y = East (Đông), Z = Down (Độ sâu đáy hồ > 0)
% - Trục Z đảo ngược (ZDir = reverse): Mặt nước 0m ở trên, đáy sâu 93m ở dưới
% =========================================================================

clear; clc; close all;

cur_dir = fileparts(mfilename('fullpath'));
xyz_path = fullfile(cur_dir, 'Depth_hoabinh_merged.xyz');
if ~exist(xyz_path, 'file')
    xyz_path = 'Depth_hoabinh_merged.xyz';
end

fprintf('>>> Đang nạp 16,429 điểm đo độ sâu thực tế hồ Hòa Bình...\n');
raw_data = readmatrix(xyz_path, 'FileType', 'text');

utm_e = raw_data(:, 1);
utm_n = raw_data(:, 2);
elev  = raw_data(:, 3); % Cao độ âm (-8m đến -93m)

% -------------------------------------------------------------------------
% 1. CHUYỂN TOÀN BỘ SANG HỆ TỌA ĐỘ NED (ĐƠN VỊ: MÉT)
% -------------------------------------------------------------------------
% Gốc tham chiếu NED [0, 0, 0]:
N0 = min(utm_n);
E0 = min(utm_e);

N_ned = utm_n - N0;     % Trục X_ned (North - mét)
E_ned = utm_e - E0;     % Trục Y_ned (East - mét)
D_ned = abs(elev);      % Trục Z_ned (Down - mét, độ sâu dương từ mặt nước xuống)

fprintf('>>> Thống kê phạm vi hệ NED:\n');
fprintf('    + Trục North (X_ned): 0 m -> %.1f m (Rộng: %.2f km)\n', max(N_ned), max(N_ned)/1000);
fprintf('    + Trục East  (Y_ned): 0 m -> %.1f m (Dài: %.2f km)\n', max(E_ned), max(E_ned)/1000);
fprintf('    + Trục Down  (Z_ned): %.1f m (Bờ cạn) -> %.1f m (Lòng sông sâu nhất)\n\n', min(D_ned), max(D_ned));

% -------------------------------------------------------------------------
% 2. THIẾT LẬP THAM SỐ MÔI TRƯỜNG THỦY ÂM HỒ HÒA BÌNH (p.env)
% -------------------------------------------------------------------------
p = struct();
p.env.ten                    = 'Hồ Hòa Bình (Sông Đà)';
p.env.loai_nuoc              = 'freshwater';         % Nước ngọt
p.env.do_man_psu             = 0.0;                  % Độ mặn S = 0 PSU (hồ nước ngọt)
p.env.nhiet_do_C             = 22.0;                 % Nhiệt độ nước trung bình (°C)
p.env.khoi_luong_rieng_kg_m3 = 997.8;                % Khối lượng riêng nước ngọt ở ~22°C (kg/m^3)
p.env.do_sau_danh_dinh_m     = 15.0;                 % Độ sâu lặn danh định của UUV (m)

% Hàm tính vận tốc truyền âm Mackenzie (1981): c = f(T, S, D)
% Mackenzie, K.V. "Nine-term equation for sound speed in the oceans", JASA 1981.
p.env.ham_van_toc_am = @(T, S, D) 1449.2 + 4.6*T - 0.055*T.^2 + ...
    0.00029*T.^3 + (1.34 - 0.010*T).*(S - 35) + 0.016*D;

% Vận tốc âm thực tế tại môi trường hồ Hòa Bình (S = 0, T = 22°C, D = 15m)
p.env.c_thuc_mps             = p.env.ham_van_toc_am(p.env.nhiet_do_C, p.env.do_man_psu, p.env.do_sau_danh_dinh_m);

% Vận tốc âm cấu hình mặc định trên DVL (thường mặc định nước biển 1500 m/s)
p.env.c_dvl_cai_dat_mps      = 1500.0; 

% Sai số tỉ lệ Scale Factor do chênh lệch vận tốc truyền âm nước ngọt vs nước mặn
% Công thức: s = (c_thuc - c_dvl) / c_dvl
p.env.scale_factor_am_hoc    = (p.env.c_thuc_mps - p.env.c_dvl_cai_dat_mps) / p.env.c_dvl_cai_dat_mps;

% Thống kê hình học trắc địa lòng hồ Hòa Bình
p.env.gioi_han_north_m       = [0, max(N_ned)];
p.env.gioi_han_east_m        = [0, max(E_ned)];
p.env.do_sau_min_m           = min(D_ned);
p.env.do_sau_max_m           = max(D_ned);
p.env.do_sau_trung_binh_m    = mean(D_ned);
p.env.so_diem_do_thuc_te     = size(raw_data, 1);
p.env.nguong_loc_tam_giac_m  = 120;                  % Ngưỡng lọc cạnh tam giác luồng đo (m)
p.env.goc_tham_chieu_ned     = [N0, E0, 0];          % Gốc tọa độ UTM tham chiếu [N0, E0, 0]

fprintf('>>> Cấu hình môi trường thủy âm hồ Hòa Bình (p.env):\n');
fprintf('    + Môi trường nước       : %s (Độ mặn: %.1f PSU, Nhiệt độ: %.1f °C)\n', p.env.loai_nuoc, p.env.do_man_psu, p.env.nhiet_do_C);
fprintf('    + Vận tốc âm thực tế c  : %.2f m/s (tại độ sâu lặn %.1f m)\n', p.env.c_thuc_mps, p.env.do_sau_danh_dinh_m);
fprintf('    + Vận tốc âm DVL cài đặt: %.2f m/s (chuẩn nước biển)\n', p.env.c_dvl_cai_dat_mps);
fprintf('    + Sai số Scale Factor s : %.4f (%+.2f%% do lệch tốc độ âm hồ Hòa Bình)\n\n', p.env.scale_factor_am_hoc, p.env.scale_factor_am_hoc * 100);

% -------------------------------------------------------------------------
% 3. TRIANGULATION CHUẨN TRẮC ĐỊA & LỌC SẠCH BIÊN (XÓA BỎ VÁCH DỰNG ẢO)
% -------------------------------------------------------------------------
fprintf('>>> Đang dựng lưới tam giác Delaunay và lọc biên luồng đo...\n');

DT = delaunayTriangulation(N_ned, E_ned);
tri = DT.ConnectivityList;
pts = DT.Points;

% Tính độ dài 3 cạnh của từng tam giác
p1 = pts(tri(:, 1), :);
p2 = pts(tri(:, 2), :);
p3 = pts(tri(:, 3), :);

d12 = hypot(p1(:,1) - p2(:,1), p1(:,2) - p2(:,2));
d23 = hypot(p2(:,1) - p3(:,1), p2(:,2) - p3(:,2));
d31 = hypot(p3(:,1) - p1(:,1), p3(:,2) - p1(:,2));

% Ngưỡng lọc cạnh: Chỉ giữ tam giác nằm trong vùng quét của thuyền đo (< 120m)
% Loại bỏ hoàn toàn các tam giác bắc cầu qua bờ gây vách dựng đứng ảo
max_edge_len = 120; 
valid_triangles = (d12 <= max_edge_len) & (d23 <= max_edge_len) & (d31 <= max_edge_len);
tri_clean = tri(valid_triangles, :);

fprintf('    + Đã lọc bỏ %d tam giác ảo ở mép ngoài.\n', sum(~valid_triangles));
fprintf('    + Giữ lại %d tam giác thực thuộc lòng hồ.\n\n', size(tri_clean, 1));

% -------------------------------------------------------------------------
% 4. VẼ ĐỒ HÌNH 3D ĐÁY HỒ CHUẨN HỆ NED (KHÔNG RÂU RIA)
% -------------------------------------------------------------------------
fig1 = figure('Name', 'Bản đồ 3D Đáy hồ Hòa Bình (Hệ NED)', 'Color', 'w', 'Position', [60, 60, 1150, 680]);

% Vẽ bề mặt thực tế bằng trisurf
h_surf = trisurf(tri_clean, N_ned, E_ned, D_ned, D_ned, ...
    'EdgeColor', 'none', 'FaceColor', 'interp');

% Cài đặt hệ quy chiếu NED:
% Trục Z (Down) đảo ngược: 0m (Mặt nước) ở trên, số dương đi sâu xuống đáy!
set(gca, 'ZDir', 'reverse');

% Tỷ lệ hiển thị trực quan (Trục Z phóng đại hợp lý để nhìn rõ luồng lạch)
% Trục East dài 10km, North 1.7km, Depth 93m:
pbaspect([3, 10, 2.5]);

colormap(jet);
cb = colorbar;
ylabel(cb, 'Độ sâu Z_{Down} (m)', 'FontSize', 11, 'FontWeight', 'bold');
caxis([0, 95]);

xlabel('Trục North X_{NED} (m)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Trục East Y_{NED} (m)', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('Độ sâu Down Z_{NED} (m)', 'FontSize', 11, 'FontWeight', 'bold');
title('BẢN ĐỒ KHÔNG GIAN 3D ĐỊA HÌNH ĐÁY HỒ HÒA BÌNH (HỆ QUY CHIẾU NED)', ...
    'FontSize', 13, 'FontWeight', 'bold');

shading interp;
camlight headlight;
lighting gouraud;
grid on;
box on;
view([-35, 32]);

% -------------------------------------------------------------------------
% 5. VẼ BẢN ĐỒ BÌNH ĐỒ 2D ĐẲNG SÂU (BATHYMETRY CONTOUR)
% -------------------------------------------------------------------------
fig2 = figure('Name', 'Bản đồ Bình đồ Đẳng sâu (Hệ NED)', 'Color', 'w', 'Position', [120, 120, 1100, 520]);

% Tạo lưới nội suy mịn chỉ trong phạm vi luồng hồ đã lọc
grid_N = linspace(min(N_ned), max(N_ned), 250);
grid_E = linspace(min(E_ned), max(E_ned), 800);
[Mesh_E, Mesh_N] = meshgrid(grid_E, grid_N);

F_interp = scatteredInterpolant(N_ned, E_ned, D_ned, 'linear', 'none');
Mesh_D = F_interp(Mesh_N, Mesh_E);

% Lọc bỏ các điểm lưới nằm ngoài luồng đo thực tế (Dùng MATLAB Core, không cần Toolbox phụ)
TR_clean = triangulation(tri_clean, [N_ned, E_ned]);
ti = pointLocation(TR_clean, [Mesh_N(:), Mesh_E(:)]);
mask_outside = reshape(isnan(ti), size(Mesh_N));
Mesh_D(mask_outside) = NaN;

% Gắn kết hàm nội suy và mặt lưới vào struct p.env phục vụ mô phỏng UUV/DVL
p.env.ham_noi_suy_do_sau = F_interp;
p.env.Mesh_N = Mesh_N;
p.env.Mesh_E = Mesh_E;
p.env.Mesh_D = Mesh_D;

[C, h_cont] = contourf(Mesh_E, Mesh_N, Mesh_D, 18, 'LineColor', [0.25 0.25 0.25]);
clabel(C, h_cont, 'FontSize', 8, 'Color', 'k', 'LabelSpacing', 300);

colormap(flipud(parula)); % Nông màu sáng, sâu màu xanh đậm
cb2 = colorbar;
ylabel(cb2, 'Độ sâu lòng hồ (m)', 'FontSize', 11, 'FontWeight', 'bold');
caxis([5, 95]);

xlabel('Trục East Y_{NED} (m)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Trục North X_{NED} (m)', 'FontSize', 11, 'FontWeight', 'bold');
title('BÌNH ĐỒ ĐƯỜNG ĐẲNG SÂU ĐÁY HỒ HÒA BÌNH (HỆ QUY CHIẾU NED)', ...
    'FontSize', 13, 'FontWeight', 'bold');
axis equal;
grid on;
box on;

fprintf('>>> Đã vẽ xong bản đồ địa hình đáy hồ Hòa Bình chuẩn hệ NED 100%%!\n');
fprintf('>>> Biến cấu trúc p.env đã sẵn sàng cho mô phỏng UUV và DVL.\n');
