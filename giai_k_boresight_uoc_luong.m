% giai_k_boresight_uoc_luong.m
% Tìm k và boresight bằng bình phương tối thiểu: so DVL với vận tốc INS ở
% khung thân, trong pha chạy thẳng đều. Giá trị thật KHÔNG dùng để giải,
% chỉ dùng ở cuối để chấm điểm và giải thích sai số.

mo_phong_sinh_du_lieu_full;              % luôn sinh lại dữ liệu

%% 1. Vận tốc INS ở khung thân, tại các thời điểm DVL
ndvl = numel(t_dvl);
v_ins_b = zeros(ndvl, 3);
for j = 1:ndvl
    i = idx_dvl(j);
    v_ins_b(j, :) = (Cnb_tu_quat(Q_ins(i, :)') * Vn_ins(i, :)')';
end

%% 2. Bình phương tối thiểu cho [k; Beta; Gamma], Alpha = 0
% Mô hình tuyến tính:  z - v = k*v + Beta*[-v_z; 0; v_x] + Gamma*[v_y; -v_x; 0]
m2 = find(pha_dvl == 2);
n2 = numel(m2);

% Cách A: giả định v_y = v_z thật = 0, chỉ lấy v_x của INS (giống cách cũ)
xA = giai_bpttt(Z_dvl(m2, :), [v_ins_b(m2, 1), zeros(n2, 2)]);
% Cách B: lấy cả v_x, v_y, v_z của INS làm mốc
xB = giai_bpttt(Z_dvl(m2, :), v_ins_b(m2, :));

%% 3. Chấm điểm (dùng giá trị thật)
e_b  = v_ins_b - v_b_that_dvl;           % sai số vận tốc INS, khung thân
e_tb = mean(e_b(m2, :), 1);
V0   = p.kich_ban.van_toc_mps;
du_B = [-e_tb(1)/V0*100; -e_tb(3)/V0*180/pi; e_tb(2)/V0*180/pi];

bs_deg = p.dvl.boresight_that_deg;
that = [p.si.k_that*100; bs_deg(2); bs_deg(3)];
A_in = [xA(1)*100; xA(2)*180/pi; xA(3)*180/pi];
B_in = [xB(1)*100; xB(2)*180/pi; xB(3)*180/pi];
ten  = {'k (%)      ', 'Beta (deg) ', 'Gamma (deg)'};

fprintf('\n=========== KẾT QUẢ (pha chạy thẳng, %d mẫu DVL) ===========\n', n2);
fprintf('               Thật    Cách A    Cách B   B-Thật  | B-Thật dự đoán từ lỗi INS\n');
for r = 1:3
    fprintf('%s %8.4f  %8.4f  %8.4f  %8.4f | %8.4f\n', ten{r}, that(r), A_in(r), B_in(r), ...
            B_in(r) - that(r), du_B(r));
end
fprintf('Alpha (deg)  %8.4f  không quan sát được: quỹ đạo không có v_y, v_z thật\n', bs_deg(1));
fprintf('\nSai số vận tốc INS khung thân, trung bình pha thẳng: [%.4f %.4f %.4f] m/s\n', e_tb);
fprintf('Cách A: giả định tàu không trượt ngang, không chìm nổi so với đáy.\n');
fprintf('Cách B: dùng v_y, v_z của INS. Sai số của B ~ sai số vận tốc INS / %.1f m/s.\n', V0);

%% ==================== Hàm cục bộ ====================
function x = giai_bpttt(Z, Vr)
% Bình phương tối thiểu cho [k; Beta; Gamma] từ đo DVL Z và vận tốc mốc Vr.
    n = size(Z, 1);
    A = zeros(3*n, 3);
    b = zeros(3*n, 1);
    for j = 1:n
        v = Vr(j, :)';
        r = 3*(j-1) + (1:3);
        A(r, :) = [v, [-v(3); 0; v(1)], [v(2); -v(1); 0]];
        b(r) = Z(j, :)' - v;
    end
    x = A \ b;
    x(2:3) = x(2:3) / (1 + x(1));        % bỏ tích k*góc ở bậc hai
end

function C = Cnb_tu_quat(q)
% C_n^b từ q = [w x y z].
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y); ...
         2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x); ...
         2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)];
end
