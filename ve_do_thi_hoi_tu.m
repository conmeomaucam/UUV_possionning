% ve_do_thi_hoi_tu.m
% Script vẽ đồ thị quá trình hội tụ ước lượng k, Beta, Gamma theo thời gian.

giai_k_boresight_uoc_luong;

% 1. Tính toán ước lượng trực tuyến (Recursive / Cumulative Estimation) theo thời gian
m2 = find(pha_dvl == 2);
n2 = numel(m2);

t_pha2 = t_dvl(m2);
k_cum = zeros(n2, 1);
beta_cum = zeros(n2, 1);
gamma_cum = zeros(n2, 1);

for k_step = 1:n2
    sub_m = m2(1:k_step);
    % Tính bình phương tối thiểu tích lũy từ mẫu 1 đến mẫu k_step (Cách A)
    vx_sub = v_ins_b(sub_m, 1);
    k_cum(k_step) = (mean(Z_dvl(sub_m, 1) ./ vx_sub) - 1) * 100;         % %
    beta_cum(k_step) = mean(Z_dvl(sub_m, 3) ./ vx_sub) * (180 / pi);     % deg
    gamma_cum(k_step) = -mean(Z_dvl(sub_m, 2) ./ vx_sub) * (180 / pi);    % deg
end

% 2. Lấy giá trị thật chuẩn
k_that_pct = p.si.k_that * 100;
beta_that_deg = p.dvl.boresight_that_deg(2);
gamma_that_deg = p.dvl.boresight_that_deg(3);

% 3. Vẽ đồ thị quá trình hội tụ trong MATLAB
figure('Name', 'QUÁ TRÌNH HỘI TỤ ƯỚC LƯỢNG K VÀ BORESIGHT', 'Color', 'w', 'Position', [100 100 900 700]);

% (a) Đồ thị k (%)
subplot(3, 1, 1);
plot(t_pha2, k_cum, 'b-', 'LineWidth', 2); hold on;
yline(k_that_pct, 'r--', 'Giá trị thật (1.0%)', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
grid on;
title('Hội tụ Hệ số Tỷ lệ k (%) theo thời gian');
xlabel('Thời gian (giây)');
ylabel('k (%)');
legend('k ước lượng tích lũy', 'k thật (Ground Truth)', 'Location', 'best');

% (b) Đồ thị Pitch Boresight Beta (deg)
subplot(3, 1, 2);
plot(t_pha2, beta_cum, 'g-', 'LineWidth', 2); hold on;
yline(beta_that_deg, 'r--', 'Giá trị thật (-0.30 deg)', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
grid on;
title('Hội tụ Góc lệch Boresight Pitch \beta (deg) theo thời gian');
xlabel('Thời gian (giây)');
ylabel('Pitch \beta (độ)');
legend('\beta ước lượng tích lũy', '\beta thật (Ground Truth)', 'Location', 'best');

% (c) Đồ thị Yaw Boresight Gamma (deg)
subplot(3, 1, 3);
plot(t_pha2, gamma_cum, 'm-', 'LineWidth', 2); hold on;
yline(gamma_that_deg, 'r--', 'Giá trị thật (1.00 deg)', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
grid on;
title('Hội tụ Góc lệch Boresight Yaw \gamma (deg) theo thời gian');
xlabel('Thời gian (giây)');
ylabel('Yaw \gamma (độ)');
legend('\gamma ước lượng tích lũy', '\gamma thật (Ground Truth)', 'Location', 'best');

sgtitle('ĐỒ THỊ QUÁ TRÌNH HỘI TỤ TỰ HIỆU CHỈNH DVL / INS (Pha chạy thẳng 30 phút)', 'FontSize', 12, 'FontWeight', 'bold');
fprintf('\nĐã vẽ xong đồ thị quá trình hội tụ 30 phút trong cửa sổ hình ảnh MATLAB.\n');
