% =========================================================================
% CHƯƠNG TRÌNH MÔ PHỎNG 3D CHUYỂN ĐỘNG TÀU NGẦM UUV VÀ CHÙM TIA DVL
% Hệ thống: Dẫn đường INS/DVL và Ước lượng Boresight (Joan Solà ES-EKF)
% =========================================================================

function mo_phong_3d_matlab()
    fprintf('=== KHỞI ĐỘNG HỆ THỐNG MÔ PHỎNG 3D UUV VÀ CHÙM TIA DVL ===\n');
    
    % Kiểm tra xem dữ liệu quỹ đạo đã có trong workspace chưa
    has_data = evalin('base', 'exist(''POS_TRUE'', ''var'') && exist(''POS_REC'', ''var'')');
    
    if ~has_data
        fprintf('Chưa tìm thấy dữ liệu quỹ đạo trong Workspace. Đang nạp/chạy kịch bản...\n');
        evalin('base', 'chay_es_ekf_boresight;');
    end
    
    % Lấy dữ liệu từ base workspace
    POS_TRUE = evalin('base', 'POS_TRUE');
    POS_REC  = evalin('base', 'POS_REC');
    yaw      = evalin('base', 'yaw');
    V        = evalin('base', 'V');
    t        = evalin('base', 't');
    S_REC    = evalin('base', 'S_REC');
    BETA_REC = evalin('base', 'BETA_REC');
    GAMMA_REC= evalin('base', 'GAMMA_REC');
    dt       = evalin('base', 'dt');
    if evalin('base', 'exist(''T_phase1'', ''var'')')
        T_start = evalin('base', 'T_phase1');
    elseif evalin('base', 'exist(''T_can_chinh'', ''var'')')
        T_start = evalin('base', 'T_can_chinh');
    else
        T_start = 60;
    end
    T_can_chinh = T_start;
    
    % Bắt đầu từ khi lặn ngầm bơi số 8
    idx_start = find(t >= T_start, 1);
    
    % Lấy mẫu thưa để chuyển động mượt mà 30-50 FPS (mỗi bước nhảy ~0.15s)
    step_skip = round(0.15 / dt); 
    anim_indices = idx_start:step_skip:length(t);
    n_frames = length(anim_indices);
    
    fprintf('Đang khởi tạo mô hình 3D UUV (Vỏ khí động học, Cánh lái, Cảm biến DVL)...\n');
    
    % ---------------------------------------------------------------------
    % KHỞI TẠO CỬA SỔ ĐỒ HỌA 3D
    % ---------------------------------------------------------------------
    hFig = figure('Name', 'MÔ PHỎNG 3D HÀNH TRÌNH UUV VÀ BẮN CHÙM TIA DVL (ES-EKF)', ...
                  'Color', [0.05 0.08 0.14], 'Position', [80 60 1350 720]);
              
    % Cửa sổ chính: Phối cảnh không gian 3D
    ax3D = subplot('Position', [0.04 0.08 0.65 0.85]);
    hold(ax3D, 'on'); grid(ax3D, 'on');
    set(ax3D, 'Color', [0.03 0.06 0.12], 'GridColor', [0.2 0.3 0.45], 'GridAlpha', 0.4);
    xlabel(ax3D, 'Trục Đông - East (m)', 'Color', [0.7 0.8 0.95], 'FontWeight', 'bold');
    ylabel(ax3D, 'Trục Bắc - North (m)', 'Color', [0.7 0.8 0.95], 'FontWeight', 'bold');
    zlabel(ax3D, 'Độ sâu - Z (m)', 'Color', [0.7 0.8 0.95], 'FontWeight', 'bold');
    axis(ax3D, 'equal');
    view(ax3D, [40 28]);
    
    % Thiết lập giới hạn không gian
    all_E = [POS_TRUE(anim_indices, 2); POS_REC(anim_indices, 2)];
    all_N = [POS_TRUE(anim_indices, 1); POS_REC(anim_indices, 1)];
    pad = 8;
    xlim(ax3D, [min(all_E)-pad, max(all_E)+pad]);
    ylim(ax3D, [min(all_N)-pad, max(all_N)+pad]);
    zlim(ax3D, [-12, 6]);
    
    % Vẽ đáy biển (Seabed Grid) tại Z = -10m
    seabed_z = -10;
    [xg, yg] = meshgrid(linspace(min(all_E)-pad, max(all_E)+pad, 15), ...
                        linspace(min(all_N)-pad, max(all_N)+pad, 15));
    zg = seabed_z * ones(size(xg));
    surf(ax3D, xg, yg, zg, 'FaceColor', [0.08 0.15 0.22], 'EdgeColor', [0.15 0.25 0.35], ...
         'FaceAlpha', 0.6, 'EdgeAlpha', 0.4);
    
    % Vẽ vệt quỹ đạo toàn phần sẵn dạng mờ
    plot3(ax3D, POS_TRUE(anim_indices, 2), POS_TRUE(anim_indices, 1), zeros(n_frames, 1), ...
          'k--', 'LineWidth', 1.2, 'Color', [0.8 0.8 0.8 0.3]);
    
    % Đối tượng đồ họa động: Quỹ đạo đã đi qua
    hTrailTrue = plot3(ax3D, nan, nan, nan, 'Color', [1.0 0.85 0.2], 'LineWidth', 2.0); % Vàng hoàng kim
    hTrailEKF  = plot3(ax3D, nan, nan, nan, 'Color', [0.1 0.7 1.0], 'LineWidth', 2.2); % Xanh Neon
    
    % Khởi tạo hình học 3D của tàu UUV
    [hull_mesh, fins_mesh, dvl_pod] = tao_hinh_hoc_uuv();
    hHull = patch(ax3D, 'Faces', hull_mesh.faces, 'Vertices', hull_mesh.verts, ...
                  'FaceColor', [0.95 0.75 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    hFins = patch(ax3D, 'Faces', fins_mesh.faces, 'Vertices', fins_mesh.verts, ...
                  'FaceColor', [0.2 0.25 0.3], 'EdgeColor', 'none');
    hDvl  = patch(ax3D, 'Faces', dvl_pod.faces, 'Vertices', dvl_pod.verts, ...
                  'FaceColor', [0.1 0.8 0.9], 'EdgeColor', 'none');
              
    % 4 Chùm tia DVL (Janus 4-beams chĩa xuống đáy biển)
    hBeams = gobjects(4, 1);
    beam_colors = [1 0.3 0.3; 0.3 1 0.3; 0.3 0.6 1; 1 1 0.3];
    for b = 1:4
        hBeams(b) = plot3(ax3D, [0 0], [0 0], [0 0], 'Color', [beam_colors(b, :), 0.75], ...
                          'LineWidth', 1.8, 'LineStyle', '-');
    end
    
    % Đèn chiếu sáng 3D
    camlight(ax3D, 'headlight');
    material(ax3D, 'shiny');
    
    % Chú thích trên góc đồ thị 3D
    title(ax3D, 'MÔ PHỎNG 3D: UUV CƠ ĐỘNG SỐ 8 & BẮN CHÙM TIA DVL TÌM ĐÁY (JANUS)', ...
          'Color', [0.4 0.85 1.0], 'FontSize', 12, 'FontWeight', 'bold');
      
    % ---------------------------------------------------------------------
    % CỬA SỔ PHỤ: CÁC ĐỒNG HỒ ĐO TRẠNG THÁI (COCKPIT HUD)
    % ---------------------------------------------------------------------
    % Subplot 1: Sai số dạt vị trí tức thời
    axErr = subplot('Position', [0.73 0.68 0.24 0.25]);
    set(axErr, 'Color', [0.03 0.06 0.12], 'XColor', [0.7 0.8 0.9], 'YColor', [0.7 0.8 0.9]);
    grid(axErr, 'on'); hold(axErr, 'on');
    hErrPlot = plot(axErr, nan, nan, 'r-', 'LineWidth', 1.8);
    title(axErr, 'Độ dạt tức thời ||p_{EKF} - p_{TRUE}|| (m)', 'Color', [1 0.4 0.4], 'FontSize', 10);
    xlabel(axErr, 'Thời gian (s)', 'Color', [0.6 0.7 0.8]); ylabel(axErr, 'Sai số (m)');
    xlim(axErr, [0, max(1, t(end) - T_start)]);
    err_all = sqrt(sum((POS_REC(anim_indices, 1:2) - POS_TRUE(anim_indices, 1:2)).^2, 2));
    ylim(axErr, [0, max(0.5, ceil(max(err_all) * 1.5 * 10) / 10)]);
    
    % Subplot 2: Hội tụ Boresight Yaw Gamma
    axGamma = subplot('Position', [0.73 0.37 0.24 0.23]);
    set(axGamma, 'Color', [0.03 0.06 0.12], 'XColor', [0.7 0.8 0.9], 'YColor', [0.7 0.8 0.9]);
    grid(axGamma, 'on'); hold(axGamma, 'on');
    yline(axGamma, 1.0, 'r--', 'LineWidth', 1.5);
    hGammaPlot = plot(axGamma, nan, nan, 'm-', 'LineWidth', 1.8);
    title(axGamma, 'Hội tụ Yaw Boresight \gamma (độ) [Đích: +1.0°]', 'Color', [0.9 0.5 1.0], 'FontSize', 10);
    xlabel(axGamma, 'Thời gian (s)'); ylabel(axGamma, '\gamma (độ)');
    xlim(axGamma, [0, max(1, t(end) - T_start)]);
    ylim(axGamma, [-0.5, 2.0]);
    
    % Subplot 3: Bảng chỉ số số liệu thời gian thực (Digital Telemetry)
    axInfo = subplot('Position', [0.73 0.08 0.24 0.22]);
    axis(axInfo, 'off');
    hTextHUD = text(axInfo, 0.05, 0.5, '', 'Color', [0.95 0.95 0.95], ...
                    'FontName', 'Courier', 'FontSize', 10, 'Interpreter', 'none');
                
    % ---------------------------------------------------------------------
    % VÒNG LẶP CHẠY MÔ PHỎNG 3D
    % ---------------------------------------------------------------------
    fprintf('Bắt đầu Animation 3D (Đang chạy)...\n');
    
    trail_E_true = []; trail_N_true = [];
    trail_E_ekf  = []; trail_N_ekf  = [];
    time_hist    = []; err_hist     = []; gamma_hist = [];
    
    % Góc Janus 25 độ của 4 chùm tia DVL trong DVL frame
    theta_janus = 25 * pi / 180;
    dvl_beams_local = [
         cos(theta_janus),  0,                sin(theta_janus);  % Chùm 1: Tiến
        -cos(theta_janus),  0,                sin(theta_janus);  % Chùm 2: Lùi
         0,                 cos(theta_janus), sin(theta_janus);  % Chùm 3: Phải
         0,                -cos(theta_janus), sin(theta_janus);  % Chùm 4: Trái
    ]';
    
    for k = 1:n_frames
        if ~ishandle(hFig), break; end % Người dùng đóng cửa sổ thì dừng
        
        idx = anim_indices(k);
        t_cur = t(idx) - T_start;
        
        % Vị trí và hướng tại bước này
        pos_true_k = POS_TRUE(idx, :); % [North, East, Down]
        pos_ekf_k  = POS_REC(idx, :);
        yaw_k      = yaw(idx);
        V_k        = V(idx);
        gamma_k    = GAMMA_REC(idx);
        err_k      = norm(pos_ekf_k(1:2) - pos_true_k(1:2));
        
        % Tích lũy vệt chuyển động
        trail_E_true(end+1) = pos_true_k(2);
        trail_N_true(end+1) = pos_true_k(1);
        trail_E_ekf(end+1)  = pos_ekf_k(2);
        trail_N_ekf(end+1)  = pos_ekf_k(1);
        
        time_hist(end+1)  = t_cur;
        err_hist(end+1)   = err_k;
        gamma_hist(end+1) = gamma_k;
        
        % Ma trận quay Body -> Nav: Tàu quay góc yaw quanh trục Z thẳng đứng
        cy = cos(yaw_k); sy = sin(yaw_k);
        R_b2n = [cy, -sy, 0; 
                 sy,  cy, 0; 
                  0,   0, 1];
        
        % Chuyển đổi đỉnh hình học 3D của vỏ tàu và cánh lái theo tọa độ EKF
        % Ghi chú: Trục x_body = Tiến (hướng theo vận tốc), y_body = Phải, z_body = Xuống
        % Đổi sang hệ vẽ 3D [East, North, Up]: East = y_n, North = x_n, Up = -z_n
        uuv_origin = [pos_ekf_k(2); pos_ekf_k(1); 0]; % Ở mặt nước z = 0
        
        V_hull_rot = (R_b2n * [hull_mesh.verts(:, 1), hull_mesh.verts(:, 2), hull_mesh.verts(:, 3)]')';
        set(hHull, 'Vertices', [V_hull_rot(:, 2) + uuv_origin(1), ...
                                V_hull_rot(:, 1) + uuv_origin(2), ...
                               -V_hull_rot(:, 3) + uuv_origin(3)]);
                            
        V_fins_rot = (R_b2n * [fins_mesh.verts(:, 1), fins_mesh.verts(:, 2), fins_mesh.verts(:, 3)]')';
        set(hFins, 'Vertices', [V_fins_rot(:, 2) + uuv_origin(1), ...
                                V_fins_rot(:, 1) + uuv_origin(2), ...
                               -V_fins_rot(:, 3) + uuv_origin(3)]);
                            
        V_dvl_rot = (R_b2n * [dvl_pod.verts(:, 1), dvl_pod.verts(:, 2), dvl_pod.verts(:, 3)]')';
        set(hDvl, 'Vertices', [V_dvl_rot(:, 2) + uuv_origin(1), ...
                               V_dvl_rot(:, 1) + uuv_origin(2), ...
                              -V_dvl_rot(:, 3) + uuv_origin(3)]);
                           
        % Cập nhật 4 chùm tia DVL bắn từ đáy bụng UUV xuống đáy biển Z = -10m
        p_dvl_center = uuv_origin + [0; 0; -0.4];
        for b = 1:4
            dir_b = dvl_beams_local(:, b);
            dir_n = R_b2n * dir_b;
            % Chiếu xuống mặt phẳng đáy biển
            scale_z = (seabed_z - p_dvl_center(3)) / (-dir_n(3));
            p_hit = p_dvl_center + [dir_n(2); dir_n(1); -dir_n(3)] * scale_z;
            set(hBeams(b), 'XData', [p_dvl_center(1), p_hit(1)], ...
                           'YData', [p_dvl_center(2), p_hit(2)], ...
                           'ZData', [p_dvl_center(3), p_hit(3)]);
        end
        
        % Cập nhật vệt quỹ đạo
        set(hTrailTrue, 'XData', trail_E_true, 'YData', trail_N_true, 'ZData', zeros(size(trail_E_true)));
        set(hTrailEKF,  'XData', trail_E_ekf,  'YData', trail_N_ekf,  'ZData', zeros(size(trail_E_ekf)));
        
        % Cập nhật các đồ thị đo phụ
        set(hErrPlot,   'XData', time_hist, 'YData', err_hist);
        set(hGammaPlot, 'XData', time_hist, 'YData', gamma_hist);
        
        % Cập nhật bảng thông số số liệu
        hud_str = sprintf([...
            '--- TELEMETRY TRỰC TUYẾN ---\n', ...
            'Thời gian: %6.1f s / 1800 s\n', ...
            'Vận tốc V: %6.2f m/s\n', ...
            'Góc Yaw  : %6.1f deg\n', ...
            '-----------------------------\n', ...
            'Độ dạt   : %6.2f m\n', ...
            'Scale s  : %+6.3f %%\n', ...
            'Yaw Gamma: %+6.3f deg\n', ...
            '-----------------------------\n', ...
            'Trạng thái: ĐANG BÁM ĐÁY (OK)' ...
        ], t_cur, V_k, yaw_k * 180 / pi, err_k, S_REC(idx), gamma_k);
        set(hTextHUD, 'String', hud_str);
        
        % Camera bám theo đuôi tàu mượt mà
        camtarget(ax3D, [pos_ekf_k(2), pos_ekf_k(1), 0]);
        
        drawnow limitrate;
    end
    fprintf('Hoàn thành phiên mô phỏng 3D!\n');
end

% =========================================================================
% HÀM DỰNG HÌNH HỌC 3D UUV (THÂN THỦY ĐỘNG HỌC, CÁNH LÁI, CỤM DVL)
% =========================================================================
function [hull, fins, dvl] = tao_hinh_hoc_uuv()
    % Chiều dài 2.4m, đường kính 0.35m
    L = 2.4; R = 0.22;
    
    % 1. Thân chính + Mũi hình bán cầu + Đuôi thuôn nhọn
    n_seg = 20; n_circ = 24;
    x_profile = linspace(-L/2, L/2, n_seg);
    r_profile = zeros(size(x_profile));
    
    for i = 1:n_seg
        xi = x_profile(i);
        if xi > L*0.25
            % Mũi elip elipsoid
            r_profile(i) = R * sqrt(max(0, 1 - ((xi - L*0.25)/(L*0.25))^2));
        elseif xi < -L*0.2
            % Đuôi côn thuôn nhọn
            r_profile(i) = R * (1 - ((-L*0.2 - xi)/(L*0.3)));
        else
            % Thân trụ tròn
            r_profile(i) = R;
        end
    end
    
    theta_c = linspace(0, 2*pi, n_circ);
    [TH, X] = meshgrid(theta_c, x_profile);
    R_mat = repmat(r_profile', 1, n_circ);
    Y = R_mat .* cos(TH);
    Z = R_mat .* sin(TH);
    
    [faces_h, verts_h] = surf2patch(X, Y, Z, 'triangles');
    hull.faces = faces_h;
    hull.verts = verts_h;
    
    % 2. Bốn cánh lái chữ thập (X-fins) ở đuôi
    fin_w = 0.35; fin_len = 0.4;
    x_tail = -L*0.35;
    
    % Đỉnh của cánh lái đứng và ngang
    verts_f = [
        x_tail,           0,       R;
        x_tail - fin_len, 0,       R + fin_w;
        x_tail - fin_len, 0,       R;
        x_tail,           0,      -R;
        x_tail - fin_len, 0,     -(R + fin_w);
        x_tail - fin_len, 0,      -R;
        x_tail,           R,       0;
        x_tail - fin_len, R+fin_w, 0;
        x_tail - fin_len, R,       0;
        x_tail,          -R,       0;
        x_tail - fin_len,-(R+fin_w), 0;
        x_tail - fin_len,-R,       0;
    ];
    faces_f = [
        1 2 3;
        4 5 6;
        7 8 9;
        10 11 12
    ];
    fins.verts = verts_f;
    fins.faces = faces_f;
    
    % 3. Cụm cảm biến DVL gắn ở bụng
    r_dvl = 0.10; h_dvl = 0.08;
    [xd, yd, zd] = cylinder(r_dvl, 12);
    xd = xd * h_dvl + 0.3; % Tọa độ x_body
    yd = yd;
    zd = zd * h_dvl + R;   % Nhô ra ở bụng dưới
    [faces_d, verts_d] = surf2patch(xd, yd, zd, 'triangles');
    dvl.faces = faces_d;
    dvl.verts = verts_d;
end
