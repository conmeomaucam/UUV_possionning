function z_d = dvl_do_khung_than(v_n, q, k, phi_bs_rad)
%DVL_DO_KHUNG_THAN  Hàm đo DVL A125, ra khung thân đầu đo.
%
% Nguyên lý DVL:
%   Bốn tia Janus chiếu xuống đáy, mỗi tia lệch 22.5 độ so với trục z.
%   Mỗi tia đo dịch tần Doppler của tiếng vọng đáy, ra vận tốc dọc tia.
%   Firmware A125 giải bốn tia thành vận tốc ba trục. Máy không xuất
%   vận tốc từng tia. PD4 và PD6 chỉ có v_x, v_y, v_z ở khung thân DVL:
%   x về mũi (phía đèn LED), y mạn phải, z xuống đáy.
%
% Tại sao phải xây hàm đo:
%   Bộ lọc không cộng z vào INS. Bộ lọc tính phần dư y = z - h(x).
%   h(x) trả lời: với trạng thái đang giữ, DVL đáng lẽ phải báo bao nhiêu.
%   z và h(x) phải cùng khung. z nằm ở khung thân DVL, nên h(x) cũng ra
%   khung đo. Không viết hàm beam vì A125 không có đo vận tốc tia.
%
% Mục tiêu đầu ra:
%   Ba số, đơn vị m/s, khung thân DVL, cùng thứ tự với PD4.
%   Dương v_x: trượt về phía LED. Dương v_z: đi xuống.
%
% Đầu vào:
%   v_n          vận tốc navigation, m/s, trục Bắc-Đông-Xuống
%   q            quaternion INS, [w x y z], đưa navigation sang khung IMU
%   k            ước lượng hệ số tỉ lệ, không phải giá trị thật. Lúc đầu = 0
%   phi_bs_rad   ước lượng boresight [rx; ry; rz], rad, khung IMU sang khung DVL
%
%   z_d = (1+k) * C(phi_bs) * C_n^b(q) * v_n

    Cnb = quat_sang_Cnb(q);
    v_b = Cnb * v_n(:);
    Cbs = eye(3) - skew(phi_bs_rad(:));
    z_d = (1 + k) * Cbs * v_b;
end

function C = quat_sang_Cnb(q)
% C_n^b. q = [w x y z].
    q = q(:) / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y); ...
         2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x); ...
         2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)];
end

function S = skew(v)
    S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

