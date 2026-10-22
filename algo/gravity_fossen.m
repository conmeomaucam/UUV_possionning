function [g, g_ned, dg_dz] = gravity_fossen(latitude, depth, is_degree)
%GRAVITY_FOSSEN Tinh toan gia toc trong truong g chuan theo mo hinh WGS-84 Somigliana
%   va phuong phap cua Giao su Thor I. Fossen (MSS Toolbox, NTNU).
%
%   Cu phap:
%       g = gravity_fossen(latitude)
%       [g, g_ned] = gravity_fossen(latitude, depth)
%       [g, g_ned, dg_dz] = gravity_fossen(latitude, depth, is_degree)
%
%   Dau vao:
%       latitude   - Vi do dia ly cua AUV. Don vi mac dinh: radian.
%                    Neu is_degree = true thi don vi la do (degree).
%       depth      - Do sau lan cua AUV tinh theo truc Down he NED (m).
%                    Gia tri duong khi lan duoi mat nuoc, mac dinh: 0 m.
%       is_degree  - Co logic quy dinh don vi vi do (tuy chon, mac dinh: false).
%                    true: vi do tinh bang do [deg].
%                    false: vi do tinh bang radian [rad].
%
%   Dau ra:
%       g     - Do lon gia toc trong truong tai vi tri lan (m/s^2).
%       g_ned - Vector gia toc trong truong bieu dien trong he toa do NED (3x1):
%               g_ned = [0; 0; g] (m/s^2), chieu thang dung doc truc Down.
%       dg_dz - Gradient bien thien trong luc theo do sau d_g / d_z (1/s^2),
%               dung cho ma tran Jacobi F trong bo loc Kalman.
%
%   Co so ly thuyet:
%   1. Cong thuc WGS-84 Somigliana (1929) tren mat elipsoid:
%      g_0 = g_e * (1 + k * sin(mu)^2) / sqrt(1 - e2 * sin(mu)^2)
%      Trong do:
%        g_e = 9.7803253359 m/s^2  (Trong luc tieu chuan tai Xich dao)
%        k   = 0.001931850400      (Hang so cong thuc Somigliana)
%        e2  = 0.006694384442      (Do lech tam thu nhat cua elipsoid WGS-84)
%        mu  = Vi do dia ly tinh bang radian.
%
%   2. Hieu chinh do sau lan (Subsurface / Underwater depth correction):
%      Khi AUV lan xuong do sau z (Down > 0), khoang cach den tam Trai Dat giam
%      ket hop voi hieu ung lop nuoc ben tren, gradient thuc te dat khoang:
%      dg/dz = +2.22e-6 (m/s^2)/m.
%      g(mu, z) = g_0 + dg/dz * z.
%
%   Tham khao:
%   - Thor I. Fossen (2021). "Handbook of Marine Craft Hydrodynamics and Motion Control", 2nd Edition.
%   - MSS Toolbox: AUV_simulation/MSS/INS/functions/gravity.m

% 1. Xu ly tham so dau vao mac dinh
if nargin < 2 || isempty(depth)
    depth = 0;
end

if nargin < 3 || isempty(is_degree)
    is_degree = false;
end

% Chuyen doi sang radian neu dau vao la do
if is_degree
    mu = latitude * (pi / 180);
else
    mu = latitude;
end

% 2. Cac hang so vat ly elipsoid WGS-84 (theo Thor I. Fossen)
g_e = 9.7803253359;      % Trong luc tai Xich dao (m/s^2)
k   = 0.001931850400;    % Hang so Somigliana
e2  = 0.006694384442;    % Do lech tam binh phuong (first eccentricity squared)

% 3. Tinh do lon trong luc tren mat nuoc (mat bien chuan / elipsoid WGS-84)
sin_mu2 = sin(mu)^2;
g_0 = g_e * (1 + k * sin_mu2) / sqrt(1 - e2 * sin_mu2);

% 4. Hieu chinh do sau lan (Underwater Depth Correction)
% Gradient trong luc theo chieu sau duoi nuoc (tinh den mat do nuoc rho ~ 1000 kg/m^3)
% He so gradient hieu dung dg/dz xap xi: 2.22e-6 s^-2
dg_dz = 2.22e-6;

% Trong luc thuc te tai do sau lan depth (m)
g = g_0 + dg_dz * depth;

% 5. Vector trong luc trong he quy chieu NED (North - East - Down)
% Huong cua trong luc luon vuong goc xuong tam Trai Dat, chi chieu len truc Down
g_ned = [0; 0; g];

end
