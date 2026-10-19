%% esekf19_sanity_tests.m
% Kiểm tra xem khối H_dvl (scale + boresight) của esekf19_step có "bịp" không.
%   TEST 1: H giải tích vs đạo hàm số (H có đúng là Jacobian của h không?)
%   TEST 2: Hình học quan sát (dθ và phi_bs có phân biệt được không? khi nào?)
%   TEST 3: Hàm NEES cho Monte Carlo (P có trung thực với sai số thật không?)
clear; clc; rng(1);

%% ---------------------------------------------------------------- TEST 1
L   = [0.3; 0.1; -0.2];                 % lever arm
q   = randn(4,1); q = q/norm(q);        % tư thế bất kỳ
v   = [1.5; 0.4; 0.1];                  % vận tốc NED
wb  = [0.02; -0.01; 0.05];              % tốc độ góc thân (đã trừ bias)
s0  = 0.01;  phi0 = [0.01; -0.02; 0.03];

nom = struct('q',q,'v',v,'wb',wb,'s',s0,'phi',phi0,'L',L);

% --- H giải tích: COPY Y NGUYÊN các dòng 220-224 của esekf19_step.m ---
Cbn = quat_to_cbn(q); Cnb = Cbn';
Cbs = eye(3) - skew_mat(phi0);
vb  = Cnb*v;
vd  = vb + cross(wb, L);
Ha = zeros(3,19);
Ha(:,4:6)   = (1+s0)*Cbs*Cnb;
Ha(:,7:9)   = (1+s0)*Cbs*skew_mat(vb);
Ha(:,13:15) = (1+s0)*Cbs*skew_mat(L);
Ha(:,16)    = Cbs*vd;
Ha(:,17:19) = (1+s0)*skew_mat(vd);

% --- H số: đạo hàm trung tâm của h(x ⊕ dx) theo từng thành phần sai số ---
h  = @(dx) dvl_h(nom, dx);
Hn = zeros(3,19);  e = 1e-6;
for j = 1:19
    d = zeros(19,1); d(j) = e;
    Hn(:,j) = (h(d) - h(-d))/(2*e);
end

names = {'dp',1:3; 'dv',4:6; 'dtheta',7:9; 'dba',10:12; ...
         'dbg',13:15; 'ds',16; 'dphi',17:19};
fprintf('=== TEST 1: |H_giai_tich - H_so| theo khoi ===\n');
for k = 1:size(names,1)
    idx = names{k,2};
    fprintf('  %-7s max|diff| = %.2e   (|H| = %.2e)\n', names{k,1}, ...
        max(abs(Ha(:,idx)-Hn(:,idx)),[],'all'), max(abs(Hn(:,idx)),[],'all'));
end
fprintf('  -> khối nào diff >~1e-6 là sai dấu/sai quy ước.\n\n');

%% ---------------------------------------------------------------- TEST 2
fprintf('=== TEST 2: dtheta va phi_bs co tach duoc khoi nhau khong? ===\n');
scen = {'Di thang doc truc x (v=[1 0 0])', [1; 0; 0];
        'Di thang nhung xien (v=[1 .3 .1])', [1; 0.3; 0.1];
        'Chi tien-lui/ngang luan phien (2 vector)', []};
for c = 1:2
    vbody = scen{c,2};
    Hth  = (1+s0)*skew_mat(vbody);       % ∂h/∂dtheta  (Cbs ~ I)
    Hphi = (1+s0)*skew_mat(vbody);       % ∂h/∂dphi
    fprintf('%s\n', scen{c,1});
    fprintf('  sv(H_phi)        = %s\n', mat2str(svd(Hphi)',3));
    fprintf('  rank([Hth Hphi]) = %d  (can 6 de tach du dtheta va phi)\n', ...
        rank([Hth Hphi], 1e-9));
end
% Nhieu epoch voi cac huong van toc khac nhau (chi tu DVL)
Hstack = [];
dirs = [1 0 0; 0 1 0; 0 0 1; 1 1 0; 0 1 1]';
for k = 1:size(dirs,2)
    vk = dirs(:,k);
    Hstack = [Hstack; (1+s0)*skew_mat(vk), (1+s0)*skew_mat(vk)]; %#ok<AGROW>
end
fprintf('Nhieu huong van toc, CHI DVL: rank([Hth Hphi]) = %d / 6\n', ...
    rank(Hstack,1e-9));
fprintf('  -> Neu rank < 6: dtheta va phi_bs la ALIAS, chi tach duoc nho\n');
fprintf('     nguon khac (GNSS yaw, rang buoc trong luc qua F(4:6,7:9)).\n\n');

%% ---------------------------------------------------------------- TEST 3
fprintf('=== TEST 3: NEES la ham nees_mc(E,P), xem cuoi file ===\n');
fprintf('  E(k,T,nmc) = x_true - x_hat, P(k,k,T,nmc) = hiep phuong sai tu bo loc.\n');

%% ============================ HÀM CỤC BỘ ===============================
function z = dvl_h(nom, dx)
% h(x ⊕ dx): áp sai số vào trạng thái danh định đúng như inject_error
v   = nom.v + dx(4:6);
dq  = [1; 0.5*dx(7:9)]; dq = dq/norm(dq);
q   = quat_multiply(nom.q, dq);
Cnb = quat_to_cbn(q)';
wb  = nom.wb - dx(13:15);                 % true bias = est + d  ->  omega = meas - bias - d
s   = nom.s + dx(16);
phi = nom.phi + dx(17:19);
vd  = Cnb*v + cross(wb, nom.L);
z   = (1+s)*(eye(3) - skew_mat(phi))*vd;
end

function [nees_avg, nees_each] = nees_mc(E, P)
% NEES trung bình theo thời gian; kỳ vọng = k (số chiều) nếu bộ lọc nhất quán.
[k, T, nmc] = size(E);
nees_each = zeros(T, nmc);
for r = 1:nmc
    for t = 1:T
        e = E(:,t,r);
        nees_each(t,r) = e' * (P(:,:,t,r) \ e);
    end
end
nees_avg = mean(nees_each, 2);
fprintf('NEES trung binh cuoi = %.2f (ky vong ~ %d). >> k: qua tu tin (bip).\n', ...
    nees_avg(end), k);
end

function Cbn = quat_to_cbn(quaternion)
quaternion = quaternion/norm(quaternion);
w = quaternion(1); x = quaternion(2); y = quaternion(3); z = quaternion(4);
Cbn = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y); ...
       2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x); ...
       2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function S = skew_mat(vector)
S = [0, -vector(3), vector(2); ...
     vector(3), 0, -vector(1); ...
     -vector(2), vector(1), 0];
end

function quaternion = quat_multiply(first, second)
quaternion = [first(1)*second(1)-first(2)*second(2)-first(3)*second(3)-first(4)*second(4); ...
    first(1)*second(2)+first(2)*second(1)+first(3)*second(4)-first(4)*second(3); ...
    first(1)*second(3)-first(2)*second(4)+first(3)*second(1)+first(4)*second(2); ...
    first(1)*second(4)+first(2)*second(3)-first(3)*second(2)+first(4)*second(1)];
end