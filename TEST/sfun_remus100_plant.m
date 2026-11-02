function sfun_remus100_plant(block)
%SFUN_REMUS100_PLANT Level-2 MATLAB S-Function mô hình hóa thủy động lực học 6-DOF REMUS 100
%   Gọi trực tiếp hàm Fossen: remus100.m
%   Quản lý 12 biến trạng thái liên tục: x = [u; v; w; p; q; r; x; y; z; phi; theta; psi]

setup(block);
end

function setup(block)
% Param 1: x0 (12x1) - trạng thái ban đầu
% Param 2: wc (1x1) - tốc độ dòng chảy thẳng đứng (m/s)
block.NumDialogPrms = 2;

% Khai báo số cổng vào (Inputs)
block.NumInputPorts = 3;
% Inport 1: ui [delta_r; delta_s; n_p] (3x1)
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(1).SamplingMode      = 'Sample';

% Inport 2: Vc (1x1) - tốc độ dòng chảy ngang (m/s)
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DirectFeedthrough = true;
block.InputPort(2).SamplingMode      = 'Sample';

% Inport 3: betaVc (1x1) - hướng dòng chảy ngang (rad)
block.InputPort(3).Dimensions        = 1;
block.InputPort(3).DirectFeedthrough = true;
block.InputPort(3).SamplingMode      = 'Sample';

% Khai báo số cổng ra (Outputs)
block.NumOutputPorts = 3;
% Outport 1: x_phys (12x1: [u; v; w; p; q; r; x; y; z; phi; theta; psi])
block.OutputPort(1).Dimensions       = 12;
block.OutputPort(1).SamplingMode     = 'Sample';

% Outport 2: xdot (12x1: đạo hàm trạng thái vật lý)
block.OutputPort(2).Dimensions       = 12;
block.OutputPort(2).SamplingMode     = 'Sample';

% Outport 3: U (1x1: tốc độ tương đối)
block.OutputPort(3).Dimensions       = 1;
block.OutputPort(3).SamplingMode     = 'Sample';

% Khai báo 12 trạng thái liên tục
block.NumContStates = 12;
block.SampleTimes = [0 0]; % Liên tục

block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Derivatives',          @Derivatives);
end

function InitializeConditions(block)
x0 = block.DialogPrm(1).Data;
block.ContStates.Data = x0(:);
end

function Outputs(block)
x = block.ContStates.Data;
ui     = block.InputPort(1).Data;
Vc     = block.InputPort(2).Data;
betaVc = block.InputPort(3).Data;
wc     = block.DialogPrm(2).Data;

[xdot, U] = remus100(x, ui, Vc, betaVc, wc);

block.OutputPort(1).Data = x;
block.OutputPort(2).Data = xdot;
block.OutputPort(3).Data = U;
end

function Derivatives(block)
x = block.ContStates.Data;
ui     = block.InputPort(1).Data;
Vc     = block.InputPort(2).Data;
betaVc = block.InputPort(3).Data;
wc     = block.DialogPrm(2).Data;

[xdot, ~] = remus100(x, ui, Vc, betaVc, wc);
block.Derivatives.Data = xdot;
end
