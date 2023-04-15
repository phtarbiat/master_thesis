function kinStruct = loadRobotKinematic(structure)
% Load kinematic parameters of the robot

kinStruct = struct;

kinStruct.structure = structure; % Robot structure

kinStruct.g = 9.81; % Graviational acceleration

kinStruct.i_g = [160; 160; 100]; % Gear ratio

% Kinematic parameters
if structure == 0
    kinStruct.r_P1P2 = [0; 0; 0] * 1e-3;
    kinStruct.r_P2P3 = [630; 0; 0] * 1e-3;
    kinStruct.r_P3P4 = [578; 0; 0] * 1e-3;
elseif structure == 1
    kinStruct.r_P1P2 = [0; 0; 0] * 1e-3;
    kinStruct.r_P2P3 = [330; 0; 0] * 1e-3;
    kinStruct.r_P3P4 = [278; 0; 0] * 1e-3;
else
    error('chosen structure does not exist')
end