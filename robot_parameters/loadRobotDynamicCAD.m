function paramStruct = loadRobotDynamicCAD(structure,kinStruct,flexible_model)
% Load base parameters of the robots CAD model

k =  [31000; 16000; 4700]; % joint stiffness

J_m = [2126; 800; 236] .* 1e-6 ./ 10; % motor inertia

r_P1S1 = [-0.7; 6.9; -14.4] .* 1e-3; % center of mass body 1
J_1 = [13848; 71; -214; 12473; 23; 9360] .* 1e-6; % inertia body 1


if structure == 0

    m = [4.4; 4.9; 2.8]; % mass

    r_P2S2 = [401; 0.1; 6.7] .* 1e-3; % center of mass body 2
    r_P3S3 = [283.7; 0; 11] .* 1e-3; % center of mass body 3

    J_2 = [5241; -192; -5402; 1060551; 6; 1060635] .* 1e-6; % inertia body 2
    J_3 = [2457; -19; -6630; 341627; 0; 342002] .* 1e-6; % inertia body 3

elseif structure == 1

    m = [4.4; 4.5; 2.5]; % mass
    
    r_P2S2 = [217.8; 0.1; 7.3] .* 1e-3; % center of mass body 2
    r_P3S3 = [144.4; 0; 11.8] .* 1e-3; % center of mass body 3

    J_2 = [5166; -95; -3065; 267348; 6; 267433] .* 1e-6; % inertia body 2
    J_3 = [2415; -9; -3278; 65261; 0; 65643] .* 1e-6; % inertia body 3

else
    error('chosen structure does not exist')
end

% Add motor inertia to body inertia for rigid body model
if ~flexible_model
    J_1(6) = J_1(6) + (kinStruct.i_g(1)^2)*J_m(1);
    J_2(6) = J_2(6) + (kinStruct.i_g(2)^2)*J_m(2);
    J_3(6) = J_3(6) + (kinStruct.i_g(3)^2)*J_m(3);
end


%% Calculate base parameters of general dynamic parameters
paramStruct = struct;

paramStruct.baseParam = [ ...
    J_1(6) + J_2(4) + J_3(4) + m(3)*kinStruct.r_P2P3(1)^2; ...
    J_m(1); ...
    J_2(1) - J_2(4) - m(3)*kinStruct.r_P2P3(1)^2; ...
    J_2(2); ...
    J_2(3) - r_P3S3(3)*kinStruct.r_P2P3(1); ...
    J_2(5); ...
    J_2(6) + m(3)*kinStruct.r_P2P3(1)^2; ...
    m(2) * r_P2S2(1); ...
    m(2) * r_P2S2(2); ...
    J_m(2); ...
    J_3(1) - J_3(4); ...
    J_3(2); ...
    J_3(3); ...
    J_3(5); ...
    J_3(6); ...
    m(3) * r_P3S3(1); ...
    m(3) * r_P3S3(2); ...
    J_m(3) ...
    ];

paramStruct.k = k;