function conStruct = loadRobotConstraints()
% Load workspace and dynamic constraints of the robot

conStruct.theta_max = [156; 83; 92];
conStruct.omega_max = [100; 100; 100];
conStruct.domega_max = [500; 500; 500];

conStruct.tcp_zmin = -500 * 1e-3;