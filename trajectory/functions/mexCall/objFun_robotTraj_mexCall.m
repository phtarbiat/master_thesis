%% Script for defining the inputs of the objective function of the optimal trajectory of the base parameters
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);

ROBOT_STRUCTURE = 1;
NUM_FOURIER_TERMS = 5;
TIME_PERIOD = 10;
SAMPLE_FREQ = 500;

paramStruct = loadRobotKinematic(ROBOT_STRUCTURE);
ab = zeros(1,2*NUM_FOURIER_TERMS*3);
timeVec = zeros(SAMPLE_FREQ*TIME_PERIOD,1);

objFunValue = objFun_robotTraj(paramStruct,ab,timeVec);