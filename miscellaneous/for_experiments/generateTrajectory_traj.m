%% Sample script to generate trajectory data with derivatives for the lab robot
%
% Generates a sample trajectory for the three robot axes and saves it as
% a single binary file that can be transferred to the PLC and read as a
% trajectory.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

% Preparation
clear
close all


%% Script settings

% File name of the generated trajectory
% Corresponds to TrajReaderParams.FileName in the PLC reading program
trajFileName = 'Trajectory.bin';

% File path, where the files will be saved (with '/' at the end)
fileDir = 'for_experiments/generatedTrajData/';

inputFile = 'identification_trajectories/traj/validation/traj_validation.mat';
inputTraj = load(inputFile);

% Sample time, at which the trajectory will be read
% ( i.e., task class cycle time of the PLC rading program)
tSample = inputTraj.time(2) - inputTraj.time(1);
tVec = inputTraj.time;

%% Save as binary data
% Save all trajectories in single file

x = inputTraj.theta;
x_dot = inputTraj.omega;
x_ddot = inputTraj.domega;

xSave = [x, x_dot, x_ddot];

fprintf('Overall array length: %d\n', length(xSave(:)));

% Test to generate invalid trajectory file
% xSave = xSave(:);
% xSave = xSave(1:end-3);

% Create output folder if it doesn't exist
if ~isfolder(fileDir) && ~isempty(fileDir)
    mkdir(fileDir);
end

% Save data as 32-Bit Single binary file
fileName = sprintf('%s/%s', fileDir, trajFileName);
fileID = fopen(fileName, 'w');
fwrite(fileID, xSave, 'single');
fclose(fileID);


%% Plot generated trajectories

figure;

subplot(3,1,1);
plot(tVec, x);
legend;
grid on;
title('Generated trajectory', 'Interpreter', 'latex')
xlabel('Time / s', 'Interpreter', 'latex')
ylabel('Data', 'Interpreter', 'latex')

subplot(3,1,2);
plot(tVec, x_dot);
legend;
grid on;
title('First derivative', 'Interpreter', 'latex')
xlabel('Time / s', 'Interpreter', 'latex')
ylabel('Data', 'Interpreter', 'latex')

subplot(3,1,3);
plot(tVec, x_ddot);
legend;
grid on;
title('Second derivative', 'Interpreter', 'latex')
xlabel('Time / s', 'Interpreter', 'latex')
ylabel('Data', 'Interpreter', 'latex')


%% Read back binary data for verification

fileName = sprintf('%s/%s', fileDir, trajFileName);


%%% Get length of trajectory array

% Get size of binary file
% See https://de.mathworks.com/matlabcentral/answers/92245-how-do-i-find-the-size-of-a-file-on-disk-from-matlab
s = dir(fileName);
fileSize = s.bytes;

fprintf('Size of generated file: %d bytes\n', fileSize)

% Number of individual trajectories in the file / columns of the matrix
nTrajs = 3*3;

% Get length of trajectory array, assuming 32 bit / 4 byte single numbers
trajLength = fileSize / nTrajs / 4;


%%% Read in data
fileID = fopen(fileName, 'r');
xBin = fread(fileID, [trajLength, nTrajs], '*single');
fclose(fileID);


%% Plot verification data
figure;

subplot(3,1,1);
plot(tVec, xBin(:, 1:3));
legend;
grid on;
title('Loaded trajectory', 'Interpreter', 'latex')
xlabel('Time / s', 'Interpreter', 'latex')
ylabel('Data', 'Interpreter', 'latex')

subplot(3,1,2);
plot(tVec, xBin(:, 4:6));
legend;
grid on;
title('First derivative', 'Interpreter', 'latex')
xlabel('Time / s', 'Interpreter', 'latex')
ylabel('Data', 'Interpreter', 'latex')

subplot(3,1,3);
plot(tVec, xBin(:, 7:end));
legend;
grid on;
title('Second derivative', 'Interpreter', 'latex')
xlabel('Time / s', 'Interpreter', 'latex')
ylabel('Data', 'Interpreter', 'latex')