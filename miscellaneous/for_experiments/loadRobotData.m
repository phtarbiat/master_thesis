function expData = loadRobotData(tSample, fileDir, baseFileName, fileEnding)
    %% Load robot measurement data from binary files
    %
    % INPUTS:
    %  * See arguments block below.
    %
    % OUTPUTS:
    %  * expData:  Struct which has the data of the logged variables as
    %              fields.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        tSample         (1,1) double % Sample time of the logged data (s)
        fileDir         (:,1) char   % Folder path to the data files (with '/' at the end)
        baseFileName    (:,1) char   % File name up to the counter digits
        fileEnding      (:,1) char   % File Ending after the counter digits
    end


    %% Read binary data from files

    % Nr. of logged variables, must correspond to the nr. of individual 
    % files and the assignment below
    nVars = 21;

    % Determine data length from file size to preallocate memory
    dataLengths = zeros(nVars, 1);
    for iVar = 1:nVars

        fileName = sprintf('%s%s%d%s', fileDir, baseFileName, iVar-1, fileEnding);

        % Make sure file exists
        if ~isfile(fileName)
            error('File does not exist. Check file name and path');
        end

        % Get file info:
        % https://de.mathworks.com/matlabcentral/answers/92245-how-do-i-find-the-size-of-a-file-on-disk-from-matlab
        fileInfo = dir(fileName);

        % Get data length, assuming 32-Bit (= 4 bytes) Single values
        dataLengths(iVar) = fileInfo.bytes / 4;
    end

    % Check if all files have the same length
    if ~all(dataLengths == dataLengths(1))
        warning('Log files have different lengths');
    end

    % Read individual files
    inputData = zeros(max(dataLengths), nVars);
    for iVar = 1:nVars

        fileName = sprintf('%s%s%d%s', fileDir, baseFileName, iVar-1, fileEnding);

        % Read data
        fileID = fopen(fileName, 'r');
        inputData(:, iVar) = fread(fileID, 'single');
        fclose(fileID);
    end

    % Get length of imported data
    dataLength = length(inputData);
    fprintf('Nr. of data samples: %d\n', dataLength)


    %% Store values in struct
    % NOTE: The assignment here must correspond to the assignment done in
    % the PLC logging program!

    expData = struct;

    % Get time vector from sample time
    tEnd =  dataLength * tSample;

    expData.t = linspace(0, tEnd, dataLength)';

    % 0-2: Position motor-sided
    expData.thetaMot = inputData(:, 1:3) / 1000;

    % 3-5: Position load-sided
    expData.thetaLoad = inputData(:, 4:6) / 1000;

    % 6-8: Velocity
    expData.velocity = inputData(:, 7:9) / 1000;

    % 9-11: Torque
    expData.torque = inputData(:, 10:12);

    % 12-14: Reference Value Position
    expData.refValuePos = inputData(:, 13:15) / 1000;

    % 15-17: Reference Value Torque
    expData.refValueTorque = inputData(:, 16:18);

    % 18-20: Acceleration Sensor Data
    expData.temperature = inputData(:, 19:21);

end

