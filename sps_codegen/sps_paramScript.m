%% Creation of a C++ function that loads hard-coded model parameters for a given parameter struct
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Clear Workspace and add necessary pathes
clear;
addpath(pathdef_local);


% Settings
FILE_NAME = 'writeParamStructFJR.h';            % Name of C++ function file
NAME_STRUCT = 'gParamStruct';                   % Name of parameter struct in function
NUMBER_DECIMAL_DIGITS = 15;                     % Number of considered decimal values of hard-coded model parameters
PATH_PARAM_STRUCT = 'identification/models';    % Folder of parameter struct
OUTPUT_FOLDER = 'sps_codegen';                  % Output folder

% Get current date
date = datetime('today');


% Create .h file
fid = fopen([OUTPUT_FOLDER '/' FILE_NAME],'w');

fstring = [ ...
    '/*   LOAD MODEL PARAM', newline, ...
    '   Initialize robot model parameters', newline, newline, ...
    '   Last time generated: ', num2str(year(date)), '/', num2str(month(date)), newline, ...
    '*/', ...
    newline, newline ...
    ];
fprintf(fid,fstring);

%% Write parameter struct into INIT function
% Load parameter struct
paramStruct = load([PATH_PARAM_STRUCT '/model_complete.mat']);
kinStruct = loadRobotKinematic(1);
cadStruct = loadRobotDynamicCAD(1,kinStruct,0);
paramStruct.robot.baseParam = cadStruct.baseParam;

% Get struct field names
fieldNames = fieldnames(paramStruct);

% Write function start
fprintf(fid,['void writeParamStructFJR(struct0_T *gParamStruct){', newline, newline]);


%% Initialize parameter struct
fprintf(fid,['// Initialize parameter struct', newline, newline]);

% Iterate through field names
for iField = 1:length(fieldNames)

    % Get current field name
    curField = fieldNames{iField};

    % Write header line for struct field
    fprintf(fid,['// Parameter of struct field ' curField, newline]);

    % Get element names of field
    elementNames = fieldnames(paramStruct.(curField));

    % Iterate through field elements
    for iElement = 1:length(elementNames)

        % Get current elements name
        curElement = elementNames{iElement};

        % Write header line for element
        fprintf(fid,['// ' curElement, newline]);

        % Get element size
        sizeElement = size(paramStruct.(curField).(curElement));

        if sizeElement(1) == 1 && sizeElement(2) == 1

            % Create string and write it to file
            fstring = sprintf([ ...
                NAME_STRUCT, '->', ...%NAME_STRUCT, '.', ...
                curField, '.', ...
                curElement, ...
                ' = %.', num2str(NUMBER_DECIMAL_DIGITS), 'f;', ...
                newline
                ],paramStruct.(curField).(curElement));

            fprintf(fid,fstring);

        elseif sizeElement(1) == 1

            % Iterate through element entries
            for iCol = 1:sizeElement(2)

                % Create string and write it to file
                fstring = sprintf([ ...
                    NAME_STRUCT, '->', ...%NAME_STRUCT, '.', ...
                    curField, '.', ...
                    curElement, ...
                    '[' num2str(iCol-1) ']', ...
                    ' = %.', num2str(NUMBER_DECIMAL_DIGITS), 'f;', ...
                    newline
                    ],paramStruct.(curField).(curElement)(iCol));

                fprintf(fid,fstring);

            end

        elseif sizeElement(2) == 1

            % Iterate through element entries
            for iRow = 1:sizeElement(1)

                % Create string and write it to file
                fstring = sprintf([ ...
                    NAME_STRUCT, '->', ...%NAME_STRUCT, '.', ...
                    curField, '.', ...
                    curElement, ...
                    '[' num2str(iRow-1) ']', ...
                    ' = %.', num2str(NUMBER_DECIMAL_DIGITS), 'f;', ...
                    newline
                    ],paramStruct.(curField).(curElement)(iRow));

                fprintf(fid,fstring);

            end

        else

            % Iterate through element entries
            
            for iCol = 1:sizeElement(2)
                for iRow = 1:sizeElement(1)

                    iElement = iRow + ((iCol-1)*sizeElement(1)) - 1;

                    % Create string and write it to file
                    fstring = sprintf([ ...
                        NAME_STRUCT, '->', ...%NAME_STRUCT, '.', ...
                        curField, '.', ...
                        curElement, ...
                        '[' num2str(iElement) ']', ...
                        ' = %.', num2str(NUMBER_DECIMAL_DIGITS), 'f;', ...
                        newline
                        ],paramStruct.(curField).(curElement)(iRow,iCol));
    
                    fprintf(fid,fstring);
    
                end
            end

        end

    end

    fprintf(fid, newline);

end

% Write function end
fprintf(fid,'}');

%% Close file
fclose(fid);