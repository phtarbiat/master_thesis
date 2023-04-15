function path = pathdef_local()
    %% Path definition file
    % Contains the path to the global directory with the required functions.
    % Use this function to add the directory to the path:
    %
    % addpath(pathdef_local);
    %

    % Relative or absolute path to the include directory;
    % Use genpath to also include subfolders
    path = [
        %genpath('../_matlab_include'), ';', ...
        %genpath('../robot_modeling_rigid/_include_modeling'), ';', ...
        %genpath('../robot_modeling_flexible'), ';', ...
		genpath(pwd)
        ];

end

