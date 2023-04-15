function  plotRobotData_traj( expData )
    %% Plot robot measurement data
    %
    % INPUTS:
    %  * See arguments block below.
    %
    % OUTPUTS:
    %  * None.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        expData (1,1) struct % Struct with all the data as fields
    end


    % Initialize axis handles for figures and subplots
    fhPos    = figure('Name', 'Position');
    fhVel    = figure('Name', 'Velocity');
    fhTorque = figure('Name', 'Torque');
    fhAcc    = figure('Name', 'Temperature');

    for iAxis = 1:3
        fhSubPos(iAxis) = subplot(3,1,iAxis, 'Parent', fhPos);
        fhSubVel(iAxis) = subplot(3,1,iAxis, 'Parent', fhVel);
        fhSubTorque(iAxis) = subplot(3,1,iAxis, 'Parent', fhTorque);
        fhSubAcc(iAxis) = subplot(3,1,iAxis, 'Parent', fhAcc);
    end
    
    % Plot data on axes
    % Note: We're directly plotting the data on the corresponding axes
    % objects (instead of first setting the current figure and then 
    % plotting to it, e.g., with figure(fhPos)), which is much faster
    for iAxis = 1:3

        %%% Positions
        plot(fhSubPos(iAxis), expData.t, expData.thetaMot(:, iAxis))
        hold(fhSubPos(iAxis), 'on');
        plot(fhSubPos(iAxis), expData.t, expData.thetaLoad(:, iAxis))
        plot(fhSubPos(iAxis), expData.t, expData.refValuePos(:, iAxis))

        legend(fhSubPos(iAxis), 'Motor', 'Load', 'Ref')
        ylabel(fhSubPos(iAxis), 'Position / deg', 'Interpreter', 'latex')
        xlabel(fhSubPos(iAxis), 'Time / s', 'Interpreter', 'latex')
        grid(fhSubPos(iAxis), 'on');


        %%% Velocity
        plot(fhSubVel(iAxis), expData.t, expData.velocity(:, iAxis))
        %hold(fhSubVel(iAxis), 'on');
        %plot(fhSubPos(iAxis), expData.t, expData.refValueVel(:, iAxis))

        legend(fhSubPos(iAxis), 'Motor', 'Ref')
        ylabel(fhSubVel(iAxis), 'Velocity / deg/s', 'Interpreter', 'latex')
        xlabel(fhSubVel(iAxis), 'Time / s', 'Interpreter', 'latex')
        grid(fhSubVel(iAxis), 'on');


        %%% Torque
        plot(fhSubTorque(iAxis), expData.t, expData.torque(:, iAxis))
        hold(fhSubTorque(iAxis), 'on');
        plot(fhSubTorque(iAxis), expData.t, expData.refValueTorque(:, iAxis))

        legend(fhSubTorque(iAxis), 'Actual', 'Ref')
        ylabel(fhSubTorque(iAxis), 'Torque / Nm', 'Interpreter', 'latex')
        xlabel(fhSubTorque(iAxis), 'Time / s', 'Interpreter', 'latex')
        grid(fhSubTorque(iAxis), 'on');


        %%% Acceleration
        plot(fhSubAcc(iAxis), expData.t, expData.temperature(:, iAxis));

        ylabel(fhSubAcc(iAxis), 'Temperature / C', 'Interpreter', 'latex')
        xlabel(fhSubAcc(iAxis), 'Time / s', 'Interpreter', 'latex')
        grid(fhSubAcc(iAxis), 'on');
    end
end

