function kinError_k = kinError_fun(paramStruct, thetaMotor_k)
% Calculate kinematic error
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load parameters
addKinError = paramStruct.kinError.addKinError;
hasPError = paramStruct.kinError.hasPError;


% Calculate kinematic error
kinError_k = zeros(3,1);

if addKinError

    for iJoint = 1:3

        % Calculate non-periodic kinematic error
        % Get Lookup table
        xLookup = paramStruct.kinError.xLookup(iJoint,:);
        yLookup = paramStruct.kinError.yLookup(iJoint,:);

        % Interpolate Lookup table
        npKinError_k = interp1(xLookup,yLookup,thetaMotor_k(iJoint));


        % Add periodic to non-periodic kinematic error 
        if hasPError(iJoint)

            % Get fourier series coefficients
            c_pError = paramStruct.kinError.c_pError(iJoint,:)';

            % At the moment only joint 1 has periodic kinematic error
            if iJoint == 1
                pKinError_k = kinError_periodic_axis1_fun(c_pError,thetaMotor_k(iJoint));
            else
                pKinError_k = 0;
            end

            kinError_k(iJoint) = npKinError_k + pKinError_k;

        else

            kinError_k(iJoint) = npKinError_k;

        end
        
    end

end