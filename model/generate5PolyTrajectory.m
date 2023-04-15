function [timeVec, theta, omega, domega, ddomega, dddomega, coeffs] = generate5PolyTrajectory(points, tMin, tMax, tSample, omegaMax, domegaMax)
% Generate time optimal 5th order polynomial trajectory between 2 points 
% considering a minimum and maximum trajectory duration and velocity and 
% acceleration limits between two points
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Initialize vectors
tEnd = zeros(3,1);
coeffs = zeros(3,6);

% Find trajectory accounting for limits
if tMin == 0
    for iJoint = 1:3

        % Create vectory of LLS method
        b = [points(iJoint,1); points(iJoint,2); 0; 0; 0; 0];
        
        % Iterate through trajectory duration
        iTime = 0;
        while true
        
            % Get current iteration values
            curTEnd = tMax - iTime*tSample;
            curCoeffs = getCoeffs(b, curTEnd);
            curTimeVec = (0:tSample:curTEnd)';
        
            curOmega = omega_fun(curCoeffs,curTimeVec);
            curDOmega = domega_fun(curCoeffs,curTimeVec);
        

            % Check limits
            if (max(abs(curOmega)) < omegaMax) && (max(abs(curDOmega)) < domegaMax)
                if curTEnd <= 0
                    error('no trajectory found')
                else
                    iTime = iTime + 1;
                end
            elseif (max(abs(curOmega)) == omegaMax) || (max(abs(curDOmega)) == domegaMax)
                tEnd(iJoint) = curTEnd;
                break;
            else
                if iTime == 0
                    error('no trajectory found')
                else
                    tEnd(iJoint) = curTEnd + (tSample);
                    break;
                end
            end
    
        end
    
    end
    
    tEnd_all = max(tEnd);

else

    tEnd_all = tMin;

end


% Find final coeeficients
for iJoint = 1:3

    b = [points(iJoint,1); points(iJoint,2); 0; 0; 0; 0];
    coeffs(iJoint,:) = getCoeffs(b, tEnd_all)';

end


% Create trajectory from coefficients
timeVec = (0:tSample:tEnd_all)';

theta = [];
omega = [];
domega = [];
ddomega = [];
dddomega = [];

for iJoint = 1:3
    theta = [theta, theta_fun(coeffs(iJoint,:)', timeVec)];
    omega = [omega, omega_fun(coeffs(iJoint,:)', timeVec)];
    domega = [domega, domega_fun(coeffs(iJoint,:)', timeVec)];
    ddomega = [ddomega, ddomega_fun(coeffs(iJoint,:)', timeVec)];
    dddomega = [dddomega, dddomega_fun(coeffs(iJoint,:)', timeVec)];
end

end


function coeffs = getCoeffs(b, tEnd)
% Find coefficients using the LLS method

A = [...
    1 0 0 0 0 0; ...
    1 tEnd tEnd^2 tEnd^3 tEnd^4 tEnd^5; ...
    0 1 0 0 0 0; ...
    0 1 2*tEnd 3*tEnd^2 4*tEnd^3 5*tEnd^4; ...
    0 0 2 0 0 0; ...
    0 0 2 6*tEnd 12*tEnd^2 20*tEnd^3 ...
    ];

coeffs = linsolve(A,b);

end


% Get trajector of position, velcoity, acceleration, ... from coefficients
function theta = theta_fun(coeffs, timeVec)

theta = [ones(length(timeVec),1) timeVec timeVec.^2 timeVec.^3 timeVec.^4 timeVec.^5] * coeffs;

end

function omega = omega_fun(coeffs, timeVec)

omega = [zeros(length(timeVec),1) ones(length(timeVec),1) 2.*timeVec 3.*timeVec.^2 4.*timeVec.^3 5.*timeVec.^4] * coeffs;

end

function domega = domega_fun(coeffs, timeVec)

domega = [zeros(length(timeVec),2) 2.*ones(length(timeVec),1) 6.*timeVec 12.*timeVec.^2 20.*timeVec.^3] * coeffs;

end

function ddomega = ddomega_fun(coeffs, timeVec)

ddomega = [zeros(length(timeVec),3) 6.*ones(length(timeVec),1) 24.*timeVec 60.*timeVec.^2] * coeffs;

end

function dddomega = dddomega_fun(coeffs, timeVec)

dddomega = [zeros(length(timeVec),4) 24.*ones(length(timeVec),1) 120.*timeVec] * coeffs;

end
