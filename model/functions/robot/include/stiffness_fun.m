function k = stiffness_fun(paramStruct, thetaDiff_k)
% Joint spring torque equation and its first time derivative
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
c_k = paramStruct.robot.c_k;
k_thetaDiff_lim = paramStruct.robot.k_thetaDiff_lim;
k_dataSheet = paramStruct.robot.k_dataSheet;


% Calculate joint stiffness
k = zeros(3,1);

for iJoint = 1:3

    if abs(thetaDiff_k(iJoint)) <= k_thetaDiff_lim(iJoint,1) % Use identified stiffness curve

        k(iJoint) = c_k(iJoint,1) .* c_k(iJoint,2) .* abs(thetaDiff_k(iJoint)).^(c_k(iJoint,2)-1);

    else % Use data sheet values (for deflection greate than limits)

        if abs(thetaDiff_k(iJoint)) <= k_thetaDiff_lim(iJoint,2)
            k(iJoint) = k_dataSheet(iJoint,1);
        else
            k(iJoint) = k_dataSheet(iJoint,2);
        end

    end

end