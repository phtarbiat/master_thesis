function k = stiffnessTauE_fun(c_k, tauE_k)
% Joint spring stiffness calculated from joint spring torque
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Calculate joint stiffness
k = zeros(3,1);

for iJoint = 1:3

    % Stiffness equation
    if tauE_k(iJoint) == 0
        k(iJoint) = Inf;
    else
        k(iJoint) = (abs(tauE_k(iJoint)) .* c_k(iJoint,2)) ./ (exp(log(abs(tauE_k(iJoint))./c_k(iJoint,1)) ./ c_k(iJoint,2)));
    end

end