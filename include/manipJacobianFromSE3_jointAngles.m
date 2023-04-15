function [J_Pi_s, J_Pi_b] = manipJacobianFromSE3_jointAngles(g_0Pi, theta)
% Calculate manipulator Jacobians corresponding to the homogeneous 
% transformation g_0Pi.
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Symbolic Jacobians
J_Pi_s = sym(zeros(6, 3));
J_Pi_b = sym(zeros(6, 3));

for iCol = 1:3
    
    % Partial derivative of g
    g_0Pi_dtheta = ...
        diff(g_0Pi, theta(iCol));
    
    % Spatial Jacobian
    J_Pi_s(:, iCol) = skewInvSE3( g_0Pi_dtheta / g_0Pi );
    
    % Body-Fixed Jacobian
    J_Pi_b(:, iCol) = skewInvSE3( g_0Pi \ g_0Pi_dtheta );
end

J_Pi_s = simplify(J_Pi_s);
J_Pi_b = simplify(J_Pi_b);

end