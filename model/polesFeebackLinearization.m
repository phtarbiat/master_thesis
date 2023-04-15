function [k_0, k_1, k_2, k_3] = polesFeebackLinearization(poles)
% Calculate Coefficients of FJR Feedback Linearization for given pole
% placement
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Coefficients position error
k_0 = poles(:,1) .* poles(:,2) .* poles(:,3) .* poles(:,4);

% Coefficients velocity error
k_1 = -(poles(:,1).*poles(:,2).*(poles(:,3)+poles(:,4)) + poles(:,3).*poles(:,4).*(poles(:,1)+poles(:,2)));

% Coefficients acceleration error
k_2 = poles(:,1).*(poles(:,2)+poles(:,3)+poles(:,4)) + poles(:,2).*(poles(:,3)+poles(:,4)) + poles(:,3).*poles(:,4);

% Coefficients jerk error
k_3 = -(poles(:,1) + poles(:,2) + poles(:,3) + poles(:,4));
