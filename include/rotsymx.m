function R = rotsymx(phi)
% Rotation matrix x-axis
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich
    
R = [ ...
    1 , 0 , 0; ...
    0 , cos(phi) , -sin(phi); ...
    0 , sin(phi) , cos(phi); ...
    ];
	
end

