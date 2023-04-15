function R = rotsymz(phi)
% Rotation matrix z-axis
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich
    
R = [ ...
	cos(phi) , -sin(phi) , 0; ...
	sin(phi) , cos(phi) , 0; ...
	0 , 0 , 1; ...
    ];
	
end

