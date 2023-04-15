function [x_hat] = skew(x)
% Calculate skew-synnetric matrix
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich
    
if length(x) == 3
    x_hat = [ ...
        0 , -x(3) , +x(2) ; ...
        +x(3) , 0 , -x(1); ...
        -x(2) , +x(1) , 0 ...
        ];
else
    error('Not implemented for this dimension.')
end

end
