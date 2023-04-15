function [x] = skewInv(X)
% Calculate elements from skew-symmetric matrix
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


if all(size(X) == 3)
    x = [ X(3,2); X(1,3); X(2,1) ];
else
    error('Not implemented for this dimension.')
end

end
