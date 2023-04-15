function e = unit(dim, dir)
% Unit vector with dimension 'dim' and in direction 'dir'
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Check input arguments: dir <= dim
if dir > dim
    error('Position of the non-zero element cannot be larger than the vector dimension.')
end

% Generate unit vector
e = zeros(dim, 1);
e(dir) = 1;

end

