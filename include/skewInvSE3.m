function V = skewInvSE3(gSk)
% Calculate elements of se(3) from skew-symmetric matrix
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Extract skew-symmetric matrix (so(3)) and position vector
R = gSk(1:3, 1:3);
v = gSk(1:3, 4);

% Give 6x1 vector
V = [v; skewInv(R)];

end