function M_dq = massMatrixDerivatives(M, theta)
% Calculate Derivatives of the Mass Matrix
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% System dimensions
n = size(M, 1);

% Mass matrix derivatives
syms M_dq [n, n, n];
for itheta = 1:n
    for iRow = 1:n
        for iCol = 1:n
            M_dq(iRow, iCol, itheta) = ...
                diff( M(iRow, iCol), theta(itheta) );
        end
    end
end

end