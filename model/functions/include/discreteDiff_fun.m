function dx = discreteDiff_fun(h, x_k, x_pk)
% Equation of single step numerical derivative
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


dx = (x_k - x_pk) / h;