function y_k = lowPassOrder1(cFilter, x_k, y_pk)
% Equation of a frist order discrete low-pass filter
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


y_k = (1 - cFilter)*y_pk + cFilter*x_k;