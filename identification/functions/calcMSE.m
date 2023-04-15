function [mse, mspe] = calcMSE(x, y)
% Calculate Mean Square Error (MSE) and Mean Square Percentage Error
% (MSPE)
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


error = x - y;

mse = mean(error.^2);
mspe = 100 * (1 - mean((error./x).^2));