function [mae, mape] = calcMAE(x, y)
% Calculate Mean Absolute Error (MAE) and Mean Absolute Percentage Error
% (MAPE)
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


error = x - y;

mae = mean(abs(error));
mape = 100 * (1 - mean(abs(error./x)));