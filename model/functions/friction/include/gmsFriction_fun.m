function tauF_k = gmsFriction_fun(sigma_0, sigma_1, tauFVS_k, z_k, dz_k)
% GMS Friction Model: friction torque equation
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


tauF_k = sum((sigma_0.*z_k + sigma_1.*dz_k),2) + tauFVS_k;
