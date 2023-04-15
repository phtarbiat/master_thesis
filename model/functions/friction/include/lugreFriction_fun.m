function tauF_k = lugreFriction_fun(sigma_0, sigma_1, tauFVS_k, z_k, dz_k)
% LuGre Friction Model: friction torque equation
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


tauF_k = sigma_0.*z_k + sigma_1.*dz_k + tauFVS_k;