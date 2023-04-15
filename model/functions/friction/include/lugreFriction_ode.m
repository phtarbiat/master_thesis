function dz = lugreFriction_ode(sigma_0, omega, tauFVW, z)
% LuGre Friction Model: ODE of the bristle element
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


dz = omega - (z .* abs(omega) .* (sigma_0 ./ tauFVW));