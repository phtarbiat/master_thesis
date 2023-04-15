function dz = gmsFriction_odeSlip(alpha, C, omega, tauFVW_i, z)
% GMS Friction Model: ODE of the slipping bristle element
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


dz = sign(omega) .* (C.*alpha) .* (1 - z./tauFVW_i);