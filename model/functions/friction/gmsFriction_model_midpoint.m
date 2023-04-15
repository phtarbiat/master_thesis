function [tauF_k, tauFVW_k, z_k, dz_k, Q_k] = gmsFriction_model_midpoint(paramStruct, h, omega_k, omega_pk, tauFVW_pk, tauLoad_k, T_k, z_pk, dz_pk, Q_pk)
% GMS Friction Model using the midpoint rule for numerical integration
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load model parameters
idxOmegaPos = omega_k >= 0; % Check movement direction for every joint

% Static friction
c_c  = paramStruct.staticFriction.c_c_neg;
c_c(idxOmegaPos,:) = paramStruct.staticFriction.c_c_pos(idxOmegaPos,:);
c_load = paramStruct.staticFriction.c_load;
c_omega = paramStruct.staticFriction.c_omega_neg;
c_omega(idxOmegaPos,:) = paramStruct.staticFriction.c_omega_pos(idxOmegaPos,:);
c_T = paramStruct.staticFriction.c_T_neg;
c_T(idxOmegaPos,:) = paramStruct.staticFriction.c_T_pos(idxOmegaPos,:);

% GMS Friction
n = paramStruct.gmsObserver.n;
C = paramStruct.gmsObserver.C;
sigma_0 = paramStruct.gmsObserver.sigma_0;
sigma_1 = 0; % Damping coefficient is set to zero
alpha = paramStruct.gmsObserver.alpha;


% Calculate Static Friction
[tauFVW_k, tauFVS_k] = staticFriction_fun(c_c, c_load, c_omega, c_T, omega_k, tauLoad_k, T_k);
tauFVS_k = sign(omega_k) .* tauFVS_k;


% Calculate internal state and its derivative
nJoint = length(omega_k);
z_k = zeros(nJoint,4);
dz_k = zeros(nJoint,4);
Q_k = zeros(nJoint,4);

tauFVW_i_k = alpha .* tauFVW_k ./ sigma_0;
tauFVW_i_pk = alpha .* tauFVW_pk ./ sigma_0;
omegaZeroCrossing = sign(omega_k) ~= sign(omega_pk);

for iJoint = 1:nJoint
    for iElement = 1:n

        % Determine if spring Elements are sticking or slipping
        if Q_pk(iJoint,iElement) == 0 % Case: element was sticking before

            % Element keeps sticking until friction force is bigger than tauStick_k
            if abs(z_pk(iJoint,iElement)) >= tauFVW_i_k(iJoint,iElement) && sign(z_pk(iJoint,iElement)) == sign(omega_k(iJoint))
                Q_k(iJoint,iElement) = 1; % Element is slipping
            else
                Q_k(iJoint,iElement) = 0; % Element is still sticking
            end

        else % Case: element was slipping before

            % Element keeps slipping until omega crosses zero
            if omegaZeroCrossing(iJoint)
                Q_k(iJoint,iElement) = 0; % Element is sticking
            else
                Q_k(iJoint,iElement) = 1; % Element is still slipping
            end

        end


        % Calculate new internal state and its derivative
        if Q_k(iJoint,iElement) == 0 % Case: element is sticking

            z_k(iJoint,iElement) = gmsFriction_odeStick_midpoint(h, omega_k(iJoint), omega_pk(iJoint), tauFVW_i_k(iJoint,iElement), tauFVW_i_pk(iJoint,iElement), z_pk(iJoint,iElement));
            dz_k(iJoint,iElement) = gmsFriction_odeStick(omega_k(iJoint));

        else % Case: element is slipping

            z_k(iJoint,iElement) = gmsFriction_odeSlip_midpoint(alpha(iJoint,iElement), C(iJoint), h, omega_k(iJoint), omega_pk(iJoint), tauFVW_i_k(iJoint,iElement), tauFVW_i_pk(iJoint), z_pk(iJoint,iElement), dz_pk(iJoint,iElement));
            dz_k(iJoint,iElement) = gmsFriction_odeSlip(alpha(iJoint,iElement), C(iJoint), omega_k(iJoint), tauFVW_i_k(iJoint), z_k(iJoint,iElement));

        end

    end
end


% Calculate GMS Friction
tauF_k = gmsFriction_fun(sigma_0, sigma_1, tauFVS_k, z_k, dz_k);