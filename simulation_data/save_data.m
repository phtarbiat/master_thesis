%% Save Simulink simulation data
simData = struct;

simData.settings.theta_0 = theta_0;
simData.settings.omega_0 = omega_0;
simData.settings.t_end = t_end;
simData.settings.paramStruct = paramStruct;

simData.simulation.timeVec = out.tout;
simData.simulation.theta = out.theta;
simData.simulation.omega = out.omega;
simData.simulation.domega = out.domega;

save(['simulation_data/validation_traj/rigid_model_cad.mat'],'-struct','simData');