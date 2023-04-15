# Code Base of the Master Thesis: Modeling, Identification and Control of a 3R Elastic-Joint Lightweight Robot Manipulator

Contains scripts and Simulink models for modeling and experimentally identifying a 3R robot with flexible joints.
The considered and identified model parameters are:

- the base parameters of the rigid and flexible joint robot model
- the kinematic error of the joints 
- the joint stiffness
- a velocity-, temperature- and load-dependent static and dynamic friction model

Overview of main folders:

- `/identification`: Identification of the robot model with experimental data
- `/model`: Modelling and control of the 3R robot with Matlab functions and Simulink
- `/sps_codgen`: Generation of C++ Code for the PLC (Programmable Logic Unit) of the robot for control purposes
- `/trajectory`: Creation of identification trajectories for the different model aspects
- `/validate`: Validation of the identified robot model