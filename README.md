# human-aware-robot-planning
 
The package gathers all the code developed to cope with the probabilitic modelling of human operators in industrial tasks, and the generation of human-aware trajectories to improve the human robot collaboration efficienfy and safety.

Refer to article:

Pellegrinelli, S., Orlandini, A., Pedrocchi, N., Umbrico, A., Tolio, T., Motion planning and scheduling for human and industrial-robot collaboration (2017) CIRP Annals - Manufacturing Technology, 66 (1), pp. 1-4. DOI: 10.1016/j.cirp.2017.04.095




# Overview

# Matlab
Run the file simulation_stack.m.

In order to load human data, indicate the path of the file "points.mat" at line 18 of the code. For instance: 'data\September2016'


In order to use random generation of trajectories: robot_random_traj=1


In order to use alraedy generated trajectories: robot_random_traj=0 and update the path at line 46. 

MinGW must be installed: https://it.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c++-compiler



# Developers Contacts

**Authors:** 

- Stefania Pellegrinelli (stefania.pellegrinelli@itia.cnr.it)
- Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)
 
Software License Agreement (BSD License) Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation. All rights reserved.

# Acknowledge

**This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 637095.**
