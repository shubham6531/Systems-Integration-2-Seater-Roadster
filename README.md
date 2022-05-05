# Systems-Integration-2-Seater-Roadster
The project performs systems integration for designing a vehicle (2 seater roadster) from a powertrain, packaging, vehicle dynamics, and economic feasibility perspective. An in-depth description of the task can be found in Problem_Statement.pdf. In essence, the project develops a 2 Seater Battery Electric Roadster by performing the following tasks:

Powertrain:
1. Select an electric motor from three available options by taking into account their torque curves, efficiencies, mass, packaging, and costs into account.
2. Evaluate and choose best drivetrain configuration from 1 motor FWD/RWD, 2 motor FWD/RWD/AWD, and 4 motor AWD taking into account the vehicle's performance, cost, and packaging constraints
3. Design a battery pack in terms of the size of cell strings and number of parallel strings, while aiming for maximum vehicle range, performance (weight), and maximum profitability (tradeoff between cost and vehicle demand. Look at profitPredict.m for more info)
4. Include ineffiencies and constraints such as traction limit in the required calculations 

Packaging:
1. Design vehicle packaging considering selected components and metrics such as center of gravity and passenger space.
2. Provide graphic visualizations of possible vehicle configurations (check BEV_DesignVisualization.mat and Final_Report.pdf for more info)

Vehicle Dynamics:
1. Evaulate vehicle performance in terms of slip angles and stiffness values arising from the selected configurations.

# MATLAB files
To perform the complete systems integration analysis:
1. Download repository and unzip it/
2. Unzip MotorMaps.zip
3. Run Master_Script.mat

# Requirements
Matlab 2021a or higher
