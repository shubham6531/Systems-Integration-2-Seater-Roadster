clear all;
clc;

%% Constants
global g R mu fr airDen gearRatio Cd L eff_trans;
g = 9.81;               % Acceleration due to gravity (m/s^2)
R = 0.6681;             % Wheel Radius (m)
mu = 1;                 % Friction Coefficient
fr = 0.009;             % Rolling friction coefficient
airDen = 1.26;          % Air Density (Kg/m^3)
gearRatio = 8;          % Gear Ratio (including final drive ratio)
Cd = 0.27;              % Coefficient of Drag
L = 2.6;                % Wheel Base (m)
eff_trans = 0.95;       % Efficiency of transmission
eff_batt = 0.9;         % Efficiency of battery
Kp = 1;                 % Parameter Kp for PI controller
Ki = 0;                 % Parameter Ki for PI controller

%% Variables

% Battery
global drivingBattCap;
capCell = 17 * 0.001;       % Cell capacity in kWh
numCellSeries = 192;        % Number of cells in series
numParallelStrings = 16;    % Number of parallel strings of the cells in series (TBD)
netBattCap = capCell * numCellSeries * numParallelStrings;   % Net battery capacity (kWh)
usableBattCap = 0.9 * netBattCap;   % Useful battery energy capacity (kWh)
drivingBattCap = 0.9 * usableBattCap;   % Battery capacity available for driving (after cooling) (kWh)

% Vehicle Mass (Kg)
global mVehicle mAxle mGear;
mPass = 2*101;              % Mass of two AM95 passengers
mCargo = 2*7;               % Mass of cargo for two passengers
mInfo = 15;                 % Mass of infotainment system
mSun = 10;                  % Mass of sunroof
mUnsprung = 4*60;           % Unsprung mass
mProp = 0;                  % Mass of propeller shaft (not used)
mDiff = 10;                 % Mass of Differential
mGear = 45;                 % Mass of Gearbox (to be decided)
mAxle = 2*5;                % Mass of Axle (Add another for AWD)
mBody = 250;                % Mass of body (TBD)
mFrame = 250;               % Mass of Frame Rail (TBD)
mBattery = netBattCap * 1000/160;       % Mass of battery in Kg (TBD)
mMotor = 0;                 % Mass of Motor (TBD)
mVehicle = mPass + mCargo + mInfo + mSun + mUnsprung + mProp + mDiff + mGear + mAxle + mBody + mFrame + mBattery;   % Mass of Vehicle (Additions to be made)

% Vehicle Cost ($)
global cVehicle cAxle cGear;
cElec = 3000;       % Cost of Misc. Electronics
cWheel = 4*450;     % Cost of Wheel Assemblies
cProp = 0;          % Cost of propeller shaft (Not used)
cDiff = 600;        % Cost of Differential
cGear = 7500;       % Cost of Gear Box (TBD. Also, double for 2AWD and consider for 4AWD)
cAxle = 2*350;      % Cost of Axles (Add another for AWD)
cMotor = 0;         % Cost of Motor (TBD)
cBody = 9400;       % Cost of body (TBD)
cFrame = 2*mFrame;  % Cost of frame (TBD)
cBattery = netBattCap * 145;       % Cost of battery (TBD)
cVehicle = cElec + cWheel + cProp + cDiff + cAxle + cBody + cFrame + cBattery;  % Cost of vehicle (Additions to be made)

% Vehicle Dimensions
global fArea WD;
w103Dim = 2.02;         % Dimension W103 (Vehicle Width)
h101Dim = 1.35;         % Dimension H101 (Vehicle Height)
fArea = w103Dim*h101Dim;% Frontal Area (TBD)
WD = 0.48;              % Weight Distribution from the front (TBD)
a = (1-WD)*L;           % Distance of CG from the front (m) (TBD)
b = WD*L;               % Distance of CG from the rear (m) (TBD)
h = 0.65;               % Height of CG (m) (TBD)
isRWD = 1;              % 1 for RWD and 0 for AWD

% Motor
global EM_Efficiency EM_Torque_Map EM_Omega_Map EM_Torque_Max EM_Omega_Max contPowerRating;
contPowerRating = 0;    % Continuous power rating of motor (W)

% Drive Cycle
global sim_time followDriveCycle grade;
load US06_Drive_Cycle.mat;
sim_time = t_cyc(end);
followDriveCycle = 1;   % Whether to follow drive cycle or not (=0 for Wide Open Throttle)
grade = 0;              % Road grade (changes for testing 130kmph ability at 12% slope)

%% Drive Cycle Tests
% We'll run tests for different possible configurations and accumulate the
% results to determine the best possible setup for us

global profit mass cost range toHundredTime topSpeed index;
profit = zeros(1,12);   % Variable to store profit values from various configurations
mass = zeros(1,12);   % Variable to store mass values for various configurations
cost = zeros(1,12);   % Variable to store cost values for various configurations
range = zeros(1,12);   % Variable to store vehicle range values for battery size
toHundredTime = zeros(1,12);   % Variable to store 0-100 kmph times in s
topSpeed = zeros(1,12);   % Variable to store top speed values from various configurations
index = 1;

%% Motor A
mSel = 'A';

% 1 motor RWD
isRWD = 1;
numMotors = 1;
performProfitAnalysis(mSel, isRWD, numMotors);

% 2 motor RWD
isRWD = 1;
numMotors = 2;
performProfitAnalysis(mSel, isRWD, numMotors);

% 2 motor AWD
isRWD = 0;
numMotors = 2;
performProfitAnalysis(mSel, isRWD, numMotors);

% 4 motor AWD
isRWD = 0;
numMotors = 4;
performProfitAnalysis(mSel, isRWD, numMotors);

%% Motor B
mSel = 'B';

% 1 motor RWD
isRWD = 1;
numMotors = 1;
performProfitAnalysis(mSel, isRWD, numMotors);

% 2 motor RWD
isRWD = 1;
numMotors = 2;
performProfitAnalysis(mSel, isRWD, numMotors);

% 2 motor AWD
isRWD = 0;
numMotors = 2;
performProfitAnalysis(mSel, isRWD, numMotors);

% 4 motor AWD
isRWD = 0;
numMotors = 4;
performProfitAnalysis(mSel, isRWD, numMotors);

%% Motor C
mSel = 'C';

% 1 motor RWD
isRWD = 1;
numMotors = 1;
performProfitAnalysis(mSel, isRWD, numMotors);

% 2 motor RWD
isRWD = 1;
numMotors = 2;
performProfitAnalysis(mSel, isRWD, numMotors);

% 2 motor AWD
isRWD = 0;
numMotors = 2;
performProfitAnalysis(mSel, isRWD, numMotors);

% 4 motor AWD
isRWD = 0;
numMotors = 4;
performProfitAnalysis(mSel, isRWD, numMotors);

%% Summarizing results
motor = ['A'; 'A'; 'A'; 'A'; 'B'; 'B'; 'B'; 'B'; 'C'; 'C'; 'C'; 'C'];
conf = ['RWD'; 'RWD'; 'AWD'; 'AWD'; 'RWD'; 'RWD'; 'AWD'; 'AWD'; 'RWD'; 'RWD'; 'AWD'; 'AWD'];
numM = [1; 2; 2; 4; 1; 2; 2; 4; 1; 2; 2; 4];

T = table(motor, conf, numM, profit' * 1e-6, mass', cost', range', toHundredTime', topSpeed');
T.Properties.VariableNames = {'Motor', 'Config', 'Number of Motors', 'Profit ($Million)', 'Mass (Kg)', 'Cost $', 'Range (Km)', '0-100kmph Time (s)', 'Top Speed (kmph)'};

T
[maxProf, maxProfConfig] = max(profit);
maxProf * 1e-6
maxProfConfig
%% Functions

function plotSimulationResults(simOut)
    close all;
    
    % Vehicle Speed plots
    figure;
    plot(simOut.desiredSpeed.Time, simOut.desiredSpeed.Data, 'b');
    hold on;
    plot(simOut.vehicleSpeed.Time, simOut.vehicleSpeed.Data, 'g');
    xlabel('Time (s)');
    ylabel('Speed (km/h)');
    title('Vehicle Speed vs Time');
    legend({'Drive Cycle', 'Actual Speed'});
    hold off;
    
    % Traction Force and traction limit plots
    figure;
    plot(simOut.tractionForce.Time, simOut.tractionForce.Data * 0.001, 'b');
    hold on;
    plot(simOut.tractionForce.Time, ones(size(simOut.tractionForce.Time)) * simOut.accTracLimit.Data * 0.001, 'r');
    plot(simOut.tractionForce.Time, ones(size(simOut.tractionForce.Time)) * simOut.decTracLimit.Data * 0.001, 'y');
    xlabel('Time (s)');
    ylabel('Force (kN)');
    title('Traction Force vs Time');
    legend({'Traction Force', 'Acceleration Traction Limit', 'Deceleration Traction Limit'});
    hold off;
    
    % Motor Torque Plot
    figure;
    plot(simOut.motTorque.Time, simOut.motTorque.Data);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Motor Torque vs Time');
    
    % Power Plot
    figure;
    plot(simOut.power.Time, simOut.power.Data * 0.001);
    xlabel('Time (s)');
    ylabel('Power (kW)');
    title('Net Power vs Time');
end

function [massVehicle, cost, range, toHundredTime, topSpeed, suitability] = calculateValueParameters(mSel, isRWD, numMotors)
    global cVehicle cAxle cGear mVehicle mAxle mGear sim_time followDriveCycle grade drivingBattCap EM_Efficiency EM_Torque_Map EM_Omega_Map EM_Torque_Max EM_Omega_Max contPowerRating;

    % Loading motor data
    if mSel == 'A'
        load MotorMaps\MotorA_Data.mat;
        cMotor = 7500;
        mMotor = 85+35;
        contPowerRating = 175*1000; 
    elseif mSel == 'B'
        load MotorMaps\MotorB_Data.mat;
        cMotor = 5200;
        mMotor = 41+16;
        contPowerRating = 70*1000;
    elseif mSel == 'C'
        load MotorMaps\MotorC_Data.mat;
        cMotor = 5500;
        mMotor = 55+12;
        contPowerRating = 100*1000;
    end

    % Data as per Number of Motors
    EM_Torque_Max = EM_Torque_Max * numMotors;
    EM_Torque_Map = EM_Torque_Map * numMotors;
    contPowerRating = contPowerRating * numMotors;
    cMotor = cMotor * numMotors;
    mMotor = mMotor * numMotors;

    originalmVehicle = mVehicle;    % Storing value for reverting mVehicle after test
    originalcVehicle = cVehicle;    % Storing value for reverting cVehicle after test
    mVehicle = mVehicle + mMotor;
    cVehicle = cVehicle + cMotor;  
    if isRWD == 0
        mVehicle = mVehicle + mAxle + mGear;
        cVehicle = cVehicle + cAxle + cGear;
    end
    massVehicle = mVehicle;
    cost = cVehicle;
    
    % Check if vehicle can reach 130kmph at 12% slope
    followDriveCycle = 0;
    grade = 0.12;
    gradeSim = sim('Powertrain_Model.slx', sim_time);
    %plotSimulationResults(gradeSim);
    if max(gradeSim.vehicleSpeed.Data) >= 130
        suitability = 1;
    else
        suitability = 0;
        %fprintf("Setup not suitable to reach 130 kmph at 0.12 slope\n\n");
    end

    % Run simulation for Drive Cycle
    followDriveCycle = 1;
    grade = 0;
    simOut = sim('Powertrain_Model.slx', sim_time);
    %plotSimulationResults(simOut);
    
    % Range calculation for battery size
    energyUsed = simOut.energyConsumed.Data(end);   % in kWh
    distance = simOut.distance.Data(end) * 0.001;   % in Km
    range = drivingBattCap * distance / energyUsed;
    
    % Run Simulation for WOT
    followDriveCycle = 0;
    wotSim = sim('Powertrain_Model.slx', 60);
    %plotSimulationResults(wotSim);
    toHundredTime = wotSim.vehicleSpeed.Time(find(wotSim.vehicleSpeed.Data >= 100, 1));
    topSpeed = wotSim.vehicleSpeed.Data(end);

    maxMotorTorque = max(simOut.motTorque.Data);
    if maxMotorTorque <= 300
        fprintf('Can use low speed gearbox for Motor %c RWD=%d numM=%d', mSel, isRWD, numMotors);
    end

    % Reverting values for next test
    mVehicle = originalmVehicle;
    cVehicle = originalcVehicle;
end

function performProfitAnalysis(mSel, isRWD, numMotors)
    global cVehicle cAxle cGear mVehicle mAxle mGear sim_time followDriveCycle grade drivingBattCap EM_Efficiency EM_Torque_Map EM_Omega_Map EM_Torque_Max EM_Omega_Max contPowerRating profit mass cost range toHundredTime topSpeed index;

    [massVal costVal rangeVal toHundredTimeVal topSpeedVal suitableVal] = calculateValueParameters(mSel, isRWD, numMotors);
    if suitableVal == 1
        profitVal = profitPredict([costVal; rangeVal; toHundredTimeVal; topSpeedVal]);
    else
        profitVal = 0;
    end
    profit(index) = profitVal;
    mass(index) = massVal;
    cost(index) = costVal;
    range(index) = rangeVal;
    toHundredTime(index) = toHundredTimeVal;
    topSpeed(index) = topSpeedVal;
    index = index + 1;
end
