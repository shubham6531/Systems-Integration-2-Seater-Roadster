%% be careful using these commands... they really delete everything.
clear all;
close all;

%% Setting all the variable values -- per category
% requirements
tireRadius = 310; % in [mm] as per requirement
wheelBase = 2700; % in [mm] as per requirement

% design choices -- variables that can be chosen by the designer
frameRailLength = 3800; % in [mm]
frameRailHeight = 120; % in [mm]
H156 = 300; % in [mm]

% intermediate variable -- computed from design variables or requirements
frontAxle = [0,tireRadius]; % in [mm]
rearAxle = frontAxle + [wheelBase,0]; % follows from definition of wheelBase
middle = wheelBase/2;
front = middle - frameRailLength/2;
rear = front +  frameRailLength;

%% Plot the vehicle packaging
figure(1)
grid on
hold on
axis equal
plotCircle(frontAxle,tireRadius,'k');
plotCircle(rearAxle,tireRadius,'k');

% plot the frame rails
plotRectangle([front,H156+frameRailHeight],[rear,H156],'c')
