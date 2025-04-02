%RUNMODEL - Main code to run robot model for ENG5009 class
%
%   NOTE: Students should only change the code between *---* markers
%
% Syntax: RunModel
%
% Inputs: none
% Outputs: none
% 
% Other m-files required: 
%   Sensor.m
%   WallGeneration.m
%   DynamicalModel.m
%   DrawRobot.m
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Preamble
close all;
clear all;
clc;

%% Simulation setup
% Time
simulationTime_total = 300;           % in seconds *------* YOU CAN CHANGE THIS
stepSize_time = 0.05;               % in seconds 

% Initial states and controls
voltage_left  = 6;                  % in volts *------* YOU CAN CHANGE THIS
voltage_right = -6;                  % in volts *------* YOU CAN CHANGE THIS

state_initial = zeros(1,24);        % VARIABLES OF IMPORTANCE:
                                    % state_initial(13): forward velocity,    v, m/s
                                    % state_initial(18): rotational velocity, r, rad/s
                                    % state_initial(19): current x-position,  x, m
                                    % state_initial(20): current y-position,  y, m
                                    % state_initial(24): heading angle,       psi, rad
state_initial(20) = 0;
state_initial(19) = 0;
state_initial(24) = 0;

% Environment
canvasSize_horizontal = 10; % DO NOT ALTER
canvasSize_vertical   = 10; % DO NOT ALTER
stepSize_canvas       = 0.01;

% *------------------------------------------*
%  YOU CAN ADD MORE SETUP HERE 
%  (e.g. setup of controller or checkpoints)
% *------------------------------------------*

%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall

drawWalls = 1; % draw walls

% outside walls
[wall_1, obstacleMatrix] = WallGenerationRev3( [-4.95, 4.95], [4.95, 4.95], 'h', obstacleMatrix);
[wall_2, obstacleMatrix] = WallGenerationRev3( [-4.95, -4.95], [4.95, -4.95], 'h', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGenerationRev3( [-4.95, -4.95], [-4.95, 4.95], 'v', obstacleMatrix);
[wall_4, obstacleMatrix] = WallGenerationRev3( [4.95, -4.95], [4.95, 4.95], 'v', obstacleMatrix);

% Obstacles
drawObstacles = 1; % when ready set this variable to 1.

if drawObstacles
    [wall_5, obstacleMatrix] = WallGenerationRev3([-3, 1], [3, 1], 'h', obstacleMatrix);
    [wall_6, obstacleMatrix] = WallGenerationRev3([-3.5, 3], [-1, 3], 'h', obstacleMatrix);
    [wall_7, obstacleMatrix] = WallGenerationRev3([1, -3], [3.5, -3], 'h', obstacleMatrix);
    [wall_8, obstacleMatrix] = WallGenerationRev3([2, 2.5], [2, 4], 'v', obstacleMatrix);
    [wall_9, obstacleMatrix] = WallGenerationRev3([-3, -3.5], [-3, -1], 'v', obstacleMatrix);
end

figure(10); 
clf; 
hold on; 
grid on; 
axis([-5.5,5.5,-5.5,5.5]);

if drawWalls 
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-');
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-');

    if drawObstacles
        plot(wall_5(:,1), wall_5(:,2),'k-');
        plot(wall_6(:,1), wall_6(:,2),'k-');
        plot(wall_7(:,1), wall_7(:,2),'k-');
        plot(wall_8(:,1), wall_8(:,2),'k-');
        plot(wall_9(:,1), wall_9(:,2),'k-');
    end
end

fuzzySystem = readfis('FuzzyControlFinalTask.fis');

% *---------------------------*
%  YOU CAN ADD MORE WALLS HERE
% *---------------------------*

%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;
robot_x = state(1,19);
robot_y = state(1,20);
headingAngle = state(1,24);

% Define Checkpoints (from Table 1)
checkpoints = [0, 0;   % Origin
               -1, -1;  % Point 1
               -4, 1;  % Point 2
               3, 3;   % Point 3
               4, -4;   % Point 4
               -4, -2]; % Point 5
          
currentCheckpointIndex = 1; % Start at first checkpoint after the origin
tolerance = 0.05; % How close the robot needs to be to the checkpoint

xlabel('y, m'); ylabel('x, m');
scatter(checkpoints(:,2), checkpoints(:,1))


    % Prepare the new file.
    vidObj = VideoWriter('peaks.avi');
    open(vidObj);

Psi_d = [];
Psi = [];
% Run simulation
for timeStep = 1:timeSteps_total

    %for timeStep = 1:timeSteps_total

        % Get current robot position
        currentLocation = [robot_x, robot_y];
        currentHeading = state(timeStep,24); % Current heading angle

        checkpoint = checkpoints(currentCheckpointIndex, :);

        % Compute required heading to the current checkpoint
        [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(currentLocation, checkpoint, tolerance);

        % If checkpoint reached, move to next one
        if booleanAtCheckpoint
            if currentCheckpointIndex < size(checkpoints,1)
                currentCheckpointIndex = currentCheckpointIndex + 1;
            else
                voltage_left = 0; voltage_right = 0; break;
            end
        end

    % **Get sensor readings (Obstacle distances)**
    [sensorOutput] = Sensor(robot_x, robot_y, headingAngle, obstacleMatrix);
    distance_left = sensorOutput(1);
    distance_right = sensorOutput(2);

    % **Compute heading error (difference from desired heading)**
    heading_error = mod(newHeadingAngle - currentHeading + pi, 2*pi) - pi;
    heading_error = rad2deg(heading_error); % Convert to degrees
    % if heading_error > 90
    %     heading_error = 90;
    % elseif heading_error < -90
    %     heading_error = -90;
    % end

    % **Fuzzy Control: Avoid obstacles while tracking checkpoint**
    fuzzyInput = [distance_left, distance_right, heading_error];
    wheel_control = evalfis(fuzzySystem, fuzzyInput);

    % **Extract left & right voltages from fuzzy system**
    voltage_left = wheel_control(1);
    voltage_right = wheel_control(2);

    % Debugging output
    disp(['Pos: ', num2str(currentLocation), ' | Target: ', num2str(checkpoint)]);
    disp(['Obstacle L: ', num2str(distance_left), ' | Obstacle R: ', num2str(distance_right)]);
    disp(['Heading Error: ', num2str(heading_error)]);
    disp(['Voltage L: ', num2str(voltage_left), ' | Voltage R: ', num2str(voltage_right)]);

    

    
    % Run model *** DO NOT CHANGE
    % Assign voltages calculated in voltages input
    voltages = [voltage_left; voltage_left; voltage_right; voltage_right];

    % Apply Voltages to the model
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;

    % Assign states
    robot_x = state(timeStep,19);
    robot_y = state(timeStep,20);
    headingAngle = state(timeStep,24);

    % Plot robot on canvas  *------* YOU CAN ADD STUFF HERE
    figure(2); clf; hold on; grid on; axis([-5.5,5.5,-5.5,5.5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');

if drawWalls 
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-');
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-');

    if drawObstacles
        plot(wall_5(:,1), wall_5(:,2),'k-');
        plot(wall_6(:,1), wall_6(:,2),'k-');
        plot(wall_7(:,1), wall_7(:,2),'k-');
        plot(wall_8(:,1), wall_8(:,2),'k-');
        plot(wall_9(:,1), wall_9(:,2),'k-');
    end
end

    xlabel('y, m'); ylabel('x, m');

    scatter(checkpoint(:,1), checkpoint(:,2));

    F = getframe;
    writeVideo(vidObj,F);
end

% Close the file.
close(vidObj);

%% Plot results
% *----------------------------------*
%  YOU CAN ADD OR CHANGE FIGURES HERE
%  don't forget to add axis labels!
% *----------------------------------*

figure(2); clf; hold on; grid on; axis([-5.5,5.5,-5.5,5.5]);

if drawWalls 
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-');
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-');
if drawObstacles
    plot(wall_5(:,1), wall_5(:,2),'k-');
    plot(wall_6(:,1), wall_6(:,2),'k-');
    plot(wall_7(:,1), wall_7(:,2),'k-');
    plot(wall_8(:,1), wall_8(:,2),'k-');
    plot(wall_9(:,1), wall_9(:,2),'k-');
end
end


plot(state(:,20), state(:,19));
xlabel('y, m'); ylabel('x, m');

figure(1);
tiledlayout(2,1);
nexttile
plot(time, state(:,13)); 
grid on;
xlabel('time, s'); ylabel('xdot, m/s');

nexttile
plot(time, state(:,24)); 
grid on;
xlabel('time, s'); ylabel('psi, radians');