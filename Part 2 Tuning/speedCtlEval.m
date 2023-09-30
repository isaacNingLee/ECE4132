% This script runs the single-vehicle control design part of the ECE4132 project
% It does the following basic steps:
% SETUP:
% -- chooses a control task to evaluate (task 1, 2, or 3). Change as needed
% -- generates road slope function
% -- generates a vehicle model (including initial nominal velocity)
% -- generates a speed reference signal 
% SYSID data: 
% -- generates step response data from the model
% -- calls YOUR function (sysID4Ctl) that finds an LTI vehicle model 
% CONTROL design:
% -- calls YOUR function (cltDesignSingle) that returns LTI controller sys
% EVALUATION:
% -- runs the closed-loop system in three scenarios:
%    -- constant reference velocity and changing slope (if speedList has a
%    single element)
%    -- constant zero slope and changing reference velocity (if maxSlope=0)
%    -- both changing reference velocity and changing road slope
%    (otherwise)
% -- RESULTS for each of the three cases:
%    -- plots the vehicle velocity response, the reference 
%    and performance metric, as feedback
% author: James Saunderson, August 2023.

clearvars;
output = zeros(100,5);
for i = [1:20]
    i
    close all;
    load_system('vehicleCtl.slx');
    load_system('vehicleID.slx');

    %%%%%%%% 
    % The control task. Set this to either 1, 2, or 3 depending on which task
    % you are testing.
    task = 3;

    % set nominal velocity for the control problem
    if task==2
        nomVel = 60;
    else 
        nomVel = 50+rand()*50; % nominal velocity between 50 and 100
    end

    %%%%%%%% system ID for control %%%%%%%%%

    % simulation time for sysID
    maxTimeID = 900;
    set_param('vehicleID','StopTime',num2str(maxTimeID));

    % generate a function handle theta that is the road slope (in degrees) 
    % as a function of position. Theta should be a function from reals to reals

    maxSlope = 0; % for identification use zero slope
    theta = generateRoadSlope(maxSlope,maxTimeID,'step');

    % generate function handles needed to simulate the vehicle
    [Ffun,Gfun,vm] = generateVehicleModel4Ctl(theta,nomVel);

    % set model to 0 when initially extracting response data from actual vehicle
    Gvehicle = tf(0);

    % generate system response data based on numResponses step inputs of
    % different amounts. 
    % -- you can change the number of "training" timeseries here if you want to.
    %    if you want to identify a model from data, then you must change this 
    %    to be non-zero
    numResponsesTraining = 4; 
    % Sample random torque input step increments
    % -- you can change this if you would like to try different 
    %    step inputs (rather than random ones) for training data. 
    % -- this will be a 1 x 0 empty array if numResponsesTraining==0
    deltaStepsTraining = 0.1*(2*rand(1,numResponsesTraining)-1);
    [systemInputData,systemResponseData,~] = generateResponseData(Gvehicle,numResponsesTraining,deltaStepsTraining);

    % run YOUR function sysID4Ctl that finds (based on measured system data)
    % an LTI model that relates torque request deviation to velocity deviation.
    % Notes: 
    % -- This can be different from the function you used for system ID, as 
    %    you may want to use a simpler model to inform your control design.
    % -- This is optional. You may not want to use any model identified from 
    %    data to inform your control design. In that case sysID4Ctl can just
    %    return the zero transfer function, for example.
    Gvehicle = sysID4Ctl(systemInputData,systemResponseData);

    %%%%%% control design and testing %%%%%%

    % run YOUR function ctlSingleDesign that returns the parameters of an 
    % controller. The controller it generates takes as input the error 
    % between the measured and reference velocity and produces as output a 
    % torque request for the vehicle model.
    [LTIblock,Ki,Kt,Tmax,Tmin] = ctlDesign(Gvehicle,task);

    % set control simulation time
    maxTimeCtl = 300;
    set_param('vehicleCtl','StopTime',num2str(maxTimeCtl));

    % generate velocity reference signal. In task two this is just a constant.
    if task==2
        velocityRef = generateVelocityReference(maxTimeCtl,nomVel,'const');
    else % in the other tasks this has step changes
        velocityRef = generateVelocityReference(maxTimeCtl,nomVel,'var');
    end

    % generate a function handle theta that is the road slope (in degrees) 
    % as a function of position. Theta should be a function from reals to reals

    % for control task 1, set this to 0 degrees (no road slope). 
    % for control tasks 2 and 3, set this to 6 degrees maximum road slope.
    if task==1
        maxSlope = 0;
    else 
        maxSlope = 6; % (in degrees) 
    end
    theta = generateRoadSlope(maxSlope,maxTimeCtl,'pl');
    [Ffun,Gfun] = generateVehicleModel4Ctl(theta,nomVel,vm);

    % run closed loop system
    [torqueRequestData, velocityResponseData,velocityRefData] = runSpeedControl(LTIblock,Ki,Kt,Tmax,Tmin,velocityRef);

    % compute evaluation metric. 
    % Use values normalized by the reference, so that 
    % comparisons across different runs make more sense
    [mseArray,nmseArray] = computeMSE({velocityResponseData},{velocityRefData});
    [speedingFine,graceProfile] = computeSpeeding(velocityRefData,velocityResponseData);
    % compute acceleration 
    accelerationResponse = timeseries(diff(velocityResponseData.Data/3.6)./diff(velocityResponseData.Time),velocityResponseData.Time(1:(end-1)));
    % compute jerk response 
    jerkResponse = timeseries(diff(accelerationResponse.Data)./diff(accelerationResponse.Time),accelerationResponse.Time(1:(end-1)));

    % plot results
    plotResponsesSpeedRefGs(velocityRef,velocityResponseData,graceProfile,sqrt(nmseArray),speedingFine,accelerationResponse,jerkResponse);
    
    output(i, 1) = mseArray;
    output(i, 2) = sqrt(nmseArray);
    output(i, 3) = speedingFine;
    output(i, 4) = max(abs(accelerationResponse.Data));
    output(i, 5) = max(abs(jerkResponse.Data));

    close_system('vehicleCtl.slx',0);
    close_system('vehicleID.slx',0);
end

save('task3.mat',"output")

%% Plotting
data1 = output(:, 2);
data2 = output(:, 4);
data3 = output(:, 5);

% Create a figure
figure;

% Plot histogram-like representation of each dataset without normalization
subplot(1, 2, 1);
histogram(data1, 10, 'FaceColor', 'g');
xlabel('NRMSE');
ylabel('Frequency');
title('Distribution histogram of NRMSE for Task 3');
subplot(1, 2, 2);
histogram(data2, 10, 'DisplayName', 'Max Acceleration (m/s^2)');
hold on
histogram(data3, 10, 'DisplayName', 'Max Jerk (m/s^3)');

% Add labels and legend
xlabel('Acceleration (m/s^2) and Jerk (m/s^3)');
ylabel('Frequency');
title('Distribution histogram of acceleration and jerk for task 3');
legend('Location', 'northeast');

hold off;
