% This script runs the system identification part of the ECE4132 project
% -- generates a model
% -- generates step response data from the model
% -- calls YOUR function (sysID) that finds an LTI vehicle model 
% -- compares the system response and the model response on a new collection of step inputs
% -- computes MSE on test data
% -- plots the vehicle and model responses as feedback
% author: James Saunderson, August 2023.
clearvars;
RMSE_arr = zeros(10,4);
for i = [1:10]
    i
    close all;
    load_system('vehicleID.slx');
    
    % simlation stop time
    Tmax = 900;
    set_param('vehicleID','StopTime',num2str(Tmax));
    
    % generate a function handle theta that is the road slope (in degrees) 
    % as a function of position. Theta should be a function from reals to reals
    % For the system ID part of the project we assume the road is flat
    maxSlope = 0;
    theta = generateRoadSlope(maxSlope,Tmax,'step');
    
    % nominal velocity (km/h) at which to identify the model
    vnominal = 50 + 50*rand(1);
    
    % generate functions needed to simulate the vehicle
    [Ffun,Gfun,vm] = generateVehicleModel(theta,vnominal);
    
    % set your model to 0 when initially extracting response data from actual vehicle
    Gvehicle = tf(0);
    
    % Generate system response data based on numResponses step inputs of different heights
    % -- You may change numResponsesTraining, the number of "training" timeseries, if you want to.
    % -- A value of zero corresponds to not generating any training data.
    % -- When assessing your sysID code, we will set numResponsesTraining = 4.
    numResponsesTraining = 4;
    % Sample random torque input step increments between -0.1 and 0.1
    % -- you can change this if you would like to try different 
    %    step inputs for training data. 
    
    %deltaStepsTraining =
    %0.1*(2*rand(1,numResponsesTraining)-1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    deltaStepsTraining = 0.1*(2*rand(1,numResponsesTraining)-1);
    [systemInputData,systemResponseData,~] = generateResponseData(Gvehicle,numResponsesTraining,deltaStepsTraining);
    
    % run YOUR function sysID that finds (based on measured system data)
    % an LTI model that relates torque request deviation to velocity deviation.
    Gvehicle = sysID(systemInputData,systemResponseData);
    
    % evaluate on four new system inputs, to see if the model generalizes
    % please do ** not ** change the value of numResponsesTest.
    numResponsesTest = 4;
    % sample random test torque input step increments between -0.1 and 0.1
    % -- you can change this if you would like to 
    %    try different step inputs for test data.
    deltaStepsTest = 0.1*(2*rand(1,numResponsesTest) - 1);
    [systemInputData, systemResponseData, myResponseData] = generateResponseData(Gvehicle,numResponsesTest,deltaStepsTest);
    
    % compute mean squared error for each evaluation case
    mseArray = computeMSE(myResponseData,systemResponseData);
    RMSE_arr(i,:) = sqrt(mseArray)./(abs(deltaStepsTest)');
    % Plot results and report normalized RMSE. 
    % Normalization allows for meaningful comparisons.
    plotResponses(myResponseData,systemResponseData,sqrt(mseArray)./(abs(deltaStepsTest)'),deltaStepsTest);
    
    
    close_system('vehicleID.slx',0);

end
save('second_order_no_td2.mat',"RMSE_arr")