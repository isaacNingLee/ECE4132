function vehicleModel = sysID4Ctl(inputData,outputData)

% This function takes in two cell arrays of timeseries data and returns an
% LTI system object (this can be a transfer function or a state space
% model). 
%
% Arguments:
% -- inputData is a k x 1 cell array where each element is a timeSeries object
%    - inputData{j} is the timeSeries object corresponding to the jth 
%      "training" input
%    - inputData{j}.Time is an array of the simulation time-stamps for the
%      jth "training" input
%    - inputData{j}.Data is an array of the corresponding input signal values
%      for the jth "training" input
% -- ouputData is a k x 1 cell array where each element is a timeSeries object
%    - outputData{j} is the timeSeries object corresponding to the jth
%      "training" output
%    - outputData{j}.Time is an array of the simulation time-stamps for the
%      jth "training" output
%    - outputData{j}.Data is an array of the corresponding input signal values
%      for the jth "training" output
%
% Assumptions:
% -- the function assumes that all of the inputData{j}.Time vectors and 
% all of the outputData{j}.Time vectors are identical.
%
% Constraints:
% -- if you want to simulate your model within this function, you are 
%    expected to use lsim.
% -- you should ** not ** call the function generateResponseData in this function. 
%    The only information about the system you should use in this function are the input/output 
%    timeSeries arrays that are passed as arguments to the function
% 
% Hints:
% -- The system starts in equilibrium. In the data preprocessing given below, 
%    there is a shift that converts the given signals into the *deviation* 
%    of the input from equilibrium and the *deviation* of the ouput from 
%    equilbrium. Your LTI system model works with these deviation signals.
% -- To access the number of timeSeries objects in inputData you can use
%    length(inputData). (Similarly for outputData.)
% -- With a little trick (illustrated in the sample code below), you can run lsim 
%    on multiple different inputs simultanously (stored as an array) and 
%    it will return multiple outputs (stored as an array)

% Everything below is just sample code that may or may not be useful.
% It can be safely removed and replaced with your code.

% this is just a fixed reference vehicle model to get you started. It does not
% use the input/output data at all!

% DATA PREPROCESSING (may be useful, can be removed if not needed):
% The following converts the cell arrays of time-series that are passed
% into the function into arrays that are (# timesteps) x (# timeSeries)
% and are shifted to remove the equilibrium values, so they are easier 
% to pass into lsim.
% -- inputTimesArray is a vector of time-stamps
% -- inputDataArray is an array of corresponding input signal values 
%    shifted to represent deviation from equilibrium, with
%    each column corresponding to a different input signal
% -- outputDataArray is an array of corrsponding output signal values, 
%    shifted to represent deviation from equilibrium, with 
%    each column corresponding to a different output signal

% Obtain the Time Vector from the input data
inputTimesVector = inputData{1}.Time; 

% Pre-allocate the input data and the output data array and the parameters
inputDataArray = zeros(length(inputData{1}.Data),length(inputData));
outputDataArray = zeros(length(outputData{1}.Data),length(outputData));
thetaOpt = zeros(length(inputData),2);
Tmax = 900;
% Shift input data to represent deviations from equilibrium
for i = 1:length(inputData)
    inputDataArray(:, i) = inputData{i}.Data - inputData{i}.Data(1);
end

% Shift output data to represent deviations from equilibrium
for i = 1:length(outputData)
    outputDataArray(:, i) = outputData{i}.Data - outputData{i}.Data(1);
end

% Identify the system parameters (thetaOpt) for each input-output pair using fminsearch
for i = 1:1:length(inputData)
    input_data = inputDataArray(:,i);
    output_data = outputDataArray(:,i);
    ts = inputTimesVector;
    v_ss = mean(output_data(end-50:end));
    
    % Define a cost function to minimize using fminsearch
    cost = @(theta) sum((output_data-lsim(tf(theta(2)*theta(1), [1,theta(1)]), input_data, ts)).^2);
    
    % Initialise parameter estimates
    v_init = output_data(1);
    T63 = (find((abs(output_data-v_init)) >= (abs(v_ss-v_init))*0.63, 1 )-1) * Tmax / length(ts) ;
    theta_init = [1/T63, mean(output_data)/mean(input_data)];
    % Optimize the cost function to find the best parameter estimates
    thetaOpt(i,:) = fminsearch(cost, theta_init);
end

% Take the mean of parameter estimates across all input-output pairs
thetaOpt = mean(thetaOpt);

% Create the identified LTI system model (vehicleModel)
vehicleModel = tf(thetaOpt(2)*thetaOpt(1), [1,thetaOpt(1)]);

% The following is an example of how to use lsim to simulate the response
% of vehicleModel to all of the inputs, and capture the responses in an array.
% The same approach may be useful for your systemID method.

% Please comment this out if you do not need to use this code!!
% modelOutputArray = lsim(vehicleModel * eye(length(inputData)), inputDataArray, inputTimesVector);
