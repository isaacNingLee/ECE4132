function vehicleModel = sysID(inputData,outputData)

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




%vehicleModel = tf(270,[100,1]);

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

inputTimesVector = inputData{1}.Time; % uses assumption that these are all the same
inputDataArray = zeros(length(inputData{1}.Data),length(inputData));
outputDataArray = zeros(length(outputData{1}.Data),length(outputData));
for j=1:length(inputData)
    inputDataArray(:,j) = inputData{j}.Data - inputData{j}.Data(1);
end
for j=1:length(outputData)
    outputDataArray(:,j) = outputData{j}.Data - outputData{j}.Data(1);
end
Tmax = 900;
% thetaOpt = zeros(length(inputData),2);
% for i = [1:1:length(inputData)]
%     input_data = inputDataArray(:,i);
%     output_data = outputDataArray(:,i);
%     ts = inputTimesVector;
%     v_ss = mean(output_data(end-50:end));
%     % value_init = in
%     % percent102 = min(find((inputData.Data-inputData.Data(1)) >= (value_ss-T_init)*1.02))
%     % sigma_d = log(1/0.02)/Ts;
% 
%     cost = @(theta) sum((output_data-lsim(tf(theta(2)*theta(1), [1,theta(1)]), input_data, ts)).^2);
%     v_init = output_data(1);
%     T63 = (find((abs(output_data-v_init)) >= (abs(v_ss-v_init))*0.63, 1 )-1) * Tmax / length(ts) ;
%     theta_init = [1/T63, mean(output_data)/mean(input_data)];
%     %theta_init = [1,1];
%     thetaOpt(i,:) = fminsearch(cost, theta_init);
% 
% end
% thetaOpt = mean(thetaOpt);
% vehicleModel = tf(thetaOpt(2)*thetaOpt(1), [1,thetaOpt(1)]);


% thetaOpt = zeros(length(inputData),3);
% for i = [1:1:length(inputData)]
%     input_data = inputDataArray(:,i);
%     output_data = outputDataArray(:,i);
%     ts = inputTimesVector;
%     v_ss = mean(output_data(end-50:end));
%     % value_init = in
%     % percent102 = min(find((inputData.Data-inputData.Data(1)) >= (value_ss-T_init)*1.02))
%     % sigma_d = log(1/0.02)/Ts;
% 
%     cost = @(theta) sum((output_data-lsim(tf(theta(2)*theta(1), [1,theta(1)],'IODelay',theta(3)), input_data, ts)).^2);
%     v_init = output_data(1);
%     T63 = (find((abs(output_data-v_init)) >= (abs(v_ss-v_init))*0.63, 1 )-1) * Tmax / length(ts) ;
%     value_init = output_data(1);
%     tau = find(abs(output_data-value_init)>0, 1 ) * Tmax / length(ts);
%     theta_init = [1/T63, mean(output_data)/mean(input_data),tau];
%     %theta_init = [1,1];
%     thetaOpt(i,:) = fminsearch(cost, theta_init);
% 
% end
% thetaOpt = mean(thetaOpt);
% vehicleModel = tf(thetaOpt(2)*thetaOpt(1), [1,thetaOpt(1)],'IODelay',thetaOpt(3));


% 
% thetaOpt = zeros(length(inputData),4);
% ts = inputTimesVector;
% for i = [1:1:length(inputData)]
%     input_data = inputDataArray(:,i);
%     output_data = outputDataArray(:,i);
%     cost = @(theta) sum((output_data-lsim(tf([theta(1)], [1,theta(2),theta(3)])*exp(tf([-abs(theta(4)),0],1)), input_data, ts)).^2);
%     theta_init = ones(4,1);
%     theta_init(1) = mean(output_data)/mean(input_data);
%     value_init = output_data(1);
%     theta_init(4) = find(abs(output_data-value_init)>0, 1 ) * Tmax / length(ts);
%     thetaOpt(i,:) = fminsearch(cost, theta_init);
% 
% end
% thetaOpt = mean(thetaOpt);

% vehicleModel = tf([thetaOpt(1)], [1,thetaOpt(2),thetaOpt(3)])*exp(tf([-abs(thetaOpt(4)),0],1));

thetaOpt = zeros(length(inputData),3);
ts = inputTimesVector;
for i = [1:1:length(inputData)]
    input_data = inputDataArray(:,i);
    output_data = outputDataArray(:,i);
    cost = @(theta) sum((output_data-lsim(tf([theta(1)], [1,theta(2),theta(3)]), input_data, ts)).^2);
    theta_init = ones(3,1);
    theta_init(1) = mean(output_data)/mean(input_data);
    thetaOpt(i,:) = fminsearch(cost, theta_init);

end
thetaOpt = mean(thetaOpt);

vehicleModel = tf([thetaOpt(1)], [1,thetaOpt(2),thetaOpt(3)]);


% The following is an example of how to use lsim to simulate the response
% of vehicleModel to all of the inputs, and capture the responses in an array.
% The same approach may be useful for your systemID method.

% Please comment this out if you do not need to use this code!!
modelOutputArray = lsim(vehicleModel*eye(length(inputData)),inputDataArray,inputTimesVector);
%modelOutputArray  = lsim(vehicleModel,inputDataArray(1), inputTimesVector)
