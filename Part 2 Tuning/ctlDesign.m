function [LTIblock,Ki,Kt,Tmax,Tmin] = ctlDesign(vehicleModel,task)

% This function takes in an LTI system object representing a simplified model 
% of the vehicle and returns an LTI system object representing a controller. 
% (This can be a transfer function or a state space model).
%
% Arguments:
% -- vehicleModel: LTI system object representing vehicle model possibly
% identified from data. This model has input the torque request (deviation)
% and output the velocity (deviation) in km/h.
% -- task: this allows you to specify a different controller for each task
%   1 (for control task 1)
%   2 (for control task 2)
%   3 (for control task 3)
%
% Return values: 
% Parameters of a controller aiming to regular vehicle velocity in the 
% presence of a changing reference and/or unknown road slope changes. 
% The controller system input is the error between the reference and the 
% measured velocity of the vehicle. The controller system output is a torque 
% request input to the vehicle. Specifically the controller is
% parameterized by:
% -- LTIblock: an LTI system (either transfer function or state space)
% that captures most of the controller architecture
% -- Ki: an integral gain value
% -- Kt: a back-calculation gain 
% -- Tmax: an estimated upper torque saturation value  
% -- Tmin: an estimated lower torque saturation value
% 
% Notes:
% + the parameters Kt, Tmax, and Tmin are only needed if you want to
% implement an anti-windup mechanism in your controller. This is optional.
% + the parameter Ki is only needed if you want integral action in your
% controller and you want to connect it with an anti wind-up mechanism. 

% The following is a minimal example of a controller for each task to get you started
% It is just a proportional controller with gain = 0.5 in each case.

T = vehicleModel.denominator{1}(2);
K_T = vehicleModel.numerator{1}(2);
if task == 1
    tau = 1;
    N= 5;
    Kp = ((0.15*tau+0.35*T)/K_T);
    Kd = 0.5*tau*K_T/(0.3*tau+K_T)/Kp; % AMIGO method from textbook
    Tf = Kd / (N * Kp);
    LTIblock = tf([Kd + Kp * Tf, Kp], [Tf, 1]);
    % % This choice of parameters turns off integral action and anti-windup
    Ki = (0.46*tau+0.02*T)/(K_T^2);
    Kt = 15 * Ki / Kp;
    Tmax = 2.0;
    Tmin = -2.0;
elseif task ==2
    tau = 1;
    N= 5;
    Kp = ((0.15*tau+0.35*T)/K_T);
    Kd = 0.5*tau*K_T/(0.3*tau+K_T)/Kp; % AMIGO method from textbook
    Tf = Kd / (N * Kp);
    LTIblock = tf([Kd + Kp * Tf, Kp], [Tf, 1]);
    % % This choice of parameters turns off integral action and anti-windup
    Ki = (0.46*tau+0.02*T)/(K_T^2);
    Kt = 15 * Ki / Kp;
    Tmax = 2.0;
    Tmin = -2.0;
else 
    tau = 1.2;
    N = 9;
    Kp = (0.15*tau+0.35*T)/K_T;
    Kd = 0.5*tau*K_T/(0.3*tau+K_T)/Kp ; % AMIGO method from textbook
    Tf = Kd / (N * Kp);
    LTIblock = tf([Kd + Kp * Tf, Kp], [Tf, 1]);
    % % This choice of parameters turns off integral action and anti-windup
    Ki = (0.46*tau+0.02*T)/(K_T^2);
    Kt = 15 * Ki / Kp;
    Tmax = 2.5;
    Tmin = -2.5;
end