   %%% <Controller Tuning> %%% 

%%%% -- This is the second code used in the controller design process -- %%%%

%%% DESCRIPTION
%%% This code uses the Control System Toolbox to tune a PID controller given the plant model identified in the SystemID.m code. 
%%% Goals of the controller are to acheive zero steady error and a transient behavior matching that which was measured on the actual bike.
%%% Finally, the mechanical power associated with the closed loop response
%%% is calculated to ensure excessive power is not required 

%%%%%% Doing the all clear %%%%%%
clear
clc
close all

%% Acquiring Settling Time of The Measured Time Response 
% Here we measure the settling time of the measured ebike time response to use it to help us match the closed loop system's transient response with the measured one 

% Loading Measured Time Response Data
load('00912.mat') 
xdata = linspace(0,20,2401);  
ydata = (2*pi()/60)*(456.3862.*NIData(300:2700,7)-1.2846)*(0.341*0.028985/0.333375); %the raw data from 00912.mat must first be processed to convert it to m/s (see http://moorepants.github.io/dissertation/davisbicycle.html#calibration) 
ydata = transpose(ydata);

% Plotting Measured Time Reponse
figure('Name','Measured Time Response')
plot(xdata,ydata)
title('Measured Ebike Time Response')
xlabel('Time (s)')
ylabel('Speed (m/s)') 

% Finding Settling Time of Measured Time Response 
spdFinal = mean(ydata(2218:end)); % First index found from looking at the plot above
msrdSetTime = xdata(2218);
fprintf('Measured Settling Time = %.1f (s)\n',msrdSetTime)

%%
%%%%%% PLANT MODEL %%%%%% (Identified from the system identification step)
%%%
%%% v(s)/V(s) = 15.32 / (s^2 + 70.81*s + 7.34 ) 
%%%
%%%%%% %%%%%% %%%%%% %%%%%

% Declaring the Plant Model
Gp = tf(15.32,[1,70.81,7.34]); 

%%%%%% Using the Control System Toolbox to tune a PID controller %%%%%% 
% INSTRUCTIONS
% Uncomment either of the following lines of code to launch either app for tuning a PID controller
% Please see MATLAB's documentation for either app for help with using the
% app

% Uncomment either one of these lines of code to use the app
%pidTuner(Gp,'pid')
%controlSystemDesigner('rlocus',Gp)

%%%%%% Declaring the Control Architecture %%%%%%
Gc = pid(1.03,0.145,0); % PID constants found from using controller tuning app
sys_cl = feedback(Gc*Gp,1); % Declaring a unity feedback control architecture with the plant model Gp and PID Controller Gc

% Simulating the Closed Loop Step Response
t = linspace(0,20,8000); 
opt = stepDataOptions('StepAmplitude',spdFinal);
y = step(sys_cl,t,opt);  

% Plotting Measured vs Closed Loop Time Responses
figure('Name','Comparing Measured to Tuned Performance')
plot(t,y,'b') 
hold on 
plot(xdata,ydata,'r')
title('Comparison Plot Between Measured and Closed Loop Time Responses')
xlabel('Time [s]')
ylabel('Speed [m/s]') 

%% Gathering and Plotting Additional Time Responses At Different Step Amplitudes
opt1 = stepDataOptions('StepAmplitude',5);
opt2 = stepDataOptions('StepAmplitude',3);
opt3 = stepDataOptions('StepAmplitude',1); 

y1 = step(sys_cl,t,opt1);
y2 = step(sys_cl,t,opt2);
y3 = step(sys_cl,t,opt3);

hold on  
plot(t,y1,'r')
plot(t,y2,'g')
plot(t,y3,'k') 
legend('Setpt = 7.68','5','3','1')

%% Measuring the Performance of the Closed Loop System
F = stepinfo(sys_cl);
fprintf("Closed loop settling time: %.2f (s) \n", F.SettlingTime)
fprintf("Closed loop overshoot percentage: %.2f (percent) \n", F.Overshoot) 

%% Calculating eBike Mechanical Power During Closed Loop Time Response  
v_t = y; % substituting y for v(t)

% Check Units on Aerodynamic Drag Force
power = v_t.*(0.368.*v_t.^2 + 0.008*95.05*9.8); % Power [W] calculated by multiplying ebike velocity by the 
% sum of the aerodynamic and rolling resistance forces acting on the point
% mass model of the ebike 
% Ka and C constants in this model are estimated from "Bicycle Science" 3rd Edition
% by David Gordon Wilson. eBike Mass taken from Modeling the Manually
% Controlled Bicycle Hess et al.  

% Plotting Power 
figure('Name', 'Power Plot');
yyaxis left, plot(t,power,'b')
hold on 
yyaxis right, plot(t,y,'r')
title('Ebike Mechanical Power During Step Response')
yyaxis left, ylabel('Power [W]')
yyaxis right, ylabel('Velocity [m/s]')
xlabel('Time [s]')
legend('Power','Velocity') 

% Documentation on the power rating of our motor is not available, so assuming it is on the low end of commercial ebike hub motors (250W) is a safe assumption. 
% Based on this assumption and the plot above, the tuned controller will not require more power from the eBike powertrain than what is available.
