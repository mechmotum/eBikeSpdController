   %%% <Controller Tuning> %%% 

%%%% -- This is the second code used in the controller design process -- %%%%

%%% DESCRIPTION
%%% This code uses the Control System Toolbox to tune a PID controller to acheive zero steady error 
%%%  and reasonable transient behavior for a unity feedback control architecture 

%%%%%% Doing the all clear %%%%%%
clear
clc
close all

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

%%%%%% PLANT MODEL %%%%%% (Identified from the system identification step)
%%%
%%% v(s)/V(s) = 8.31 / (s^2 + 70.82*s + 7.543 ) 
%%%
%%%%%% %%%%%% %%%%%% %%%%%

% Declaring the Plant Model
Gp = tf(8.308,[1,70.83,7.542]); 

%%%%%% Using the Control System Toolbox to tune a PID controller %%%%%% 
% INSTRUCTIONS
% Uncomment either of the following lines of code to launch either app for tuning a PID controller

pidTuner(Gp,'pid')
% controlSystemDesigner('rlocus',Gp)

%%%%%% Declaring the Control Architecture %%%%%%
%Gc = pid(4.18,0.884,0); % PID constants found from using controller tuning app
% Gc = pid(0.363,0.0955,0); % PID constants found from using controller tuning app
% Gc = pid(0.74,0.0908,0); % PID constants found from using controller tuning app
Gc = pid(1.01,0.108,0); % PID constants found from using controller tuning app
sys_cl = feedback(Gc*Gp,1);

% Simulating the Closed Loop Step Response
t = linspace(0,20,8000); 
opt = stepDataOptions('StepAmplitude',spdFinal);
y = step(sys_cl,t,opt); 

figure('Name','Comparing Measured to Tuned Performance')
plot(t,y,'b') 
hold on 
plot(xdata,ydata,'r')
title('Comparison Plot Between Measured and Tuned Time Response')
xlabel('Time [s]')
ylabel('Speed [m/s]') 

% Measuring the Performance of the Closed Loop System
F = stepinfo(sys_cl);
fprintf("Closed loop settling time: %.2f (s) \n", F.SettlingTime)
fprintf("Closed loop overshoot percentage: %.2f (precent) \n", F.Overshoot)

