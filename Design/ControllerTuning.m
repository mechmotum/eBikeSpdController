   %%% <Controller Tuning> %%% 

%%%% -- This is the second code used in the controller design process -- %%%%

%%% DESCRIPTION
%%% This code uses the Control System Toolbox to tune a PID controller to acheive zero steady error 
%%%  and reasonable transient behavior for a unity feedback control architecture 

%%%%%% Doing the all clear %%%%%%
clear
clc
close all

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

% pidTuner(Gp,'pid')
% controlSystemDesigner('rlocus',Gp)

%%%%%% Declaring the Control Architecture %%%%%%
Gc = pid(68.5,106,1.44); % PID constants found from using controller tuning app
sys_cl = feedback(Gc*Gp,1);

% Simulating the Closed Loop Step Response
t = linspace(0,5,2000); 
y = step(sys_cl,t); 

figure()
plot(t,y,'b')  

% Plot Settings
grid on
title('Closed Loop System Step Response')
xlabel('Time [s]')
ylabel('Speed [m/s]') 

% Measuring the Performance of the Closed Loop System
F = stepinfo(sys_cl);
fprintf("Closed loop settling time: %.2f (s) \n", F.SettlingTime)
fprintf("Closed loop overshoot percentage: %.2f (precent) \n", F.Overshoot)

