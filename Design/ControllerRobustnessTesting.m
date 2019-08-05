       %%% <Controller Robustness Testing> %%% 

%%%% -- This is the third code used in the controller design process -- %%%%

%%% DESCRIPTION
%%% This code uses the Robust Control Toolbox to simulate uncertainties in 
%%%  the identified plant model to evaluate the robustness of the tuned 
%%%   controller found in the second code of this series

%%%%%% Doing the all clear %%%%%%
clear
clc
close all

% Using MATLAB Robust Control Toolbox 
% Based off DC Motor Plant Uncertainty Tutorial
% https://www.mathworks.com/help/robust/examples/robustness-of-servo-controller-for-dc-motor.html

%%%%%% PLANT MODEL %%%%%% (Identified from the system identification step)
%%%
%%% v(s)/V(s) = 8.31 / (s^2 + 70.82*s + 7.543 ) 
%%% v(s)/V(s) == d / (a*s^2 + b*s + c )
%%%
%%%%%% %%%%%% %%%%%% %%%%%

% Establishing Uncertain Plant Model Based on Percentage Based Uncertainties In Plant Model Coefficeints
a = ureal('a', 1, 'Percentage',30);
b = ureal('b', 70.83, 'Percentage',30);
c = ureal('c', 7.542, 'Percentage',30);
d = ureal('d', 8.308, 'Percentage',30); 
Gp = tf(d, [a b c]); 

% Tuned Controller Acquired From ControllerTuning.m 
% Gc = pid(68.5,106,1.44);
Gc = pid(1.01,0.108,0);

% Closed Loop System
sys_cl = feedback(Gc*Gp,1); 

% Plotting Closed Loop and Open Loop Step Responses For Samples of the Uncertain Plant Model 
%subplot(2,1,1); step(Gp.NominalValue,'r-+',usample(Gp,20),'b',3), title('Plant response (20 samples)')
%subplot(2,1,2); 
figure();
step(sys_cl.NominalValue,'r-+',usample(sys_cl,20),'b',3), title('Closed-loop response (20 samples)')
legend('Nominal','Samples')

%% Looking at discturbance rejection performance

% Robustness of Disturbance Rejections 
S = feedback(1,Gp*C); % Sensitivity Function
bodemag(usample(S,20),'b',S.Nominal,'r-+');
legend('Samples','Nominal')

% Time Responses for Step Disturbances 
step(usample(S,20),'b',S.Nominal,'r-+',3);
title('Disturbance Rejection')
legend('Samples','Nominal') 

% Looking at Worst Case Scenario 
om = logspace(-1,2,80);
Sg = ufrd(S,om);
[maxgain,worstuncertainty] = wcgain(Sg);
Sworst = usubs(S,worstuncertainty);
Sgworst = frd(Sworst,Sg.Frequency);
norm(Sgworst,inf)
maxgain.LowerBound
step(Sworst,'b',S.NominalValue,'r-+',6);
title('Disturbance Rejection')
legend('Worst-case','Nominal')





