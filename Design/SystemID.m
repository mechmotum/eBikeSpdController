                   %%% <System Identification> %%% 

%%%% -- This is the first code used in the controller design process -- %%%%

%%% DESCRIPTION
%%% This code identifies the constants of the derived ebike plant model using a least-sqaures curve fitting method
%%%  based on initial guesses of the plant model constants 
%%%   This code requires 00912.mat data file in order to run

%%%%%% Doing the all clear %%%%%%
clear
clc
close all

%%%%%% PLANT MODEL %%%%%% (Derived from first principles)
%%%
%%% v(s)/V(s) = Kt*L*Rw / (J*L + L*Rw^2*m)*s^2 + (J*Rm + 2*Ka*L*Rw^2 + Rm*Rw^2*m)*s + Kt^2 + 2*Ka*Rm*Rw^2 ) 
%%%
%%%%%% %%%%%% %%%%%% %%%%%

% Initializing Plant Model Constants 
Ka = 0.368; 
kt = 0.002;  
Rm = 0.15;  
m = 95.05; 
J = 0.15;   
L = 0.06;   
Rw = 0.34; 

% Loading Measured Time Response Data
load('00912.mat') 
xdata = linspace(0,20,2401);  
ydata = (2*pi()/60)*(456.3862.*NIData(300:2700,7)-1.2846)*(0.341*0.028985/0.333375); %the raw data from 00912.mat must first be processed to convert it to m/s (see http://moorepants.github.io/dissertation/davisbicycle.html#calibration) 
ydata = transpose(ydata);

% Declaring Coefficeints of Simplified Plant Model 
% Gp = e / (s^2 + f*s + g)
a = J*L + L*Rw^2*m;
b = J*Rm + 2*Ka*L*Rw^2 + Rm*Rw^2*m;
c = kt^2 + 2*Ka*Rm*Rw^2;
d = kt*L*Rw;
e = d/a;
f = b/a;
g = c/a;

% Function that computes the time response of the transfer function. 
% This function is used in the least-squares curve fitting
% The expression below is the inverse laplace of 8*e / s^3 + f*s^2 + g*s where x is an array containing the values of coefficients e, f and g
time_response = @(x,xdata) 8*x(1)/x(3) - (8*x(1)*exp(-(x(2).*xdata)/2).*(cosh(xdata.*((x(2)^2)/4 - x(3))^(1/2)) + (x(2)*sinh(xdata.*((x(2)^2)/4 - x(3))^(1/2)))/(2*((x(2)^2)/4 - x(3))^(1/2))))/x(3); 

% Initial Guess
x0 = [e, f, g]; 

% Plotting The Initial Guess Against the Measured Data 
figure()
t_2 = linspace(xdata(1), xdata(end));
plot(xdata,ydata,'r',t_2,time_response(x0,t_2),'b') 

title('Closed Loop Step Response of Plant Model With Initial Guesses vs Measured Time Response')
xlabel('Time [s]') 
ylabel('Speed [m/s]')
legend('Measured Time Response', 'Plant Model With Initial Guesses')

% Curve Fitting Using Least-Squares Method
x = lsqcurvefit(time_response,x0,xdata,ydata);
fprintf("The optimized coefficients:"); display(x); fprintf("/n");

% Plotting the Curve Fit
figure();
plot(xdata,ydata,'r',t_2,time_response(x,t_2),'b', 'LineWidth', 2 )  
title('Curvefit Result vs Measured Time Response')
xlabel('Time [s]') 
ylabel('Speed [m/s]')
legend('Measured Time Response', 'Curvefit')

% Displaying the Identified Plant Model
Gp = tf(x(1),[1,x(2),x(3)]);
fprintf('The identified plant model is:'); display(Gp)
