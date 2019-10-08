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
% These constants are initial guesses
Ka = 0.368;      % Aerodynamic Drag coefficient taken from Bicycling Science 3rd Edition
kt = 0.002;      % Motor torque constant for a 500 Kv motor
Rm = 0.15;       % [ohms] Motor resistance taken as an average from other similar motors
m = 95.05;       % [kg] Mass of bike+rider from Modeling the Manually Controlled Bicycle
J = 2*0.15;      % [kg*m^2] From Modeling the Manually Controlled Bicycle (multiplied by two to lump both front and rear wheels)   
L = 0.06;        % [H] Motor inductance taken as an average from other similar motors
Rw = 0.34;       % [m] Radius of the rear wheel

% Loading Measured Time Response Data 
cd('C:\Users\1tzm_000\Documents\Research\Testing\Dry Testing\Raw Data') 

rawData = readmatrix('TEST_85.txt'); 
ccState = rawData(:,1);
setpoint = rawData(:,2);
Tsig = rawData(:,3);
input= rawData(:,4); % m/s
genVolt= rawData(:,5); % V
output= rawData(:,6); 
time = rawData(:,7); 
loopStartState = rawData(:,8);
ifState1 = rawData(:,9);
ifState2 = rawData(:,10);
ifState3 = rawData(:,11);
ifState4 = rawData(:,12);
TsigAnalogRead = rawData(:,13); 
unfiltrdGenVoltage = rawData(:,13);
cd('C:\Users\1tzm_000\Documents\Research\SourceCode\Design') 
xdata_1 = transpose(time(1207:1635)); % indices found from plot of time response
ydata_1 = transpose(input(1207:1635));  
cd('C:\Users\1tzm_000\Documents\Research\SourceCode\Design') 

% Loading Measured Time Response Data
load('00912.mat') 
xdata = linspace(0,20,2401);  
ydata = (2*pi()/60)*(456.3862.*NIData(300:2700,7)-1.2846)*(0.341*0.028985/0.333375); %the raw data from 00912.mat must first be processed to convert it to m/s (see http://moorepants.github.io/dissertation/davisbicycle.html#calibration) 
ydata = transpose(ydata); 
stepVoltage = 4.27; % [V] hall effect sensor voltage measured with a multimeter at full throttle

%xdata = xdata_1;
%ydata = ydata_1;

% Declaring Coefficeints of Simplified Plant Model 
%-          Gp = e / (s^2 + f*s + g)            -%
a = J*L + L*Rw^2*m;
b = J*Rm + 2*Ka*L*Rw^2 + Rm*Rw^2*m;
c = kt^2 + 2*Ka*Rm*Rw^2;
d = kt*L*Rw;
e = d/a;
f = b/a;
g = c/a;

% Function that computes the time response of the transfer function. 
% This function is used in the least-squares curve fitting
% The expression below is the inverse laplace of (stepVoltage/s) * (e / s^2 + f*s + g) where x is an array containing the values of coefficients e, f and g
%time_response = @(x,xdata) 8*x(1)/x(3) - (8*x(1)*exp(-(x(2).*xdata)/2).*(cosh(xdata.*((x(2)^2)/4 - x(3))^(1/2)) + (x(2)*sinh(xdata.*((x(2)^2)/4 - x(3))^(1/2)))/(2*((x(2)^2)/4 - x(3))^(1/2))))/x(3); 
time_response = @(x,xdata) (427*x(1))/(100*x(3)) - (427*x(1)*exp(-(x(2).*xdata)/2).*(cosh(xdata.*(x(2)^2/4 - x(3))^(1/2)) + (x(2)*sinh(xdata.*(x(2)^2/4 - x(3))^(1/2)))/(2*(x(2)^2/4 - x(3))^(1/2))))/(100*x(3));

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

%% Plotting the Curve Fit %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(); 

plot(xdata,ydata,'r',t_2,time_response(x,t_2),'b', 'LineWidth', 2 )  
title('Non Linear Least Squares Curve Fit Result')
xlabel('Time [s]') 
ylabel('Speed [m/s]')
h = legend('Measured', 'Model Prediction');
rect = [0.21, -0.27, 1, 1];
set(h, ... 
    'Position', rect, ... 
    'FontSize', 12);  

%% Displaying the Identified Plant Model
Gp = tf(x(1),[1,x(2),x(3)]);
fprintf('The identified plant model is:'); display(Gp)

%% Generating Plot for Paper 
figure();

pos = get(gcf, 'Position');
width = 7; % width in inches 
height = 6; % height in inches
set(gcf, 'Position', [pos(1) pos(2) width*100 height*100])

plot(xdata,ydata,'r',t_2,time_response(x,t_2),'b', 'LineWidth', 2 )  

% Gathering and Plotting Additional Time Responses At Different Magnitudes 
%opt1 = stepDataOptions('StepAmplitude',5);
opt2 = stepDataOptions('StepAmplitude',3);
opt3 = stepDataOptions('StepAmplitude',1); 

%y1 = step(Gp,t_2,opt1);
y2 = step(Gp,t_2,opt2);
y3 = step(Gp,t_2,opt3);

hold on  
%plot(t_2,y1,'b')
plot(t_2,y2,'g')
plot(t_2,y3,'k') 

xlabel('Time [s]') 
ylabel('Speed [m/s]')
h = legend('Measured','Model Fit','Step Magnitude = 3V','1V');
rect = [-0.15, 0.3, 1, 1];
set(h, ... 
    'Position', rect, ... 
    'FontSize', 12);  

% Preserving the size of the image when saving 
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);

print('SysIDPlot','-depsc2','-r600')
