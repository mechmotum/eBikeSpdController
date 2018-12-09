% Code for Testing Various Constants 
% Reads Values for Each Constant From a csv File and Then Plots a Graph For Each Set of Constants

clear
clc

% Initializing Constants 
constants = csvread('constants_1.csv',0,1);
Ka = constants(1,:); 
kt = constants(2,:);  
Rm = constants(3,:);  
m = constants(4,:); 
J = constants(5,:);   
L = constants(6,:);   
Rw = constants(7,:);  

t = linspace(0,20,10000); % time for model
t_1 = linspace(0,20,3701); % time for measured data 

for INDEX = 1:length(Ka) 
    
    % Declaring Plant Model
    s = time_response('s');
    Gp_num = kt(INDEX)*L(INDEX)*Rw(INDEX);
    Gp_den = (J(INDEX)*L(INDEX)+L(INDEX)*(Rw(INDEX)^2)*m(INDEX))*s^2+(J(INDEX)*Rm(INDEX)+2*Ka(INDEX)*L(INDEX)*(Rw(INDEX)^2)+Rm(INDEX)*(Rw(INDEX)^2)*m(INDEX))*s+(kt(INDEX)^2)+2*Ka(INDEX)*Rm(INDEX)*(Rw(INDEX)^2);
    fprintf(' ------------------- Index = %d ----------------- \n',INDEX);
    Gp = (Gp_num/Gp_den) 
    
    % Generating step and Ramp Responses
    figure()
    y = step(8*Gp,t);   % multiplying Gp by 8 to scale the step response
    plot(t,y,'b')
    hold on  
    
    augGp = series(Gp,(1/s));
    y_ramp = step(augGp,t);   % generates a ramp response 
    plot(t,y_ramp,'g')
    
    % Plotting Measured Data
    load('00912.mat') 
    BikeSpeed = (2*pi()/60)*(456.3862.*NIData(300:4000,7)-1.2846)*(0.341*0.028985/0.333375);
    plot(t_1,BikeSpeed,'r') 
    
    % Plot Settings
    title('Plot of System Time Responses')
    legend( 'Model Step Response', 'Model Ramp Response','Measured Time Response') 
    ylabel('Speed (m/s)')
    xlabel('Time (s)') 
    grid on 
    
    % Printing the Inverse Laplace of Gp 
    syms s 
    Gp_num_s = kt(INDEX)*L(INDEX)*Rw(INDEX);
    Gp_dem_s = (J(INDEX)*L(INDEX)+L(INDEX)*(Rw(INDEX)^2)*m(INDEX))*s^2+(J(INDEX)*Rm(INDEX)+2*Ka(INDEX)*L(INDEX)*(Rw(INDEX)^2)+Rm(INDEX)*(Rw(INDEX)^2)*m(INDEX))*s+(kt(INDEX)^2)+2*Ka(INDEX)*Rm(INDEX)*(Rw(INDEX)^2);
    Gp_ramp = (Gp_num_s/Gp_dem_s) * (1/s^2);
    Gp_step = (Gp_num_s/Gp_dem_s) * (1/s);
    fprintf('Time Response with Ramp Input: \n')
    ramp_t_response = ilaplace(Gp_ramp);
    display(vpa(ramp_t_response,4)) 
    fprintf('Time Response with Step Input: \n')
    step_t_response = ilaplace(Gp_step);
    display(vpa(step_t_response,4)) 

end 

%% Using lsqcurvefit 
clear
clc
constants = csvread('constants_1.csv',0,1);
Ka = constants(1,:); 
kt = constants(2,:);  
Rm = constants(3,:);  
m = constants(4,:); 
J = constants(5,:);   
L = constants(6,:);   
Rw = constants(7,:); 
load('00912.mat') 

% Measured Data
xdata = linspace(0,20,2401); % has enough points to match the length of measured speeds 
ydata = (2*pi()/60)*(456.3862.*NIData(300:2700,7)-1.2846)*(0.341*0.028985/0.333375); 
ydata = transpose(ydata);

INDEX = 3;
% Declaring Coefficeints
a = J(INDEX)*L(INDEX)+L(INDEX)*Rw(INDEX)^2*m(INDEX);
b = J(INDEX)*Rm(INDEX)+2*Ka(INDEX)*L(INDEX)*Rw(INDEX)^2+Rm(INDEX)*Rw(INDEX)^2*m(INDEX);
c = kt(INDEX)^2+2*Ka(INDEX)*Rm(INDEX)*Rw(INDEX)^2;
d = kt(INDEX)*L(INDEX)*Rw(INDEX);
e = d/a;
f = b/a;
g = c/a;

% Function that computes the time response of the transfer function. The expression below is the inverse laplace of 8*e / s^3 + f*s^2 + g*s SEE Pg 29 OF NOTEBOOK
time_response = @(x,xdata) 8*x(1)/x(3) - (8*x(1)*exp(-(x(2).*xdata)/2).*(cosh(xdata.*((x(2)^2)/4 - x(3))^(1/2)) + (x(2)*sinh(xdata.*((x(2)^2)/4 - x(3))^(1/2)))/(2*((x(2)^2)/4 - x(3))^(1/2))))/x(3); 

% Initial Guess
x0 = [e, f, g]; 

% Plotting Result 
figure()
t_2 = linspace(xdata(1), xdata(end));
plot(xdata,ydata,'r',t_2,time_response(x0,t_2),'b') 
hold on 

% Curve Fitting 
x = lsqcurvefit(time_response,x0,xdata,ydata);
display(x) 
plot(xdata,ydata,'r',t_2,time_response(x,t_2),'g')  

% Showing Identified Plant Model
Gp = tf(x(1),[1,x(2),x(3)]);
fprintf('The identified plant model of the measured data is:')
display(Gp)

% Plot Settings
title('lsq curve fit result')
xlabel('time (s)')
ylabel('speed (m/s)')
legend('Measured Data','Original Guess x0','lsq result x') 

% [y, t_3] = step(8*Gp);
% plot(t_3,y,'ob');
