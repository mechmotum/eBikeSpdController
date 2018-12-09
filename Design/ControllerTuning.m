% Program for tuning a controller to get a desired time response 
clear 
clc

% Declaring the Plant Model and System 
Gp = tf(8.308,[1,70.83,7.542]); % identified model from measured time response of the system 
% controlSystemDesigner('rlocus',Gp)
% pidTuner(Gp,'pi')
C = pid(68.5,106,1.44);
sys_cl = feedback(C*Gp,1);

% Simulating the System
t = linspace(0,5,2000); 
y = step(Gp,t); 
y2 = step(sys_cl,t); 
figure()
% plot(t,y,'r')
hold on 
plot(t,y2,'b')  

% Plot Settings
grid on
title('System Simulation: Step Response')
xlabel('time (s)')
ylabel('speed (m/s)')
% legend('uncompensated open loop step response', 'compensated closed loop step response') 

