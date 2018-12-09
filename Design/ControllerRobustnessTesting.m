%% Testing the Robustness of The Controller By Changing The Plant Model 

clear 
clc 
close()
figure();

% Controller 
C = pid(68.5,106,1.44);

% Plant model constants from initial system identification
a = 1; b = 70.83; c = 7.542; d = 8.308;

for x=1:3

% Calculates new constants in the plant model that are +/- 10% of the original value 
% NOTE: in the future make original values a, b, c, d static vars so that they do not need to be hardcoded
if x == 1 
    a = 1 - 6.5*1;
    b = 70.83 - 6.5*70.83;
    c = 7.542 - 6.5*7.542;
    d = 8.308 - 6.5*8.308;
elseif x == 2
    a = 1;
    b = 70.83;
    c = 7.542;
    d = 8.308;
elseif x == 3 
    a = 1 + 0.5*1;
    b = 70.83 + 0.5*70.83;
    c = 7.542 + 0.5*7.542;
    d = 8.308 + 0.5*8.308;
end 

% Declaring the Plant Model and System 
Gp = tf(d,[a,b,c]); 
sys_cl = feedback(C*Gp,1); % Creates a unity feedback loop model

% Simulating the System
t = linspace(0,5,2000); 
y = step(sys_cl,t); 
% y2 = step(Gp,t);
hold on 
subplot(3,1,x);
plot(t,y,'b')  
% plot(t,y2,'b')

% Plot Settings
grid on 
if x == 2
    title('Compensated System Simulation With Original Plant Model: Unit Step Response') 
else 
    title('Compensated System Simulation With Pertubed Plant Model: Unit Step Response') 
end
xlabel('time (s)')
ylabel('speed (m/s)') 

% Storing Stats
if x ==1
S = stepinfo(sys_cl);
elseif x==2
    D = stepinfo(sys_cl);
else
    F = stepinfo(sys_cl);
end

end  

% Displaying stats 
fprintf("Settling Times\n");
disp(S.SettlingTime)
disp(D.SettlingTime)
disp(F.SettlingTime)
fprintf("\n Overshoots \n");
disp(S.Overshoot)
disp(D.Overshoot)
disp(F.Overshoot)

%% Check to see if uniform changes to the plant model will change the closed loop step response 
close()
clear

syms a b c d s ki kd kp

%Known Plant Model
Gp = d / (a*s^2 + b*s + c);

%Uncertain Plant Models
Gp1 = (d+1) / ((a+1)*s^2 + (b+1)*s + (c+1));
Gp2 = (d+5) / ((a+5)*s^2 + (b+1)*s + (c+4));

%Controller
Gc = kp + (ki/s) + kd*s;

%ClosedLoopTFs
CLTF = (Gp*Gc) / (1+Gp*Gc);
CLTF1 = (Gp1*Gc) / (1+Gp1*Gc);  
CLTF2 = (Gp2*Gc) / (1+Gp2*Gc);

%Initializing Coefficients
a = 1; b = 70.83; c = 7.542; d = 8.308; ki = 106; kd = 1.44; kp = 68.5; 

%Making TransferFunctions
CLTFtf = tf( [ (d*kd) (d*kp) (d*ki) ] , [a (b+d*kd) (c+d*kp) d*ki] );
CLTF1tf = tf( [ (kd+d*kd) (kp+d*kp) (ki+d*ki) ] , [ (a+1) (b+kd+d*kd+1) (c+kp+d*kp+1) (ki+d*ki) ]);
CLTF2tf = tf( [ (5*kd + d*kd) (5*kp + d*kp) (5*ki + d*ki) ] , [ (a+5) (b+5*kd+d*kd+1) (c+5*kp+d*kp+4) (5*ki+d*ki) ] );

%PlottingStepResponses
figure();
subplot(3,1,1)
step(CLTFtf)
subplot(3,1,2)
step(CLTF1tf)
subplot(3,1,3)

%GettingSTEPINFO
S = stepinfo(CLTFtf);
fprintf("SettlingTimes\n");
display(S.SettlingTime)
S1 = stepinfo(CLTF1tf);
display(S1.SettlingTime);
S2 = stepinfo(CLTF2tf);
display(S2.SettlingTime); 

%% Using MATLAB Robust Control Toolbox 
% Based off DC Motor Plant Uncertainty Tutorial
% https://www.mathworks.com/help/robust/examples/robustness-of-servo-controller-for-dc-motor.html

clear 
clc 

%%%%%%%%%%%% Establishing The Plant Model %%%%%%%%%%%% 
% Identified model from measured time response of the system 
% Gp = tf(8.308,[1,70.83,7.542])

% Establishing Uncertain Plant Model
a = ureal('a', 1, 'Percentage',30);
b = ureal('b', 70.83, 'Percentage',30);
c = ureal('c', 7.542, 'Percentage',30);
d = ureal('d', 8.308, 'Percentage',30); 
Gp = tf(d, [a b c]); 

% Tuned Controller From ControllerTuning.m 
C = pid(68.5,106,1.44);

% Closed Loop System
sys_cl = feedback(C*Gp,1); 

% Plotting Time Responses 
subplot(2,1,1); step(Gp.NominalValue,'r-+',usample(Gp,20),'b',3), title('Plant response (20 samples)')
subplot(2,1,2); step(sys_cl.NominalValue,'r-+',usample(sys_cl,20),'b',3), title('Closed-loop response (20 samples)')
legend('Nominal','Samples')

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





