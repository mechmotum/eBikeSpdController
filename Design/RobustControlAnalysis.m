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
% This code it based off the DC Motor Plant Uncertainty Tutorial
% https://www.mathworks.com/help/robust/examples/robustness-of-servo-controller-for-dc-motor.html

%%%%%% PLANT MODEL %%%%%% (Identified from the system identification step)
%%%
%%% v(s)/V(s) = 15.32 / (s^2 + 70.81*s + 7.34 ) 
%%% v(s)/V(s) == d / (a*s^2 + b*s + c )
%%%
%%%%%% %%%%%% %%%%%% %%%%%

% Establishing Uncertain Plant Model Based on Percentage Based Uncertainties In Plant Model Coefficeints
a = ureal('a', 1, 'Percentage',30);
b = ureal('b', 70.81, 'Percentage',30);
c = ureal('c', 7.34, 'Percentage',30);
d = ureal('d', 15.32, 'Percentage',30); 
Gp = tf(d, [a b c]); 

% Tuned Controller Acquired From ControllerTuning.m 
Gc = pid(1.03,0.145,0.05);

% Declaring The Closed Loop System
sys_cl = feedback(Gc*Gp,1); 

% Plotting Closed Loop and Open Loop Step Responses For Samples of the Uncertain Plant Model  
figure();
pos = get(gcf, 'Position');
width = 7; % width in inches 
height = 6; % height in inches
set(gcf, 'Position', [pos(1) pos(2) width*100 height*100])

step(usample(sys_cl,15),'b');

hold on 

[y2, t2] = step(sys_cl.NominalValue);
p2 = plot(t2,y2,'r','LineWidth',2.5);


title('Uncertain Plant Closed-Loop Step Response (15 Samples)', 'FontSize',14)
legend({'Uncertain Plants','Nominal Plant'},'FontSize',16,'location','SouthEast') 
ylabel('Speed (m/s)','FontSize',16,'FontName','Minion Pro')
xlabel('Time','FontSize',16)

% Saving Plot For Publishing 
publish = 0;

if publish
    % Preserving the size of the image when saving 
    set(gcf,'InvertHardcopy','on');
    set(gcf,'PaperUnits', 'inches');
    papersize = get(gcf, 'PaperSize');
    left = (papersize(1)- width)/2;
    bottom = (papersize(2)- height)/2;
    myfiguresize = [left, bottom, width, height];
    set(gcf,'PaperPosition', myfiguresize);

    print('RobustnessPlot_700dpi','-depsc2','-r700') 
end

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