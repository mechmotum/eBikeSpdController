% testAnalyzer.m  
% This code plots and analyzes diagnostic data from the cruise control
% system for review and performance analysis. This code begins by parsing
% and storing data from a diagnostics file recorded on the microSD card
% module of the cruise control system. Important data from the run is then
% plotted and the accuracy and precision errors of the controller are
% calculated. Finally, a plot for publishing is prepared. Note that this code 
% can be modified to plot or calculate anything associated with the
% diagnostic data.

% Doing the all clear
clc 
clear 
close all

stdTestNo = '94'; % Standard Test Number corresponding to the desired test

% Using custom function to grab data from a google spreadsheet. Note this
% is still in development
% sprdSht = GetGoogleSpreadsheet('1l6Bvzsjhv-Rlak49-qD7AuwBcCYL-e_uhCq2TeBvfnQ');
% 
% row = find(cell2mat(sprdSht(2:end,4)) == stdTestNo);
% starti = sprdSht(row,9); 
% endi = sprdSht(row,10); 
% setpointVal = sprdSht(row,5); % [m/s] setpoint value used during the test
% FIND ROW

% start and endi are indices measured from viewing a plot of the speed
% data from the run
starti = 1233; 
endi = 1469; 
setpointVal = 5.1; % [m/s] setpoint value used during the test

% Switching to directory containing the raw data from the microSD card
% module. Note: this directory may need to updated to work with your
% computer
cd('C:\Users\1tzm_000\Documents\Research\Testing\Dry Testing\Raw Data') 

% Parsing and storing data
rawData = readmatrix(['TEST_',stdTestNo,'.txt']); 
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

% Switching back to directory where the script is at
cd('C:\Users\1tzm_000\Documents\Research\SourceCode\Testing')

%% Plotting Output and Input Response of The Entire Data Set
figure();

subplot(2,1,1)
plot(time, setpoint, 'g','LineWidth', 1.25), ylabel('Speed [m/s]')
hold on 
plot(time, input,'b') 
setpoint1 = setpointVal + 0.1;
yline(setpoint1,'k--','LineWidth',1.25);
setpoint2 = setpointVal - 0.1;
yline(setpoint2,'k--','LineWidth',1.25);

legend('Setpoint','Input','Setpoint + 0.1m/s','Setpoint - 0.1m/s') 
xlabel('Time [s]')

subplot(2,1,2)
plot(time, output, 'r'), ylabel('Voltage [V]')
xlabel('Time [s]')
legend('Output') 

%% Analyzing Run Performance 

% Calculating Cruise Control Accuracy and Precision
errors = input(starti:endi) - mean(setpoint(starti:endi)); % Calculates the error between setpoint and actual speed  

meanError = mean(errors);
fprintf('Mean Error = %.4f [m/s]\n',meanError);
stdDevError = std(errors); 
fprintf('Std Dev Error = %.4f [m/s]\n',stdDevError);

%% Plotting Figure for Conference Paper 
% NOTE: Only works for Test 84
% SetpointVal = 4.5
figure(); 

starti1 = 5136;
endi1 = 7796;
plot(time(starti1:endi1), setpoint(starti1:endi1), 'r','LineWidth', 1.25)
hold on 
plot(time(starti1:endi1), input(starti1:endi1),'b') 
setpoint1 = setpointVal + 0.1;
yline(setpoint1,'k--','LineWidth',1.25);
setpoint2 = setpointVal - 0.1;
yline(setpoint2,'k--','LineWidth',1.25);

leg = legend('Setpoint Speed','Measured Speed','Setpoint + 0.1m/s','Setpoint - 0.1m/s');
set(leg,... 
    'Location', 'SouthEast',...
    'FontSize', 12);
ylabel('Speed [m/s]', 'FontSize', 10)
xlabel('Time [s]', 'FontSize', 10) 
xlim([223 330]) 

print('exampleDataPlot','-dpng','-r300');