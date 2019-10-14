%BoxPlotFigCode.m 
% This code produces the box plot figure for the straight line test that is
% shown on the 2019 Bicycle and Motorcycle Dynamics Conference Poster 
% This code also calculates the percentage of data points that lie within
% +/-0.1m/s of the setpoint speed

clc 
clear 
close all

stdTestNo = '87'; % Standard Test Number corresponding to the desired test

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
starti = 4913; 
endi = 8458; 
setpointVal = 2.6; % [m/s] setpoint value used during the test


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


%% Loading Data Set 2 
% This is the exact same procedure for loading the first data

stdTestNo1 = '84'; 

% sprdSht = GetGoogleSpreadsheet('1l6Bvzsjhv-Rlak49-qD7AuwBcCYL-e_uhCq2TeBvfnQ');
% 
% row = find(cell2mat(sprdSht(2:end,4)) == stdTestNo);
% starti = sprdSht(row,9); 
% endi = sprdSht(row,10); 
% setpointVal = sprdSht(row,5); % [m/s] setpoint value used during the test
% FIND ROW

starti1 = 2532; 
endi1 = 4615; 
setpointVal1 = 3.7; % [m/s] setpoint value used during the test

cd('C:\Users\1tzm_000\Documents\Research\Testing\Dry Testing\Raw Data') 

rawData1 = readmatrix(['TEST_',stdTestNo1,'.txt']); 
ccState1 = rawData1(:,1);
setpoint1 = rawData1(:,2);
Tsig1 = rawData1(:,3);
input1= rawData1(:,4); % m/s
genVolt1= rawData1(:,5); % V
output1= rawData1(:,6); 
time1 = rawData1(:,7); 
loopStartState1 = rawData1(:,8);
ifState11 = rawData1(:,9);
ifState21 = rawData1(:,10);
ifState31 = rawData1(:,11);
ifState41 = rawData1(:,12);
TsigAnalogRead1 = rawData1(:,13); 
unfiltrdGenVoltage1 = rawData1(:,13);

cd('C:\Users\1tzm_000\Documents\Research\SourceCode\Testing')


%% Loading Data Set 3 

stdTestNo2 = '89'; 

% sprdSht = GetGoogleSpreadsheet('1l6Bvzsjhv-Rlak49-qD7AuwBcCYL-e_uhCq2TeBvfnQ');
% 
% row = find(cell2mat(sprdSht(2:end,4)) == stdTestNo);
% starti = sprdSht(row,9); 
% endi = sprdSht(row,10); 
% setpointVal = sprdSht(row,5); % [m/s] setpoint value used during the test
% FIND ROW

starti2 = 2848; 
endi2 = 4708; 
setpointVal2 = 5.1; % [m/s] setpoint value used during the test

cd('C:\Users\1tzm_000\Documents\Research\Testing\Dry Testing\Raw Data') 

rawData2 = readmatrix(['TEST_',stdTestNo2,'.txt']); 
ccState2 = rawData2(:,1);
setpoint2 = rawData2(:,2);
Tsig2 = rawData2(:,3);
input2= rawData2(:,4); % m/s
genVolt2= rawData2(:,5); % V
output2= rawData2(:,6); 
time2 = rawData2(:,7); 
loopStartState2 = rawData2(:,8);
ifState12 = rawData2(:,9);
ifState22 = rawData2(:,10);
ifState32 = rawData2(:,11);
ifState42 = rawData2(:,12);
TsigAnalogRead2 = rawData2(:,13); 
unfiltrdGenVoltage2 = rawData2(:,13);

cd('C:\Users\1tzm_000\Documents\Research\SourceCode\Testing')


%% Preprocessing Data For Box Plots 

% Removing Outliers
TF = ~isoutlier(input(starti:endi),'quartiles');
TF1 = ~isoutlier(input1(starti1:endi1),'quartiles');
TF2 = ~isoutlier(input2(starti2:endi2),'quartiles'); 

input(starti:endi) = TF.*input(starti:endi);
input1(starti1:endi1) = TF1.*input1(starti1:endi1);
input2(starti2:endi2) = TF2.*input2(starti2:endi2);

for i = 0:length(input(starti:endi))-1
    if input(starti+i) == 0
        input(starti+i) = nan;
    end
end

for i = 0:length(input1(starti1:endi1))-1
    if input1(starti1+i) == 0
        input1(starti1+i) = nan;
    end
end

for i = 0:length(input2(starti2:endi2))-1
    if input2(starti2+i) == 0
        input2(starti2+i) = nan;
    end
end 



%% Making Box Plots

% finding number of samples in each box 
n = num2str(length(starti:endi));
nArray = ['n = ',n];

n1 = num2str(length(starti1:endi1));
nArray1 = ['n = ',n1];

n2 = num2str(length(starti2:endi2));
nArray2 = ['n = ',n2];

figure(); 

% Setting Figure Size
pos = get(gcf, 'Position');
width = 6; % width in inches 
height = 6; % height in inches
set(gcf, 'Position', [pos(1) pos(2) width*100 height*100])

subplot(1,3,1)
b = barh(setpointVal,100,0.2,'k','LineStyle','none');
alpha(b,0.07);
hold on
boxplot(input(starti:endi)); 
yline(setpointVal,'k','LineWidth',2);  
topDiff = setpointVal + 0.1 - 2.7;
botDiff = setpointVal - 0.1 - 2.46; 
totalDiff = abs(topDiff) + abs(botDiff);
percentTotalDiff = totalDiff/0.2*100; 
%ylabel('Speed [m/s]') 
ylim([2.1 3.1]) 
%text(0.57,2.2,nArray,'FontSize',8);

subplot(1,3,2)
b = barh(setpointVal1,100,0.2,'k','LineStyle','none');
alpha(b,0.07);
hold on 
boxplot(input1(starti1:endi1))
yline(setpointVal1,'k','LineWidth',2);
ylim([3.2 4.2])
%text(0.57,3.3,nArray1,'FontSize',8);

subplot(1,3,3)
b = barh(setpointVal2,100,0.2,'k','LineStyle','none');
alpha(b,0.07);
hold on 
boxplot(input2(starti2:endi2))
yline(setpointVal2,'k','LineWidth',2);  
ylim([4.6 5.6])
%text(0.57,4.7,nArray2,'FontSize',8);

% Preserving the size of the image when saving 
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);

print('boxPlotSLine_1','-depsc2','-r600') 

%% Determining Percentage of Data Points Lying In Bounds 
% +/- 0.1m/s for each setpoint
Q1 = 2.5; Q2 = 2.7;
Q1_1 = 3.6; Q2_1 = 3.8;
Q1_2 = 5.0; Q2_2 = 5.2;

inBounds = find(input(starti:endi) > Q1 & input(starti:endi) < Q2);
percentIn = length(inBounds)/length(starti:endi)*100;

inBounds1 = find(input1(starti1:endi1) > Q1_1 & input1(starti1:endi1) < Q2_1);
percentIn1 = length(inBounds1)/length(starti1:endi1)*100;

inBounds2 = find(input2(starti2:endi2) > Q1_2 & input2(starti2:endi2) < Q2_2);
percentIn2 = length(inBounds2)/length(starti2:endi2)*100; 

percentInMean = mean([percentIn,percentIn1,percentIn2]);