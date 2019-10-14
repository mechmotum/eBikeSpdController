%BoxPlotFigCode.m 
clc 
clear 
close all
% same as dry tester except make it convinient to enter test number and
% good data indices 

stdTestNo = '87'; 

% sprdSht = GetGoogleSpreadsheet('1l6Bvzsjhv-Rlak49-qD7AuwBcCYL-e_uhCq2TeBvfnQ');
% 
% row = find(cell2mat(sprdSht(2:end,4)) == stdTestNo);
% starti = sprdSht(row,9); 
% endi = sprdSht(row,10); 
% setpointVal = sprdSht(row,5); % [m/s] setpoint value used during the test
% FIND ROW

starti = 4913; 
endi = 8458; 
setpointVal = 2.6; % [m/s] setpoint value used during the test

% - Start Code From Dry Tester - %
% code to analyze dry test data

cd('C:\Users\1tzm_000\Documents\Research\Testing\Dry Testing\Raw Data') 

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

cd('C:\Users\1tzm_000\Documents\Research\SourceCode\Testing')

% figure();
% 
% subplot(2,1,1)
% plot(time, setpoint, 'g'), ylabel('Speed [m/s]')
% hold on 
% plot(time, input,'b') 
% setpoint1 = setpointVal + 0.1;
% yline(setpoint1);
% setpoint2 = setpointVal - 0.1;
% yline(setpoint2);
% legend('Setpoint','Input') 
% 
% subplot(2,1,2)
% plot(time, output, 'r'), ylabel('Voltage [V]')
% xlabel('Time [s]')
% legend('Output') 

% - End Code From Dry Tester - % 

%% Loading Data 1

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

% - Start Code From Dry Tester - %
% code to analyze dry test data

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

% figure();
% 
% subplot(2,1,1)
% plot(time, setpoint, 'g'), ylabel('Speed [m/s]')
% hold on 
% plot(time, input,'b') 
% setpoint1 = setpointVal + 0.1;
% yline(setpoint1);
% setpoint2 = setpointVal - 0.1;
% yline(setpoint2);
% legend('Setpoint','Input') 
% 
% subplot(2,1,2)
% plot(time, output, 'r'), ylabel('Voltage [V]')
% xlabel('Time [s]')
% legend('Output') 

% - End Code From Dry Tester - % 

%% Loading Data 2 

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

% - Start Code From Dry Tester - %
% code to analyze dry test data

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

% figure();
% 
% subplot(2,1,1)
% plot(time, setpoint, 'g'), ylabel('Speed [m/s]')
% hold on 
% plot(time, input,'b') 
% setpoint1 = setpointVal + 0.1;
% yline(setpoint1);
% setpoint2 = setpointVal - 0.1;
% yline(setpoint2);
% legend('Setpoint','Input') 
% 
% subplot(2,1,2)
% plot(time, output, 'r'), ylabel('Voltage [V]')
% xlabel('Time [s]')
% legend('Output') 

% - End Code From Dry Tester - % 

%% Preprocessing Data For Box Plots 

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

% input(starti:endi) = rmoutliers(input(starti:endi),'quartiles');
% input1(starti1:endi1) = rmoutliers(input1(starti1:endi1),'quartiles');
% input2(starti2:endi2) = rmoutliers(input2(starti2:endi2),'quartiles');


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


 %% Error Bars 
% 
% % Calculating Accuracy and Precision
% errors = input(starti:endi) - mean(setpoint(starti:endi)); % Calculates the error between setpoint and actual speed 
% errors = abs(errors); % make all differences positive 
% meanError = mean(errors);
% stdDevError = std(errors); 
% 
% errors1 = input1(starti1:endi1) - mean(setpoint1(starti1:endi1)); % Calculates the error between setpoint and actual speed 
% errors1 = abs(errors1); % make all differences positive 
% meanError1 = mean(errors1);
% stdDevError1 = std(errors1); 
% 
% errors2 = input2(starti2:endi2) - mean(setpoint2(starti2:endi2)); % Calculates the error between setpoint and actual speed 
% errors2 = abs(errors2); % make all differences positive 
% meanError2 = mean(errors2);
% stdDevError2 = std(errors2); 
% 
% %% Plotting Error Bars
% figure(); 
% 
% subplot(3,1,1)
% errorbar([meanError,meanError1,meanError2],[stdDevError*3,stdDevError1*3,stdDevError2*3],'o')
% set(gca,'XTick',[1 2 3],'XTickLabel',{'Sp = 2m/s','Sp = 2.6m/s','Sp = 3.7m/s'}); 
% xlim([0 4]);
% ylim([-0.5 0.8]) 
% yline(0.1); yline(-0.1);
% 
% 
% % subplot(1,3,2) 
% % boxplot(input1(starti1:endi1),repmat('MedSpdSp',length(input1(starti1:endi1)),1))
% % hold on 
% % yline(setpointVal1,'--k','Setpoint','LineWidth',2);
% % 
% % subplot(1,3,3)
% % boxplot(input2(starti2:endi2),repmat('HighSpdSp',length(input2(starti2:endi2)),1))
% % hold on 
% % yline(setpointVal2,'--k','Setpoint','LineWidth',2); 
