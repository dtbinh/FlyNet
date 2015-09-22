%% Battery Test Analysis
% 
% Summary:
% This program plots data from a test flight using a 4S battery to ensure that maximum allowable power for motors and esc's was not exceeded.
% Prior to this test 4S Lipo batteries were thought to possibly be the cause of earlier ESC and Motor failures.
% The quad was flown manually and flown more harshly than normal autonomous operation expectations.
% The results show that the 2013 X8 is compatible with 4S Lipo batteries. Since this test, 4S batteries have 
% been used on the X8 regularly and cause no problems.
%
close all; clear all; clc; format compact;

%% Test 1 data and plot: heavy payload
data1 = load('4S_battery_test.txt');
time1 = data1(:,1);       % Time in seconds
voltage1 = data1(:,2)./1000;    % Voltage in Volts
current1 = data1(:,3)./100;    % Current in Amps
batt_remaining1 = data1(:,4);

% Zero first time:
time1 = time1-time1(1);

% Power Analysis:
P = voltage1.*current1; % Power in Watts
Max_Power = max(P)/8    % Max power per Motor
Max_Current = max(current1)/8 % Max current per motor
Avg_Power = mean(P)

figure
plot(time1,voltage1,'b',time1,current1,'k', time1,P,'r')
title('Voltage and Current')
legend('Voltage (V)','Current (A)','Power (W)')
xlabel('Time, seconds')


% %% Test 2 data and plot: Light payload
% data2 = csvread('battery_info_light.txt',1,0);
% time2 = data2(:,1);       % Time in seconds
% voltage2 = data2(:,2)./1000;    % Voltage in Volts
% current2 = data2(:,3)./100;    % Current in Amps
% 
% % Zero first time:
% time2 = time2-time2(1);
% 
% figure
% plot(time2,voltage2,'r',time2,current2,'k')
% title('Light Payload')
% legend('Voltage','Current')
% xlabel('Time, seconds')
% 
% %% Statistical Analysis:
% % find mean current and voltage over last 30 sec of each flight:
% t1_start = find(time1>time1(end)-30,1);
% t2_start = find(time2>time2(end)-30,1);
% 
% starttime1 = time1(t1_start)
% mean_Voltage1 = mean(voltage1(t1_start:end))
% std_Voltage1 = std(voltage1(t1_start:end))
% mean_Current1 = mean(current1(t1_start:end))
% std_Current1 = std(current1(t1_start:end))
% 
% starttime2 = time2(t2_start)
% mean_Voltage2 = mean(voltage2(t2_start:end))
% std_Voltage2 = std(voltage2(t2_start:end))
% mean_Current2 = mean(current2(t2_start:end))
% std_Current2 = std(current2(t2_start:end))
% 
% 
% 
