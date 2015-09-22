%% Battery Test Analysis
close all; clear all; clc; format compact;

%% Test 1 data and plot: heavy payload
data1 = csvread('battery_info_heavy.csv',1,0);
time1 = data1(:,1);       % Time in seconds
voltage1 = data1(:,2)./1000;    % Voltage in Volts
current1 = data1(:,3)./100;    % Current in Amps

% Zero first time:
time1 = time1-time1(1);

figure
plot(time1,voltage1,'r',time1,current1,'k')
title('Heavy Payload')
legend('Voltage','Current')
xlabel('Time, seconds')

%% Test 2 data and plot: Light payload
data2 = csvread('battery_info_light.txt',1,0);
time2 = data2(:,1);       % Time in seconds
voltage2 = data2(:,2)./1000;    % Voltage in Volts
current2 = data2(:,3)./100;    % Current in Amps

% Zero first time:
time2 = time2-time2(1);

figure
plot(time2,voltage2,'r',time2,current2,'k')
title('Light Payload')
legend('Voltage','Current')
xlabel('Time, seconds')

%% Statistical Analysis:
% find mean current and voltage over last 30 sec of each flight:
t1_start = find(time1>time1(end)-30,1);
t2_start = find(time2>time2(end)-30,1);

starttime1 = time1(t1_start)
mean_Voltage1 = mean(voltage1(t1_start:end))
std_Voltage1 = std(voltage1(t1_start:end))
mean_Current1 = mean(current1(t1_start:end))
std_Current1 = std(current1(t1_start:end))

starttime2 = time2(t2_start)
mean_Voltage2 = mean(voltage2(t2_start:end))
std_Voltage2 = std(voltage2(t2_start:end))
mean_Current2 = mean(current2(t2_start:end))
std_Current2 = std(current2(t2_start:end))

