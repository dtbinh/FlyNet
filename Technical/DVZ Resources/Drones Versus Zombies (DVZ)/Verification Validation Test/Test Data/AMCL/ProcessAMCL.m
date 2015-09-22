% AMCL comparison with VICON
% Project DVZ
% Author: Ben Zatz
% Created 2/6/2015
% Updated 2/24/2015


clear all
clc
close all
for kkk = 6

format long
date = '02_25_15';
addpath(['characterization_tests_', date]);
%% Animate( 1 = Yes, 0 = No)
animate = 0;

%% Do you want to save the error values? (1=Yes,0=No)
saveXL = 0;

%% Filter (1 = Yes, 0 = No)

filter_on = 0;

% cutoff frequency [Hz]
fcut = 100;


%% Load data
test_num = kkk;

filename = ['amcl_test',num2str(test_num),'.txt'];
[time,amcl,vicon] = parseAMCL(filename);
time = time - time(1);
ydes = 0.1; % 10 cm absolute error requirement on crosstrack error

%% Filter Implementation

if(filter_on == 1)
    
    RC = 1/(fcut*2*pi);
    amcl.x_filt = zeros(length(time), 1);
    amcl.y_filt = zeros(length(time), 1);
    amcl.x_filt(1) = amcl.x(1);
    amcl.y_filt(1) = amcl.y(1);
    for ii = 2:length(time);
        
        dt = time(ii) - time(ii - 1);
        alpha = dt/(RC + dt);
        
        amcl.x_filt(ii) = alpha*amcl.x(ii) + (1-alpha)*amcl.x_filt(ii-1);
        amcl.y_filt(ii) = alpha*amcl.y(ii) + (1-alpha)*amcl.y_filt(ii-1);
            
    end
    
    amcl.x = amcl.x_filt;
    amcl.y = amcl.y_filt;
    
    
end
%% Find location where we start moving and stops moving

% Find when movement starts and stops
move_timex = find(abs(vicon.x - vicon.x(1)) > 0.1);
move_timey = find(abs(vicon.y - vicon.y(1)) > 0.1);
stop_timex = find(abs(vicon.x - vicon.x(end)) > 0.1);
stop_timey = find(abs(vicon.y - vicon.y(end)) > 0.1);

% find minimum/maximum
move_time = min([move_timex(1), move_timey(1)]);
stop_time = max([stop_timex(end), stop_timey(end)]);

%% Predict Theta

% Maximum Likelihood estimator
theta1 = sum((amcl.x(move_time:stop_time)-vicon.x(move_time:stop_time)).*vicon.y(move_time:stop_time))...
    /sum(vicon.y(move_time:stop_time).^2);
theta2 = sum((vicon.y(move_time:stop_time)-amcl.y(move_time:stop_time)).*vicon.x(move_time:stop_time))...
    /sum(vicon.x(move_time:stop_time).^2);

% Two theta estimates
theta = mean([theta1,theta2]);

% Rotate the frame
S = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
[amcl_rotated] = S * [amcl.x';amcl.y'];

% assign to vectors
amcl.x = amcl_rotated(1,:)';
amcl.y = amcl_rotated(2,:)';


%% Plot x,y
figure
subplot(211)
plot(time(move_time:stop_time),amcl.x(move_time:stop_time),'b'),grid
hold on
xlabel('Time [s]')
ylabel('Position X [m]')
title('AMCL vs VICON')
xlim([time(move_time) time(stop_time)]);
plot(time(move_time:stop_time),vicon.x(move_time:stop_time),'g')
legend('AMCL','VICON','Location','Best')
hold off

subplot(212)
plot(time(move_time:stop_time),amcl.y(move_time:stop_time),'b'),grid
hold on
xlim([time(move_time) time(stop_time)]);
xlabel('Time [s]')
ylabel('Position Y [m]')
plot(time(move_time:stop_time),vicon.y(move_time:stop_time),'g')
legend('AMCL','VICON','Location','Best')
hold off

%% Plot 2D Position
figure
plot(amcl.x,amcl.y,'b*')
hold on
plot(vicon.x,vicon.y,'go')
plot(amcl.x(1:20),amcl.y(1:20),'r*')
plot(vicon.x(1:20),vicon.y(1:20),'ro')
hold off
axis equal
grid on 
xlabel('Position X [m]')
ylabel('Position Y [m]')
title('2D Position for AMCL vs VICON')
legend('AMCL','VICON','Location','Best')

%% Calculate errors
% Find error after movement begins
errx = vicon.x(move_time:stop_time) - amcl.x(move_time:stop_time);
erry = vicon.y(move_time:stop_time) - amcl.y(move_time:stop_time);

errx_abs = abs(vicon.x(move_time:stop_time) - amcl.x(move_time:stop_time));
erry_abs = abs(vicon.y(move_time:stop_time) - amcl.y(move_time:stop_time));

% Statistics on Error
mean_errx = mean(errx);
mean_erry = mean(erry);
median_errx = median(errx);
median_erry = median(erry);
std_errx = std(errx);
std_erry = std(erry);

errx_nomean = errx - mean_errx;
erry_nomean = erry - mean_erry;
mean_errx_nomean = mean(errx_nomean);
mean_erry_nomean = mean(erry_nomean);
median_errx_nomean = median(errx_nomean);
median_erry_nomean = median(erry_nomean);


fprintf('Error Statistics\n')
fprintf('======================================\n')
fprintf(' Stat             X           Y \n')
fprintf('======================================\n')
fprintf(' Mean [cm]:     %6.4f       %6.4f  \n', mean_errx*100, mean_erry*100)
fprintf(' Median [cm]:   %6.4f       %6.4f  \n', median_errx*100, median_erry*100);
fprintf(' Std Dev [cm]:  %6.4f       %6.4f  \n', std_errx*100, std_erry*100);
fprintf(' 3 Sigma [cm]:  %6.4f       %6.4f  \n', std_errx*100*3, std_erry*100*3);

% Plot absolute error
figure
subplot(2,1,1)
plot(time(move_time:stop_time),errx,'b')
hold on
plot(time(move_time:stop_time),mean_errx*ones(1,length(errx_abs)),'r')
plot(time(move_time:stop_time),median_errx*ones(1,length(errx_abs)),'g')

legend('Absolute','Mean','Median')
grid on
xlim([time(move_time) time(stop_time)]);
ylabel('X error [m]')
title('2D Pose Error Plots')
hold off
subplot(2,1,2)
plot(time(move_time:stop_time),erry,'b')
hold on
plot(time(move_time:stop_time),mean_erry*ones(1,length(erry_abs)),'r')
plot(time(move_time:stop_time),median_erry*ones(1,length(erry_abs)),'g')
plot(time(move_time:stop_time), ydes*ones(1, length(errx_abs)), '--k');
plot(time(move_time:stop_time), -ydes*ones(1, length(errx_abs)), '--k');

legend('Absolute','Mean','Median', 'Req')
grid on 
xlim([time(move_time) time(stop_time)]);
ylabel('Y error [m]')
xlabel('Time [s]')
hold off

% Plot histogram of error
figure
hist(errx, 100)
xlabel('X Error [m]')
title('X Error Histogram')

figure
hist(erry, 100)
xlabel('Y Error [m]')
title('Y Error Histogram')

% Plot x, y with bias removed
figure
plot(amcl.x(move_time:stop_time) + mean_errx, amcl.y(move_time:stop_time) + mean_erry, '*b');
hold on
plot(vicon.x(move_time:stop_time), vicon.y(move_time:stop_time), 'og');
hold off
axis equal
grid on 
xlabel('X [m]')
ylabel('Y [m]')
title('X-Y Position with Mean Error Removed')
legend('AMCL', 'VICON', 'Location', 'Best');

% Plot x, y error with bias removed
figure
subplot(2,1,1)
plot(time(move_time:stop_time), abs(errx_nomean), '-b')
hold on
plot(time(move_time:stop_time),mean_errx_nomean*ones(1,length(errx_abs)),'r')
plot(time(move_time:stop_time),median_errx_nomean*ones(1,length(errx_abs)),'g')
hold off
ylabel('X error [m]')
legend('Absolute', 'Mean', 'Median');
grid on 
xlim([time(move_time) time(stop_time)]);
title('2D Pose Error Plots - Mean Removed')

subplot(2,1,2)
plot(time(move_time:stop_time), erry_nomean, '-b')
hold on
plot(time(move_time:stop_time),mean_erry_nomean*ones(1,length(errx_abs)),'r')
plot(time(move_time:stop_time),median_erry_nomean*ones(1,length(errx_abs)),'g')
plot(time(move_time:stop_time), ydes*ones(1, length(errx_abs)), '--k');
plot(time(move_time:stop_time), -ydes*ones(1, length(errx_abs)), '--k');

hold off
ylabel('Y error [m]')
legend('Error', 'Mean', 'Median', 'Req.');
grid on 
xlabel('Time [s]')
xlim([time(move_time) time(stop_time)]);




%% 2D Active Plot
if animate == 1
    figure
    hold on
    xlim([-1 3]);
    ylim([-1.2 1.2]);
    xlabel('Position X [m]')
    ylabel('Position Y [m]')
    title('2D Position for AMCL vs VICON')
    for i = 1:length(time)
        plot(amcl.x(i),amcl.y(i),'b*');
        plot(vicon.x(i),vicon.y(i),'go');
    end
    hold off
    legend('AMCL','VICON','Location','Best') 
end

%% Write data to excel file
if saveXL == 1
    d = {mean_errx*100, mean_erry*100, std_errx*100*3, std_erry*100*3};
    xlswrite('AMCL parameters.xlsx', d,date,['AD',num2str(test_num+1)]);
end
end
