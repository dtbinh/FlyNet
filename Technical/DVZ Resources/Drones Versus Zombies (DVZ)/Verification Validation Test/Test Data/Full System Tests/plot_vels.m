close all
clear all
clc

addpath('04_24_2015')

data = load_DVZ_log('full_system_test26.txt');
dt = mean(diff(data.time));

% xacc = data.pixhawk_xacc.*(-1).*9.81/1000;
% yacc = data.pixhawk_yacc.*(1).*9.81/1000;
% zacc = data.pixhawk_zacc.*(1).*9.81/1000;

% Vx = data.local_velx_body;
% Vy = data.local_vely_body;

vicon_vx = diff(data.vicon_x)./dt;
% vicon_xacc = diff(vicon_vx)./dt;

vicon_vy = diff(-data.vicon_y)./dt;
% vicon_yacc = diff(vicon_vy)./dt;

vicon_vz = diff(data.vicon_z)./dt;
% vicon_zacc = diff(vicon_vz)./dt;

vx_body_unf = data.local_velx_body(1:end-1);
vy_body_unf = data.local_vely_body(1:end-1);

vx_body_f = data.velx_body(1:end-1);
vy_body_f = data.vely_body(1:end-1);

%vx_body = smooth(vx_body, 5);
%vy_body = smooth(vy_body, 5);

%vicon_vx = smooth(vicon_vx, 5);
%vicon_vy = smooth(vicon_vy, 5);


h3 = figure;
hold on
p5 = plot(data.time(1:end-1),vx_body_unf,'x-b');
p7 = plot(data.time(1:end-1),vx_body_f,'x-g');
p6 = plot(data.time(1:end-1),vicon_vx,'r');
legend('Local_{unf}','Local_f','Vicon','Location','best');
title('Local vs Vicon (Vx)');
xlabel('Time [s]');
ylabel('X Velocity [m/s]')
get(h3,'Position');
set(h3,'Position',[50 570 560 420]);
% Local Velocity Y
h4 = figure;
hold on
p8 = plot(data.time(1:end-1),vy_body_unf,'x-b');
p10 = plot(data.time(1:end-1),vy_body_f,'x-g');
p9 = plot(data.time(1:end-1),vicon_vy,'r');
legend('Local_{unf}','Local_f','Vicon','Location','best');
title('Local vs Vicon (Vy)');
xlabel('Time [s]');
ylabel('Y Velocity [m/s]')
get(h4,'Position');
set(h4,'Position',[640 570 560 420]);