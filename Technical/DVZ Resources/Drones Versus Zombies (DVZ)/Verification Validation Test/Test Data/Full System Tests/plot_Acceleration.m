close all
clear all
clc

addpath('04_24_2015')

data = load_DVZ_log('full_system_test3.txt');
dt = mean(diff(data.time));

% xacc = data.pixhawk_xacc.*(-1).*9.81/1000;
% yacc = data.pixhawk_yacc.*(1).*9.81/1000;
% zacc = data.pixhawk_zacc.*(1).*9.81/1000;

% Vx = data.local_velx_body;
% Vy = data.local_vely_body;

vicon_vx = diff(data.vicon_x)./dt;
% vicon_xacc = diff(vicon_vx)./dt;

vicon_vy = diff(data.vicon_y)./dt;
% vicon_yacc = diff(vicon_vy)./dt;

vicon_vz = diff(data.vicon_z)./dt;
% vicon_zacc = diff(vicon_vz)./dt;

% Smooth data for checking
% Vx = smooth(Vx,10);
% Vy = smooth(Vy,10);
% X Position
% h1 = figure;
% hold on
% p1 = plot(data.time,data.local_xmap,'x-b');
% p2 = plot(data.time,data.vicon_x,'r');
% legend('Local','Vicon','Location','best');
% title('Local vs Vicon (X)');
% xlabel('Time [s]');
% ylabel('X Position [m]')
% get(h1,'Position');
% set(h1,'Position',[50 570 560 420]);
% % Y Position
% h2 = figure;
% hold on
% p3 = plot(data.time,data.local_ymap,'x-b');
% p4 = plot(data.time,data.vicon_y,'r');
% legend('Local','Vicon','Location','best');
% title('Local vs Vicon (Y)');
% xlabel('Time [s]');
% ylabel('Y Position [m]')
% get(h2,'Position');
% set(h2,'Position',[640 570 560 420]);
% Local Velocity X
h3 = figure;
hold on
p5 = plot(data.time(1:end-1),data.local_velx_body(1:end-1),'x-b');
p6 = plot(data.time(1:end-1),vicon_vx,'r');
legend('Local','Vicon','Location','best');
title('Local vs Vicon (Vx)');
xlabel('Time [s]');
ylabel('X Velocity [m/s]')
get(h3,'Position');
set(h3,'Position',[50 570 560 420]);
% Local Velocity Y
h4 = figure;
hold on
p7 = plot(data.time(1:end-1),data.local_vely_body(1:end-1),'x-b');
p8 = plot(data.time(1:end-1),vicon_vy,'r');
legend('Local','Vicon','Location','best');
title('Local vs Vicon (Vy)');
xlabel('Time [s]');
ylabel('Y Velocity [m/s]')
get(h4,'Position');
set(h4,'Position',[640 570 560 420]);

% h3 = figure
% get(h3,'Position');
% set(h3,'Position',[1230 570 560 420]);
% subplot(2,1,1)
% plot(data.time(1:end-2),zacc(1:end-2),'b')
% title('Pixhawk')
% subplot(2,1,2)
% plot(data.time(1:end-2),vicon_zacc,'r')
% title('Vicon');
% xlabel('Time [s]');
% ylabel('Z Acceleration [m/s^2]')
% suptitle('Pixhawk vs. Vicon [Z]');

%% Kalman Filter Output
% X Velocity
h11 = figure;
hold on
p11 = plot(data.time(1:end-1),data.kalman_xdot_body(1:end-1),'b');
p12 = plot(data.time(1:end-1),...
    data.kalman_xdot_body(1:end-1)+...
    2*data.kalman_xdot_body_var(1:end-1).^(1/2),'g--');
p13 = plot(data.time(1:end-1),...
    data.kalman_xdot_body(1:end-1)+...
    (-2)*data.kalman_xdot_body_var(1:end-1).^(1/2),'g--');
p14 = plot(data.time(1:end-1),vicon_vx,'r');
legend([p11 p12 p14],'Kalman Xdot Body','2 Sigma Error','Vicon')
title('Kalman vs Vicon (X Velocity)')
xlabel('Time [s]');
ylabel('X Velocity [m/s]')
set(h11,'Position',[50 570 560 420]);
% Y Velocity
h21 = figure;
hold on
p21 = plot(data.time(1:end-1),data.kalman_ydot_body(1:end-1),'b');
p22 = plot(data.time(1:end-1),...
    data.kalman_ydot_body(1:end-1)+...
    2*data.kalman_ydot_body_var(1:end-1).^(1/2),'g--');
p23 = plot(data.time(1:end-1),...
    data.kalman_ydot_body(1:end-1)+...
    (-2)*data.kalman_ydot_body_var(1:end-1).^(1/2),'g--');
p24 = plot(data.time(1:end-1),vicon_vy,'r');
legend([p21 p22 p24],'Kalman Ydot Body','2 Sigma Error','Vicon')
title('Kalman vs Vicon (Y Velocity)')
xlabel('Time [s]');
ylabel('Y Velocity [m/s]')
set(h21,'Position',[640 570 560 420]);
% Z Velocity
h31 = figure;
hold on
p31 = plot(data.time(1:end-1),data.kalman_zdot_body(1:end-1),'b');
p32 = plot(data.time(1:end-1),...
    data.kalman_zdot_body(1:end-1)+...
    2*data.kalman_zdot_body_var(1:end-1).^(1/2),'g--');
p33 = plot(data.time(1:end-1),...
    data.kalman_zdot_body(1:end-1)+...
    (-2)*data.kalman_zdot_body_var(1:end-1).^(1/2),'g--');
p34 = plot(data.time(1:end-1),vicon_vz,'r');
legend([p31 p32 p34],'Kalman Zdot Body','2 Sigma Error','Vicon')
title('Kalman vs Vicon (Z Velocity)')
xlabel('Time [s]');
ylabel('Z Velocity [m/s]')
set(h31,'Position',[1230 570 560 420]);
% X Acceleration bias
h41 = figure;
hold on
p41 = plot(data.time,data.kalman_ax_bias_body,'b');
p42 = plot(data.time,...
    data.kalman_ax_bias_body+...
    2*data.kalman_ax_bias_body_var.^(1/2),'g--');
p43 = plot(data.time,...
    data.kalman_ax_bias_body+...
    (-2)*data.kalman_ax_bias_body_var.^(1/2),'g--');
legend([p41 p42],'Kalman aX bias Body','2 Sigma Error')
title('Kalman (X Acceleration)')
xlabel('Time [s]');
ylabel('X Acceleration [m/s^2]')
set(h41,'Position',[50 50 560 420]);
% Y Acceleration bias
h51 = figure;
hold on
p51 = plot(data.time,data.kalman_ay_bias_body,'b');
p52 = plot(data.time,...
    data.kalman_ay_bias_body+...
    2*data.kalman_ay_bias_body_var.^(1/2),'g--');
p53 = plot(data.time,...
    data.kalman_ay_bias_body+...
    (-2)*data.kalman_ay_bias_body_var.^(1/2),'g--');
legend([p51 p52],'Kalman aY bias Body','2 Sigma Error')
title('Kalman (Y Acceleration)')
xlabel('Time [s]');
ylabel('Y Acceleration [m/s^2]')
set(h51,'Position',[640 50 560 420]);
% Z Acceleration bias
h61 = figure;
hold on
p61 = plot(data.time,data.kalman_az_bias_body,'b');
p62 = plot(data.time,...
    data.kalman_az_bias_body+...
    2*data.kalman_az_bias_body_var.^(1/2),'g--');
p63 = plot(data.time,...
    data.kalman_az_bias_body+...
    (-2)*data.kalman_az_bias_body_var.^(1/2),'g--');
legend([p61 p62],'Kalman aZ bias Body','2 Sigma Error')
title('Kalman (Z Acceleration)')
xlabel('Time [s]');
ylabel('Z Acceleration [m/s^2]')
set(h61,'Position',[1230 50 560 420]);
% Phi bias
h71 = figure;
hold on
p71 = plot(data.time,data.kalman_phi_bias,'b');
p72 = plot(data.time,...
    data.kalman_phi_bias+...
    2*data.kalman_phi_bias_var.^(1/2),'g--');
p73 = plot(data.time,...
    data.kalman_phi_bias+...
    (-2)*data.kalman_phi_bias_var.^(1/2),'g--');
legend([p71 p72],'Kalman \phi bias','2 Sigma Error')
title('Kalman (\phi Angle)')
xlabel('Time [s]');
ylabel('\phi Angle [rad]')
set(h71,'Position',[320 270 560 420]);
% Theta bias
h81 = figure;
hold on
p81 = plot(data.time,data.kalman_theta_bias,'b');
p82 = plot(data.time,...
    data.kalman_theta_bias+...
    2*data.kalman_theta_bias_var.^(1/2),'g--');
p83 = plot(data.time,...
    data.kalman_theta_bias+...
    (-2)*data.kalman_theta_bias_var.^(1/2),'g--');
legend([p81 p82],'Kalman \theta bias','2 Sigma Error')
title('Kalman (\theta Angle)')
xlabel('Time [s]');
ylabel('\theta Angle [rad]')
set(h81,'Position',[910 270 560 420]);





