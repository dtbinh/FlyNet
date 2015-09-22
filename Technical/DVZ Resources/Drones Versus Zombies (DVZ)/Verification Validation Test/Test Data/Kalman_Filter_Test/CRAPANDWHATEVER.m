clear all
close all
clc

data = load_DVZ_log('full_system_test3.txt');

vicon_velx = diff(data.vicon_x);
vicon_vely = diff(data.vicon_y);
amcl_velx = diff(data.amcl_x);
amcl_vely = diff(data.amcl_y);

vicon_speed = sqrt(vicon_velx.^2 + vicon_vely.^2);
amcl_speed = sqrt(data.local_velx_body(2:end).^2 + data.local_vely_body(2:end).^2);

figure
hold on
grid on
plot(data.time(2:end),vicon_speed)
% plot(data.time(2:end),0.12.*amcl_speed)
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Vicon','AMCL','Location','Best')

% figure
% hold on
% grid on
% plot(data.time(2:end),vicon_velx)
% plot(data.time(2:end),data.local_vely_body(2:end))
% plot(data.time(2:end),amcl_velx)
% xlabel('Time [s]')
% ylabel('Body X Velocity [m/s]')
% legend('Vicon','AMCL','Diff AMCL','Location','Best')
% 
% figure
% hold on
% grid on
% plot(data.time(2:end),vicon_vely)
% plot(data.time(2:end),data.local_velx_body(2:end))
% plot(data.time(2:end),amcl_vely)
% xlabel('Time [s]')
% ylabel('Body Y Velocity [m/s]')
% legend('Vicon','AMCL','Diff AMCL','Location','Best')

figure
subplot(211)
plot(data.vicon_x,data.vicon_y)
xlabel('X [m]')
ylabel('Y [m]')
subplot(212)
plot(data.local_xmap,data.local_ymap)
xlabel('X [m]')
ylabel('Y [m]')

% figure
% hold on
% grid on
% plot(data.time(2:end),vicon_velx)
% plot(data.time(2:end),amcl_velx)
% xlabel('Time [s]')
% ylabel('Body X Velocity [m/s]')
% legend('Vicon','AMCL','Location','Best')
% 
% figure
% hold on
% grid on
% plot(data.time(2:end),vicon_vely)
% plot(data.time(2:end),amcl_vely)
% xlabel('Time [s]')
% ylabel('Body Y Velocity [m/s]')
% legend('Vicon','AMCL','Location','Best')
% 
% figure
% subplot(211)
% plot(data.vicon_x,data.vicon_y)
% xlabel('X [m]')
% ylabel('Y [m]')
% subplot(212)
% plot(data.amcl_x,data.amcl_y)
% xlabel('X [m]')
% ylabel('Y [m]')