clear variables
close all
clc

%% Controller Gains
defineGains
gains = readGains('current_gains.txt');

%% Simulink Model
wp = 'hourglass';
[t,target_psi,target_x,target_y] = defineWaypoints(wp);

%%%%% Model Parameters %%%%%
% Acceleration of gravity
g = 9.81;

% Velocity Saturation Limits
vx_sat_max = 0.5;
vx_sat_min = -0.5;
vy_sat_max = 0.5;
vy_sat_min = -0.5;

% Roll and Pitch noise parameters
noise_phi_mean = 0;
noise_theta_mean = 0;
noise_phi_var = 0;%0.00001;
noise_theta_var = 0;%0.00001;

sim('controller_pid_sim_noise.slx',t);

plotWaypointPath(t,x_desired_out,y_desired_out,x_out,y_out,psi_desired_out,psi_out,wp)

% Plot xy position response
figure
subplot(231)
hold on
grid on
plot(x_desired_out.time,x_desired_out.signals.values,'b')
plot(x_out.time,x_out.signals.values,'r')
ylabel('X Position [m]')
title('X Position')
legend('Ref','Act','Location','Best')

subplot(234)
hold on
grid on
plot(y_desired_out.time,y_desired_out.signals.values,'b')
plot(y_out.time,y_out.signals.values,'r')
xlabel('Time [s]')
ylabel('Y Position [m]')
title('Y Position')
legend('Ref','Act','Location','Best')

% Plot Vx,Vy velocity response
subplot(232)
hold on
grid on
plot(vx_desired_out.time,vx_desired_out.signals.values,'b')
plot(vx_out.time,vx_out.signals.values,'r')
plot([vx_out.time(1) vx_out.time(end)],[vx_sat_max vx_sat_max],'k--')
plot([vx_out.time(1) vx_out.time(end)],[vx_sat_min vx_sat_min],'k--')
ylabel('Velocity X [m/s]')
title('Velocity X')
legend('Ref','Act','Location','Best')

subplot(235)
hold on
grid on
plot(vy_desired_out.time,vy_desired_out.signals.values,'b')
plot(vy_out.time,vy_out.signals.values,'r')
plot([vy_out.time(1) vy_out.time(end)],[vy_sat_max vy_sat_max],'k--')
plot([vy_out.time(1) vy_out.time(end)],[vy_sat_min vy_sat_min],'k--')
xlabel('Time [s]')
ylabel('Velocity Y [m/s]')
title('Velocity Y')
legend('Ref','Act','Location','Best')

% Plot phi,theta response
subplot(233)
hold on
grid on
plot(phi_desired_out.time,phi_desired_out.signals.values.*(180/pi),'b')
plot(phi_out.time,phi_out.signals.values.*(180/pi),'r')
ylabel('Phi [deg]')
title('Phi')
legend('Ref','Act','Location','Best')

subplot(236)
hold on
grid on
plot(theta_desired_out.time,theta_desired_out.signals.values.*(180/pi),'b')
plot(theta_out.time,theta_out.signals.values.*(180/pi),'r')
xlabel('Time [s]')
ylabel('Theta [deg]')
title('Theta')
legend('Ref','Act','Location','Best')
suptitle('Simulated Response')
