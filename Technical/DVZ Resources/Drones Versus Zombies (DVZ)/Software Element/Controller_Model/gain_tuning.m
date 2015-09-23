clear all
close all
clc

%% Controller Gains
%%%%% Current Gains: 3/9/15 %%%%%
% Roll Rate Gains: Pr_phi = 0.2*100; Ir_phi = -0.005*100; Dr_phi = 0.12*100;
% Pitch Rate Gains: Pr_theta = 0.2*100; Ir_theta = -0.005*100; Dr_theta = 0.12*100;
% Roll Angle Gains: Pa_phi = 10;
% Pitch Angle Gains: Pa_theta = 10;
% Velocity X Gains: Pvx = -0.26; Ivx = -0.05; Dvx = 0;
% Velocity Y Gains: Pvy = 0.14; Ivy = 0.05; Dvy = 0;
% Position X Gains: Px = 0.5; Ix = 0.02; Dx = 0;
% Position Y Gains: Py = 0.4; Iy = 0.03; Dy = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Roll Rate Gains
Pr_phi = 0.2*100;
Ir_phi = -0.005*100;
Dr_phi = 0.12*100;

% Pitch Rate Gains
Pr_theta = 0.2*100;
Ir_theta = -0.005*100;
Dr_theta = 0.12*100;

% Yaw Rate Gains
Pr_psi = 0.2*100;
Ir_psi = -0.005*100;
Dr_psi = 0.12*100;

% Roll Angle Gains
Pa_phi = 10;

% Pitch Angle Gains
Pa_theta = 10;

% Yaw Angle Gains
Pa_psi = 10;

% Velocity X Gains
Pvx = -0.35;
Ivx = -0.05;
Dvx = 0;

% Velocity Y Gains
Pvy = 0.35;
Ivy = 0.05;
Dvy = 0;

% Position X Gains
Px = 1.5;
Ix = 0;
Dx = 0;

% Position Y Gains
Py = 1.5;
Iy = 0;
Dy = 0;

%% Simulink Model
[t,target_psi,target_x,target_y] = defineWaypoints('figure8');

%%%%% Model Parameters %%%%%
% Acceleration of gravity
g = 9.81;

% Velocity Saturation Limits
vx_sat_max = 1;
vx_sat_min = -1;
vy_sat_max = 1;
vy_sat_min = -1;

sim('controller_pid_sim.slx',t);

% Plot xy position response
figure
subplot(231)
hold on
grid on
plot(x_desired_out.time,x_desired_out.signals.values,'b')
plot(x_out.time,x_out.signals.values,'r')
xlabel('Time [s]')
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
xlabel('Time [s]')
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
xlabel('Time [s]')
ylabel('Phi [rad]')
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

figure
hold on
grid on
plot(x_desired_out.signals.values,y_desired_out.signals.values,'b')
plot(x_out.signals.values,y_out.signals.values,'r')
for i = 1:length(t)
    p1 = plot(x_desired_out.signals.values(i),y_desired_out.signals.values(i),'go','MarkerFaceColor','g');
    p2 = plot(x_out.signals.values(i),y_out.signals.values(i),'k^','MarkerFaceColor','k');
    pause(0.01)
    xlabel('X [m]')
    ylabel('Y [m]')
    if i ~= length(t)
        delete(p1)
        delete(p2)
    end
end    
legend([p1 p2],'Target','Quad','Location','Best')
    

