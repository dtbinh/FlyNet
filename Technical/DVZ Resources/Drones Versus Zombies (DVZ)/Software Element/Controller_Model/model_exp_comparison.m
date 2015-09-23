clear variables
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
Pvx = -0.26;
Ivx = -0.05;
Dvx = 0;

% Velocity Y Gains
Pvy = 0.14;
Ivy = 0.05;
Dvy = 0;

% Position X Gains
Px = 0.5;
Ix = 0.02;
Dx = 0;

% Position Y Gains
Py = 0.4;
Iy = 0.03;
Dy = 0;

%% Simulink Model
% Load experimental data
data1 = load('test data/pos_test17.txt');
data2 = load('test data/vel_test17.txt');

exp_time = data1(:,1) - data1(1,1);
exp_target_phi = data2(:,12);
exp_phi = data1(:,13);
exp_target_theta = data2(:,6);
exp_theta = data1(:,14);
exp_target_velx = data2(:,20);
exp_target_vely = data2(:,21);
exp_velx = data2(:,22);
exp_vely = data2(:,23);
exp_target_x = data1(:,16);
exp_target_y = data1(:,17);
exp_x = data1(:,10);
exp_y = data1(:,11);

expx.time = exp_time;
expx.signals.values = exp_target_x;
expx.signals.dimensions = 1;
expy.time = exp_time;
expy.signals.values = exp_target_y;
expy.signals.dimensions = 1;

%%%%% Model Paramters %%%%%
% Acceleration of gravity
g = 9.81;

% Velocity Saturation Limits
vx_sat_max = 0.2;
vx_sat_min = -0.2;
vy_sat_max = 0.2;
vy_sat_min = -0.2;

tfinal = exp_time(end);
sim('controller_pid_sim_model_exp_comp.slx',tfinal);

%% Plot Model and Experimental Data Comparison
% Compare experimental and model phi
figure
subplot(221)
hold on
grid on
plot(phi_desired_out.time,phi_desired_out.signals.values.*(180/pi),'b')
plot(exp_time,exp_target_phi,'r')
xlabel('Time [s]')
ylabel('Target Roll [deg]')
title('Roll: Model vs Experimental')
legend('Model','Exp','Location','Best')
subplot(223)
hold on
grid on
plot(phi_out.time,phi_out.signals.values.*(180/pi),'b')
plot(exp_time,exp_phi.*(180/pi),'r')
xlabel('Time [s]')
ylabel('Roll [deg]')
legend('Model','Exp','Location','Best')

% Compare experimental and model theta
subplot(222)
hold on
grid on
plot(theta_desired_out.time,theta_desired_out.signals.values.*(180/pi),'b')
plot(exp_time,exp_target_theta,'r')
xlabel('Time [s]')
ylabel('Target Pitch [deg]')
title('Pitch: Model vs Experimental')
legend('Model','Exp','Location','Best')
subplot(224)
hold on
grid on
plot(phi_out.time,phi_out.signals.values.*(180/pi),'b')
plot(exp_time,exp_theta.*(180/pi),'r')
xlabel('Time [s]')
ylabel('Pitch [deg]')
legend('Model','Exp','Location','Best')

% Compare experimental and model x velocity
figure
subplot(221)
hold on
grid on
plot(vx_desired_out.time,vx_desired_out.signals.values,'b')
plot(exp_time,exp_target_velx,'r')
xlabel('Time [s]')
ylabel('Target Vel_x [m/s]')
title('X Velocity: Model vs Experimental')
legend('Model','Exp','Location','Best')
subplot(223)
hold on
grid on
plot(vx_out.time,vx_out.signals.values,'b')
plot(exp_time,exp_velx,'r')
xlabel('Time [s]')
ylabel('Vel_x [m/s]')
legend('Model','Exp','Location','Best')

% Compare experimental and model y velocity
subplot(222)
hold on
grid on
plot(vy_desired_out.time,vy_desired_out.signals.values,'b')
plot(exp_time,exp_target_vely,'r')
xlabel('Time [s]')
ylabel('Target Vel_y [m/s]')
title('Y Velocity: Model vs Experimental')
legend('Model','Exp','Location','Best')
subplot(224)
hold on
grid on
plot(vy_out.time,vy_out.signals.values,'b')
plot(exp_time,exp_vely,'r')
xlabel('Time [s]')
ylabel('Vel_y [m/s]')
legend('Model','Exp','Location','Best')

% Compare experimental and model x position
figure
subplot(221)
hold on
grid on
plot(x_desired_out.time,x_desired_out.signals.values,'b')
plot(exp_time,exp_target_x,'r')
xlabel('Time [s]')
ylabel('Target X [m]')
title('X Position: Model vs Experimental')
legend('Model','Exp','Location','Best')
subplot(223)
hold on
grid on
plot(x_out.time,x_out.signals.values,'b')
plot(exp_time,exp_x,'r')
xlabel('Time [s]')
ylabel('X [m]')
legend('Model','Exp','Location','Best')

% Compare experimental and model y position
subplot(222)
hold on
grid on
plot(y_desired_out.time,y_desired_out.signals.values,'b')
plot(exp_time,exp_target_y,'r')
xlabel('Time [s]')
ylabel('Target Y [m]')
title('Y Position: Model vs Experimental')
legend('Model','Exp','Location','Best')
subplot(224)
hold on
grid on
plot(y_out.time,y_out.signals.values,'b')
plot(exp_time,exp_y,'r')
xlabel('Time [s]')
ylabel('Y [m]')
legend('Model','Exp','Location','Best')
