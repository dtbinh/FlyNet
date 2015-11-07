% Basic Point-Mass Quad Model
% Runs the simulink model quad_6d_model.slx
% Uses 6 DOF point mass model.
% Moments of Inertia are modeled as eye(3) for point mass. 
% Based on MATLAB simulation, run the following to generate original:
% asbQuadcopterStart
%
% Required files:
%
% quad_6d_model.slx
% boldify.m (optional)
%
% Navigate to directory with the required files and
% hit ctrl+enter to run
%
% Inputs are thrust and angular acelerations.
% Outputs are many.
%
clear,clc,close('all')
% Env Params
g = 9.8; % m/s^2 for gravity accel
mass = 2; % kg for quad
inertia = eye(3); % for now point-mass inertia
% Initial Conditions
d_fn = 0;
initVb = [0,0,0];
initEuler = [0,0,0];
initAngRates = [0,0,0];
initPosLLA = [40.143237,-105.243462,1684]; % table mountain starting location
initPosNED = [0,0,-100];
% Time Vector
sim_time = 10; % seconds
sim_rate = 1e3; % Hz
t = (0:1/sim_rate:sim_time-1/sim_rate)'; % s
% Inputs
rol_acc = 0.0*ones(size(t)); % roll angular accel in rad/s^2
pit_acc = 0.001*ones(size(t)); % pitch angular accel in rad/s^2
yaw_acc = 0.01*ones(size(t)); % yaw angular accel in rad/s^2
thrust_vec = -1.01*mass*g*ones(size(t));
ang_acc_vec = [rol_acc,pit_acc,yaw_acc];
% form the input matrix
u = [thrust_vec, ang_acc_vec];
%
tic % check how long the sim takes
[t_out,x,y] = sim('quad_6d_model.slx',t,[],[t,u]); 
clc % ignore that pesky warning
toc % print how long the simulation took
%% Data Processing 
LLA_out = y(:,1:3);
NED_vel_out = y(:,4:6);
NED_pos_out = y(:,7:9);
euler_out = y(:,10:12);
body_vel_out = y(:,13:15);
body_q_out = y(:,16:18);
body_dq_out = y(:,19:21);
body_acc_out = y(:,22:24);
% NED 2D Plot
figure(1)
clf(1)
plot(NED_pos_out)
% NED 3D Plot
figure(2)
clf(2)
plot3(NED_pos_out(:,1),NED_pos_out(:,2),-NED_pos_out(:,3))
grid('on')
% LLA Plot
figure(3)
clf(3)
plot(LLA_out)
% Euler Angles Plot
figure(4)
clf(4)
plot(euler_out)
% boldify