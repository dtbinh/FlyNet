
%% Clean up
clear all
close all
clc


%% Load data

addpath('04_28_2015');

data = load('vicon_test1.txt');

time = data(:,1) - data(1,1);
vicon_x = data(:,2);
vicon_y = data(:,3);
vicon_z = data(:,4);
vicon_roll = data(:,5);
vicon_pitch = data(:,6);
vicon_yaw = data(:,7);

%% Sampling Rate Subdivision

sample_division = 1;

%% Differentiate

% average time between samples
dt_vec = diff(time);
fprintf('Sample Rate: %4.2f Hz \n', mean(1./dt_vec));

% Body Euler Angles
body_roll = vicon_roll;
body_pitch = -vicon_pitch;
body_yaw = -vicon_yaw;

% Smooth vicon position
vicon_x = smooth(vicon_x, 100);
vicon_y = smooth(vicon_y, 100);
vicon_z = smooth(vicon_z, 100);

for ii = 1:length(dt_vec)
	
	dt = dt_vec(ii);
	% Vicon velocities
	vicon_velx = diff(vicon_x)./dt;
	vicon_vely = diff(vicon_y)./dt;
	vicon_velz = diff(vicon_z)./dt;
	
	% Vicon accelerations
	vicon_accx = diff(vicon_velx)./dt;
	vicon_accy = diff(vicon_vely)./dt;
	vicon_accz = diff(vicon_velz)./dt;
end

% %% Filter the accelerations
%
% Fs = mean(1./dt_vec);
%
% Nf = 50;
% Fpass = 20;
% Fstop = 30;
%
% d = designfilt('differentiatorfir', 'FilterOrder', Nf, ...
% 	'PassbandFrequency', Fpass, 'StopbandFrequency', Fstop, ...
% 	'SampleRate', Fs);
%
% figure
% fvtool(d, 'MagnitudeDisplay', 'zero-phase', 'Fs', Fs)
%
% dt_avg = mean(dt_vec);
%
% vicon_velxf = filter(d, vicon_x)/dt_avg;
% vicon_velyf = filter(d, vicon_y)/dt_avg;
%
% vicon_accxf = filter(d, vicon_velxf)/dt_avg;
% vicon_accyf = filter(d, vicon_velyf)/dt_avg;
%
% vicon_accx = smooth(vicon_accx, 10000);
% vicon_accy = smooth(vicon_accy, 10000);
% vicon_accz = smooth(vicon_accz, 10000);

%% Differentiate the position

for ii = 2:length(vicon_x) - 1
	
	dt1 = dt_vec(ii-1);
	dt2 = dt_vec(ii);
	
	dt_scale = 1/(1/2*dt1 + 1/2*dt2);
	
	vicon_velx(ii) = (vicon_x(ii+1) - vicon_x(ii-1))/(dt1+dt2);
	vicon_vely(ii) = (vicon_y(ii+1) - vicon_y(ii-1))/(dt1+dt2);
	vicon_velz(ii) = (vicon_z(ii+1) - vicon_z(ii-1))/(dt1+dt2);
	
	
	vicon_accx(ii) = dt_scale * ((vicon_x(ii+1) - vicon_x(ii))/dt2 - (vicon_x(ii) - vicon_x(ii-1))/dt1);
	vicon_accy(ii) = dt_scale * ((vicon_y(ii+1) - vicon_y(ii))/dt2 - (vicon_y(ii) - vicon_y(ii-1))/dt1);
	vicon_accz(ii) = dt_scale * ((vicon_z(ii+1) - vicon_z(ii))/dt2 - (vicon_z(ii) - vicon_z(ii-1))/dt1);
	
	
	
	
end

smooth_window = 100;

vicon_velx = smooth(vicon_velx, smooth_window);
vicon_vely = smooth(vicon_vely, smooth_window);
vicon_velz = smooth(vicon_velz, smooth_window);

vicon_accx = smooth(vicon_accx, smooth_window);
vicon_accy = smooth(vicon_accy, smooth_window);
vicon_accz = smooth(vicon_accz, smooth_window);

vicon_velx = vicon_velx(100:end);
vicon_vely = vicon_vely(100:end);
vicon_velz = vicon_velz(100:end);

vicon_accx = vicon_accx(100:end);
vicon_accy = vicon_accy(100:end);
vicon_accz = vicon_accz(100:end);

vicon_roll = vicon_roll(100:end);
vicon_pitch = vicon_pitch(100:end);
vicon_yaw = vicon_yaw(100:end);

body_roll = body_roll(100:end);
body_pitch = body_pitch(100:end);
body_yaw = body_yaw(100:end);

dt_vec = dt_vec(100:end);


figure
subplot(311)
plot(vicon_velx)

subplot(312)
plot(vicon_vely)

subplot(313)
plot(vicon_velz)

figure
subplot(311)
plot(vicon_accx)

subplot(312)
plot(vicon_accy)

subplot(313)
plot(vicon_accz)


%% Rotate to body frame

% Body velocities and accelerations
cnt = 1;
for ii = 1:sample_division:length(vicon_accx)
	
	% Rotation Matrix from vicon to body
	R_vicon2body = [cos(vicon_yaw(ii)), sin(vicon_yaw(ii)), 0;
					sin(vicon_yaw(ii)), -cos(vicon_yaw(ii)), 0;
					0, 0, -1];
	
	% Body velocity vector
	body_vel(:,cnt) = R_vicon2body*[vicon_velx(ii); vicon_vely(ii); vicon_velz(ii)];
	
	% Body acceleration vector
	body_acc(:,cnt) = R_vicon2body*[vicon_accx(ii); vicon_accy(ii); vicon_accz(ii)];
	
	% Update counter
	cnt = cnt + 1;
	
end

% Change dt to reflect subsampling
dt_mean  = mean(dt_vec)*sample_division;
fprintf('Sub-sample Rate: %4.2f Hz \n', 1/dt_mean);


%% Kalman Filter


% State
% x = [xdot,cur]
%     [ydot,cur]
%     [zdot,cur]
%     [acc_x,body,bias]
%     [acc_y,body,bias]
%     [acc_z,body,bias]
%     [phi,bias]
%     [theta,bias]
%     [xdot,prev]
%     [ydot,prev]
%     [zdot,prev]

% Drag coefficient
mu = 0.77;

% Mass of quad [kg]
m = 2.5;

% Gravity [m/s^2]
g = 9.81;

% Process noise covariance matrix
sig_euler = 1e-3;%1e-7;
sig_acc_bias = 1e-2;%;1e-6;
sig_euler_bias = 1e-4;%;1e-6;
Q = blkdiag(sig_euler, sig_acc_bias, sig_euler_bias);

% Initialize Kalman Filter variables
xhat(:,1) = zeros(11,1);
Phat{1} = eye(11);
xhat_bnd(:,1) = diag(Phat{1});

% Measurement noise covariance
sig_acc = 1e-1;%1e-3;
R = blkdiag(eye(3).*sig_acc);

for k = 1:length(body_acc)
	
	dt = dt_vec(k);
	
	% State matrix
% 	F = [(1-(mu/m)*dt), 0, 0, 0, 0, 0, 0, g*dt, 0, 0, 0;...
% 		0, (1-(mu/m)*dt), 0, 0, 0, 0, -g*dt, 0, 0, 0, 0;...
% 		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;...
% 		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;...
% 		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;...
% 		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;...
% 		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;...
% 		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;...
% 		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
% 		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
% 		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0];
% 	% Input matrix
% 	G = [0, -g*dt;...
% 		g*dt, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0;...
% 		0, 0];
% % Process noise matrix
% 	Gamma = [1, 0, 0;...
% 		1, 0, 0;...
% 		1, 0, 0;...
% 		0, 1, 0;...
% 		0, 1, 0;...
% 		0, 1, 0;...
% 		0, 0, 1;...
% 		0, 0, 1;...
% 		0, 0, 0;...
% 		0, 0, 0;...
% 		0, 0, 0];
	
	% State matrix
	F = [(1-(mu/m)*dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
		0, (1-(mu/m)*dt), 0, 0, 0, 0, 0, 0, 0, 0, 0;...
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;...
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;...
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;...
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;...
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;...
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;...
		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0];
	% Input matrix
	G = [0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0;...
		0, 0];
	% Process noise matrix
	Gamma = [1, 0, 0;...
		1, 0, 0;...
		1, 0, 0;...
		0, 1, 0;...
		0, 1, 0;...
		0, 1, 0;...
		0, 0, 0;...
		0, 0, 0;...
		0, 0, 0;...
		0, 0, 0;...
		0, 0, 0];
	
	H = [1/dt, 0, 0, 1, 0, 0, 0, 0, -1/dt, 0, 0;...
		0, 1/dt, 0, 0, 1, 0, 0, 0, 0, -1/dt, 0;...
		0, 0, 1/dt, 0, 0, 1, 0, 0, 0, 0, -1/dt];
	
	% Measurements
	z(:,k) = body_acc(:,k);
	
	% Input
	u(:,k) = [body_roll(k); body_pitch(k)];
	
	% Propagation step
	xbar = F*xhat(:,k) + G*u(:,k);
	Pbar = F*Phat{k}*F' + Gamma*Q*Gamma';
	
	% Measurement update step
	zbar = H*xbar;
	Pxz = Pbar*H';
	Pzz = H*Pbar*H' + R;
	W = Pxz*inv(Pzz);
	
	% Innovation
	eta = z(:,k) - zbar;
	
	% Normalized Innovation Squared
	NIS(k) = eta'*inv(Pzz)*eta;
	
	% Update State Estimate
	xhat(:,k+1) = xbar + W*eta;
	Phat{k+1} = (eye(11) - W*H)*Pbar*(eye(11) - W*H)' + W*R*W';
	
	xhat_bnd(:,k+1) = diag(Phat{k+1});
end

%% NIS Test
alpha = 0.05;
ep = mean(NIS)
r1 = chi2inv(alpha/2,5*length(NIS))/length(NIS)
r2 = chi2inv(1 - (alpha/2),5*length(NIS))/length(NIS)



%% Plot Results

plot_time = time(1:sample_division:end);
plot_time_full = time(1:end-1);


% Plot velocities
% X Body velocity
% figure
% subplot(211)
% grid on
% hold on
% p1 = plot(plot_time_full,vicon_velx);
% p2 = plot(plot_time,xhat(1,:));
% % p3 = plot(plot_time,xhat(1,:) + 2*sqrt(xhat_bnd(1,:)),'g--');
% % plot(plot_time,xhat(1,:) - 2*sqrt(xhat_bnd(1,:)),'g--')
% ylabel('X Body Velocity [m/s]')
% legend([p1 p2],'Vicon','Kalman','+/-2\sigma','Location','Best')
% title('Body Velocities')
% 
% % Y Body Velocity
% subplot(212)
% grid on
% hold on
% plot(plot_time_full,vicon_vely)
% plot(plot_time,xhat(2,:))
% % plot(plot_time,xhat(2,:) + 2*sqrt(xhat_bnd(2,:)),'g--')
% % plot(plot_time,xhat(2,:) - 2*sqrt(xhat_bnd(2,:)),'g--')
% xlabel('Time [s]')
% ylabel('Y Body Velocity [m/s]')


figure
subplot(211)
grid on
hold on
p1 = plot(vicon_velx);
p2 = plot(xhat(1,:));
% p3 = plot(plot_time,xhat(1,:) + 2*sqrt(xhat_bnd(1,:)),'g--');
% plot(plot_time,xhat(1,:) - 2*sqrt(xhat_bnd(1,:)),'g--')
ylabel('X Body Velocity [m/s]')
legend([p1 p2],'Vicon','Kalman','+/-2\sigma','Location','Best')
title('Body Velocities')

% Y Body Velocity
subplot(212)
grid on
hold on
plot(vicon_vely)
plot(xhat(2,:))
% plot(plot_time,xhat(2,:) + 2*sqrt(xhat_bnd(2,:)),'g--')
% plot(plot_time,xhat(2,:) - 2*sqrt(xhat_bnd(2,:)),'g--')
xlabel('Time [s]')
ylabel('Y Body Velocity [m/s]')


% Plot accelerometer biases
% X Body acceleration bias
figure
subplot(311)
grid on
hold on
plot(xhat(4,:))
% p1 = plot(plot_time,xhat(4,:));
% p2 = plot(plot_time,xhat(4,:) + 2*sqrt(xhat_bnd(4,:)),'g--');
% plot(plot_time,xhat(4,:) - 2*sqrt(xhat_bnd(4,:)),'g--')
ylabel('acc_x [m/s^2]')
% legend([p2], '+/-2\sigma','Location','Best')
title('Accelerometer biases')

% Y Body acceleration bias
subplot(312)
grid on
hold on
plot(xhat(5,:))
% plot(plot_time,xhat(5,:))
% plot(plot_time,xhat(5,:) + 2*sqrt(xhat_bnd(5,:)),'g--')
% plot(plot_time,xhat(5,:) - 2*sqrt(xhat_bnd(5,:)),'g--')
ylabel('acc_y [m/s^2]')

% Z Body acceleration bias
subplot(313)
grid on
hold on
plot(xhat(6,:))
% plot(plot_time,xhat(6,:))
% plot(plot_time,xhat(6,:) + 2*sqrt(xhat_bnd(6,:)),'g--')
% plot(plot_time,xhat(6,:) - 2*sqrt(xhat_bnd(6,:)),'g--')
xlabel('Time [s]')
ylabel('acc_z [m/s^2]')

% Plot euler angle biases
figure
subplot(211)
grid on
hold on
plot(xhat(7,:))
% p1 = plot(plot_time,xhat(7,:));
% p2 = plot(plot_time,xhat(7,:) + 2*sqrt(xhat_bnd(7,:)),'g--');
% plot(plot_time,xhat(7,:) - 2*sqrt(xhat_bnd(7,:)),'g--')
ylabel('\phi bias [rad]')
% legend([p2],'+/-2\sigma','Location','Best')
title('Euler Angle Biases')
subplot(212)
grid on
hold on
plot(xhat(8,:))
% plot(plot_time,xhat(8,:))
% plot(plot_time,xhat(8,:) + 2*sqrt(xhat_bnd(8,:)),'g--')
% plot(plot_time,xhat(8,:) - 2*sqrt(xhat_bnd(8,:)),'g--')
xlabel('Time [s]')
ylabel('\theta bias [rad]')
























