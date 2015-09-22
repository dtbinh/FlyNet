clear variables
close all
clc

addpath('04_26_2015')

data = load_DVZ_log2('full_system_test7.txt');
dt = mean(diff(data.time));

% Calculate vicon velocities
vicon_vx_inertial = diff(data.vicon_x)./dt;
vicon_vy_inertial = diff(data.vicon_y)./dt;
vicon_vz_inertial = diff(data.vicon_z)./dt;

for i = 1:length(data.vicon_yaw)-1
    R = [cos(data.vicon_yaw(i)), sin(data.vicon_yaw(i));...
         sin(data.vicon_yaw(i)), -cos(data.vicon_yaw(i))];
    vicon_vel_body(:,i) = R*[vicon_vx_inertial(i); vicon_vy_inertial(i)];
end

acc(1:3,:) = [-data.pixhawk_xacc'*(9.81/1000);...
              -data.pixhawk_yacc'*(9.81/1000);...
              -data.pixhawk_zacc'*(9.81/1000)];
euler(1:2,:) = [data.pixhawk_roll';...
                data.pixhawk_pitch'];
euler_rate(1:2,:) = [data.pixhawk_rollspeed';...
                     data.pixhawk_pitchspeed'];
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
%     [phi,no_bias]
%     [theta,no_bias]
%     [phi_dot]
%     [theta_dot]
num_state = 15;

% Drag coefficient
mu = 0.77;

% Mass of quad [kg]
m = 2.5;

% Gravity [m/s^2]
g = 9.81;

% State matrix
F = [(1-(mu/m)*dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -g*dt, 0, 0;...
     0, (1-(mu/m)*dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, g*dt, 0, 0, 0;...
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;...
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, dt, 0;...
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, dt;...
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;...
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
     
% Process noise matrix
Gamma = [1, 0, 0, 0, 0;...
         1, 0, 0, 0, 0;...
         1, 0, 0, 0, 0;...
         0, 1, 0, 0, 0;...
         0, 1, 0, 0, 0;...
         0, 1, 0, 0, 0;...
         0, 0, 1, 0, 0;...
         0, 0, 1, 0, 0;...
         0, 0, 0, 0, 0;...
         0, 0, 0, 0, 0;...
         0, 0, 0, 0, 0;...
         0, 0, 0, 1, 0;...
         0, 0, 0, 1, 0;...
         0, 0, 0, 0, 1;...
         0, 0, 0, 0, 1];

% Measurement matrix
H = [1/dt, 0, 0, 1, 0, 0, 0, 0, -1/dt, 0, 0, 0, 0, 0, 0;...
     0, 1/dt, 0, 0, 1, 0, 0, 0, 0, -1/dt, 0, 0, 0, 0, 0;...
     0, 0, 1/dt, 0, 0, 1, 0, 0, 0, 0, -1/dt, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0;...
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;...
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
 
% Process noise covariance matrix
sig_vel_body = 1e-3;
sig_acc_bias = 1e-6*1;
sig_euler_bias = 1e-6;
sig_euler_model = 1e-4;
sig_euler_rate_model = 1e-4;
Q = blkdiag(sig_vel_body, sig_acc_bias, sig_euler_bias, sig_euler_model, sig_euler_rate_model);

% Measurement noise covariance matrix
sig_acc = 1e-3*10000;
sig_euler = 1e-4*0.001;
sig_euler_rate = 1e-4*0.001;
R = [sig_acc, 0, 0, 0, 0, 0, 0;...
     0, sig_acc, 0, 0, 0, 0, 0;...
     0, 0, sig_acc, 0, 0, 0, 0;...
     0, 0, 0, sig_euler, 0, 0, 0;...
     0, 0, 0, 0, sig_euler, 0, 0;...
     0, 0, 0, 0, 0, sig_euler_rate, 0;...
     0, 0, 0, 0, 0, 0, sig_euler_rate];
 
% Initialize Kalman Filter variables
xhat(:,1) = zeros(num_state,1);
Phat{1} = eye(num_state).*0.5;
xhat_bnd(:,1) = diag(Phat{1});
for k = 1:length(data.time)-2
    z(:,k) = [acc(:,k); euler(:,k); euler_rate(:,k)];
    
    % Propagation step
    xbar = F*xhat(:,k);
    Pbar = F*Phat{k}*F' + Gamma*Q*Gamma';
    
    % Measurement update step
    zbar = H*xbar;
    Pxz = Pbar*H';
    Pzz = H*Pbar*H' + R;
    W = Pxz*inv(Pzz);
    
    eta = z(:,k) - zbar;
    
    xhat(:,k+1) = xbar + W*eta;
    Phat{k+1} = (eye(num_state) - W*H)*Pbar*(eye(num_state) - W*H)' + W*R*W';
    
    xhat_bnd(:,k+1) = diag(Phat{k+1});
end

% Plot velocities
figure
subplot(221)
grid on
hold on
p1 = plot(data.time(1:end-1),vicon_vel_body(1,:));
p2 = plot(data.time(1:end-1),xhat(1,:));
p3 = plot(data.time(1:end-1),xhat(1,:) + 2*sqrt(xhat_bnd(1,:)),'g--');
plot(data.time(1:end-1),xhat(1,:) - 2*sqrt(xhat_bnd(1,:)),'g--')
ylabel('X Body Velocity [m/s]')
legend([p1 p2 p3],'Vicon','Kalman','+/-2\sigma','Location','Best')
title('Body Velocities')
subplot(222)
grid on
hold on
plot(data.time(1:end-1),vicon_vel_body(1,:) - xhat(1,:))
ylabel('X Body Vel Diff [m/s]')
subplot(224)
grid on
hold on
plot(data.time(1:end-1),vicon_vel_body(2,:) - xhat(2,:))
xlabel('Time [s]')
ylabel('Y Body Vel Diff [m/s]')
subplot(223)
grid on
hold on
plot(data.time(1:end-1),vicon_vel_body(2,:))
plot(data.time(1:end-1),xhat(2,:))
plot(data.time(1:end-1),xhat(2,:) + 2*sqrt(xhat_bnd(2,:)),'g--')
plot(data.time(1:end-1),xhat(2,:) - 2*sqrt(xhat_bnd(2,:)),'g--')
xlabel('Time [s]')
ylabel('Y Body Velocity [m/s]')


return
% Plot accelerometer biases
figure
subplot(311)
grid on
hold on
p1 = plot(data.time(1:end-1),xhat(4,:));
p2 = plot(data.time(1:end-1),xhat(4,:) + 2*sqrt(xhat_bnd(4,:)),'g--');
plot(data.time(1:end-1),xhat(4,:) - 2*sqrt(xhat_bnd(4,:)),'g--')
ylabel('acc_x [m/s^2]')
legend([p2], '+/-2\sigma','Location','Best')
title('Accelerometer biases')
subplot(312)
grid on
hold on
plot(data.time(1:end-1),xhat(5,:))
plot(data.time(1:end-1),xhat(5,:) + 2*sqrt(xhat_bnd(5,:)),'g--')
plot(data.time(1:end-1),xhat(5,:) - 2*sqrt(xhat_bnd(5,:)),'g--')
ylabel('acc_y [m/s^2]')
subplot(313)
grid on
hold on
plot(data.time(1:end-1),xhat(6,:))
plot(data.time(1:end-1),xhat(6,:) + 2*sqrt(xhat_bnd(6,:)),'g--')
plot(data.time(1:end-1),xhat(6,:) - 2*sqrt(xhat_bnd(6,:)),'g--')
xlabel('Time [s]')
ylabel('acc_z [m/s^2]')

% Plot euler angle biases
figure
subplot(211)
grid on
hold on
p1 = plot(data.time(1:end-1),xhat(7,:));
p2 = plot(data.time(1:end-1),xhat(7,:) + 2*sqrt(xhat_bnd(7,:)),'g--');
plot(data.time(1:end-1),xhat(7,:) - 2*sqrt(xhat_bnd(7,:)),'g--')
ylabel('\phi bias [rad]')
legend([p2],'+/-2\sigma','Location','Best')
title('Euler Angle Biases')
subplot(212)
grid on
hold on
plot(data.time(1:end-1),xhat(8,:))
plot(data.time(1:end-1),xhat(8,:) + 2*sqrt(xhat_bnd(8,:)),'g--')
plot(data.time(1:end-1),xhat(8,:) - 2*sqrt(xhat_bnd(8,:)),'g--')
xlabel('Time [s]')
ylabel('\theta bias [rad]')
