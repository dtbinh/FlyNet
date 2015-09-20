clear variables
close all
clc

addpath('04_26_2015')

data = load_DVZ_log2('full_system_test14.txt');
dt = mean(diff(data.time));
% 04_26_2015/full_system_test12.txt = long symmetric hallway, x down length
% of hallway
% 04_26_2015/full_system_test13.txt = long symmetric hallway, 45 deg down length
% of hallway

% Calculate vicon velocities
vicon_vx_inertial = diff(data.vicon_x)./dt;
vicon_vy_inertial = diff(data.vicon_y)./dt;
vicon_vz_inertial = diff(data.vicon_z)./dt;

for i = 1:length(data.vicon_yaw)-1
    R = [cos(data.vicon_yaw(i)), sin(data.vicon_yaw(i));...
         sin(data.vicon_yaw(i)), -cos(data.vicon_yaw(i))];
    vicon_vel_body(:,i) = R*[vicon_vx_inertial(i); vicon_vy_inertial(i)];
end

vicon_vel_body(1,:) = smooth(vicon_vel_body(1,:),10);
vicon_vel_body(2,:) = smooth(vicon_vel_body(2,:),10);

acc(1:3,:) = [-data.pixhawk_xacc'*(9.81/1000);...
              -data.pixhawk_yacc'*(9.81/1000);...
              -data.pixhawk_zacc'*(9.81/1000)];
u(1:2,:) = [data.pixhawk_roll';...
            data.pixhawk_pitch'];
% State
% x = [xdot,cur]
%     [ydot,cur]
%     [zdot,cur]
%     [xddot,cur]
%     [yddot,cur]
%     [zddot,cur]
%     [acc_x,body,bias]
%     [acc_y,body,bias]
%     [acc_z,body,bias]
%     [phi,bias]
%     [theta,bias]


% Drag coefficient
mu = 2.3;

% Mass of quad [kg]
m = 2.5;

% Gravity [m/s^2]
g = 9.81;

% State matrix
 F = [(1-mu/m*dt), 0, 0, dt, 0, 0, 0, 0, 0, 0, 0;
     0, (1-mu/m*dt),0, 0, dt, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g;
     0, 0, 0, 0, 0, 0, 0, 0, 0, -g, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
% Input matrix
G = [0, 0;
     0, 0;
     0, 0;
     0, -g;
     g, 0;
     0, 0;
     0, 0;
     0, 0;
     0, 0;
     0, 0;
     0, 0];
% Process noise matrix
Gamma = [1, 0 ,0, 0;...
         1, 0, 0, 0;...
         1, 0, 0, 0;...
         0, 1, 0, 0;...
         0, 1, 0, 0;...
         0, 1, 0, 0;...
         0, 0, 1, 0;...
         0, 0, 1, 0;...
         0, 0, 1, 0;...
         0, 0, 0, 1;...
         0, 0, 0, 1];
% Measurement matrix
H = [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0;...
     0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0;...
     0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0;...
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
 
% Process noise covariance matrix
sig_euler = 1e-3*10;
sig_acc_proc = 1e-3*10;
sig_acc_bias = 1e-6*0.00001;
sig_euler_bias = 1e-6*0.00001;
Q = blkdiag(sig_euler, sig_acc_proc, sig_acc_bias, sig_euler_bias);

% Initialize Kalman Filter variables
xhat(:,1) = zeros(11,1);
Phat{1} = eye(11,11);
xhat_bnd(:,1) = diag(Phat{1});

for k = 1:length(data.time)-2
    % Measurement noise covariance matrix
    sig_acc = 1e-3*100;
    sig_laser = 2000;
    R = blkdiag(eye(3).*sig_acc,sig_laser.*[data.kalman_xdot_body_var(k), data.kalman_zdot_body_var(k); data.kalman_zdot_body_var(k), data.kalman_ydot_body_var(k)]);

    z(:,k) = [acc(:,k); data.kalman_xdot_body(k); data.kalman_ydot_body(k)];
    
    % Propagation step
    xbar = F*xhat(:,k) + G*u(:,k);
    Pbar = F*Phat{k}*F' + Gamma*Q*Gamma';
    
    % Measurement update step
    zbar = H*xbar;
    Pxz = Pbar*H';
    Pzz = H*Pbar*H' + R;
    W = Pxz*inv(Pzz);
    
    eta = z(:,k) - zbar;
    
    NIS(k) = eta'*inv(Pzz)*eta;
    
    xhat(:,k+1) = xbar + W*eta;
    Phat{k+1} = (eye(11) - W*H)*Pbar*(eye(11) - W*H)' + W*R*W';
    
    xhat_bnd(:,k+1) = diag(Phat{k+1});
end
alpha = 0.05;
ep = mean(NIS)
r1 = chi2inv(alpha/2,5*length(NIS))/length(NIS)
r2 = chi2inv(1 - (alpha/2),5*length(NIS))/length(NIS)


figure
subplot(211)
grid on
hold on
plot(data.time(1:end-1),vicon_vel_body(1,:), '-b');
plot(data.time(1:end-1),data.kalman_xdot_body(1:end-1), '-r')
plot(data.time(1:end-1),xhat(1,:), '-g');
ylabel('X Body Velocity [m/s]')
legend('Vicon','Laser','Kalman','Location','Best')
title('Body Velocity Comparison')
subplot(212)
grid on
hold on
plot(data.time(1:end-1),vicon_vel_body(2,:), '-b');
plot(data.time(1:end-1),data.kalman_ydot_body(1:end-1), '-r')
plot(data.time(1:end-1),xhat(2,:), '-g');
xlabel('Time [s]')
ylabel('Y Body Velocity [m/s]')


% Plot velocities
figure
subplot(221)
grid on
hold on
p1 = plot(data.time(1:end-1),vicon_vel_body(1,:));
p2 = plot(data.time(1:end-1),xhat(1,:), '-r');
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
plot(data.time(1:end-1),xhat(2,:), '-r')
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
