function plotYawController(yaw_cont)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotYawController.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   yaw_cont - yaw structured variable containing data about yaw
%              controller.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the yaw controller structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot yaw over time
figure
plot(yaw_cont.time,yaw_cont.yaw,'r'),grid
title('Time versus Yaw')
xlabel('Time [s]')
ylabel('Yaw [rad]')

% Plot yaw controller PID individual contributions
figure
subplot(311)
plot(yaw_cont.time,yaw_cont.rc_p,'r'),grid
ylabel('P [pwm]')
title('Yaw PID Contributions')
subplot(312)
plot(yaw_cont.time,yaw_cont.rc_i,'r'),grid
ylabel('I [pwm]')
subplot(313)
plot(yaw_cont.time,yaw_cont.rc_d,'r'),grid
xlabel('Time [s]')
ylabel('D [pwm]')

% Plot yaw controller PID contributions on same plot
figure
hold on
grid on
plot(yaw_cont.time,yaw_cont.rc_p,'r')
plot(yaw_cont.time,yaw_cont.rc_i,'g')
plot(yaw_cont.time,yaw_cont.rc_d,'b')
xlabel('Time [s]')
ylabel('PWM')
title('Yaw PWM')
legend('P','I','D','Location','Best')

% Plot yaw controller yaw_send pwm command
figure
plot(yaw_cont.time,yaw_cont.yaw_send,'k'),grid
xlabel('Time [s]')
ylabel('Yaw Send [pwm]')
title('Yaw Send PWM')

% Plot quad xyz inertial position
figure
subplot(311)
plot(yaw_cont.time,yaw_cont.x,'r'),grid
ylabel('X [m]')
title('Quad Position')
subplot(312)
plot(yaw_cont.time,yaw_cont.y,'r'),grid
ylabel('Y [m]')
subplot(313)
plot(yaw_cont.time,yaw_cont.z,'r'),grid
xlabel('Time [s]')
ylabel('Z [m]')

% Plot quad euler angles
figure
subplot(311)
plot(yaw_cont.time,yaw_cont.phi,'r'),grid
ylabel('\phi [rad]')
title('Quad Euler Angles')
subplot(312)
plot(yaw_cont.time,yaw_cont.theta,'r'),grid
ylabel('\theta [rad]')
subplot(313)
plot(yaw_cont.time,yaw_cont.psi,'r'),grid
xlabel('Time [s]')
ylabel('\psi [rad]')

end