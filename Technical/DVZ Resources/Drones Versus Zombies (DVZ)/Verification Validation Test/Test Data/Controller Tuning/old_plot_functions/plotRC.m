function plotRC(rc)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotRC.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   rc - rc structured variable containing data about the pwm commands of 
%        the RC handset.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the rc structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot roll, pitch, yaw, throttle pwms
figure
subplot(411)
plot(rc.time,rc.roll,'r'),grid
ylabel('\phi [pwm]')
title('RC Channels PWM')
subplot(412)
plot(rc.time,rc.pitch,'r'),grid
ylabel('\theta [pwm]')
subplot(413)
plot(rc.time,rc.yaw,'r'),grid
ylabel('\psi [pwm]')
subplot(414)
plot(rc.time,rc.throttle,'r'),grid
xlabel('Time [s]')
ylabel('T [pwm]')

% Plot roll_send, pitch_send ,yaw_send, throttle_send pwms
figure
subplot(411)
plot(rc.time,rc.roll_send,'r'),grid
ylabel('\phi [pwm]')
title('Controller Send PWM')
subplot(412)
plot(rc.time,rc.pitch_send,'r'),grid
ylabel('\theta [pwm]')
subplot(413)
plot(rc.time,rc.yaw_send,'r'),grid
ylabel('\psi [pwm]')
subplot(414)
plot(rc.time,rc.throttle_send,'r'),grid
xlabel('Time [s]')
ylabel('T [pwm]')

figure
subplot(2,1,1)
plot(rc.time,rc.phi*(180/pi),'r')
subplot(2,1,2)
plot(rc.time,rc.theta*(180/pi),'r')

end