function plotAttitude(att)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotAttitude.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   att - attitude structured variable containing data about attitude.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the attitude structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot roll pitch yaw from pixhawk and vicon
figure
subplot(311)
hold on
grid on
plot(att.time,att.p_roll,'r')
plot(att.time,att.v_roll,'b')
ylabel('Roll [rad]')
legend('P','V','Location','Best')
title('Pixhawk and Vicon Attitude')
subplot(312)
hold on
grid on
plot(att.time,att.p_pitch,'r')
plot(att.time,att.v_pitch,'b')
ylabel('Pitch [rad]')
legend('P','V','Location','Best')
subplot(313)
hold on
grid on
plot(att.time,att.p_yaw,'r')
plot(att.time,att.v_yaw,'b')
xlabel('Time [s]')
ylabel('Yaw [rad]')
legend('P','V','Location','Best')

end