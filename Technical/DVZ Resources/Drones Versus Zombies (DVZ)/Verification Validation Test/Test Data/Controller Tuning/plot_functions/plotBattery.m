function plotBattery(batt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotBattery.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   batt - battery structured variable containing data about pixhawk
%          battery.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the battery structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot battery info over time
figure
subplot(311)
plot(batt.time,batt.voltage,'r'),grid
title('Battery Info')
ylabel('Voltage [V]')
subplot(312)
plot(batt.time,batt.current,'r'),grid
ylabel('Current [A]')
subplot(313)
plot(batt.time,batt.batt_remaining,'r'),grid
xlabel('Time [s]')
ylabel('%')

end