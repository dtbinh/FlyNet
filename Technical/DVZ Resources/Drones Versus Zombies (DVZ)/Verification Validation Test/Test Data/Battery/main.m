% Battery Tests Post Processing
% Project DVZ
% Programmer: Mark Sakaguchi
% Created: 2/8/2015
% Updated: 2/8/2015

clear all
close all
clc

format long

addpath('endurance_flight_02_08_15');

%% Load data
filename1 = 'alt_test2.txt';
filename2 = 'battery_test2.txt';
filename3 = 'rc_test2.txt';
filename4 = 'attitude_test6.txt';
alt = parseAltController(filename1);
batt = parseBattery(filename2);
rc = parseRC(filename3);
att = parseAttitude(filename4);
alt.time = alt.time - alt.time(1);
batt.time = batt.time - batt.time(1);
rc.time = rc.time - rc.time(1);
att.time = att.time - att.time(1);

%% Plot altitude controller
% figure
% plot(alt.time,alt.alt,'r'),grid
% title('Time versus Altitude')
% xlabel('Time [s]')
% ylabel('Altitude [m]')
% 
% figure
% subplot(311)
% plot(alt.time,alt.rc_p,'r'),grid
% ylabel('P [pwm]')
% subplot(312)
% plot(alt.time,alt.rc_i,'r'),grid
% ylabel('I [pwm]')
% subplot(313)
% plot(alt.time,alt.rc_d,'r'),grid
% xlabel('Time [s]')
% ylabel('D [pwm]')
% title('Altitude PID Contributions')
% 
% figure
% hold on
% grid on
% plot(alt.time,alt.rc_p,'r')
% plot(alt.time,alt.rc_i,'g')
% plot(alt.time,alt.rc_d,'b')
% xlabel('Time [s]')
% ylabel('PWM')
% title('Altitude PWM')
% legend('P','I','D','Location','Best')
% 
% figure
% plot(alt.time,alt.t_send,'k'),grid
% xlabel('Time [s]')
% ylabel('Throttle Send [pwm]')
% title('Throttle Send PWM')
% 
% figure
% subplot(311)
% plot(alt.time,alt.x,'r'),grid
% ylabel('X [m]')
% title('Quad Position')
% subplot(312)
% plot(alt.time,alt.y,'r'),grid
% ylabel('Y [m]')
% subplot(313)
% plot(alt.time,alt.z,'r'),grid
% xlabel('Time [s]')
% ylabel('Z [m]')
% 
% figure
% subplot(311)
% plot(alt.time,alt.phi,'r'),grid
% ylabel('\phi [rad]')
% title('Quad Euler Angles')
% subplot(312)
% plot(alt.time,alt.theta,'r'),grid
% ylabel('\theta [rad]')
% subplot(313)
% plot(alt.time,alt.psi,'r'),grid
% xlabel('Time [s]')
% ylabel('\psi [rad]')

%% Plot battery
% figure
% subplot(311)
% plot(batt.time,batt.voltage,'r'),grid
% title('Battery Info')
% ylabel('Voltage [V]')
% subplot(312)
% plot(batt.time,batt.current,'r'),grid
% ylabel('Current [A]')
% subplot(313)
% plot(batt.time,batt.batt_remaining,'r'),grid
% xlabel('Time [s]')
% ylabel('%')

%% Plot RC
% figure
% subplot(411)
% plot(rc.time,rc.roll,'r'),grid
% ylabel('\phi [pwm]')
% title('RC Channels PWM')
% subplot(412)
% plot(rc.time,rc.pitch,'r'),grid
% ylabel('\theta [pwm]')
% subplot(413)
% plot(rc.time,rc.yaw,'r'),grid
% ylabel('\psi [pwm]')
% subplot(414)
% plot(rc.time,rc.throttle,'r'),grid
% xlabel('Time [s]')
% ylabel('T [pwm]')
% 
% figure
% subplot(411)
% plot(rc.time,rc.roll_send,'r'),grid
% ylabel('\phi [pwm]')
% title('Controller Send PWM')
% subplot(412)
% plot(rc.time,rc.pitch_send,'r'),grid
% ylabel('\theta [pwm]')
% subplot(413)
% plot(rc.time,rc.yaw_send,'r'),grid
% ylabel('\psi [pwm]')
% subplot(414)
% plot(rc.time,rc.throttle_send,'r'),grid
% xlabel('Time [s]')
% ylabel('T [pwm]')
% 

%% Plot attitude
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