clear variables
close all
clc

addpath('04_22_2015');

data = load_DVZ_log('full_system_test7.txt');
dt = mean(diff(data.time));

xacc = data.pixhawk_xacc.*(-1).*9.81/1000;
yacc = data.pixhawk_yacc.*(1).*9.81/1000;
zacc = data.pixhawk_zacc.*(1).*9.81/1000;

Vx = data.local_velx_body;
Vy = (-1).*data.local_vely_body;

vicon_vx = diff(data.vicon_x)./dt;
vicon_xacc = diff(vicon_vx)./dt;

vicon_vy = diff(data.vicon_y)./dt;
vicon_yacc = diff(vicon_vy)./dt;

vicon_vz = diff(data.vicon_z)./dt;
vicon_zacc = diff(vicon_vz)./dt;


figure
subplot(211)
hold on
grid on
plot(data.time,data.vicon_x)
plot(data.time,data.local_xmap)
xlabel('Time [s]')
ylabel('X [m]')
subplot(212)
hold on
grid on
plot(data.time,data.vicon_y)
plot(data.time,data.local_ymap)
xlabel('Time [s]')
ylabel('Y [m]')

figure
subplot(311)
plot(data.time,data.voltage./1000)
ylabel('Voltage [V]')
subplot(312)
plot(data.time,data.current./1000)
ylabel('Current [A]')
subplot(313)
plot(data.time,data.battery_remaining)
xlabel('Time [s]')
ylabel('% Remaining')
