function plotAltController(alt)

figure
plot(alt.time,alt.alt,'r'),grid
title('Time versus Altitude')
xlabel('Time [s]')
ylabel('Altitude [m]')

figure
subplot(311)
plot(alt.time,alt.rc_p,'r'),grid
ylabel('P [pwm]')
subplot(312)
plot(alt.time,alt.rc_i,'r'),grid
ylabel('I [pwm]')
subplot(313)
plot(alt.time,alt.rc_d,'r'),grid
xlabel('Time [s]')
ylabel('D [pwm]')
title('Altitude PID Contributions')

figure
hold on
grid on
plot(alt.time,alt.rc_p,'r')
plot(alt.time,alt.rc_i,'g')
plot(alt.time,alt.rc_d,'b')
xlabel('Time [s]')
ylabel('PWM')
title('Altitude PWM')
legend('P','I','D','Location','Best')

figure
plot(alt.time,alt.t_send,'k'),grid
xlabel('Time [s]')
ylabel('Throttle Send [pwm]')
title('Throttle Send PWM')

figure
subplot(311)
plot(alt.time,alt.x,'r'),grid
ylabel('X [m]')
title('Quad Position')
subplot(312)
plot(alt.time,alt.y,'r'),grid
ylabel('Y [m]')
subplot(313)
plot(alt.time,alt.z,'r'),grid
xlabel('Time [s]')
ylabel('Z [m]')

figure
subplot(311)
plot(alt.time,alt.phi,'r'),grid
ylabel('\phi [rad]')
title('Quad Euler Angles')
subplot(312)
plot(alt.time,alt.theta,'r'),grid
ylabel('\theta [rad]')
subplot(313)
plot(alt.time,alt.psi,'r'),grid
xlabel('Time [s]')
ylabel('\psi [rad]')

end