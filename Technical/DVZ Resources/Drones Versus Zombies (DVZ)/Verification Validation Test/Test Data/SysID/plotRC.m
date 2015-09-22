function plotRC(rc)

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

end