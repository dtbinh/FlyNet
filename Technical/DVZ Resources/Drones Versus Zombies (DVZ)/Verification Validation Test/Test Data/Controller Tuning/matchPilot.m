function matchPilot(rc)
pitch_trim = 1459-20;
roll_trim = 1599+20;
pitch_max = 1877;
pitch_min = 1035;
roll_max = 2022;
roll_min = 1164;
for i = 1:length(rc.theta)
rc.theta_from_pwm(i) = pwm2angle(rc.pitch(i),pitch_trim,pitch_max,pitch_min);
rc.phi_from_pwm(i) = pwm2angle(rc.roll(i),roll_trim,roll_max,roll_min);
end

for i = 1:length(rc.theta_from_pwm)
   rc.theta_pwm(i) = angle2pwm(rc.theta_from_pwm(i),pitch_trim,pitch_max,pitch_min);
   rc.phi_pwm(i) = angle2pwm(rc.phi_from_pwm(i),roll_trim,roll_max,roll_min);
end

figure
plot(rc.time,rc.theta_pwm,'b')
hold on
waitforbuttonpress()
plot(rc.time,rc.pitch,'r')
legend('should be sending','actually sent')

for i = 1:length(rc.theta)
rc.theta_from_pwm(i) = pwm2angle(rc.theta_pwm(i),pitch_trim,pitch_max,pitch_min);
rc.phi_from_pwm(i) = pwm2angle(rc.phi_pwm(i),roll_trim,roll_max,roll_min);
end
end