function plotAltController(alt_cont)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotAltController.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   alt_cont - alt structured variable containing data about alt
%              controller.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the altitude controller structured 
%    variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot altitude over time
figure
plot(alt_cont.time,alt_cont.alt,'r'),grid
title('Time versus Altitude')
xlabel('Time [s]')
ylabel('Altitude [m]')

% Plot altitude controller PID individual contributions
figure
subplot(311)
plot(alt_cont.time,alt_cont.rc_p,'r'),grid
title('Altitude PID Contributions')
ylabel('P [pwm]')
subplot(312)
plot(alt_cont.time,alt_cont.rc_i,'r'),grid
ylabel('I [pwm]')
subplot(313)
plot(alt_cont.time,alt_cont.rc_d,'r'),grid
xlabel('Time [s]')
ylabel('D [pwm]')

% Plot altitude controller PID contributions on same plot
figure
hold on
grid on
plot(alt_cont.time,alt_cont.rc_p,'r')
plot(alt_cont.time,alt_cont.rc_i,'g')
plot(alt_cont.time,alt_cont.rc_d,'b')
xlabel('Time [s]')
ylabel('PWM')
title('Altitude PWM')
legend('P','I','D','Location','Best')

% Plot altitude controller throttle_send pwm command
figure
plot(alt_cont.time,alt_cont.t_send,'k'),grid
xlabel('Time [s]')
ylabel('Throttle Send [pwm]')
title('Throttle Send PWM')

% Plot quad xyz inertial position
figure
subplot(311)
plot(alt_cont.time,alt_cont.x,'r'),grid
ylabel('X [m]')
title('Quad Position')
subplot(312)
plot(alt_cont.time,alt_cont.y,'r'),grid
ylabel('Y [m]')
subplot(313)
plot(alt_cont.time,alt_cont.z,'r'),grid
xlabel('Time [s]')
ylabel('Z [m]')

% Plot quad euler angles
figure
subplot(311)
plot(alt_cont.time,alt_cont.phi,'r'),grid
ylabel('\phi [rad]')
title('Quad Euler Angles')
subplot(312)
plot(alt_cont.time,alt_cont.theta,'r'),grid
ylabel('\theta [rad]')
subplot(313)
plot(alt_cont.time,alt_cont.psi,'r'),grid
xlabel('Time [s]')
ylabel('\psi [rad]')

end