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
plot(alt_cont.time,alt_cont.filtered_alt,'r'),grid
title('Time versus Altitude')
xlabel('Time [s]')
ylabel('Altitude [m]')

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

% Plot quad xyz vicon and amcl inertial position
figure
subplot(311)
hold on
grid on
plot(alt_cont.time,alt_cont.vicon_x,'r')
plot(alt_cont.time,alt_cont.amcl_x,'b')
ylabel('X [m]')
title('Quad Position')
legend('Vicon','AMCL','Location','Best')
subplot(312)
hold on
grid on
plot(alt_cont.time,alt_cont.vicon_y,'r')
plot(alt_cont.time,alt_cont.amcl_y,'b')
ylabel('Y [m]')
subplot(313)
hold on
grid on
plot(alt_cont.time,alt_cont.vicon_z,'r')
plot(alt_cont.time,alt_cont.amcl_z','b')
xlabel('Time [s]')
ylabel('Z [m]')

% Plot 2D quad xy vicon and amcl inertial position
figure
hold on
grid on
plot(alt_cont.vicon_x,alt_cont.vicon_y,'r')
plot(alt_cont.amcl_x,alt_cont.amcl_y,'b')
xlabel(' X [m]')
ylabel(' Y [m]')
title('Quad 2D X-Y Position')
legend('Vicon','AMCL','Location','Best')

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
hold on
grid on
plot(alt_cont.time,alt_cont.psi,'r')
plot(alt_cont.time,alt_cont.amcl_psi,'b')
xlabel('Time [s]')
ylabel('\psi [rad]')
legend('Pixhawk','AMCL','Location','Best')

end