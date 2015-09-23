function plotRollController(roll_cont)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotRollController.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   roll_cont - roll structured variable containing data about roll
%               controller.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the roll controller structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot roll over time
figure
plot(roll_cont.time,roll_cont.roll,'r'),grid
title('Time versus Roll')
xlabel('Time [s]')
ylabel('Roll [rad]')

% Plot roll controller PID individual contributions
figure
subplot(311)
plot(roll_cont.time,roll_cont.rc_p,'r'),grid
ylabel('P [pwm]')
title('Roll PID Contributions')
subplot(312)
plot(roll_cont.time,roll_cont.rc_i,'r'),grid
ylabel('I [pwm]')
subplot(313)
plot(roll_cont.time,roll_cont.rc_d,'r'),grid
xlabel('Time [s]')
ylabel('D [pwm]')

% Plot roll controller PID contributions on same plot
figure
hold on
grid on
plot(roll_cont.time,roll_cont.rc_p,'r')
plot(roll_cont.time,roll_cont.rc_i,'g')
plot(roll_cont.time,roll_cont.rc_d,'b')
xlabel('Time [s]')
ylabel('PWM')
title('Roll PWM')
legend('P','I','D','Location','Best')

% Plot roll controller roll_send pwm command
figure
plot(roll_cont.time,roll_cont.roll_send,'k'),grid
xlabel('Time [s]')
ylabel('Roll Send [pwm]')
title('Roll Send PWM')

% Plot quad xyz inertial position
figure
subplot(311)
plot(roll_cont.time,roll_cont.x,'r'),grid
ylabel('X [m]')
title('Quad Position')
subplot(312)
plot(roll_cont.time,roll_cont.y,'r'),grid
ylabel('Y [m]')
subplot(313)
plot(roll_cont.time,roll_cont.z,'r'),grid
xlabel('Time [s]')
ylabel('Z [m]')

% Plot quad euler angles
figure
subplot(311)
plot(roll_cont.time,roll_cont.phi,'r'),grid
ylabel('\phi [rad]')
title('Quad Euler Angles')
subplot(312)
plot(roll_cont.time,roll_cont.theta,'r'),grid
ylabel('\theta [rad]')
subplot(313)
plot(roll_cont.time,roll_cont.psi,'r'),grid
xlabel('Time [s]')
ylabel('\psi [rad]')

% Plot target roll and actual roll
figure
hold on
grid on
plot(roll_cont.time,roll_cont.target_roll,'b')
plot(roll_cont.time,roll_cont.roll,'r')
xlabel('Time [s]')
ylabel('Roll [rad]')
title('Target Roll and Vehicle Roll')
legend('Target Roll','Actual Roll')

end