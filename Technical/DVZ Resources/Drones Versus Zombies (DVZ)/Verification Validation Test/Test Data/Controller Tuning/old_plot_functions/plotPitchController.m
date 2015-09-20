function plotPitchController(pitch_cont)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotPitchController.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   pitch_cont - pitch structured variable containing data about pitch
%               controller.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the pitch controller structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot pitch over time
figure
plot(pitch_cont.time,pitch_cont.pitch,'r'),grid
title('Time versus Pitch')
xlabel('Time [s]')
ylabel('Pitch [rad]')

% Plot pitch controller PID individual contributions
figure
subplot(311)
plot(pitch_cont.time,pitch_cont.rc_p,'r'),grid
ylabel('P [pwm]')
title('Pitch PID Contributions')
subplot(312)
plot(pitch_cont.time,pitch_cont.rc_i,'r'),grid
ylabel('I [pwm]')
subplot(313)
plot(pitch_cont.time,pitch_cont.rc_d,'r'),grid
xlabel('Time [s]')
ylabel('D [pwm]')

% Plot pitch controller PID contributions on same plot
figure
hold on
grid on
plot(pitch_cont.time,pitch_cont.rc_p,'r')
plot(pitch_cont.time,pitch_cont.rc_i,'g')
plot(pitch_cont.time,pitch_cont.rc_d,'b')
xlabel('Time [s]')
ylabel('PWM')
title('Pitch PWM')
legend('P','I','D','Location','Best')

% Plot pitch controller roll_send pwm command
figure
plot(pitch_cont.time,pitch_cont.pitch_send,'k'),grid
xlabel('Time [s]')
ylabel('Pitch Send [pwm]')
title('Pitch Send PWM')

% Plot quad xyz inertial position
figure
subplot(311)
plot(pitch_cont.time,pitch_cont.x,'r'),grid
ylabel('X [m]')
title('Quad Position')
subplot(312)
plot(pitch_cont.time,pitch_cont.y,'r'),grid
ylabel('Y [m]')
subplot(313)
plot(pitch_cont.time,pitch_cont.z,'r'),grid
xlabel('Time [s]')
ylabel('Z [m]')

% Plot quad euler angles
figure
subplot(311)
plot(pitch_cont.time,pitch_cont.phi,'r'),grid
ylabel('\phi [rad]')
title('Quad Euler Angles')
subplot(312)
plot(pitch_cont.time,pitch_cont.theta,'r'),grid
ylabel('\theta [rad]')
subplot(313)
plot(pitch_cont.time,pitch_cont.psi,'r'),grid
xlabel('Time [s]')
ylabel('\psi [rad]')

% Plot target pitch and actual pitch
figure
hold on
grid on
plot(pitch_cont.time,pitch_cont.target_pitch,'b')
plot(pitch_cont.time,pitch_cont.pitch,'r')
xlabel('Time [s]')
ylabel('Pitch [rad]')
title('Target Pitch and Vehicle Pitch')
legend('Target Pitch','Actual Pitch')

end