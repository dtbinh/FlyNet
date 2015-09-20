function plotFlow(alt,flow)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotFlow.m
% Programmer: Mark Sakaguchi
% Created: 2/20/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   alt - alt structured variable containing data about alt controller.
%   flow - px4flow structured variable containing data about px4flow
%          sensor.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the altitude controller structured 
%    variable and the flow structured variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot flow altitude over time
figure
subplot(211)
hold on
grid on
plot(alt.time,alt.alt,'b')
plot([flow.time(1) flow.time(end)],[flow.target_alt(1) flow.target_alt(end)],'k--')
title('Flow Altitude')
xlabel('Time [s]')
ylabel('Filtered Flow Alt [m]')
legend('Alt','Ref','Location','Best')
subplot(212)
hold on
grid on
plot(flow.time,flow.flow_alt,'r')
plot([flow.time(1) flow.time(end)],[flow.target_alt(1) flow.target_alt(end)],'k--')
xlabel('Time [s]')
ylabel('Unfiltered Flow Alt [m]')
legend('Alt','Ref','Location','Best')

% Plot quality over time
figure
plot(flow.time,flow.quality,'b'),grid
xlabel('Time [s]')
ylabel('Quality')
title('Flow Quality')

% Plot flow altitude versus vicon altitude
figure
hold on
grid on
plot(alt.time,alt.alt,'b')
plot(flow.time,flow.z,'r')
xlabel('Time [s]')
ylabel('Altitude [m]')
title('Flow vs. Vicon Altitude')

end