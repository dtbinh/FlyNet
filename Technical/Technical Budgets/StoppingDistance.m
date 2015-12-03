% Tyler Clayton
% StoppingDistance.m
% 10/30/2015

clear all; close all; clc;
%%
v0 = linspace(0,18,2*18); % velocities                 [m/s]
g = 9.81;              % gravitorial acceleration   [m/s/s]
theta_max = 30;        % max pitch angle,           [deg]
Izz = 0.0191;          % moment of inertia          [kg*m*m]
L = .13335;            % length of quad             [m]
F1 = 9.81*.6;          % max motor thrust           [N]
%F2 = ;
%Fnet = ;
in2m = 0.0254;         % inches to meters

%% Moment of Inertia Calc

% % Arm bars
% ms = (75/1000);
% Is = ms/12*((1*in2m)^2+(5.5*in2m)^2);
% % Motors
% mm = (79.3/1000);
% Im = mm/12*(3*(9/8/2*in2m)^2+(7/8*in2m)^2);
% % Battery
% mb = (482.7/1000);
% Ib = mb/12*((6.5*in2m)^2+(1.5*in2m)^2);
% % Center
% Ic = (811.25/1000)/12*((14*in2m)^2+(9/4*in2m)^2);
% 
% Izz = Ic + (Ib+mb*(2*in2m)^2) + (Is+ms*(4.25*in2m)^2) + (Im+mm*(5.25*in2m)^2);

%% Analysis 1
%{
 Assumptions: 
    -Quad is pitched fully backwards (-30deg) moving forwared at v0 at t0
    -Weight is balanced to maintain alt, left over force is allowed to slow
    -pitching moments are ignored
    -drag is ignored
    -1D kinematic equations are used
    -
%}

ax = -tand(theta_max)*g;     % acceleration in x direction

ts1 = -v0./ax;        % stopping time at constant acceleration
d1 = -v0.^2./(2*ax);  % stopping distance at constant acceleration

% get location for maximum sensor distance = 18.35m
Vmax = sqrt(-18.35*2*ax);
Tmax = -Vmax/ax;

figure(1)
subplot(2,1,1)
hold on
plot(v0,d1,'g'); 
plot([0;Vmax],[18.35;18.35],'r');
plot([Vmax;Vmax],[0;18.35],'r');
xlabel('V_0 [m/s]'); ylabel('Stopping Distance [m]');title('Analysis 1');

subplot(2,1,2)
hold on
plot(v0,ts1,'k'); xlabel('V_0 [m/s]'); 
plot([0;Vmax],[Tmax;Tmax],'r');
plot([Vmax;Vmax],[0;Tmax],'r');
ylabel('Stopping time [s]');

%% Analysis 2
%{
 Assumptions: 
    -Quad is pitched fully forward (30deg) moving forwared at v0 at t0
    -Weight is balanced to maintain alt, left over force is allowed to slow
    -pitching moments are ignored, except to rotate from 30 to -30 deg
    -v0 does not change throughout the orientation transition
    -drag is ignored
    -1D kinematic equations are used
    -
%}

tr1 = 2*2*(theta_max)*Izz^2/(2*F1^2*L^2);
dr1 = v0.*tr1;

% get location for maximum distance
% set coeff vector for velocity
c = [1;-2*ax*tr1;2*ax*18.35];
c2 = [1;-2*ax*tr1;2*ax*5];
rs = roots(c);
rs2 = roots(c2);


Tmax2 = tr1-rs2(2)/ax;

figure (2)
subplot(2,1,1)
hold on
plot(v0,dr1+d1,'g'); 
p1 = plot([0;rs(2)],[18.35;18.35],'r');
plot([0;rs2(2)],[5;5],'b');
p2 = plot([rs2(2); rs2(2)],[0; 5],'b');
plot([rs(2);rs(2)],[0;18.35],'r');
xlabel('V_0 [m/s]'); ylabel('Stopping Distance [m]'); 
%title('Analysis 2: Rotation included, 30deg max pitch');
legend([p1,p2],'Max Guidance','Expected Guidance');

subplot(2,1,2)
hold on
plot(v0,ts1+tr1,'k'); xlabel('V_0 [m/s]'); ylabel('Stopping time [s]');
plot([0;rs(2)],[Tmax;Tmax],'r');
plot([rs(2);rs(2)],[0;Tmax],'r');
plot([rs2(2);rs2(2)],[0;Tmax2],'b');
plot([0;rs2(2)],[Tmax2,Tmax2],'b');


