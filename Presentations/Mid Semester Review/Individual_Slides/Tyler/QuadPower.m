
clear all; close all; clc

%% DJI - Matrice 100
%m              % quad mass     [kg]
g = 9.81;       % grav accel    [m/s/s]
r = .1651;      % prop radius   [m] .11938
V = 22.2;       % batt voltage  [V]  14.8
rho = 1.225;    % air density   [kg/m3]
eta = .85;      % trans efficiency
batt = 4500;    % batt rating   [mAh] 
I_extra = 1340;     % current used by elec [mAh]

m = linspace(2,4,35);

Curr = @(M) (1/eta).*(M.*g).^(3/2)./(4*r*V*sqrt(2*pi*rho));
EstTime = @(i) (batt/1000/4./i).*60;

% endurance based on motors
I = Curr(m);
t = EstTime(I);
% endurance with extra guidance/odroid
I2 = Curr(m)+I_extra/1000;
t2 = EstTime(I2);
% flynet point
flyt = EstTime(Curr(2.71)+I_extra/1000);
str = sprintf('%2.2f min',flyt);

A = [2.3 22; 2.8 17; 3.2 13];
figure(1)
hold on
plot(A(:,1),A(:,2),'ro','MarkerFaceColor','r');
plot(m,t,'bd','MarkerFaceColor','b');
plot(m,t2,'md','MarkerFaceColor','m');
plot(2.71,flyt,'go','MarkerFaceColor','g');
text(2.65,flyt+1,str,'rotation',0);
xlabel('M_T_O [kg]'); ylabel('Duration [min]'); title('M100 Theoretical Endurance');
legend('Mfg claim','Est Endurance','Est Endurance w/ Guidance','FlyNet - Guidance');


%% AlienBee 450 
%m              % quad mass     [kg]
g = 9.81;       % grav accel    [m/s/s]
r = .11938;     % prop radius   [m] 
V = 14.8;       % batt voltage  [V]  
rho = 1.225;    % air density   [kg/m3]
eta = .85;      % trans efficiency
batt = 4500;    % batt rating   [mAh] 
I_extra = [1123.2 1820];     % current used by elec [mAh]

m = linspace(1,4,35);
%RGBD
I = Curr(m)+I_extra(1)/1000;
t = EstTime(I);
tr = EstTime(Curr(1.52)+1/1000*I_extra(1));
strt = sprintf('%2.2f min',tr);
%Guidance
I2 = Curr(m)+I_extra(2)/1000;
t2 = EstTime(I2);
tg = EstTime(Curr(1.63)+1/1000*I_extra(2));
strg = sprintf('%2.2f min',tg);

figure(2)
hold on
plot(m,t,'bd','MarkerFaceColor','b');
plot(m,t2,'mo','MarkerFaceColor','m');
plot(1.52,tr,'rd','MarkerFaceColor','r');
plot(1.63,tg,'go','MarkerFaceColor','g');
text(1.5,tr+1,strt,'rotation',0);
text(1.56,tg+1,strg,'rotation',0);
xlabel('M_T_O [kg]'); ylabel('Duration [min]'); title('AlienBee Theoretical Endurance');
legend('RGBD','Guidance','FlyNet - RGBD', 'FlyNet - Guidance');

