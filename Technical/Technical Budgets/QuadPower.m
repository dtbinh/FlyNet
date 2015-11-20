
clear all; close all; clc

%% DJI - Matrice 100
%m              % quad mass     [kg]
g = 9.81;       % grav accel    [m/s/s]
r = .1651;      % prop radius   [m] .11938
V = 22.2;       % batt voltage  [V]  14.8
rho0 = 1.225;    % air density   [kg/m3]
rho = 0.98;
eta = .85;      % trans efficiency
batt = 4500;    % batt rating   [mAh] 
I_extra = 1394;     % current used by elec [mA]
MaxI = 20;

m = linspace(1.5,4,35);

Curr0 = @(M) 4*(1/eta).*(M.*g).^(3/2)./(4*r*V*sqrt(2*pi*rho0));
Curr = @(M) 4*(1/eta).*(M.*g).^(3/2)./(4*r*V*sqrt(2*pi*rho));
EstTime = @(i) (batt/1000)./i.*60;

% endurance based on motors
I = Curr0(m);
t = EstTime(I);
% endurance with extra guidance/odroid
I2 = Curr(m)+I_extra/1000;
t2 = EstTime(I2);
% flynet point
fly = EstTime(Curr(2.77)+I_extra/1000);
fly_1lb = EstTime(Curr(3.22)+I_extra/1000);
str1 = sprintf('(2.77,%2.2f)',fly);
str2 = sprintf('(3.22,%2.2f)',fly_1lb);

A = [2.3 22; 2.8 17; 3.2 13];
figure(1)
hold on
plot(A(:,1),A(:,2),'go','MarkerFaceColor','g');
plot(m,t,'b','LineWidth',2);
plot(m,t2,'m','LineWidth',2);
plot(2.77,fly,'ro','MarkerFaceColor','r');
text(2.52,fly-1.25,str1,'rotation',0);
plot(3.22,fly_1lb,'ko','MarkerFaceColor','k');
text(2.98,fly_1lb-1.25,str2,'rotation',0);
xlabel('M_T_O [kg]'); ylabel('Duration [min]'); title('M100 Hovering Endurance');
legend('Mfg claim','Est Endurance','Endurance w/ Guidance','FlyNet - Guidance','FlyNet - Guidance& 1 lb PL');


 %% AlienBee 450 
%m              % quad mass     [kg]
g = 9.81;       % grav accel    [m/s/s]
r = .11938;     % prop radius   [m] 
V = 14.8;       % batt voltage  [V]  
rho = 0.98; %1.225;    % air density   [kg/m3]
eta = .85;      % trans efficiency
batt = 4500;    % batt rating   [mAh] 
I_extra = [2228.57 1669.11];     % current used by elec [mAh]
MaxI = 15;

Curr = @(M) 4*(1/eta).*(M.*g).^(3/2)./(4*r*V*sqrt(2*pi*rho));
EstTime = @(i) (batt/1000)./i.*60;

m = linspace(1.5,4,35);
%RGBD
Irg = Curr(m)+I_extra(1)/1000;
trg = EstTime(Irg);

Fly_rg = EstTime(Curr(1.91)+1/1000*I_extra(1));
Fly_rg_1lb = EstTime(Curr(2.36)+I_extra(1)/1000);
strrg = sprintf('(1.91, %2.f)',Fly_rg);
strrg_1lb = sprintf('(2.36, %2.2f)',Fly_rg_1lb);


%Guidance
Igu = Curr(m)+I_extra(2)/1000;
tgu = EstTime(Igu);

Fly_gu = EstTime(Curr(1.93)+1/1000*I_extra(2));
Fly_gu_1lb = EstTime(Curr(2.38)+I_extra(2)/1000);
strgu = sprintf('(1.93, %2.2f)',Fly_gu);
strgu_1lb = sprintf('(2.38, %2.2f)',Fly_gu_1lb);

figure(2)
hold on
plot(m,trg,'b','LineWidth',2);
plot(m,tgu,'m','LineWidth',2);
plot(1.91,Fly_rg,'rd','MarkerFaceColor','r');
plot(2.36,Fly_rg_1lb,'ro','MarkerFaceColor','r');
text(1.91,Fly_rg+1,strrg,'rotation',0);
text(2.36,Fly_rg_1lb+1,strrg_1lb,'rotation',0);

plot(1.93,Fly_gu,'kd','MarkerFaceColor','k');
plot(2.38,Fly_gu_1lb,'ko','MarkerFaceColor','k');
% text(1.93,Fly_gu-1.5,strgu,'rotation',0);
% text(2.38,Fly_gu_1lb-1.5,strgu_1lb,'rotation',0);

xlabel('M_T_O [kg]'); ylabel('Duration [min]'); title('AlienBee Hovering Endurance');
legend('RGBD','Guidance','FlyNet - RGBD','FlyNet - RGBD & 1lb PL','FlyNet - Guidance', 'FlyNet - Guidance & 1lb PL');

