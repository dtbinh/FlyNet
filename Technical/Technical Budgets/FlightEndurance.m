%FlightEndurance.m
%Tyler Clayton
% 12/2/2015

clear all; close all; clc;

%% variables

m_empt = 2.37; % empty mass [kg]
pay_max = 4*1.2*.8; %max payload [kg] = 80% of max thrust
g = 9.81;   %grav accel [m/s/s]
V = 14.8;   %Batt volt [V]
I_elec = 5.2;  %current from electronics [A]

batt = 8000; %battery capacity [mAhr]

% create array for different payload masses
pay = linspace(-1,pay_max,1000);
batt2 = 4500:250:10000;

% call SunnySkyCurDraw.m to get equation, coeffs given by c1038
SunnySkyCurDraw;

%% Current functions

% determine motor current for hover based on weight
I_mot = @(mass) polyval(c1038,mass./4);
% sum to get total current
I_tot = @(mass) 4.*I_mot(mass)+I_elec;

% calculate time
EstTime = @(mass) 60.*(batt/1000)./I_tot(mass); % [min]
EstTime2 = @(mass,bat) 60.*(bat./1000)./I_tot(mass); % [min]
%%% calculations
m = (m_empt+pay)*1000;
t = EstTime(m);

% find the entry where it crosses requriement
i = find(t<10,1);
disp(m(i)/1000)
disp(t(i))
%% plot
figure
hold on
plot(m_empt+pay,t,'k');
plot(m_empt,EstTime(m_empt*1000),'ro','MarkerFaceColor','r');
plot(m_empt+0.5/2.2,EstTime(m_empt*1000+0.5/2.2*1000),'go','MarkerFaceColor','g');
plot(m_empt+1/2.2,EstTime(m_empt*1000+1/2.2*1000),'ko','MarkerFaceColor','k');
plot(m(i)/1000,t(i),'md','MarkerFaceColor','m');
legend('Theo','Empty','0.5-lb','1.0-lb','Max','Location','Best');
xlabel('mass [kg]'); ylabel('Endurance [min]');


figure
hold on
plot(pay,t,'k');
plot(0,EstTime(m_empt*1000),'ro','MarkerFaceColor','r');
plot(0.5/2.2,EstTime(m_empt*1000+0.5/2.2*1000),'go','MarkerFaceColor','g');
plot(1/2.2,EstTime(m_empt*1000+1/2.2*1000),'ko','MarkerFaceColor','k');
plot(m(i)/1000-m_empt,t(i),'md','MarkerFaceColor','m');
legend('Theo','Empty','0.5-lb','1.0-lb','Max','Location','Best');
xlabel('payload [kg]'); ylabel('Endurance [min]');
xlim([0,4]);

%% vary batteries, no payload
Batt_data = [5000 556;
             4500 485;
             5800 562;
             6200 589;
             8000 643;
             6600 537;
             10000 804
             ]; %various battery mass and caps for different sizes

m_bat = m_empt*1000 + Batt_data(:,2)-804;
tbat = EstTime2(m_bat,Batt_data(:,1));

figure
plot(Batt_data(:,1),tbat,'rs')

