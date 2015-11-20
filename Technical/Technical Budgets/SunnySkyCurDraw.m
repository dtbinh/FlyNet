% Motor Current Draw

clear all; 
%close all; 
clc;

%% Motor data
% data is presented in the form [Throttle%, Current Draw (A), Thrust (g)]
p0947 = [.5 2.9 410; .6 3.8 460; .65 4.4 530; .75 5.8 640; .8 6.9 710; .85 8.9 830; 1 10.5 930];
p1038 = [.5 3.3 480; .65 5.4 650;.75 8.1 810;.85 11.9 1030; 1 15.2 1210];
p1147 = [.5 3.9 530; .6 5.5 680; .65 6.9 790; .75 9.5 910; .8 11 1020; .85 13.9 1150; 1 16.8 1320];

%% find curve to fit the line
% xvalues - Current
thrust = linspace(300,1500,1000);
% coefficients
c0947 = polyfit(p0947(:,3),p0947(:,2),2);
c1038 = polyfit(p1038(:,3),p1038(:,2),2);
c1147 = polyfit(p1147(:,3),p1147(:,2),2);
% evaluate poly to get thrust values
f0947 = polyval(c0947,thrust);

f1038 = polyval(c1038,thrust);
f1147 = polyval(c1147,thrust);

% combine p0947 and p1147
% pa1047 = [p0947; p1147];
% ca1047 = polyfit(pa1047(:,2),pa1047(:,3),2);
% fa1047 = polyval(ca1047,current);

%% Plot the data

figure()
hold on
f1 = plot(thrust,f0947,'r');
plot(p0947(:,3),p0947(:,2),'ro','MarkerFaceColor','r');
f2 = plot(thrust,f1038,'g');
plot(p1038(:,3),p1038(:,2),'go','MarkerFaceColor','g');
f3 = plot(thrust,f1147,'k');
plot(p1147(:,3),p1147(:,2),'ko','MarkerFaceColor','k');
ylabel('Current [A]'); xlabel('Thrust [g]');
% plot(current,fa1047,'m','LineWidth',2)
 legend([f1,f2,f3],'9x4.7','10x3.8','11x4.7','Location','Best'); title('Current vs Thrust Per Motor');
% xlabel('Current [A]'); ylabel('Thrust [g]');


