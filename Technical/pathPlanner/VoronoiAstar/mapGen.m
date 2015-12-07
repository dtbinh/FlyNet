% close all, clear all, 
clear

% Wall 1 - left
for i = 1:1:60
    mapt(i,1) = 0;
    mapt(i,2) = i;
end
% Wall 2 - far
for i = 61:1:90
    mapt(i,1) = i-60;
    mapt(i,2) = 60;
end
% Wall 3 - right
for i = 91:1:150
    mapt(i,1) = 30;   % x
    mapt(i,2) = 150-i; % y 
end
% Wall 4 - near
for i = 151:1:180
    mapt(i,1) = 180-i;  % x
    mapt(i,2) = 0;      % y
end
mapt = [0, 0; mapt(:,1), mapt(:,2)];

% 1st free wall
for i = 181:1:194
    mapt(i,1) = 18;
    mapt(i,2) = i-181;
end
% plot([mapt(i,1), 30],[mapt(i,2),18],'w') 

mapt = [mapt(:,1), mapt(:,2); 30, 18];
% 2nd free wall (ccw)
for i = 196:1:208
    mapt(i,1) = (208-i) + 18;    % x
    mapt(i,2) = 18;              % y
end

mapt = [mapt(:,1), mapt(:,2); 30, 18]; % take back to wall
mapt = [mapt(:,1), mapt(:,2); 30, 45]; % move up to next start
% 3rd free wall (ccw)
for i = 211:1:229
    mapt(i,1) = (229-i) + 12;   % x
    mapt(i,2) = 45;             % y
end

% 
for i = 230:1:240
    mapt(i,1) = 12;             % x
    mapt(i,2) = (240-i) + 45;   % y
end
mapt = [mapt(:,1), mapt(:,2); 12, 45]; 

%
for i = 242:1:257
    mapt(i,1) = 6;              % x
    mapt(i,2) = (257-i) + 40;   % y
end

for i = 258:1:277
    mapt(i,1) = (277-i) + 6;    % x
    mapt(i,2) = 40;             % y
end

for i = 278:1:293
    mapt(i,1) = 25;             % x
    mapt(i,2) = (293-i)+ 25;    % y
end

for i = 294:1:299
    mapt(i,1) = (299-i)+ 20;    % x
    mapt(i,2) = 25;             % y
end

for i = 300:1:312
    mapt(i,1) = (312-i);        % x
    mapt(i,2) = 25;             % y
end

for i = 313:1:320
    mapt(i,1) = 6;              % x
    mapt(i,2) = (320-i) +18;    % y
end

for i = 321:1:327
    mapt(i,1) = (327 - i) +6;   % x
    mapt(i,2) = 18;             % y
end

for i = 328:1:340
    mapt(i,1) = 12;             % x
    mapt(i,2) = (340 - i)+ 6;   % y
end

figure
plot(mapt(:,1),mapt(:,2))
hold on
plot([6 12],[55,45],'w')
plot([18; 30],[13,18],'w') 
plot([20 12],[25,25],'w')
axis([-1 31 -1 61])

save('mapt.mat')