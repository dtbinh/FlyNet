function [] = Space(sizex, sizey, startx, starty, goalx, goaly)
% Written by Bryce Hill
% 
% Purpose: Define 2-D space to search 
% 
% Inputs:
%   * none
% 
% Outputs:
%   * 2-D point .mat file
%

%% Generate 2D plane
X = sizex;
Y = sizey;
space = zeros(X,Y);
i = 0; j = 1; k = 0;
for i = 1:1:X;
    for k = 1:1:Y;
        space(i,k) = j;
        j = j +1;
    end
    j = i +1;
end

%% Generate point array defining vector space
i = 0; j = 1; k = 0; length = X*Y ;l = 1;
spaceVec(1:length,1:2) = 0;
spaceVecX(1:length) = 0;
spaceVecY(1:length) = 0;
for i = 1:1:X;
    for k = 1:1:Y;
        spaceVecX(l) = i;
        spaceVecY(l) = k;
        l = l + 1;
    end
end    

x = 1:1:X;
y = 1:1:Y;

%% Generate obstacle vector
numObst = X;
% obstX = round(X*rand([10,1]));
% obstY = round(Y*rand([10,1]));
obstX = [1, 1.5, 3, 2.5, 3];
obstY = [1, 3,   2, 1, 3]; 

%% Generate points equi-distant between obstacles 
% - Populate nx1 matrices Vx and Vy which hold in bounds only voronoi
%   generated waypoints
[vx, vy] = Voronoi(obstX, obstY)   % Calculate voronoi
axis([0 6 -2 5])


%% Concatenate vx and vy
for i = 1:1:numel(vx)
    vxf(i) = vx(i);
    vyf(i) = vy(i);
end

%% Calculate neighbors
% x = [startx; vxf'; goalx];     % Add start point as 
% y = [starty; vyf'; goaly];
x = vxf;
y = vyf;
% successor = zeros(numel(x),1);
numel(x)
for i = 1:1:numel(x)
    index = 1;
%     for j = i:1:numel(x)
    flag = 0;
    for j = 1:1:numel(x)
        if x(i) == x(j)
            if y(i) == y(j)
                if mod(i,2) == 0            
                    neighbor(i,index) = j-1;
                    index = index +1;
                    flag = 1;
                else
                    neighbor(i,index) = j+1;
                    index = index +1;
                    flag = 1;
                end
            end
        end
    end
    if(flag == 0)
        neighbor(i,index) = 18;
    end
    index = 1;
end

%% Publish values

save('space.mat');
save('spaceVec.mat','spaceVec','spaceVecX','spaceVecY');
save('xy.mat','x','y');
save('obstacles.mat','obstX','obstY');
end









































%% Generate points equi-distant between obstacles 
% - Populate nx1 matrices Vx and Vy which hold in bounds only voronoi
%   generated waypoints
% [vx, vy] = Voronoi(obstX, obstY)   % Calculate voronoi
% axis([0 12 0 12])
% Vx = [vx(1,:),vx(2,:)];             % Concatenate voronoi vertices s.t.  
% Vy = [vy(1,:),vy(2,:)];             % Vx & Vy = [:,1] vectors

% count = 1; i = 1; w = numel(Vx);    % Define counter variables
% while count <= w
%     if (Vx(i) >= X) || (Vx(i) <= 0)
%     Vx = [Vx(1:(i-1)),Vx((i+1):end)];
%     Vy = [Vy(1:(i-1)),Vy((i+1):end)];
%     else if (Vy(i) >= Y) || (Vy(i) <= 0)
%         Vx = [Vx(1:(i-1)),Vx((i+1):end)];
%         Vy = [Vy(1:(i-1)),Vy((i+1):end)];   
%         else
%         i = i + 1;
%         end
%     end
%     count = count +1;
% end

%% Generate Stucture
% i = 1;
% for i = 2:1:numel(Vx)
%     node(i+1).x = Vx(i);
%     node(i+1).y = Vy(i);
%     node(i+1).suc = 0;
% %     node(i).cost = 0;
% end

%% Generate points equi-distant between obstacles
% - Trim nx2 matrices to be within bounds of space
% count = 1; vx2 = vx'; vy2 = vy';  % Restart count variable
% for i = 1:1:numel(vx)/2
%     if (vx2(i,1) >= 0) || (vx2(i,1) <= X) || (vy2(i,1) >= 0) || (vy2(i,1) <=Y) 
%         m = (vy2(i,2) - vy2(i,1))/(vx2(i,2) - vx2(i,1));
%         vy2(i,1) = vy2(i,1)-m*(vx2(i,2) - vx2(i,1)); 
%         vx2(i,1) = (vy2(i,1) - vy2(i,1))/m*vx2(i,1);
%     else if ((vx2(i,2) >= 0) || (vx2(i,2) <= X) || (vy2(i,2) >= 0) || (vy2(i,2) <=Y)) 
%         m = (vy2(i,2) - vy2(i,1))/(vx2(i,2) - vx2(i,1));
%         vy2(i,2) = vy2(i,1)-m*(vx2(i,2) - vx2(i,1)); 
%         vx2(i,2) = (vy2(i,1) - vy2(i,1))/m*vx2(i,1);
%         end
% end
    
%% Plot values (for development only)
% figure
% plot(spaceVecX,spaceVecY,'.')
% hold on
% plot(obstX,obstY,'*')
% plot(Vx,Vy,'+')
% axis([0 X 0 Y])

% figure
% plot(Vx,Vy,'+')
% axis([-10 40 -10 40])
% plot(vx(1,:),vx(1,:),'g')
% plot(vy(1,:),vy(1,:),'r')