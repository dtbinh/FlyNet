function [] = Space(sizex, sizey, startx, starty, goalx, goaly, margin)
% Written by Bryce Hill
% 
% Purpose: Define 2-D space to search 
% 
% Inputs:
%   * sizex/y
%   * startx/y
%   * goalx/y
% 
% Outputs:
%   * 2-D point .mat file
% 
% ToDo:
%   * Trim space points
%   * Incorporate obstacle margin

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
load('mapt.mat')
obstX = mapt(:,1);
obstY = mapt(:,2);
% obstX = [16, 4,  5, 8, 16,  5, 14, 10, 10, 18, 10, 10, 10];
% obstY = [ 8, 3, 10, 4,  3, 18, 14, 8,  12, 18, 17, 11, 6]; 
% obstX = [16, 4,  5, 8, 16,  5];
% obstY = [ 8, 3, 10, 4,  3, 18]; 

%% Generate points equi-distant between obstacles 
% - Populate nx1 matrices Vx and Vy which hold in bounds only voronoi
%   generated waypoints
[vx, vy] = Voronoi(obstX, obstY);   % Calculate voronoi
% axis([-110 41 -70 41])
axis([-1 31 -1 61])

%% Concatenate vx and vy
for i = 1:1:numel(vx)
    vxf(i) = vx(i);
    vyf(i) = vy(i);
end
x = vxf;
y = vyf;

%% Trim points to be within the valid space
% for i = 1:2:numel(x)    % Loop over number of lines
%     % case 9: both out of bounds
%     if (x(i) < 0) || (x(i) > sizex)
%         if (x(i+1) < 0) || (x(i+1) > sizex) 
%             if (y(i) < 0) || (y(i) > sizey)
%                 if (y(i+1) < 0) || (y(i+1) > sizey)
%             % erase neighbors of 1st point, path is invalid
% %             neighbor(i,2) == 0;
% %             neighbor(i,3) == 0;
% %             neighbor(i,4) == 0;
%             % erase neighbors of 2nd point
% %             neighbor(i+1,2) == 0;
% %             neighbor(i+1,3) == 0;
% %             neighbor(i+1,4) == 0;
%             % Delete nodes
%             disp('i')
%             xplot = [x(i+1), x(i)];
%             yplot = [y(i+1), y(i)];
%             plot(xplot, yplot, '-.w')
%             x(i) = 0;
%             x(i+1) = 0;
%             y(i) = 0;
%             y(i+1) = 0;
%             x = [x(1:(i-1)),x((i+1):end)];
%             y = [y(1:(i-1)),y((i+1):end)];
%                 end
%             end
%         end
%     else
%         disp('wo')
%     end
% 
%     % case 1: x < 0
% %     elseif (x(i) < 0) || (x(i) > sizex)
% %         m = (y(i+1) - y(i)) / (x(i+1) - x(i));
% %         for i = x(i):.5:x(i+1)
% %             xnew = (y(i+1) - y(i))/m - x(i+2);
% %             if (xnew > 0) && (xnew < sizex)
% %                 break
% %             end
% %         end
% end

%% Calculate neighbors
numel(x);
for i = 1:1:numel(x)
    index = 1;
    flag = 0;
    for j = 1:1:numel(x)
        if x(i) == x(j)     % Check that x's are equal
            if y(i) == y(j)     % Check that y's are equal
                if mod(j,2) == 0    % Even result          
                    neighbor(i,index) = j-1;
                    index = index +1;
                    flag = 1;
                else                % Odd result
                    neighbor(i,index) = j+1;
                    index = index +1;
                    flag = 1;
                end
            end
        end
    end
    if(flag == 0)
%         neighbor(i,index) = 18;
    end
    index = 1;
end

%% Eliminate connections within marginal distance of obstacles
for i = 1:2:numel(x)    % Loop over number of lines
    for j = 1:1:numel(obstX)    % For each line, loop over each obstacle
        % Define variables for ease of readability
        y2 = y(i+1);
        y1 = y(i);
        x2 = x(i+1);
        x1 = x(i);
        px = obstX(j);
        py = obstY(j);
   
        % Calculate distance from line to obstacle
        d = abs((y2-y1)*px-(x2-x1)*py+x2*y1-y2*x1)/sqrt((y2-y1)^2+(x2-x1)^2);
        
        % Check to find vertex, the point on the line closest to the
        % obstaculo
        d1obst = distance(x1, y1, px, py);
        d2obst = distance(x2, y2, px, py);
        if d1obst < d2obst
            d13 = d1obst;
            d23 = d2obst;
        else
            d13 = d2obst;
            d23 = d1obst;
        end
        d12 = distance(x1, y1, x2, y2);
        angle = acosd((d12^2+d13^2-d23^2)/(2*d12*d13));
        
        if (d <= margin) && (angle <= 90)      % if obstacle is too close to path
            % -------------------------------------------------------------
            % Eliminate connection between path that has been found to be
            % too close to an obstacle
            for k = 1:1:3 
                if neighbor(i,k) == i+1     % find neighbor of ith node that is i+1 and delete that connection
                    neighbor(i,k) = 0;                    
                    % visualization aids ---------
                    plot(obstX(j),obstY(j),'or')    % Plot encroaching obstaculo
                    xth = (x2-x1)/2 + min(x2,x1);
                    yth = (y2-y1)/2 + min(y2,y1);
                    xths = [xth obstX(j)];
                    yths = [yth obstY(j)];
                    %plot(xths,yths,'--g')
                    xx = [x1 x2];
                    yy = [y1 y2];
                    plot(xx, yy,'w')
                    % ----------------------------
                end     
                if neighbor(i+1,k) == i     % find neighbor of i+1 node that is ith and delete that connection
                    neighbor(i+1,k) = 0;    
                end
            end
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            % -------------------------------------------------------------
            % Check all other sister nodes and eliminate their connections
            % through the path that has been deemed unpassable
            for l = 1:1:numel(x)
                if (x(l) == x(i))
                    l;
                    i;
                  for k = 1:1:3
                      if neighbor(l,k) == i+1
                          neighbor(l,k) = 0;
                      end
                  end
                end
            end
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        end
    end
end
                
                    

%% Publish values
save('space.mat');
end










% %% Eliminate connections within marginal distance of obstacles
% for i = 1:2:numel(x)    % Loop over number of lines
%     for j = 1:1:numel(obstX)    % For each line, loop over each obstacle
%         % Define variables for ease of readability
%         y2 = y(i+1);
%         y1 = y(i);
%         x2 = x(i+1);
%         x1 = x(i);
%         px = obstX(j);
%         py = obstY(j);
%         % Calculate distance from line to obstacle
%         d = abs((y2-y1)*px-(x2-x1)*py+x2*y1-y2*x1)/sqrt((y2-y1)^2+(x2-x1)^2);
%         % Calculate distance from line origin 
%         d1 = distance(x1, px, y2, py);
%         if d <= margin      % if obstacle is too close to path
%             for k = 1:1:3
%                 if neighbor(i,k) == i+1     % find neighbor of ith node that is i+1 and delete that connection
%                     neighbor(i,k) == 0;
%                     plot(obstX(j),obstY(j),'or')    % Plot encroaching obstaculo
%                     xth = (x2-x1)/2 + min(x2,x1);
%                     yth = (y2-y1)/2 + min(y2,y1);
%                     xths = [xth obstX(j)];
%                     yths = [yth obstY(j)];
%                     plot(xths,yths,'-.g')
%                 end
%                 if neighbor(i+1,k) == i     % find neighbor of i+1 node that is ith and delete that connection
%                     neighbor(i+1,k) == 0    
%                 end
%             end
%         end
%     end
% end
             

% beforetrimm = numel(x)
% x'
% % xx= x;
% % yy = y;
% % neighborr = neighbor;
% [~,ia] = unique(x, 'stable')
% %% Get rid of repeat points
% for i = 1:1:numel(ia)
%     xx(i) = x(ia(i));
%     yy(i) = y(ia(i));
%     neighborr(i,:) = neighbor(ia(i),:);
% end
% % x = xx;
% % y = yy;
% % neighbor = neighborr;
% aftertrim = numel(x)
% x'

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