function [xNode, yNode, cameFrom, closed, nowindex, step, noPath] = successor(xNode, yNode, openx, openy, cameFrom, length, neighbor, index, closed, goalx, goaly, step, term)
% Written by Bryce Hill
% 
% Purpose: _ 2-D space to search 
% 
% Inputs:
%   * node (x,y)
%   * open (struct)
%   * cameFrom (path)
% 
% Outputs:
%   * node (x,y)
%   * open (struct)
%   * cameFrom (path)
% 
% ToDo:
%   * If no nodes are within termination radius (term) then algorithm
%       exhausts field in its search before finally jumping to the goal

%% Calculate cost to neighbors
for i = 1:1:4       %CHECK - need to know length of row(neighbor)
    % index = current index |&| x&ynode = current point
    if i == 4;      % If the goal is within termination radius(term) units then search is over, jump to goal
        if distance(xNode, yNode, goalx, goaly) < term
            bufferCost(i) = 1;
        else
            bufferCost(i) = 999;
        end
    else
        temp = neighbor(index, i);      % Get index of neighbor (un ; for debugging
        if temp ~= 0
            if any(closed == temp) == 0     % check if temp = previously visited index
                if (xNode ~= openx(temp)) || (yNode ~= openy(temp))     % Is the prospective point = to 
                    if visited(temp, openx, openy, cameFrom) == 0       % check if point has been previously visited (due to duplicate points)
                        % Calculate distance from current location to neighbor
                            bufferCostTemp(i) = distance(xNode, yNode, openx(temp), openy(temp));
                        % Calculate distance from neighbor to goal and sum both costs
                            bufferCost(i) = bufferCostTemp(i) + distance(openx(temp), openy(temp), goalx, goaly); % un ; for debugging
                        % Set buffer index
                            bufferIndex(i) = temp;
                                if bufferCostTemp(i) == bufferCost(i)
                                    bufferCost(i) = 899;
                                end
                        % add index to closed 
                            closed = [closed; temp];
                    else
                        bufferCost(i) = 900;     % temp had been previously visited but has a different index
                        closed = [closed; temp];
                    end
                else
                    bufferCost(i) = 901;     % temp = current point
                    closed = [closed; temp];
                end
            else
                bufferCost(i) = 902;     % temp is on closed list
            end
        else
            bufferCost(i) = 903;     % temp cannot be 0
        end
    end
end
    

%% Select least cost
if (exist('bufferCost') == 1) && (exist('bufferIndex') == 1)
    [dum, bindex] = min(bufferCost);    % Bindex = index of buffer
    step = step + 1;                    % Step increments cameFrom index
    if bindex == 4
        cameFrom(step,1) = goalx;
        cameFrom(step,2) = goaly;
        nowindex = index;
        xNode = goalx;
        yNode = goaly;
        noPath = 0;
        bufferIndex = 0;
        stem = step +1;
    else
        nowindex = bufferIndex(bindex);      % get index of actual point
        cameFrom(step,1) = openx(bufferIndex(bindex));
        cameFrom(step,2) = openy(bufferIndex(bindex));
        noPath = 1;
        xNode = openx(bufferIndex(bindex));
        yNode = openy(bufferIndex(bindex));
    end
    bindex;
else
    nowindex = index;
    bufferIndex = 0;
    noPath = 0;
    step = step + 1;
end
end