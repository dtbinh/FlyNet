function [xNode, yNode, cameFrom, closed, jindex, step, noPath] = successor(xNode, yNode, openx, openy, cameFrom, length, neighbor, index, closed, goalx, goaly, step)
% function [] = astar(goalx, goaly, startx, starty)
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

%% Calculate cost to neighbors
for i = 1:1:3       %CHECK - need to know length of row(neighbor)
    % index = current index | x&ynode = current point
    temp = neighbor(index, i)      % Get index of neighor
    if temp ~= 0
        if any(closed == temp) == 0
        % Calculate distance from current location to neighbor
        bufferCostTemp(i) = distance(xNode, yNode, openx(temp), openy(temp));
        % Calculate distance from neighbor to goal and sum both costs
        bufferCost(i) = bufferCostTemp(i) + distance(openx(temp), openy(temp), goalx, goaly)
        % Set buffer index
        bufferIndex(i) = temp;
            if bufferCostTemp(i) == bufferCost(i)
                bufferCost(i) = 1000;
            end
        % add index to closed 
        closed = [closed; temp]
        else
            bufferCost(i) = 1000
        end
    else
        bufferCost(i) = 1000
    end
end
    

%% Select least cost
if (exist('bufferCost') == 1) && (exist('bufferIndex') == 1)
    [dum, jindex] = min(bufferCost);    % Jindex = index of current point
    step = step + 1;                    % Step increments cameFrom index
    cameFrom(step,1) = openx(bufferIndex(jindex));
    cameFrom(step,2) = openy(bufferIndex(jindex));
    trail = [openx(jindex),openy(jindex)];  % Update current node
    noPath = 1;
else
    jindex = index;
    bufferIndex = 0;
    noPath = 0;
    step = step + 1;
end


% bufferClosed 
%% Move neighbors to closed list, save path
% closed = [closed; bufferClosed']
% noPath = 1;
%% Check if done
% if numel(closed) > numel(openx - 1)
%     noPath = 0;
% end
end