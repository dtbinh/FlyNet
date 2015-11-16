function [xNode, yNode, cameFrom, closed, nowindex, step, noPath] = successor(xNode, yNode, openx, openy, cameFrom, length, neighbor, index, closed, goalx, goaly, step)
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
for i = 1:1:4       %CHECK - need to know length of row(neighbor)
    % index = current index | x&ynode = current point
    if i == 4;
        if distance(xNode, yNode, goalx, goaly) < 5
            bufferCost(i) = 1;
%             bufferIndex(i) = numel(
        else
            bufferCost(i) = 999;
        end
    else
        temp = neighbor(index, i)      % Get index of neighor
        if temp ~= 0
            if any(closed == temp) == 0
                if (xNode ~= openx(temp)) && (yNode ~= openy(temp))
                    if any(cameFrom == openx(temp)) == 0
                            % Calculate distance from current location to neighbor
                            bufferCostTemp(i) = distance(xNode, yNode, openx(temp), openy(temp));
                            % Calculate distance from neighbor to goal and sum both costs
                            bufferCost(i) = bufferCostTemp(i) + distance(openx(temp), openy(temp), goalx, goaly)
                            % Set buffer index
                            bufferIndex(i) = temp;
                                if bufferCostTemp(i) == bufferCost(i)
                                    bufferCost(i) = 899;
                                end
                            % add index to closed 
                            closed = [closed; temp]
                    else
                    bufferCost(i) = 900
                    closed = [closed; temp]
                    end
                else
                    bufferCost(i) = 901
                    closed = [closed; temp]
                end
            else
                bufferCost(i) = 902
            end
        else
            bufferCost(i) = 903
        end
    end
end
    

%% Select least cost
if (exist('bufferCost') == 1) && (exist('bufferIndex') == 1)
    [dum, bindex] = min(bufferCost);    % Jindex = index of buffer
    step = step + 1;                    % Step increments cameFrom index
    if bindex == 4
        
%     if dum < distance(xNode, yNode, goalx, goaly)
        cameFrom(step,1) = goalx;
        cameFrom(step,2) = goaly;
        nowindex = index;
        xNode = goalx;
        yNode = goaly;
        noPath = 0;
        bufferIndex = 0;
        stem = step +1;
    else
        nowindex = bufferIndex(bindex)      % get index of actual point
        cameFrom(step,1) = openx(bufferIndex(bindex));
        cameFrom(step,2) = openy(bufferIndex(bindex));
        noPath = 1;
        xNode = openx(bufferIndex(bindex));
        yNode = openy(bufferIndex(bindex));
    end
    bindex
else
    nowindex = index;
    bufferIndex = 0;
    noPath = 0;
    step = step + 1;
end
end