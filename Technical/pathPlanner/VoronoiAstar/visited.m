function [true] = visited(temp, openx, openy, cameFrom)
% Written by Bryce Hill
% 
% Purpose: Determine if proposed node has been previously visited
% 
% Inputs:
%   * node (x,y)
%   * openx (x)
%   * openy (y)
%   * cameFrom (path)
% 
% Outputs:
%   * true

true = 0;
for i = 1:1:numel(cameFrom)/2       % Divide by 2 because cameFrom = 1x2 array
    if (cameFrom(i,1) == openx(temp)) && (cameFrom(i,2) == openy(temp))
        true = 1;
    end
end
