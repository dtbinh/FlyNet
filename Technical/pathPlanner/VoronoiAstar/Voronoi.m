function [vx,vy] = Voronoi(obstX, obstY)
% Written by Bryce Hill
% 
% Purpose: _ 2-D space to search 
% 
% Inputs:
%   * none
% 
% Outputs:
%   * 

%load('space.mat')


% tri = voronoi(spaceVecX,spaceVecY)
voronoi(obstX,obstY);
hold on 
plot(obstX,obstY,'*')
[vx,vy] = voronoi(obstX,obstY);
% [vx,vy] = voronoin([obstX(:) obstY(:)]);
% vx(vx>sizex)=[];
% vy(vy>sizey)=[];
% vx(vx<0)=[]
% vy(vy<0)=[]
% hold on
% plot(vx,vy,'*')
% axis([min(obstX) max(obstX) min(obstY) max(obstY)])

end