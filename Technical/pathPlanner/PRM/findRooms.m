clc; clear;
map = load('map.mat');
map = map.map;
tempmap = flipud(map);
% Define room dimensions Width, Height (assume rectangular)
maxy = size(map, 1);
maxx = size(map, 2);
ystep = floor(maxy / 4);
xstep = floor(maxx / 4);

poiy = ystep:ystep:maxy;
poix = xstep:xstep:maxx;
poi = zeros(maxy, maxx);
poi(poiy, poix) = 1;
poi(poi & tempmap) = 0;

% Node states: 0 - New, 1 - Visited
[gy, gx] = find(flipud(poi));
goalcoords = [gx, gy];
gv = zeros(length(gx), 1);
state = zeros(size(gx,1), 1);
n = 0;
start = [12.5, 2.5]*12;

% Create PRM
prm = PRM(map, 'npoints', 150);
randinit
prm.plan();
prm.plot();

%%%
tstart = tic;
%%%
for g = 1:length(gv)
    gv(g) = prm.graph.closest(goalcoords(g,:));
end
gv = unique(gv, 'stable');
prm.graph.highlight_node(gv)

% Find cost of moving between goal vertices
adjacency = prm.graph.adjacency();
adjgv = triu(adjacency(gv, gv));

for i = 1:length(gv)
    for j = 1:length(gv)
        
        if adjgv(i,j) ~= 0 || i == j
            continue
        end
        
        [~, adjgv(i,j)] = prm.graph.Astar(gv(i), gv(j));
        
    end
end

% http://www.mathworks.com/matlabcentral/fileexchange/35178-tspsearch/content/tspsearch.m

Lmin = inf;
% Nearest neighbour tour
p = greedy(1, adjgv);
% Improve tour by 2-opt heuristics
[p,L] = exchange2(p,adjgv);
% Keep best tour
if L < Lmin
    Lmin = L;
    pmin = p;
end

% Output
p = double(pmin);
L = Lmin;

%%%
telapsed = toc(tstart);
%%%
%%
% filename = 'pathexample.gif';

for n = 1:length(p)
    
    goal = goalcoords(p(n),:);
    prm.path(start, goal)
    start = goal;
    
    %     frame = getframe(1);
    %     im = frame2im(frame);
    %     [imind,cm] = rgb2ind(im,256);
    %     if n == 1;
    %         imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    %     else
    %         imwrite(imind,cm,filename,'gif','WriteMode','append');
    %     end
    
end

