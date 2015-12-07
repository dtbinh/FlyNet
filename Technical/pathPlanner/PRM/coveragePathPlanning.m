function gotoNodes = coveragePathPlanning(map, nodeLocations)

nx = nodeLocations(1,:);
ny = nodeLocations(2,:);
maxrows = size(map,1);
maxcols = size(map,2);
numNodes = length(nx);
tempmap = map;
tempmap(sub2ind(size(map), ny, nx)) = 2;
space = 40;  % square of empty space to identify room

for n = 1:numNodes
    % Loop over each nodes to check if it lies in a "big" empty space
    nodeNbound = ny(n) + space;
    if nodeNbound > maxrows
        nodeNbound = maxrows;
    end
    nodeSbound = ny(n) - space;
    if nodeSbound < 1
        nodeSbound = 1;
    end
    nodeEbound = nx(n) + space;
    if nodeEbound > maxcols
        nodeEbound = maxcols;
    end
    nodeWbound = nx(n) - space;
    if nodeWbound < 1
        nodeWbound = 1;
    end
    
    % Throw out any nodes that collide within 'space'
    if any(any(map(nodeSbound:nodeNbound, nodeWbound:nodeEbound) == 1))
        tempmap(ny(n), nx(n)) = 0;
    end
        
end

distmax = 40; % Should be less than equal to 'space'
numNodes = length(find(tempmap == 2));
distmat = zeros(numNodes);
[ny, nx] = find(tempmap == 2);
for nn = 1:numNodes
    dist = sqrt((nx(nn) - nx).^2 + (ny(nn) - ny).^2);
    distmat(nn,:) = dist';
end

distmat = triu(distmat);
    
[r, ~] = find(distmat < distmax & distmat > 0);
tempmap(ny(r), nx(r)) = 0;


gotoNodes = (tempmap);
figure;
hold on
grid on
axis([0 maxcols 0 maxrows]);
[oy, ox] = find(gotoNodes == 1);
[y, x] = find(gotoNodes == 2);
scatter(ox, oy, 'rs', 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
scatter(x, y, 'bo', 'MarkerEdgeColor','k', 'MarkerFaceColor','b');



end