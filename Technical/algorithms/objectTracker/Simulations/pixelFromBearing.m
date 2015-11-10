function [finalIntersect] = pixelFromBearing(az, el, pos, los)

x = pos(1);
y = pos(2);
z = pos(3);

%% Vertices
a = [0,0,0];
b = [0,18,0];
c = [9,18,0];
d = [9,0,0];
e = [0,0,9];
f = [0,18,9];
g = [9,18,9];
h = [9,0,9];

%% Equations of Planes
bottom.eq = cross(b-a, d-a);
bottom.vert1 = a;
bottom.vert2 = b;
bottom.vert3 = c;
bottom.vert4 = d;
bottom.color = [128, 128, 128];
left.eq = cross(b-a, e-a);
left.vert1 = a;
left.vert2 = b;
left.vert3 = e;
left.vert4 = f;
left.color = [128, 128, 128];
right.eq = cross(c-d, h-d);
right.vert1 = c;
right.vert2 = d;
right.vert3 = g;
right.vert4 = h;
right.color = [128, 128, 128];
top.eq = cross(f-e, h-e);
top.vert1 = e;
top.vert2 = f;
top.vert3 = g;
top.vert4 = h;
top.color = [128, 128, 128];
front.eq = cross(f-b, c-b);
front.vert1 = b;
front.vert2 = c;
front.vert3 = f;
front.vert4 = g;
front.color = [128, 128, 128];
back.eq = cross(e-a, d-a);
back.vert1 = a;
back.vert2 = d;
back.vert3 = e;
back.vert4 = h;
back.color = [128, 128, 128];
target.vert1 = [4, 12, 0];
target.vert2 = [4, 12, 6];
target.vert3 = [6, 12, 0];
target.vert4 = [6, 12, 6];
target.eq = cross(target.vert2 - target.vert1, target.vert4 - target.vert1);
target.color = [205, 205, 205];
planes{1} = bottom;
planes{2} = left;
planes{3} = right;
planes{4} = top;
planes{5} = front;
planes{6} = back;
planes{7} = target;

%% Rotate Az El from body coordinates to global coordinates

ssc = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
RU = @(A,B) eye(3) + ssc(cross(A,B)) + ...
     ssc(cross(A,B))^2*(1-dot(A,B))/(norm(cross(A,B))^2);
     
% Rotation from body to global
if norm(cross(los/norm(los), [1; 0; 0]))
    R = RU([1; 0; 0], los/norm(los));
else
    R = eye(3)*dot(los/norm(los), [1; 0; 0]);
end

% Rotate Az El vector to global coordinates
globalAzEl = R*[cos(az); sin(az); sin(el)];

%% Begin instersection calculations

intersections = {};
intIdx = 1;

for idx=1:length(planes)
    
    planes{idx}.d = planes{idx}.eq(1)*planes{idx}.vert1(1) +...
        planes{idx}.eq(2)*planes{idx}.vert1(2) +...
        planes{idx}.eq(3)*planes{idx}.vert1(3);
    
    intT = -(x*planes{idx}.eq(1) + y*planes{idx}.eq(2) + z*planes{idx}.eq(3) - planes{idx}.d)/(globalAzEl(1)*planes{idx}.eq(1) + globalAzEl(2)*planes{idx}.eq(2) + globalAzEl(3)*planes{idx}.eq(3));
    if ~isnan(intT) && ~isinf(intT) && intT > 0
        xint = globalAzEl(1)*intT + x;
        yint = globalAzEl(2)*intT + y;
        zint = globalAzEl(3)*intT + z;
        dist = norm([xint, yint, zint] - [x, y, z]);
        maxx = max([planes{idx}.vert1(1), planes{idx}.vert2(1), planes{idx}.vert3(1), planes{idx}.vert4(1)]);
        minx = min([planes{idx}.vert1(1), planes{idx}.vert2(1), planes{idx}.vert3(1), planes{idx}.vert4(1)]);
        maxy = max([planes{idx}.vert1(2), planes{idx}.vert2(2), planes{idx}.vert3(2), planes{idx}.vert4(2)]);
        miny = min([planes{idx}.vert1(2), planes{idx}.vert2(2), planes{idx}.vert3(2), planes{idx}.vert4(2)]);
        maxz = max([planes{idx}.vert1(3), planes{idx}.vert2(3), planes{idx}.vert3(3), planes{idx}.vert4(3)]);
        minz = min([planes{idx}.vert1(3), planes{idx}.vert2(3), planes{idx}.vert3(3), planes{idx}.vert4(3)]);
        
        if xint - maxx > 1e-9 || minx - xint > 1e-9 || yint - maxy > 1e-9 || miny - yint > 1e-9 || zint - maxz > 1e-9 || minz - zint > 1e-9
            continue
        end
            
        intersections{intIdx}.loc = [xint, yint, zint];
        intersections{intIdx}.dist = dist;
        intersections{intIdx}.color = planes{idx}.color;
            
        intIdx = intIdx + 1;
    else
        continue
    end    
end

minDist = 99999999;
finalIntersect.valid = 0;

for idx=1:length(intersections)
    intersections{idx}.valid = 1;
    if intersections{idx}.dist < minDist
        minDist = intersections{idx}.dist;
        finalIntersect = intersections{idx};
    end
end