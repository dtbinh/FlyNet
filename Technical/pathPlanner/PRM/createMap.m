ftrows = 50;
ftcols = 25;
inrows = ftrows * 12;
incols = ftcols * 12;

map = zeros(inrows+1, incols+1);

obstacleNodes = ...
   [5,    5;
    5,    10;
    17.5, 5;
    12.5, 10;
    12.5, 25;
    17.5, 20;
    30,   0;
    30,   10;
    30,   17.5;
    30,   20;
    35,   5;
    35,   10;
    35,   15;
    40,   15;
    45,   10;
    50,   15] * 12 + 1;

obstacleInd = sub2ind(size(map), obstacleNodes(:,1), obstacleNodes(:,2));
map(obstacleInd) = 1;