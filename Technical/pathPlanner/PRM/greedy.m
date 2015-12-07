function p = greedy(s,D)
%GREEDY Travel to nearest neighbour, starting with node s.
%http://www.mathworks.com/matlabcentral/fileexchange/35178-tspsearch/content/tspsearch.m

n = size(D,1);
p = zeros(1,n,'uint16');
p(1) = s;

for k = 2:n
    D(s,:) = inf;
    [junk,s] = min(D(:,s)); %#ok
    p(k) = s;
end