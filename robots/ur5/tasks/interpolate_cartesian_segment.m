function X = interpolate_cartesian_segment(x0, x1, N)

t = linspace(0,1,N);
X = zeros(3,N);

for i = 1:N
    s = 10*t(i)^3 - 15*t(i)^4 + 6*t(i)^5;  % minimum jerk
    X(:,i) = x0 + s*(x1 - x0);
end

end