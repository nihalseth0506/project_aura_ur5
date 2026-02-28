function X = interpolate_cartesian_segment(x_start, x_end, steps)

X = zeros(3,steps);

for i = 1:steps
    s = (i-1)/(steps-1);
    X(:,i) = (1-s)*x_start + s*x_end;
end

end