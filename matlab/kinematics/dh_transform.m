function A = dh_transform(theta, d, a, alpha)
% DH_TRANSFORM Computes standard Denavit-Hartenberg transformation matrix
%
% Inputs:
%   theta - Joint angle (symbolic or numeric)
%   d     - Link offset
%   a     - Link length
%   alpha - Link twist
%
% Output:
%   A     - 4x4 homogeneous transformation matrix

A = [ cos(theta),            -sin(theta)*cos(alpha),   sin(theta)*sin(alpha),    a*cos(theta);
      sin(theta),             cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),    a*sin(theta);
      0,                      sin(alpha),              cos(alpha),               d;
      0,                      0,                       0,                        1 ];

end
