function [M] = roty(a)
% Returns rotation matrix for 3D rotation around y axis
% a - angle in radians
M = [cos(a) 0 sin(a) 0
     0 1 0 0
     -sin(a) 0 cos(a) 0
     0 0 0 1];
end