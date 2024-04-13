function [M] = rotx(a)
% Returns rotation matrix for 3D rotation around x axis
% a - angle in radians
M = [1 0 0 0
     0 cos(a) -sin(a) 0
     0 sin(a) cos(a) 0
     0 0 0 1];
end