function [P,h]=DrawRobot(t,scale)
    % P - points of robot to calculate its new coordinate
    % h - graphic handle of robot
    % t - type of robot
    % scale - of robot to draw- (default 0.01)
    
    if nargin < 2
        scale = 0.01;
    end
    
    switch t
        case 1      % DD
            P = [
                20 20 20 22 22 33 33 33 22 22 20 20 -20 -20 -22 -22 -33 -33 -22 -22 -22 -20 -20
                40 40 7 7 11 11 11 -7 -7 -3 -3 -10 -10 -3 -3 -7 -7 11 11 11 7 7 40
            ];
        case 2      % TRI
            P = [
                2 2 5 5 15 20 20 27 27 20 20 5 5 -5 -5 -20 -20 -27 -27 -20 -20 -14 -5 -5 -2 -2
                54 44 44 16 2 2 8 8 -8 -8 -3 -3 -12 -12 -3 -3 -8 -8 8 8 2 2 16 44 44 54
            ];
        case 3         % OMNI
            P = [
                12 12 4 4 21 44 35 40 44 51 39 32 36 31 21 -24 -33 -37 -34 -41 -52 -45 -41 -36 -46 -24 -4 -4 -12 -12
                52 44 44 39 39 0 -15 -18 -11 -15 -35 -31 -24 -21 -39 -39 -23 -26 -33 -37 -17 -13 -20 -17 0 39 39 44 44 52
            ];
    end

    P = scale * P;
    h = fill(P(1,:), P(2,:), 'y');
end