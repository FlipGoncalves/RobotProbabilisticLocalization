function [Vx, Vy, w] = localvels(t, r, L, w1, aw2, w3)
    % Vx, Vy, w - velocities in local frame
    % t   - type of robot (DD/1, TRI/2, OMNI/3)
    % r   - traction wheel radius
    % L   - meaning depending on type (wheel sep/1, wheel dist/2, wheel dist/3)
    % w1  - angular velocity of wheel 1 (right wheel/1, steering/2)
    % aw2 - angular velocity of wheel 2 (left wheel/1 or alpha/2)
    % w3  - angular velocity of wheel 3 (OMNI)

    if t == 1           % DD
        R = [r/2   r/2
             0     0
             -r/L  r/L];
        V = R * [w1 aw2]';
        Vx = V(1); Vy = V(2); w = V(3);
    elseif t == 2       % TRI
        Vx = w1 * r * cos(aw2); 
        Vy = 0; 
        w = w1 * r / L * sin(aw2);
    else                % OMNI
        R = [ 0        r/sqrt(3)  -r/sqrt(3)
             -2*r/3    r/3         r/3
              r/(3*L)  r/(3*L)     r/(3*L)];
        V = R * [w1 aw2 w3]';
        Vx = V(1); Vy = V(2); w = V(3);
    end
end

