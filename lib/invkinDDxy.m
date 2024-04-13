function [VR,VL] = invkinDDxy(X,Y,L,t,Xn,Yn)
    % X - target position in x (meters)
    % Y - target position in y (meters)
    % L - wheel separation (meters)
    % t - time to accomplish the trajectory (seconds)
    % Xn - next target position in x (meters)
    % Yn - next target position in y (meters)
    
    if X == 0 && Y == 0
        theta = atan2(Yn, Xn);
        W = theta / t;

        VR = W * L / 2;
        VL = -VR;
    else 
        D = norm([X Y]);
        V = D/t;
        VL = V; VR = V;
    end
end

