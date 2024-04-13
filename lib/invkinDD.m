function [VR,VL] = invkinDD(X,Y,th,L,t)
    % X - target position in x (meters)
    % Y - target position in y (meters)
    % th- target orientation in th (radians)
    % L - wheel separation (meters)
    % t - time to accomplish the trajectory (seconds)
    % Notice: X, Y and th can not be set all at the same time
    % one of them must be NaN for the function to return valid results.

    if th == 0
        V = X / t;
        VL = V;
        VR = V;
        return;
    end

    W = th / t;

    if isnan(Y) || Y == 0
        V = W*X/sin(th);
    end

    if isnan(X) || X == 0
        V = W*Y/cos(th);
    end
    
    VR = V + W*L / 2;
    VL = V - W*L / 2;
end

