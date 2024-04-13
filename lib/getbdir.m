function b = getbdir(B, P, th, error)
    % B - beacon coordinates
    % P - current position in path
    % th - direction of motion in the global frame
    % b - angle <B,P,Q> mod 180º
    % error - if its applied a gaussian or not

    if nargin < 4
        error = 0;
    end

    PB = B - P;
    b = atan2(PB(2), PB(1)) - th;

    if error == 1
        sigma = 0.0175;
        b = b + sigma * randn;
    end

    b = mod(b, pi);
end

