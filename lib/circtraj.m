function MM = circtraj(P1,P2,R,N)
    % MM.xy - the matrix of xy points
    % MM.angle - the vector of orientations
    % MM.T - the associated geometric transformation

    d = norm(P2 - P1);
    R = max(R,d/2);             % ensure R radius >= d/2
    if isinf(R); R = 1e6; end   % use very large R

    C = (P2 + P1)/2 + sqrt(R^2 - d^2/4)/d * [0, -1; 1, 0] * (P2 - P1);
    ang_P1 = atan2(P1(2)-C(2), P1(1)-C(1));
    ang_P2 = atan2(P2(2)-C(2), P2(1)-C(1));
    ang_P2 = mod(ang_P2-ang_P1, 2*pi) + ang_P1; % ensures the circunference goes counterclockwise

    t = linspace(ang_P1, ang_P2, N);
    MM.xy = C + R * [cos(t); sin(t)];

    dM = diff(MM.xy,1,2);                       % first diff along two dimensions
    MM.angle = atan2(dM(2,:), dM(1,:));
    MM.angle(end+1) = MM.angle(end);

    MM.T = zeros(3, 3, width(MM.xy));
    for n = 1:size(MM.T,3)
        MM.T(:,:,n) = transl(MM.xy(:,n)) * rotat(MM.angle(n));
    end

end

