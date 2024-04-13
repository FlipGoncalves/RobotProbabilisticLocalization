function MM = traject(P1,P2,N)
    % MM.xy - the matrix of xy points
    % MM.angle - the vector of orientations
    % MM.T - the associated geometric transformation

    MM.xy = [ linspace(P1(1), P2(1), N)
              linspace(P1(2), P2(2), N)
            ];

    dM = diff(MM.xy,1,2);                       % first diff along two dimensions
    MM.angle = atan2(dM(2,:), dM(1,:));
    MM.angle(end+1) = MM.angle(end);

    MM.T = zeros(3, 3, width(MM.xy));
    for n = 1:size(MM.T,3)
        MM.T(:,:,n) = transl(MM.xy(:,n)) * rotat(MM.angle(n));
    end

end

