function T = transl(v)

    if numel(v) < 2
        v=[v(1) v(1)];
    end

    T = [ 1 0 v(1)
          0 1 v(2)
          0 0   1
        ];
    
end

