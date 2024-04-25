function path = searchPath(T, end_Idx)
path = [];
idx = end_Idx;
while(1)
    if T.v(idx).indParent == 0 % find the goal point
        break;
    end

    path = [T.v(idx).x,T.v(idx).y,T.v(idx).z, T.v(idx).radius;
        path];
    idx = T.v(idx).indParent;
end

end