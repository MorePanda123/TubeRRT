function [x_min, min_idx] = ChooseParentTube(X_nears, x_new, x_newRadius, T)
% 在邻近集合X_nears中，找到使得x_new通往起点距离最短的父节点
nearest = [];
min_cost = 10000; %计算x_new->nearest node->起点需要的cost
count = size(X_nears, 2);

for i = 1:count
    nearest(1) = T.v(X_nears(i)).x;
    nearest(2) = T.v(X_nears(i)).y;
    nearest(3) = T.v(X_nears(i)).z;
    % cost of total distance
    costDis = norm(nearest - x_new) + T.v(X_nears(i)).dist; 
    % cost of total volume and intersection volume
    k_v = 0;
    if T.useintVol
        k_i = T.costIntVol;
    else
        k_i = 0;
    end
    
    interVol = intersectVolume(nearest, T.v(X_nears(i)).radius, x_new, x_newRadius);
%     costVolume = k_v*exp(-4/3*pi*(x_newRadius/T.maxRadius)^3) + ...
%         k_i*exp(-10*interVol/(4/3*pi*T.maxRadius^3)+1) + ...
%         T.v(X_nears(i)).vol;
    costVolume = k_v*exp(-4/3*pi*(x_newRadius/T.maxRadius)^3) + ...
        k_i*(interVol/(4/3*pi*T.maxRadius^3)+1)^(-1) + ...
        T.v(X_nears(i)).vol;

    
    cost = costDis/T.minDis + costVolume;

    if cost<min_cost 
        min_cost = cost;
        x_min = nearest;
        min_idx = X_nears(i);
    end
end


end
    