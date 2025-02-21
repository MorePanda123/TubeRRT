function T = rewireTube(T, X_nears, x_new, Imp)
% 对于除了最近节点x_near外的邻近节点来说 若通过x_new使其到起点的距离更短，更新x_new作为它的父节点，
count = size(X_nears, 2);
new_idx = size(T.v, 2);

for i = 1:count
    if X_nears(i) ~= T.v(new_idx).indParent
        pre_costDis = T.v(X_nears(i)).dist;
        pre_costVolume = T.v(X_nears(i)).vol;
        % pre_cost = exp(-pre_costVolume/(4/3*pi*T.maxRadius^3)) + pre_costDis/T.minDis;
        pre_cost =  pre_costDis/T.minDis + pre_costVolume;

        near_node(1) = T.v(X_nears(i)).x;
        near_node(2) = T.v(X_nears(i)).y;
        near_node(3) = T.v(X_nears(i)).z;

        tentative_costDis = norm(near_node - x_new) + T.v(new_idx).dist; % 通过x_new到起点的cost
        k_v = 0;
        if T.useintVol
            k_i = T.costIntVol;
        else
            k_i = 0;
        end
        interVol = intersectVolume(near_node, T.v(X_nears(i)).radius, x_new, T.v(new_idx).radius);
%         tentative_costVolume = k_v * exp(-4/3*pi*(T.v(X_nears(i)).radius/T.maxRadius)^3)...
%             + k_i * exp(-10*interVol/(4/3*pi*T.maxRadius^3)+1)...   
%             + T.v(new_idx).vol;
        tentative_costVolume = k_v * exp(-4/3*pi*(T.v(X_nears(i)).radius/T.maxRadius)^3)...
            + k_i * 1 / (interVol/(4/3*pi*T.maxRadius^3)+1)...   
            + T.v(new_idx).vol;
        % tentative_cost = tentative_costDis/T.minDis + exp(-tentative_costVolume/(4/3*pi*T.maxRadius^3));
        
        tentative_cost = tentative_costDis/T.minDis + tentative_costVolume;
        if ~collisionChecking(near_node, x_new,Imp)  % rewire过程中也要碰撞检测
            if tentative_cost < pre_cost  % 若通过起点使cost降低，改变其父节点为x_new
                % plot3([T.v(X_nears(i)).x, T.v(X_nears(i)).xParent],...
                %     [T.v(X_nears(i)).y,T.v(X_nears(i)).yParent],...
                %     [T.v(X_nears(i)).z,T.v(X_nears(i)).zParent],'-w');
                T.v(X_nears(i)).xParent = x_new(1);
                T.v(X_nears(i)).yParent = x_new(2);
                T.v(X_nears(i)).zParent = x_new(3);
                T.v(X_nears(i)).dist = tentative_costDis;
                T.v(X_nears(i)).indParent = new_idx;
                T.v(X_nears(i)).vol = tentative_costVolume;
                % plot3([T.v(X_nears(i)).x, T.v(X_nears(i)).xParent],...
                %     [T.v(X_nears(i)).y,T.v(X_nears(i)).yParent],...
                %     [T.v(X_nears(i)).z,T.v(X_nears(i)).zParent],'--b');
            end
        end
    end
    
end

end