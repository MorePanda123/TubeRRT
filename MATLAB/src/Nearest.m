function [x_near, near_Idx] = Nearest(x_rand, T)
% 在树T中搜索距离随机点x_rand最近的节点，返回它及其它在树中的索引
    count = size(T.v,2);
    min_dis = 10000;
    for node = 1: count
        dis = sqrt(power((T.v(node).x-x_rand(1)) ,2) + ...
            power((T.v(node).y - x_rand(2)), 2) + ...
            power((T.v(node).z - x_rand(3)), 2));
        if dis<min_dis
            min_dis = dis;
            near_Idx = node;
            x_near(1) = T.v(node).x;
            x_near(2) = T.v(node).y;
            x_near(3) = T.v(node).z;
        end
    end
end
