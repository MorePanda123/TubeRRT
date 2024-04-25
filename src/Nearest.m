function [x_near, near_Idx] = Nearest(x_rand, T)
% ����T���������������x_rand����Ľڵ㣬�����������������е�����
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
