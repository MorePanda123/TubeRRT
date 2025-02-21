function feasible=collisionChecking(startPose,goalPose,map)
% 输入为两节点坐标和地图信息，若两节点直线连接不会经过障碍物，则返回true, 碰到障碍物则为返回false
feasible=true;
dis = norm(startPose - goalPose);
direction= (goalPose - startPose)/dis;
[~,feasible] = rayIntersection(map,[startPose 1 0 0 0],direction,dis);
end