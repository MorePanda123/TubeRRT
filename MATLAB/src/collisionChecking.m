function feasible=collisionChecking(startPose,goalPose,map)
% ����Ϊ���ڵ�����͵�ͼ��Ϣ�������ڵ�ֱ�����Ӳ��ᾭ���ϰ���򷵻�true, �����ϰ�����Ϊ����false
feasible=true;
dis = norm(startPose - goalPose);
direction= (goalPose - startPose)/dis;
[~,feasible] = rayIntersection(map,[startPose 1 0 0 0],direction,dis);
end