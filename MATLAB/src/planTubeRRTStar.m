function [path,T] = planTubeRRTStar(startPose, goalPose, map3D, setting)
% useIntVol: true is considering intersection volume in cost while false is
% not considering.

[minDis, minPoint]=findNearestPoint(map3D,startPose);
minDis = min(minDis,15);% constraint the max radius
% showCorridor(minDis, startPose);
%% 建树初始化
T.v(1).x = startPose(1);         % T是树，v是节点，先把起始点加入树中
T.v(1).y = startPose(2);
T.v(1).z = startPose(3);
T.v(1).xParent = startPose(1);     %起始节点的父节点依然是其本身
T.v(1).yParent = startPose(2);
T.v(1).zParent = startPose(2);
T.v(1).dist=0;          % 父节点到该节点的距离，可取欧氏距离
T.v(1).indParent = 0;     %父节点的index
T.v(1).radius = minDis;
T.v(1).vol = 0*exp(- 4/3*pi*(minDis/2)^3);
T.minDis = norm(startPose-goalPose);
T.maxRadius = 15;
T.minRadius = 3;
T.useintVol = setting.useIntVol;
T.costIntVol = setting.costIntVol;

planner.GoalBias = setting.GoalBias;
planner.MaxIterations = 2000;
planner.ContinueAfterGoalReached = setting.ContinueAfterGoalReached;
planner.MaxNumTreeNodes = setting.MaxNumTreeNodes;
bFind = false;
Delta= setting.MaxConnectionDistance;              % 设置扩展步长
xLimMin = setting.xLim(1);
xLimMax = setting.xLim(2);
yLimMin = setting.yLim(1);
yLimMax = setting.yLim(2);
zLimMin = setting.zLim(1);
zLimMax = setting.zLim(2);
for iter=1:planner.MaxIterations
    %Step 1: sample a free point in map
    E = diag([xLimMax-xLimMin,yLimMax-yLimMin,zLimMax-zLimMin]);
    x_rand = sampleFree(map3D,E)+[xLimMin yLimMin 0];
%     scatter3(x_rand(1), x_rand(2), x_rand(3), 'or');

    %Step 2: find the nearest point X_nearest in the tree
    [x_nearest, near_Idx] = Nearest(x_rand, T);

    %Step 3: 拓展得到x_new节点, 相似三角形
    %注意使用拓展步长Delta
    % x_new = Steer(x_rand, x_nearest, Delta);
    % update x_new to intersect with the sphere of the x_nearest point
    [x_new,x_newRadius] = tubeSteer(near_Idx,x_rand,T,map3D);

    if x_newRadius < T.minRadius
        continue;
    end

     %检查节点是否是collision free
    if ~collisionChecking(x_nearest,x_new,map3D) 
       
        
%         scatter3(x_new(1), x_new(2), x_new(3), 'ob','filled');
%         showCorridor(x_newRadius, x_new);

        % 找到x_new的临近节点 and it has intersection with x_new's shpere
        X_nears = NearTube(T, x_new, x_newRadius, near_Idx, map3D);

        % 找到使得x_new到起点cost最小的父节点x_min
        % wait for add sphere cost
        [x_min, min_Idx] = ChooseParentTube(X_nears, x_new, x_newRadius, T);
%         plot3([x_min(1),x_new(1)],[x_min(2),x_new(2)],[x_min(3),x_new(3)],'--b');
        %  将x_min作为x_new的父节点加入树中
        T =  AddNodeTube(T, x_new, x_min, min_Idx, x_newRadius);
        

        %   更新临近节点的父节点
        T = rewireTube(T, X_nears, x_new, map3D);

        
         %Step 5:检查是否达到目标点附近
        if bFind == false    % 判断是否已经找到路径
            dis_goal = norm(x_new-goalPose);
            if dis_goal < min([x_newRadius,planner.GoalBias])
                bFind = true;
                end_Idx = length(T.v);
            end
        else
            path = searchPath(T, end_Idx);
%             path = [startPose, minDis;
%                 path];
            % plot3(path(:,1),path(:,2),path(:,3),'g','linewidth',2);
            if ~planner.ContinueAfterGoalReached
                break;
            end
        end
        % if bFind
        %     break;
        % end


    end
end
end