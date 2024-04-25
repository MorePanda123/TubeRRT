% This demo is used for the paper of tube path planning
clear;
close all;
addpath("src\");
%% generate 3-D occupancy map
map3D = occupancyMap3D(0.1);
pose = [ 0 0 0 1 0 0 0];
randNum = 30;
maxRange = 200;
pointsx = rand(randNum,1)*maxRange+20;
pointsy = rand(randNum,1)*maxRange;
points1 = [pointsx pointsy, ones(randNum,1)];
points2 = [pointsx pointsy, 11*ones(randNum,1)];
points3 = [pointsx pointsy, 21*ones(randNum,1)];
points = [points1;points2;points3];
insertPointCloud(map3D,pose,points,maxRange*1.5);
pointsFloor = [];
for x = 0:1:300
    for y = 0:1:300
        pointsFloor = [pointsFloor; x, y, -0.5];
    end
end
insertPointCloud(map3D,pose,pointsFloor,maxRange*1.5);
map3D.FreeThreshold = map3D.OccupiedThreshold;

xLimMin = 0;
xLimMax = 240;
yLimMin = 0;
yLimMax = 200;
zLimMin = 0;
zLimMax = 30;
%% path planning
startPose = [10 110 7];  % [x y z ]
goalPose = [220 100 15];

% ----------------RRT*-----------------
setting.ContinueAfterGoalReached = true;
setting.MaxConnectionDistance = 15;
setting.GoalBias = 15;
setting.MaxNumTreeNodes = 1000;
setting.xLim = [xLimMin xLimMax];
setting.yLim = [yLimMin yLimMax];
setting.zLim = [zLimMin zLimMax];
setting.useIntVol = true;
setting.costIntVol = 0.015;
[path,tree] = planTubeRRTStar(startPose, goalPose, map3D, setting);
%% plot
figure,
hold on;
show(map3D)
colormap gray
xlim([0 240.0])
ylim([0 200.0])
zlim([0.00 30.0])
scatter3(startPose(1),startPose(2),startPose(3),"g",'filled');
scatter3(goalPose(1),goalPose(2),goalPose(3),"g",'filled');

plot3(path(:,1),path(:,2),path(:,3),'-g','linewidth',2);
for k=1:length(path(:,1))
   showCorridor(path(k,4), path(k,1:3)); 
end
tubePaths = genTubePath2(path);
wp1 = tubePaths(1).path;
wp2 = tubePaths(2).path;
wp3 = tubePaths(3).path;
wp4 = tubePaths(4).path;

hold on;
plot3(wp1(:,1),wp1(:,2),wp1(:,3),'-r','linewidth',1);
plot3(wp2(:,1),wp2(:,2),wp2(:,3),'-r','linewidth',1);
plot3(wp3(:,1),wp3(:,2),wp3(:,3),'-r','linewidth',1);
plot3(wp4(:,1),wp4(:,2),wp4(:,3),'-r','linewidth',1);

scatter3(wp1(:,1),wp1(:,2),wp1(:,3),"r",'filled');
scatter3(wp2(:,1),wp2(:,2),wp2(:,3),"r",'filled');
scatter3(wp3(:,1),wp3(:,2),wp3(:,3),"r",'filled');
scatter3(wp4(:,1),wp4(:,2),wp4(:,3),"r",'filled');




