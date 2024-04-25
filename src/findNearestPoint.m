function [minDis, minPoint] = findNearestPoint(sv,point)
%UNTITLED2 此处提供此函数的摘要
%   此处提供详细说明
numRays = 30;
angles = linspace(-pi,pi,numRays);
intersectionPoints = [];
for alpha = -pi/2:0.1:pi/2
    directions = [cos(alpha)*cos(angles); cos(alpha)*sin(angles); sin(alpha)*ones(1,numRays)]';
    intersectionPoints = [intersectionPoints; rayIntersection(sv,[point 1 0 0 0],directions,100)];
end
disPoints = intersectionPoints - point;
minDis = 1000;
for k=1:length(intersectionPoints(:,1))
    disTemp = norm(disPoints(k,:));
    if disTemp < minDis
        minDis = disTemp;
        minPoint = intersectionPoints(k,:);
    end
end
end