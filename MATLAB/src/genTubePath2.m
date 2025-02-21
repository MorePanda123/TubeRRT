function tubePaths = genTubePath2(path)
% path = [pose radius]
lenPath = length(path(:,1));
% tubePaths = zeros(lenPath-1,8);
tubePaths(1).path = [];tubePaths(2).path = [];
tubePaths(3).path = [];tubePaths(4).path = [];
tubePaths(5).path = [];
for k=2:lenPath
%    showCorridor(path(k,4), path(k,1:3)); 
   if k==1
       direction = (path(k+1,1:3) - path(k,1:3)) / ...
           norm(path(k+1,1:3) - path(k,1:3));
%        tubePaths(k,:) = [path(k,1:3) direction path(k,4) path(k,4)];
   else
       last.pose = path(k-1,1:3);
       last.radius = path(k-1,4);
       now.pose = path(k,1:3);
       now.radius = path(k,4);
       [~,interTubePoint] = scoreCorridor(last,now);
       
       tubePaths(5).path = [tubePaths(5).path; interTubePoint.pose];
       % the first path       
       extreme_points1 = interTubePoint.pose + interTubePoint.radius * ...
           interTubePoint.normal_vect;
       tubePaths(1).path = [tubePaths(1).path;
           extreme_points1];
       % the second curve
       extreme_points1 = interTubePoint.pose - interTubePoint.radius * ...
           interTubePoint.normal_vect;
       tubePaths(2).path = [tubePaths(2).path;
           extreme_points1];
       % the third curve
       extreme_points2 = interTubePoint.pose + interTubePoint.radius * ...
           interTubePoint.abnormal_vect;
       tubePaths(3).path = [tubePaths(3).path;
           extreme_points2];
       % the forth curve
       extreme_points2 = interTubePoint.pose + interTubePoint.direction * ...
           interTubePoint.forwardRadius;
       tubePaths(4).path = [tubePaths(4).path;
           extreme_points2];
       
%        tubePaths(k-1,:) = [interTubePose interTubeDirection interTubeRadius interTubeForwardRadius];
   end
end
% direction = (path(lenPath,1:3) - path(lenPath-1,1:3)) / ...
%     norm(path(lenPath,1:3) - path(lenPath-1,1:3));
% tubePaths(lenPath+1,:) = [path(lenPath,1:3) direction path(lenPath,4)];
end