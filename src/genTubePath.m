function tubePaths = genTubePath(path)
% path = [pose radius]
lenPath = length(path(:,1));
% tubePaths = zeros(lenPath-1,8);
tubePaths(1).path = [];tubePaths(2).path = [];
tubePaths(3).path = [];tubePaths(4).path = [];
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
       interTubePose = interTubePoint.pose;
       interTubeDirection = interTubePoint.direction;
       interTubeRadius = interTubePoint.radius;
       interTubeForwardRadius = interTubePoint.forwardRadius;
       
       % the first path       
       extreme_points1 = interTubePoint.pose + 3/5*interTubePoint.radius * ...
           interTubePoint.normal_vect;
       d1 = norm(extreme_points1 - last.pose);
       r1 = 3/5*interTubePoint.radius;
       R1 = norm(interTubePoint.pose - last.pose);
       cos_gamma = -(d1^2+r1^2-R1^2) / (2*d1*r1);
       forward_len = cos_gamma * d1 + sqrt(last.radius^2 - (1-cos_gamma^2)*d1^2);
       extreme_points1forward = extreme_points1 + interTubeDirection*forward_len;
       d1 = norm(extreme_points1 - now.pose);
       R1 = norm(interTubePoint.pose - now.pose);
       cos_gamma = -(d1^2+r1^2-R1^2) / (2*d1*r1);
       back_len = cos_gamma * d1 + sqrt(now.radius^2 - (1-cos_gamma^2)*d1^2);
       extreme_points1back = extreme_points1 - interTubeDirection*back_len;
       tubePaths(1).path = [tubePaths(1).path;
           extreme_points1back;
           extreme_points1;
           extreme_points1forward];
       % the second curve
       extreme_points1 = interTubePoint.pose - 3/5*interTubePoint.radius * ...
           interTubePoint.normal_vect;
       extreme_points1forward = extreme_points1 + interTubeDirection*forward_len;
       extreme_points1back = extreme_points1 - interTubeDirection*back_len;
       tubePaths(2).path = [tubePaths(2).path;
           extreme_points1back;
           extreme_points1;
           extreme_points1forward];

       % the third curve
       extreme_points2 = interTubePoint.pose + 3/5*interTubePoint.radius * ...
           interTubePoint.abnormal_vect;
       extreme_points2forward = extreme_points2 + interTubeDirection*forward_len;
       extreme_points2back = extreme_points2 - interTubeDirection*back_len;
       tubePaths(3).path = [tubePaths(3).path;
           extreme_points2back;
           extreme_points2;
           extreme_points2forward];
       % the forth curve
       extreme_points2 = interTubePoint.pose - 3/5*interTubePoint.radius * ...
           interTubePoint.abnormal_vect;
       extreme_points2forward = extreme_points2 + interTubeDirection*forward_len;
       extreme_points2back = extreme_points2 - interTubeDirection*back_len;
       tubePaths(4).path = [tubePaths(4).path;
           extreme_points2back;
           extreme_points2;
           extreme_points2forward];
       
%        tubePaths(k-1,:) = [interTubePose interTubeDirection interTubeRadius interTubeForwardRadius];
   end
end
% direction = (path(lenPath,1:3) - path(lenPath-1,1:3)) / ...
%     norm(path(lenPath,1:3) - path(lenPath-1,1:3));
% tubePaths(lenPath+1,:) = [path(lenPath,1:3) direction path(lenPath,4)];
end