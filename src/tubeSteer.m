function [x_sphere,x_newRadius] = tubeSteer(near_Idx,x_new,T,map3D)
% find a point which has a maximum non-collision sphere to intersect with
% adjoint sphere.
x_near = [T.v(near_Idx).x T.v(near_Idx).y T.v(near_Idx).z];
x_sphere = x_new;
x_radius = T.v(near_Idx).radius;
dis = norm(x_near - x_new);
direction = (x_new - x_near)/dis;
findFlag = true;


while(findFlag)
    % find the maximum radius of the point x_sphere
    [x_newRadius, ~]=findNearestPoint(map3D,x_sphere);
    x_newRadius = min(x_newRadius,T.maxRadius);% constraint the max radius

    % if two adjoint spheres do not have sufficient intersection space, the
    % x_sphere is update by closing the 2/3 distance
    if max(x_newRadius,x_radius) < dis
        dis = max(x_newRadius,x_radius);
        x_sphere = x_near + dis * direction;
    else
        findFlag = false;
    end
    % if (x_newRadius+x_radius)*0.6 < dis
    %     dis = dis*0.9;
    %     x_sphere = x_near + dis * direction;
    % else
    %     findFlag = false;
    % end
end

end