function T = AddNodeTube(T, x_new, x_near, near_Idx, radius)
% 将collision_free的x_new节点加入树T中，并以x_near为父节点
    count = size(T.v,2) + 1;
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).z = x_new(3);
    T.v(count).xParent = T.v(near_Idx).x;
    T.v(count).yParent = T.v(near_Idx).y;
    T.v(count).zParent = T.v(near_Idx).z;
    T.v(count).dist=norm(x_new-x_near) + T.v(near_Idx).dist;  % 该节点到原点的距离
    T.v(count).indParent = near_Idx;     %父节点的index
    T.v(count).radius = radius;

    near_node = [T.v(near_Idx).x T.v(near_Idx).y T.v(near_Idx).z];
    if T.useintVol
        k_i = T.costIntVol;
    else
        k_i = 0;
    end
    interVol = intersectVolume(near_node, T.v(near_Idx).radius, x_new, radius);
    T.v(count).vol = 0*exp(-4/3*pi*(radius/T.maxRadius)^3)...
        + k_i*exp(-10*interVol/(4/3*pi*T.maxRadius^3)+1) + T.v(near_Idx).vol;
end
