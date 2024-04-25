function [score,tube] = scoreCorridor(last,now)
% last.pose: 1xN vector
% last.radius: double
%   

V_vol = 4/3*pi*now.radius^3;
% 
rho_v = 1;
% 

d = norm(last.pose - now.pose);
r = last.radius;
R = now.radius;

cos_alpha = (r^2 + d^2 -R^2)/(2*r*d);
h2 = r - r*cos_alpha;

sin_alpha = (1 - cos_alpha^2)^0.5;

cos_beta = (R^2 + d^2 - r^2)/(2*R*d);
h1 = R - R*cos_beta;

V_int = pi*h1^2*(R - 1/3*h1) + pi*h2^2*(r - 1/3*h2);
% 
rho_i = 1;
% 

tube.direction = (now.pose - last.pose)/norm(now.pose - last.pose);
tube.pose = last.pose + (d-R+h1)*tube.direction ;
tube.radius = r*sin_alpha;
tube.forwardRadius = r*(1-cos_alpha);

tube.normal_vect = cross(tube.direction, [0 0 1]);
tube.abnormal_vect = cross(tube.direction, tube.normal_vect);



if d / (r+R) > 0.9
    score = 0;
else
    score = rho_v * V_vol + rho_i * V_int;
end
end