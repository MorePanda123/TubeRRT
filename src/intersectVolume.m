function interVol = intersectVolume(x_near, x_nearRadius, x_new, x_newRadius)


d = norm(x_near - x_new);
r = x_nearRadius;
R = x_newRadius;

cos_alpha = (r^2 + d^2 -R^2)/(2*r*d);
h2 = r - r*cos_alpha;

% sin_alpha = (1 - cos_alpha^2)^0.5;

cos_beta = (R^2 + d^2 - r^2)/(2*R*d);
h1 = R - R*cos_beta;

interVol = pi*h1^2*(R - 1/3*h1) + pi*h2^2*(r - 1/3*h2);
end