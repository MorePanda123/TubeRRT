function [] = showCorridor(minDis, lastPoint)
%UNTITLED4 此处提供此函数的摘要
%   此处提供详细说明

[X,Y,Z] = sphere;
r = minDis;
X2 = X * r + lastPoint(1);
Y2 = Y * r + lastPoint(2);
Z2 = Z * r + lastPoint(3);
sf = surf(X2,Y2,Z2);
sf.EdgeColor = [0 0 0.8];
sf.EdgeAlpha = 0;
sf.FaceColor = [0 1 1];
sf.FaceAlpha = 0.2;
end