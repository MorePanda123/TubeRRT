function [] = showTubePathPoints(tubePaths)
for theta=0:0.5:2*pi-0.5
    for k=1:length(tubePaths(:,1))
        origin = tubePaths(k,1:3);
        direction = tubePaths(k,4:6);
        radius = tubePaths(k,end);

        z_ = [0 0 1];
        normal = cross(direction, z_);
        abnormal = cross(direction,normal);

        poseTemp = origin + radius*cos(theta)*normal + radius*sin(theta)*abnormal;
        scatter3(poseTemp(1),poseTemp(2),poseTemp(3),'or','filled');


        if k > 1
            plot3([poseTemp(1) poseTempBefore(1)], [poseTemp(2) poseTempBefore(2)], [poseTemp(3) poseTempBefore(3)],'-g');
        end
        poseTempBefore = poseTemp;


    end
end