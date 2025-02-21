function [x_rand] = sampleFree(map3D,E)
    iOccval = 1;
    while(iOccval)
        x_rand = rand(1,3)*E;
        iOccval = checkOccupancy(map3D,x_rand);
    end
end