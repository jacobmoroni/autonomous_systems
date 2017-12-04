function out = inverse_range_sensor(xmap,ymap,x,y,theta,z,thk)
    alpha = 1;
    beta = 5*pi/180;
    z_max = 150;
    l_o = log(.5/.5);
    l_occ = log(.7/.3);
    l_free = log(.3/.7);
    r = sqrt((xmap-x)^2 + (ymap-y)^2);
    phi = atan2((ymap-y),(xmap-x))-theta;
%                 k = min(phi(xx,yy,i,j)
    if r > min(z_max,z+alpha/2) || abs(phi-thk) > beta/2
        out = l_o;
    elseif z < z_max && abs(r-z) < alpha/2
        out = l_occ;
    elseif r < z
        out = l_free;
    else 
        out = l_o;
%         xmap 
%         ymap
    end

end