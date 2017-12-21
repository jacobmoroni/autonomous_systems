function angle = wrap (angle)
    while angle >= pi
        angle = angle -2*pi;
    end
    while angle < -pi
        angle = angle +2*pi;
    end
end