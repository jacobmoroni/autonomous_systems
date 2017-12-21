function angle = unwrap (angle)
    for i = 2:length(angle)
        while angle(i)-angle(i-1) > 3*pi/4
            angle(i) = angle(i)+2*pi;
        end
        while angle(i) - angle(i-1) < -3*pi/4
            angle(i) = angle(i)+2*pi;
        end
    end
end