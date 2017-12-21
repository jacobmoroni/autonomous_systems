function vector = plot_squares (center, offset ,scale)
    if offset == 0
        vector = [center(1)-scale,center(1)+scale,center(1)+scale,center(1)-scale;
                  center(2)      ,center(2)      ,center(2)      ,center(2);
                  center(3)-scale,center(3)-scale,center(3)+scale,center(3)+scale];
    elseif offset == 90
        vector = [center(1)      ,center(1)      ,center(1)      ,center(1);
                  center(2)-scale,center(2)+scale,center(2)+scale,center(2)-scale;
                  center(3)-scale,center(3)-scale,center(3)+scale,center(3)+scale]; 
    end
end