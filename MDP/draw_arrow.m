function [] = draw_arrow(x0,y0,length,angle)

% draw and arrow in a Matlab plot
% angle = 0 corresponds to an arrow pointing up (+y direction)
% increasing angle results in arrow rotating CCW from +y direction
% scale = 1 corresponds to an arrow of length 1
% shift is an (x,y) shift in the location of the tail of the arrow.

    arrow_x = [0 0 -0.2 0 0.2];
    arrow_y = [0 1 0.8 1 0.8];

    scale = length;
    shift = [x0; y0];

    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

    a = R*scale*[arrow_x; arrow_y] + shift;
    a_x = a(1,:);
    a_y = a(2,:);

    plot(a_x,a_y,'b');

end