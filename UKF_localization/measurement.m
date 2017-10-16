function z = measurement(j,landmark,sig_r,sig_phi,x,y,theta)
    dx = landmark(1,j)-x; %x distance from bot to landmark
    dy = landmark(2,j)-y; %y distance from bot to landmark
    range = sqrt(dx^2+dy^2)+randn*sig_r; %l2 norm from bot to landmark (noise added)
    bearing = atan2(dy,dx)-theta + randn*sig_phi; %bearing angle from front of bot to landmark (noise added)
    z = [range;bearing]; %collapse into single measurement matrix
end