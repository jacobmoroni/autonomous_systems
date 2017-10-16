function z = UKF_measurement(j,landmark,state)
    x = state(1);
    y = state(2);
    theta = state(3);

    dx = landmark(1,j)-x; %x distance from bot to landmark
    dy = landmark(2,j)-y; %y distance from bot to landmark
    range = sqrt(dx^2+dy^2); %l2 norm from bot to landmark (noise added)
    bearing = atan2(dy,dx)-theta; %bearing angle from front of bot to landmark (noise added)
    z = [range;bearing]; %collapse into single measurement matrix
end