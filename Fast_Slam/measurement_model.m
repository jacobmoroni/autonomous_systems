function q = measurement_model(landmark,sig,x_t,z_t)
    sig_r = sig(1);
    sig_phi = sig(2);
    x = x_t(1);
    y = x_t(2);
    theta = x_t(3);
    r = z_t(1,:);
    b = z_t(2,:);
    MM = size(landmark,2);
    for j = 1:MM
        dx = landmark(1,j)-x; %x distance from bot to landmark
        dy = landmark(2,j)-y; %y distance from bot to landmark
        range = sqrt(dx^2+dy^2); %l2 norm from bot to landmark (noise added)
        bearing = atan2(dy,dx)-theta; %bearing angle from front of bot to landmark (noise added)
        
        %determine error of estimated measurement vs real measurement
        err_r = range - r(j);
        err_b = bearing - b(j);
        %find probability of measurement occuring based on system noise
        q(j) = prob_normal_dist(err_r,sig_r)*prob_normal_dist(err_b,sig_phi);
    end
    %combine probabilities based on all landmarks
    q = prod(q);
end