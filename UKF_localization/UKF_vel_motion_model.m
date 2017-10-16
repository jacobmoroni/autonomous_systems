function state_prime = UKF_vel_motion_model(input, state, Ts)
    v = input(1)+state(4);
    w = input(2)+state(5);
    x = state(1);
    y = state(2);
    theta = state(3);
    
    % Implement velocity motion model
   
    x = x-v/w*sin(theta)+v/w*sin(theta+w*Ts); %extract x from taylor series expansion?
    y = y+v/w*cos(theta)-v/w*cos(theta+w*Ts); %extract ? from taylor series expansion?
    theta =theta + w*Ts; %extract theta from taylor series expansion?
    state_prime = [x,y,theta];
end