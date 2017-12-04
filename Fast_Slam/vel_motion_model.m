function state_prime = vel_motion_model(input, state_true, noise, Ts)
    v_c = input(1);
    w_c = input(2);
    alpha1 = noise(1);
    alpha2 = noise(2);
    alpha3 = noise(3);
    alpha4 = noise(4);
    alpha5 = noise(5);
    alpha6 = noise(6);
    x_true = state_true(1);
    y_true = state_true(2);
    theta_true = state_true(3);
    
    % Implement velocity motion model
    v_hat = v_c+randn*sqrt(alpha1*v_c^2+alpha2*w_c^2); %add noise to linear velocity
    w_hat = w_c+randn*sqrt(alpha3*v_c^2+alpha4*w_c^2); %add noise to angular velocity
    gamma_hat = randn*sqrt(alpha5*v_c^2+alpha6*w_c^2); %gamma velocity (not used, so set to 0)
    
    x_prime = x_true-v_hat/w_hat*sin(theta_true)+v_hat/w_hat*sin(theta_true+w_hat*Ts); %extract x from taylor series expansion?
    y_prime = y_true+v_hat/w_hat*cos(theta_true)-v_hat/w_hat*cos(theta_true+w_hat*Ts); %extract ? from taylor series expansion?
    theta_prime = theta_true+w_hat*Ts+gamma_hat*Ts; %extract theta from taylor series expansion?
    state_prime = [x_prime,y_prime,theta_prime];
end