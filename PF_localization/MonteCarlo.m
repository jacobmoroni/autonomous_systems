function chi_t = MonteCarlo(chi_t_1, u_t,z_t,landmark,alphas,Ts,sig)
    %initialize variables
    M = length(chi_t_1);
    chibar_t = zeros(size(chi_t_1));
    w_t = zeros(1,M);
    x_t_1 = chi_t_1;
%     %plot points prior to resampling;
%     plot (chi_t_1(1,:),chi_t_1(2,:),'ob')
    
    for m = 1:M
        %sample from motion model
        x_t = vel_motion_model(u_t, x_t_1(:,m), alphas, Ts);
        %find probablility of measurement being correct
        w_t(m) = measurement_model(landmark,sig,x_t,z_t);
        %append chibar_t
        chibar_t(:,m) = x_t';
    end
    %normalize weights for low variance sampler
    w_t=w_t/sum(w_t);
    %resample points with low variance sampler based on weights
    [chi_t,un] = low_var_sampler(chibar_t,w_t);
    if un < 50
            chi_t = chi_t+randn(3,M)./un;
            plot (chi_t(1,:),chi_t(2,:),'.b')
            hold on
    else
        %plot points post resampling
        plot (chi_t(1,:),chi_t(2,:),'.r')
        hold on
    end
end