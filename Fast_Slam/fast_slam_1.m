function [Y,wmax] = fast_slam_1(z,u,Y_old,noise,Ts,MM,view_angle)
%measurement noise
alphas = noise(1:6);
sig_r = noise(7); %range measurement
sig_phi = noise(8); %bearing measurement
Q_t = [sig_r^2 0; 0 sig_phi^2];
p_0 = .05; %default importance weight
M = length(Y_old(1,:));
w = zeros(1,M);
    for k =1:M %number of particles
        X_old(:,k) = Y_old(1:3,k);%Extract X from Y
        X(:,k) = vel_motion_model(u, X_old, alphas, Ts); %Propagate X with veolocity motion model
        for ll = 1:MM %number of landmarks
            mu_old = Y_old(3+6*ll-5:3+6*ll-4,k); %Extract States from Y
            Sigma_old(1,:)=Y_old(3+6*ll-3:3+6*ll-2,k); %Extract Sigma Matricies from Y
            Sigma_old(2,:)=Y_old(3+6*ll-1:3+6*ll,k);
            if abs(z(2,ll)) < view_angle %measurement view width
                if mu_old(1) == 0 %if never seen before
                    mu(:,ll,k) = [X(1,k);X(2,k)] + [z(1,ll)*cos(z(2,ll)+X(3,k));z(1,ll)*sin(z(2,ll)+X(3,k))];
                    delta = [mu(1,ll,k)-X(1,k);mu(2,ll,k)-X(2,k)];
                    q = delta'*delta;
                    H = 1/q*[sqrt(q)*delta(1) sqrt(q)*delta(2);
                               -delta(2)        delta(1)     ];
                    Sigma(:,:,ll,k) = H^-1*Q_t*(H^-1)';
                    w(k)=p_0;
                else
                    delta = [mu_old(1)-X(1,k);mu_old(2)-X(2,k)];
                    q = delta'*delta;
                    zhat = [sqrt(q);atan2(delta(2),delta(1))-X(3,k)];
                    H = 1/q*[sqrt(q)*delta(1) sqrt(q)*delta(2);
                               -delta(2)        delta(1)     ];
                    Q = H*Sigma_old*H'+Q_t;
                    K = Sigma_old*H'*Q^-1;
                    z_diff = z(:,ll) - zhat;
                    z_diff(2) = wrap(z_diff(2));
                    mu(:,ll,k) = mu_old + K*(z_diff);
                    Sigma(:,:,ll,k)=(eye(2)-K*H)*Sigma_old;
                    w(k) = w(k) + log(mvnpdf(z_diff,[0;0],Q));
%                     w(k) = det(2*pi*Q)^-.5*exp(-.5*z_diff'*Q^-1*z_diff);
%                     plot (mu(1,ll,k),mu(2,ll,k),'.b')
                end
            else 
                mu(:,ll,k) = mu_old;
                Sigma(:,:,ll,k) = Sigma_old;
            end
        end       
    end
    for k = 1:M
        Y(1:3,k) = X(:,k);
        for ll = 1:MM
            Y(3+6*ll-5:3+6*ll,k)=[mu(:,ll,k);Sigma(1,:,ll,k)';Sigma(2,:,ll,k)'];
        end
    end
    w = exp(w-max(w));
    w=w/sum(w);
    [Y,un]=low_var_sampler(Y,w);
    wmax = find(max(w));
end
            