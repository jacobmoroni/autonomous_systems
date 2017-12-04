function [mu, Sigma, zhat] = EKF_SLAM_known_correspondences (mu,Sigma,u,z,MM,Ts)
    v = u(1);
    w = u(2);
    alpha1 = .1;
    alpha2 = .01;
    alpha3 = .01;
    alpha4 = .1;
    
    %measurement noise
    sig_r = .1; %range measurement
    sig_phi = .05; %bearing measurement
    
    theta = mu(3);
    F_x = [eye(3),zeros(3,2*MM)];
    mubar = mu + F_x'*[(-v/w*sin(theta))+v/w*sin(theta+w*Ts);
                        (v/w*cos(theta))-v/w*cos(theta+w*Ts);
                                         w*Ts               ];
                                     
    G = eye(2*MM+3,2*MM+3) + F_x'*[0 0 -v/w*cos(theta)+v/w*cos(theta+w*Ts);
                                   0 0 -v/w*sin(theta)+v/w*sin(theta+w*Ts);
                                   0 0 0]*F_x;
                  
    % Jacobian of g(u(t), x(t-1)) with respect to inputs    
    V = [(-sin(theta)+sin(theta+w*Ts))/w (v*(sin(theta)-sin(theta+w*Ts)))/(w^2)+(v*cos(theta+w*Ts)*Ts)/(w);
         (cos(theta)-cos(theta+w*Ts))/w -(v*(cos(theta)-cos(theta+w*Ts)))/(w^2)+(v*sin(theta+w*Ts)*Ts)/(w);
         0 Ts];
    
    % Noise in control space [forward, 0; 0, Angular] 
    M = [alpha1*(v^2) + alpha2*(w^2),              0              ;
                      0             , alpha3*(v^2) + alpha4*(w^2)];
                  
    R = V*M*V';
    Sigbar = G*Sigma*G' + F_x'*R*F_x;
    Q = [sig_r^2 0; 0 sig_phi^2];
    for j = 1:MM
        if abs(z(2,j)) < pi/8 
            if mubar(3+2*j-1) == 0%norm(z(:,j))==0
                [mubar_j(:,j)] = [mubar(1);mubar(2)] + [z(1,j)*cos(z(2,j)+mubar(3));z(1,j)*sin(z(2,j)+mubar(3))];
                mubar(3+2*j-1) = mubar_j(1,j);
                mubar(3+2*j) = mubar_j(2,j);
            end
    %         
            delta = [mubar(3+2*j-1)-mubar(1);mubar(3+2*j)-mubar(2)];
            q = delta'*delta;
            zhat(:,j) = [sqrt(q);atan2(delta(2),delta(1))-mubar(3)];
            Fj(:,:,j) = [eye(5,3) zeros(5,2*j-2) rot90(eye(5,2),2) zeros(5,2*MM-2*j)];
            H(:,:,j) = 1/q*[-sqrt(q)*delta(1) -sqrt(q)*delta(2) 0 sqrt(q)*delta(1) sqrt(q)*delta(2);
                                 delta(2)        -delta(1)     -q   -delta(2)        delta(1)]*Fj(:,:,j);
            K(:,:,j) = Sigbar*H(:,:,j)'*(H(:,:,j)*Sigbar*H(:,:,j)' + Q)^-1;
            z_diff = (z(:,j)-zhat(:,j));
            while z_diff(2) >= pi
               z_diff(2) = z_diff(2) - 2*pi;
            end
            while z_diff(2) < -pi
               z_diff(2) = z_diff(2) + 2*pi;
            end
        
            mubar = mubar + K(:,:,j)*z_diff;
        
            mubar(3) = wrap(mubar(3));
            Sigbar = (eye(2*MM+3) - K(:,:,j)*H(:,:,j))*Sigbar;
%             plot (mu(3+2*j-1),mu(3+2*j),'og')
        else
            zhat(:,j) = [0;0];
        end
    end
    mu = mubar;
    Sigma = Sigbar;
     
end