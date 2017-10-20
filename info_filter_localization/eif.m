function [xi, Omega, x, P] = eif(x, P, v, w, z, m, dt)

alpha1 = 0.1;
alpha2 = 0.01;
alpha3 = 0.01;
alpha4 = 0.1;
alpha5 = 0.00001;
alpha6 = 0.00001;

% propagate
% Jacobian from radius motion model
% F = [1, 0, v/w*(-cos(x(3)) + cos(x(3)+w*dt));
%      0, 1, v/w*(-sin(x(3)) + sin(x(3)+w*dt));
%      0, 0, 1];

% Jacobian from straight line motion model 
F = [1 0 -v*sin(x(3))*dt;
     0 1 v*cos(x(3))*dt;
     0 0 1];

% Jacobian from radius motion model 
% a = -sin(x(3)) + sin(x(3)+w*dt);
% b = cos(x(3)) - cos(x(3)+w*dt);
% G = [a/w, -v*a/w^2 + v*cos(x(3)+w*dt)/w;
%      b/w, -v*b/w^2 + v*sin(x(3)+w*dt)/w;
%      0, dt];

% Jacobian from straight line motion model 
G = [cos(x(3))*dt 0;
     sin(x(3))*dt 0;
     0 dt];

Qu = diag([alpha1*v^2 + alpha2*w^2, alpha3*v^2 + alpha4*w^2]);
Qx = diag([0, 0, alpha5*v^2 + alpha6*w^2]);

P = F*P*F' + G*Qu*G' + Qx;
xi = P\dynamics(x,v,w,dt);
Omega = inv(P);

% Omega = inv(F*P*F' + G*Qu*G' + Qx);
% xi = Omega*dynamics(x, v, w, dt);

% measurement update
sigma_r = 0.1; % m
sigma_phi = 0.05; % rad
R = diag([sigma_r, sigma_phi].^2);

for i = 1:size(m,2)
    q = (m(1,i) - x(1))^2 + (m(2,i) - x(2))^2;
    zhat = [sqrt(q); atan2(m(2,i)-x(2), m(1,i)-x(1)) - x(3)];
    
    while zhat(2)-z(2,i) > pi
        zhat(2) = zhat(2) - 2*pi;
    end
    while z(2,i)-zhat(2) > pi
        zhat(2) = zhat(2) + 2*pi;
    end

    H = zeros(2,3);
    H(1,1) = -(m(1,i)-x(1))/sqrt(q);
    H(1,2) = -(m(2,i)-x(2))/sqrt(q);
    H(2,1) = (m(2,i)-x(2))/q;
    H(2,2) = -(m(1,i)-x(1))/q;
    H(2,3) = -1.0;
    
    Omega = Omega + H'/R*H;
    xi = xi + H'/R*(z(:,i) - zhat + H*x);
    x = Omega\xi;
end, clear i

% transform back into state/covariance
% x = Omega\xi;
P = inv(Omega);

end

function xp = dynamics(x, v, w, dt)
    xp = zeros(3,1);
%     xp(1) = x(1) + v/w*(-sin(x(3)) + sin(x(3) + w*dt));
%     xp(2) = x(2) + v/w*(cos(x(3)) - cos(x(3) + w*dt));
%     xp(3) = x(3) + w*dt;
    xp(1) = x(1) + v*cos(x(3))*dt;
    xp(2) = x(2) + v*sin(x(3))*dt;
    xp(3) = x(3) + w*dt;
end
