clear

%==========================================================================
% setup
%==========================================================================

% time
tf = 20; % s
dt = 0.1; % s

t = 0:dt:tf;

% initial conditions
x0 = [-5; -3; 90 * pi/180];

% map
m = [6 -7  6;
     4  8 -4];

xlims = [-10 10];
ylims = [-10 10];

% nominal inputs
vc = 1 + 0.5*cos(2*pi*0.2*t);
wc = -0.2 + 2*cos(2*pi*0.6*t);
u = [vc; wc];

%==========================================================================
% simulate
%==========================================================================

% noisy inputs
alpha1 = 0.1;
alpha2 = 0.01;
alpha3 = 0.01;
alpha4 = 0.1;

u_Sigma = reshape([alpha1*vc.^2 + alpha2*wc.^2;
              zeros(2,length(t));
              alpha3*vc.^2 + alpha4*wc.^2], 2, 2, length(t));

u_noisy = mvnrnd(u', u_Sigma)';
vc_noisy = u_noisy(1,:);
wc_noisy = u_noisy(2,:);

alpha5 = 0.00001;
alpha6 = 0.00001;
Qgamma = alpha5*vc.^2 + alpha6*wc.^2;
gamma = mvnrnd(zeros(1,length(t)), Qgamma);

% generate trajectory
% TODO use RK4 integration to simulate
x = zeros(3,length(t));
x(:,1) = x0;
for k = 2:length(t)
    x(1,k) = x(1,k-1) + vc_noisy(k)/wc_noisy(k) * (-sin(x(3,k-1)) + sin(x(3,k-1) + wc_noisy(k)*dt));
    x(2,k) = x(2,k-1) + vc_noisy(k)/wc_noisy(k) * (cos(x(3,k-1)) - cos(x(3,k-1) + wc_noisy(k)*dt));
    x(3,k) = x(3,k-1) + wc_noisy(k)*dt + gamma(k)*dt;
end, clear k

% measurements
sigma_r = 0.1; % m
sigma_phi = 0.05; % rad
R = diag([sigma_r, sigma_phi].^2);

z = zeros(2,size(m,2),length(t));
for k = 1:length(t)
    for i=1:size(m,2)
        z(1,i,k) = sqrt((m(1,i) - x(1,k))^2 + (m(2,i) - x(2,k))^2) + sigma_r*randn;
        z(2,i,k) = atan2(m(2,i) - x(2,k), m(1,i) - x(1,k)) - x(3,k) + sigma_phi*randn;
    end
end

%==========================================================================
% extended information filter
%==========================================================================

% initialize
xhat = zeros(size(x));
xhat(:,1) = x0;

P = zeros(3,3,length(t));
P(:,:,1) = diag([0.1, 0.1, 0.01].^2);

xi = zeros(size(xhat));
Omega = zeros(size(P));

Omega(:,:,1) = inv(P(:,:,1));
xi(:,1) = Omega(:,:,1)\xhat(:,1);

for k = 2:length(t)
    [xi(:,k), Omega(:,:,k), xhat(:,k), P(:,:,k)] = ...
        eif(xhat(:,k-1), P(:,:,k-1), vc(k), wc(k), z(:,:,k), m, dt);
end, clear k

%==========================================================================
% error
%==========================================================================
error = x - xhat;
sigma = zeros(size(x));
for k = 1:length(t)
    sigma(:,k) = sqrt(diag(P(:,:,k)));
end, clear k

%==========================================================================
% plots
%==========================================================================

state_labels = { 'x [m]', 'y [m]', '\theta [rad]'};

% map
figure(1), clf
set(gcf, 'Name', 'EKF Localization')
plot(m(1,:), m(2,:), 'bo')
hold on
plot(x(1,:), x(2,:))
plot(xhat(1,:), xhat(2,:))
hold off
axis equal
set(gca, 'XLim', xlims, 'YLim', ylims)
xlabel('x [m]')
ylabel('y [m]')
legend('map', 'truth', 'estimate')

% states
figure(2), clf
set(gcf, 'Name', 'States and Estimates')

for i = 1:3
    subplot(3,1,i)
    plot(t, x(i,:), t, xhat(i,:))
    xlabel('t [s]')
    ylabel(state_labels{i})
    legend('truth', 'estimate')
end, clear i

% error and covariance
figure(3), clf
set(gcf, 'Name', 'Error and Covariance')

for i = 1:3
    subplot(3,1,i)
    plot(t, error(i,:), t, 2*sigma(i,:), 'k--', t, -2*sigma(i,:), 'k--')
    xlabel('t [s]')
    ylabel(state_labels{i})
    legend('error', '\pm2\sigma')
end, clear i
