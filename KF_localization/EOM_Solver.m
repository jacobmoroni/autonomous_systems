clc
clear

%EOM solver KF_Localization

x0=[0; 0];
tspan=0:.1:50;

[t,x]=ode45(@KF_localization,tspan,x0);

pos=x(:,1);
vel=x(:,2);


figure(1)
plot (t,pos); hold on
title ('time vs. pos');
xlabel ('time (s)');
ylabel ('pos(m)');

% figure(2)
plot (t,vel); hold on
% title ('time vs. Velocity');
% xlabel ('time (s)');
% ylabel ('Velocity');

% figure(3)
% plot (t,ArmAngle); hold on
% title ('time vs. Arm Angle');
% xlabel ('time (s)');
% ylabel ('Angle (Deg)');
% 
% figure(4)
% plot (t,CapacitorVoltage); hold on
% title ('time vs. Capacitor Voltage');
% xlabel ('time (s)');
% ylabel ('Voltage');
% 
% 
% mass = 100; %kg
% t=0:.01:50;
% for (t >= 0 && t<5)