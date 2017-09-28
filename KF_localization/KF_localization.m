function dx=KF_localization(t,x)
dx=zeros(2,1);

m = 100; %kg
b = 20; %N-s/m
if t>=0 && t<5
    F = 50; %N
elseif t >= 25 && t <30
    F=-50; %N
else
    F = 0; %N
end

dx(1)=x(2);
dx(2)=(F -(b*x(2)))/m;
