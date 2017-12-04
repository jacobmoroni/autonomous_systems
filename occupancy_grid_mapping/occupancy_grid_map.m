%Occupancy Grid Map
clear
clc
load ('state_meas_data.mat')
xmap = 1:.5:100;
ymap = 1:.5:100;
alpha = 1.0;
beta = 5*pi/180;
z_max = 150;
l_o = log(.5/.5);
l_occ = log(.7/.3);
l_free = log(.3/.7);
%%
map = zeros(length(xmap));

for i = 1:length(X)
    for xx = 1:length(xmap)
        for yy = 1:length(ymap)
            for j = 1:length(thk)
%                 if abs(atan2((ymap(yy)-X(2,i)),(xmap(xx)-X(1,i)))-X(3,i))<= 95*pi/180
%                     out = inverse_range_sensor(xmap(xx),ymap(yy),X(1,i),X(2,i),X(3,i),z(1,j,i),thk(j));
%                     map(xx,yy) = map(xx,yy)+out-l_o;
%                 else
% %                     map(xx,yy) = l_o;
                    out = inverse_range_sensor(xmap(xx),ymap(yy),X(1,i),X(2,i),X(3,i),z(1,j,i),thk(j));
                    map(xx,yy) = map(xx,yy)+out-l_o;
%                 end
            end
        end
    end
end
%%
%reverse log probability
for xx = 1:length(xmap)
    for yy = 1:length(ymap)
        map(xx,yy) = 1-1/(1+exp(map(xx,yy)));
    end
end
%%
figure(1);clf
heatmap(map)%,'rowlabels',[1 100])
grid on
