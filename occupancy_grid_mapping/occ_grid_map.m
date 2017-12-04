clear
clc
load ('state_meas_data.mat')
%%
xmap = 1:1:100;
ymap = 1:1:100;
alpha = 1;
beta = 5*pi/180;
z_max = 150;
l_o = log(.5/.5);
l_occ = log(.7/.3);
l_free = log(.3/.7);

for xx = 1:length(xmap)
    for yy = 1:length(ymap)
        for i=1:length(X)
            for j = 1:length(thk)
                r(xx,yy,i) = sqrt((xmap(xx)-X(1,i))^2 + (ymap(yy)-X(2,i))^2);
                phi(xx,yy,i,j) = atan2((ymap(yy)-X(2,i)),(xmap(xx)-X(1,i)))-X(3,i);
%                 k = min(phi(xx,yy,i,j)
                if r(xx,yy,i) > min(z_max,z(1,j,i)+alpha/2) || abs(phi(xx,yy,i,j)-thk(j)) > beta/2
                    out(xx,yy,i,j) = l_o;
                elseif z(1,j,i) < z_max && abs(r(xx,yy,i)-z(1,j,i)) < alpha/2
                    out(xx,yy,i,j) = l_occ;
                elseif r(xx,yy,i) < z(1,j,i)
                    out(xx,yy,i,j) = l_free;
                end
            end
        end
    end  
end
%%
map = zeros(length(xmap));
for xx = 1:length(xmap)
    for yy = 1:length(ymap)
        for i = 1:length(X)
            for j = 1:length(thk)
                map(xx,yy) = map(xx,yy)+out(xx,yy,i,j)-l_o;
            end
        end
    end
end
for xx = 1:length(xmap)
    for yy = 1:length(ymap)
        map(xx,yy) = 1/(1+exp(map(xx,yy)));
%         if map(xx,yy)>2
%             map(xx,yy) = 2;
%         end
    end
end


%%
figure(1);clf
plot(X(1,:),X(2,:))
hold on
for i = 1:length(X)
%     range(i,:) = z(1,:,i);
%     bearing(i,:) = z(2,:,i);
    for j = 1:length(thk)
        if isnan(z(1,j,i)) || isnan(z(2,j,i))
        else
            x_point = X(1,i)+cos(thk(j)+X(3,i))*z(1,j,i);
            y_point = X(2,i)+sin(thk(j)+X(3,i))*z(1,j,i);
%             x_point = X(1,i)+cos(z(2,j,i)+X(3,i))*z(1,j,i);
%             y_point = X(2,i)+sin(z(2,j,i)+X(3,i))*z(1,j,i);
            plot (x_point,y_point,'or')
        end
    end
end

%%
figure(2);clf
bar3(map)
%%
figure(3);clf
hold on
for xx = 1:length(xmap)
    for yy = 1:length(ymap)
        plot3(xmap(xx),ymap(yy),map(xx,yy),'or')
    end
end
%%
phi2 = phi(:,:,:,1);