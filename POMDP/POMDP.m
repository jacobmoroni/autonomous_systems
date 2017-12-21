%POMDP
clear
clc

T = 300;

p_z(1,1) = .7;
p_z(2,1) = 1-p_z(1,1);
p_z(1,2) = p_z(2,1);
p_z(2,2) = p_z(1,1);

p_s = zeros(2,3,2);
p_s(2,3,1) = .8;
p_s(1,3,2) = p_s(2,3,1);
p_s(1,3,1) = 1-p_s(2,3,1);
p_s(2,3,2) = p_s(1,3,1);


Y = [0,0,0];

r = [-100, 100, -1;
     100, -50,-1];
 gamma = 1;
for tao = 1:T
    Ypr = [];
    [KK,~] = size(Y);
    for k = 1:KK
        for iu=1:3 %all actions
            for iz = 1:2 %all measurements
                for j = 1:2 %all states
                    vk1 = Y(k,2)*p_z(iz,1)*p_s(1,iu,j);
                    vk2 = Y(k,3)*p_z(iz,2)*p_s(2,iu,j);
                    Y_vv(k,iu,iz,j) = vk1+vk2;
%                     disp(vk1+vk2)
                end
            end
        end
    end
    [K,~,~,~] = size(Y_vv);
    for iu=1:3
        for k1 = 1:K
            for k2 = 1:K
                for i=1:2
                    vpr(i) = gamma*(r(i,iu) + Y_vv(k1,iu,1,i) + Y_vv(k2,iu,2,i));
                end
                Ypr = [Ypr; iu vpr];
            end
        end
    end
    Y = Ypr;
    Y = prunenator(Y);
end
v2 = Y(:,2:3);

 %plot value function
 figure(1);clf
 hold on
 xlabel ('Probability of State 1')
 ylabel ('Reward')
 title ('Value Function')
 for i=1:length(v2)
     plot ([0,1],[v2(i,2),v2(i,1)])
 end
 