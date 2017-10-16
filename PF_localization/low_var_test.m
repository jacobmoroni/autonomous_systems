%low var tester
chibar = zeros(100);
chibar(:,1) = 1:1:100;
weight = rand(1,100);
weight = weight/sum(weight);
for i =2:1000
    chibar(:,i) = low_var_sampler(chibar(:,i-1),weight);
    figure(1); clf
    plot (weight,chibar(:,i),'*b')
end
% figure(1); clf
% % plot (1, chi, 'or')
% % hold on
% plot (1,chibar(:,10000),'*b')