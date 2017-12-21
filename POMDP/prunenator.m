function Y = prunenator(Y)
Y = unique(Y,'rows');
p=0:.0001:1; %probability of state 1
vplot = Y(:,3) + (Y(:,2)-Y(:,3))*p;
[~,maxpos]=max(vplot,[],1);
pruned = unique(maxpos);
Y = Y(pruned,:);
end