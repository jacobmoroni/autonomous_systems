function [chibar,un] = low_var_sampler(chi,weight)
    M = length(chi);
    chibar = zeros(size(chi)); %initialize chibar
    r = rand*M^-1; %set first random interval (proportional to size)
    c = weight(1); %first weight
    i = 1;
    for m = 1:M
       U = r+(m-1)*M^-1;
       while U>c && i< M %assign new points based on weights
          i = i+1;
          c = c + weight(i);
       end
       chibar(:,m) = chi(:,i);
    end
    un = length(unique(chibar))/3;
end