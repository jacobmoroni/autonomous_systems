%find probablity based on error and standard deviation
function prob = prob_normal_dist(err,sigma)
    prob = 1/(sqrt(2*pi*sigma^2))*exp((-err^2)/(2*sigma^2));
end