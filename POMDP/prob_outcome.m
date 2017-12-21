clc
clear
%probability outcome
prob = .6;
state = 1;
% while prob >= .289 && prob <= .882
for i = 1:10
    prob = .6;
    state = 1;
    while prob >= .278 && prob <= .653
            %move
        if rand <= .8
            if state ==1
                state = 2;
            else
                state = 1;
            end
        end

        %measure
        if rand <= .7
            meas = state;
        else
            meas = mod(state+2,2)+1;
        end

        if meas == 1
            p1pr = .7*prob/(.4*prob+.3);
            p2pr = .3*(1-prob)/(.4*prob+.3);
        elseif meas == 2
            p1pr = .3*prob/(-.4*prob+.7);
            p2pr = .7*(1-prob)/(-.4*prob+.7);
        end
        
        %update beliefkk,
        if state == 1
            prob = p1pr;
        else
            prob = p2pr;
        end
    end
    if state == 2 && prob <= .5
        score(i) = 100;
    elseif state ==2 && prob >=.5
        score(i) = -50;
    elseif state == 1 && prob >.5
        score(i) = 100;
    elseif state == 1 && prob <.5
        score(i) = -100;
    end
    score_final = sum(score);
end
score
score_final
% prob
% state
