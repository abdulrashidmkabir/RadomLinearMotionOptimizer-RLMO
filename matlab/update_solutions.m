%% Local Helper Function 2: update_solutions

function [cost_out, pop_out] = update_solutions(pop_new, cost_new, pop_old, cost_old)
    % Greedy selection update rule
    mask = (cost_old - cost_new) > 0;
    cost_out = mask .* cost_new + (1 - mask) .* cost_old;
    pop_out  = mask .* pop_new  + (1 - mask) .* pop_old;
end
