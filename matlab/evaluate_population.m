%% Local Helper Function 1: evaluate_population

function [cost, pop_norm, nfe] = evaluate_population(obj_fun, pop_norm, lb, ub, norm_min, norm_max, nfe)
    % Enforce normalized constraints
    pop_norm = apply_bounds(pop_norm, norm_min, norm_max);
    
    % Denormalize candidate positions into physical domain
    pop_real = lb + (ub - lb) .* (pop_norm - norm_min) ./ (norm_max - norm_min);
    
    % Evaluate cost values across individual agents
    num_agents = size(pop_norm, 1);
    cost = zeros(num_agents, 1);
    
    for i = 1:num_agents
        cost(i) = obj_fun(pop_real(i, :));
    end
    
    nfe = nfe + num_agents;
end
