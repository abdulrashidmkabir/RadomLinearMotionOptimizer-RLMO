% Main Function: Random Linear Motion Optimizer (RLMO) Algorithm
function [best_pos, best_cost, convergenceCurve, nfe] = main(obj_fun, dim, lb, ub, max_iter, init_pop)

nfe = 0;           % Initialize Function Evaluation Counter

% Enforce vector consistency on spatial bounds
    if isscalar(lb), lb = repmat(lb, 1, dim); end
    if isscalar(ub), ub = repmat(ub, 1, dim); end

    % Initialize starting agent population positions
    if nargin < 6 || isempty(init_pop)
        pop_init = rand(1, dim) .* (ub - lb) + lb;
    else
        pop_init = apply_bounds(init_pop, lb, ub);
    end

    cycles=ceil(max_iter^(1/4))+1;
   
    % Construct population matrix
    if size(pop_init) < cycles
        pop_init = [pop_init; rand(cycles-size(pop_init,1), dim) .* (ub - lb) + lb];
    end
    convergenceCurve=[];
    for cycle=1:cycles
    [best_pos, best_cost, convergenceCurve0, nfe] = simulate_random_linear_motion(obj_fun, dim, lb, ub, cycles^3, pop_init, nfe);
    pop_init=(best_pos+pop_init)/2;
    pop_init(cycle,:)=best_pos;
    convergenceCurve=[convergenceCurve,convergenceCurve0];
    end
end
