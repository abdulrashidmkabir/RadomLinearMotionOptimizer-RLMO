%% Local Helper Function 4: calculate_kinematics

function [best_pos_norm, best_cost, vel_init, vel_final, accel, dir_v, dir_s, dir_a, nfe] = ...
    calculate_kinematics(obj_fun, pop_norm, cost_norm, best_pos_norm, best_cost, ...
                         norm_min, norm_max, lb, ub, t, accel_prev, nfe)

    t_effective = t + 1;
    
    % Compute position-to-cost sensitivity ratio
    grad_ratio = (pop_norm - best_pos_norm) ./ ((cost_norm - best_cost) + eps);
    
    % Check and patch undefined boundary values (NaN or Inf)
    invalid_mask = isnan(grad_ratio) | isinf(grad_ratio);
    if any(invalid_mask(:))
        max_mat = repmat(norm_max, size(pop_norm, 1), 1);
        min_mat = repmat(norm_min, size(pop_norm, 1), 1);
        grad_ratio(invalid_mask) = (max_mat(invalid_mask) - 2 * pop_norm(invalid_mask) + min_mat(invalid_mask)) / 2;
    end
    
    % Update agent step along calculated gradient angle
    pop_step = pop_norm - 0.01 * cost_norm .* grad_ratio;
    [cost_step, pop_step, nfe] = evaluate_population(obj_fun, pop_step, lb, ub, norm_min, norm_max, nfe);
    
    [min_cost, min_idx] = min(cost_step);
    step_best_cost = cost_step(min_idx, :);
    step_best_pos  = pop_step(min_idx, :);
    
    [best_cost, best_pos_norm] = update_solutions(step_best_pos, step_best_cost, best_pos_norm, best_cost);

    % Update physical kinematics components
    delta_s   = (best_pos_norm - pop_norm);
    vel_init  = pop_norm / t_effective;
    accel     = 2 * delta_s / (t_effective^2);
    vel_final = vel_init + accel * t_effective;

    dir_v = sign(vel_final - vel_init);
    dir_s = sign(delta_s);
    dir_a = sign(accel - accel_prev);
end
