%% Local Helper Function 0: simulate_random_linear_motion
function [best_pos, best_cost, convergenceCurve, nfe] = simulate_random_linear_motion(obj_fun, dim, lb, ub, max_iter, pop_init, nfe)
    %% 1. Parameter Initialization & Space Normalization

    pop_size=size(pop_init,1);

    % Establish normalized optimization hypercube [-0.5, 0.5]
    norm_min = -0.5 * ones(1, dim);
    norm_max =  0.5 * ones(1, dim);
    
    % Map physical problem domain to normalized range
    pop_norm = norm_min + (pop_init - lb) .* (norm_max - norm_min) ./ (ub - lb);

    % Compute initial objective evaluations
    [cost_norm, pop_norm, nfe] = evaluate_population(obj_fun, pop_norm, lb, ub, norm_min, norm_max, nfe);
    
    % Track initial global optimum agent
    [best_cost, min_idx] = min(cost_norm);
    best_pos_norm = pop_norm(min_idx, :);

    convergenceCurve=best_cost;

    % Initialize initial kinematics variables
    accel_prev = zeros(size(pop_norm));
    [best_pos_norm, best_cost, vel_init, vel_final, accel, ~, ~, ~, nfe] = ...
        calculate_kinematics(obj_fun, pop_norm, cost_norm, best_pos_norm, best_cost, ...
                             norm_min, norm_max, lb, ub, 0, accel_prev, nfe);

    % Preallocate stochastic rates
    r_v = rand(pop_size, dim);
    r_s = rand(pop_size, dim);
    r_a = rand(pop_size, dim);
    r_t = rand(pop_size, dim);

    cost_current = cost_norm;
    pop_current  = pop_norm;

    iter_per_cycle=max_iter*dim;
    max_cycle=max_iter;
    max_gen=dim;

    %% 2. Optimization Loop
    for gen=1:max_gen
    check_convergence=0;
    for cycle = 1:max_cycle
        best_cost_init0=best_cost;
        for t = 1:iter_per_cycle
            best_cost_init=best_cost;
            if t > 1

                % Generate logical improvement vector mask
                is_improved = (cost_current - cost_current0) > 0;
                
                r_v_prev = r_v; r_s_prev = r_s; r_a_prev = r_a; r_t_prev = r_t;
                if rand>0.1, r_v = rand(pop_size, dim); end
                if rand>0.1, r_s = rand(pop_size, dim); end 
                if rand>0.1, r_a = rand(pop_size, dim); end
                
                cycle_delay= exp(-(0.005+2.3*(t-1)/max_cycle));
                time_decay = exp(-(0.005+2.3*(t-1)/iter_per_cycle));
                
                r_v = cycle_delay.*time_decay .* ((1 - is_improved) .* r_v + is_improved .* r_v_prev);
                r_s = cycle_delay.*time_decay .* ((1 - is_improved) .* r_s + is_improved .* r_s_prev);
                r_a = cycle_delay.*time_decay .* ((1 - is_improved) .* r_a + is_improved .* r_a_prev);
                r_t = cycle_delay.*time_decay .* ((1 - is_improved) .* r_t + is_improved .* r_t_prev);
            end
            cost_current0=cost_current;

            cost_norm = cost_current;
            pop_norm  = pop_current;

            % Stage 0: Time-step optimization and cost evaluation
            dt = r_t .* t;

            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, vel_init .* dt + 0.5 * accel_prev .* dt.^2, ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, (vel_init + vel_final) .* dt / 2, ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end

            % Stage 1: Velocity adjustment and cost evaluation
            vel_final = vel_init + sign(vel_final - vel_init) .* r_v .* abs(vel_final - vel_init);
           
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, (vel_init + vel_final) * t / 2, ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, 0.5 * (vel_final.^2 - vel_init.^2) ./ (accel + eps), ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end

            % Stage 2: Acceleration modification
            accel_prev = accel_prev + sign(accel - accel_prev) .* r_a .* abs(accel - accel_prev);
            
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, vel_init * t + 0.5 * accel_prev * t^2, ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, 0.5 * (vel_final.^2 - vel_init.^2) ./ (accel_prev + eps), ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end

            % Stage 3: Position trajectory update
            vel_init  = vel_final;
            vel_final = vel_final + accel .* (r_t .* t);
            
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, (vel_init + vel_final) * t / 2, ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end
            if rand>0.1
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, 0.5 * (vel_final.^2 - vel_init.^2) ./ (accel + eps), ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_norm, pop_norm] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end
            
            if rand>0.1
            % Stage 4: Random positional displacement
            disp_target = pop_current + sign(pop_current - pop_norm) .* r_s .* abs(pop_current - pop_norm);
            [cost_current, pop_current, nfe] = evaluate_population(obj_fun, disp_target, ...
                lb, ub, norm_min, norm_max, nfe);
            [cost_current, pop_current] = update_solutions(pop_current, cost_current, pop_norm, cost_norm);
            end

            % Extract optimum candidate across current iteration
            [min_val, min_idx] = min(cost_current);
            [best_cost, best_pos_norm] = update_solutions(pop_current(min_idx, :), min_val, best_pos_norm, best_cost);

            % Update kinematic measurements for subsequent iteration step
            [best_pos_norm, best_cost, vel_init, vel_final, accel, ~, ~, ~, nfe] = ...
                calculate_kinematics(obj_fun, pop_current, cost_current, best_pos_norm, best_cost, ...
                                     norm_min, norm_max, lb, ub, t, accel_prev, nfe);
            convergenceCurve=[convergenceCurve,best_cost];
            
            if nfe==dim*max_iter*40000
                break
            end

            if (best_cost_init==best_cost)
                if (t>1)
                check_convergence=check_convergence+1;
                Scale=rand*cycle_delay.*time_decay/nfe;
                pop_new = rand(pop_size, dim) .* (ub - lb) + lb;
                pop_current=best_pos_norm+Scale.*(pop_new+best_pos_norm+pop_current)/3;
                end
            else
                check_convergence=0;
            end
            
            if (best_cost==0) || (check_convergence==max(max_iter,10000))
                break
            end
        end
            if nfe==dim*max_iter*40000
                break
            end
            
            if (best_cost_init0==best_cost)
                if (t>1)
                check_convergence=check_convergence+1;
                    Scale=rand*cycle_delay;
                    pop_new = rand(pop_size, dim) .* (ub - lb) + lb;
                    pop_current=(best_pos_norm+Scale.*(pop_new+best_pos_norm+pop_current)/3)/2;
                end
            else
                check_convergence=0;
            end
            
        % Reset random generator seed to secure execution baseline
        rng('default');
    end
    if nfe==dim*max_iter*40000
       break
    end
    % Reset random generator seed to secure execution baseline
        rng('default');
    end

    %% 3. Denormalize Best Position to Original Problem Boundaries
    best_pos = lb + (ub - lb) .* (best_pos_norm - norm_min) ./ (norm_max - norm_min);
end
