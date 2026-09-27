%% Local Helper Function 3: apply_bounds

function pop_bounded = apply_bounds(pop, min_bound, max_bound)
    % Fully vectorized boundary clipping function
    pop_bounded = bsxfun(@min, bsxfun(@max, pop, min_bound), max_bound);
end
