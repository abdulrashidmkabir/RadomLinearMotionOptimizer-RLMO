
clear; clc; close all;
addpath(cd)
cd ..\
addpath('matlab')
cd demos\
%% 1. Optimization Configuration
dim      = 10;               % Problem dimensionality
max_iter = 10;               % Maximum iteration count
lb       = -5.12;            % Lower bound
ub       =  5.12;            % Upper bound

% Define benchmark objective functions
sphere_fn     = @(x) sum(x.^2);
rosenbrock_fn = @(x) sum(100 * (x(2:end) - x(1:end-1).^2).^2 + (1 - x(1:end-1)).^2);
rastrigin_fn  = @(x) 10 * length(x) + sum(x.^2 - 10 * cos(2 * pi * x));
Schwefel_fn  = @(x) sum(-x.*sin(sqrt(abs(x))));

benchmarks = {
    'Sphere Function',     sphere_fn,     -100, 100;
    'Rosenbrock Function', rosenbrock_fn,  -30,  30;
    'Rastrigin Function',  rastrigin_fn,  -5.12, 5.12
    'Schwefel Function',  Schwefel_fn,  -500, 500
};

%% 2. Run Optimization Across Benchmarks
fprintf('=======================================================\n');
fprintf('     Random Linear Motion Optimizer (RLMO) Demo        \n');
fprintf('=======================================================\n\n');

for i = 1:size(benchmarks, 1)
    name    = benchmarks{i, 1};
    obj_fun = benchmarks{i, 2};
    b_lb    = benchmarks{i, 3};
    b_ub    = benchmarks{i, 4};
    
    % Execute RLMOA
    tic;
    [best_pos, best_cost, convergenceCurve, nfe] = main(obj_fun, dim, b_lb, b_ub, max_iter);
    elapsed_time = toc;
    
    % Display Results
    fprintf('Benchmark: %s\n', name);
    fprintf('  -> Best Cost Achieved : %.6e\n', best_cost);
    fprintf('  -> Number of Function Evaluations (NFE): %d\n',nfe);
    fprintf('  -> NFE at Convergent: %d\n',find(convergenceCurve==min(convergenceCurve),1));
    fprintf('  -> Execution Time     : %.4f seconds\n', elapsed_time);
    fprintf('  -> Best Position (1x%d): [%s ...]\n\n', dim, num2str(best_pos(1:min(3, dim)), '%.4f '));
end

fprintf('=======================================================\n');
fprintf('Demo completed successfully.\n');

rmpath(cd)
cd ..\
rmpath('matlab')
cd demos\
