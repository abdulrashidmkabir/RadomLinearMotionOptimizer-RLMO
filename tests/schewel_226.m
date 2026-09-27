clear; clc; close all;

%% Optimization Parameters
numAgents     = 30;         % Population size
maxIterations = 10000;     % Maximum iteration count
%                             Note:The higher the maxIterations the better 
%                             the solution, but the more time it takes ti finish
dimensions    = 30;         % Problem dimension (D=30)
lowerBound    = -500;       % Lower bound
upperBound    = 500;        % Upper bound

% Define Benchmark Objective Function (e.g., Schwefel 2.26)
objectiveFunc = @(x) sum(-x.*sin(sqrt(abs(x)))); % The Schwefel 2.26

%% Execution & Timing
fprintf('Running RLMO Algorithm...\n');
tic;
% Initial_population is optional you can ommit it
[bestPos, bestScore, convergenceCurve, nfe] = main(objectiveFunc, dimensions, ...
                                               lowerBound, upperBound, ...
                                                         maxIterations); %,initial_position);
executionTime = toc;

%% Output Results
fprintf('--------------------------------------------------\n');
fprintf('Optimization Completed in : %.4f seconds\n', executionTime);
fprintf('Best Fitness Score        : %.6e\n', bestScore);
fprintf('Number of Function Evaluation        : %.6e\n', nfe);
fprintf('--------------------------------------------------\n');

%% Visualization
figure('Name', 'RLMO Performance Analysis', 'Color', [1 1 1]);

% Plot Convergence Curve
semilogy(1:numel(convergenceCurve), convergenceCurve, 'LineWidth', 2, 'Color', [0.85 0.325 0.098]);
grid on;
title('Convergence Trajectory of RLMO', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Iteration', 'FontSize', 11);
ylabel('Best Score (Log Scale)', 'FontSize', 11);
legend('RLMO', 'Location', 'northeast');

