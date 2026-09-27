# RadomLinearMotionOptimizer-RLMO
% RMLO is improved optimizer with higher rate of convergence that can solve any 
% Optimization problem defined using its Objective function. 
% Further descriptions are provided in the following
## Features
- Fast convergence for complex bounds.
- Easy integration with MATLAB/Python workflows.
  
## Installation & Requirements
- MATLAB R2021a 
- No external toolboxes required.

## Quick Start
Run `demo_RLMO.m` to execute a benchmark run on a standard test function.
Example Test Objective Function are provided in ' test_RLMO.m'

## Usage Example
```MATLAB
%% Random Linear Motion Optimizer (RLMO) Algorithm
%
%  Syntax:
%    [best_pos, best_cost, nfe] = RLMOA(obj_fun, dim, lb, ub, max_iter)
%    [best_pos, best_cost, nfe] = RLMOA(obj_fun, dim, lb, ub, max_iter, init_pop)
%
%  Functions:
%%   Main Function : main.m
%%   Local Helper Function 0: simulate_random_linear_motion.m
%%   Local Helper Function 1: evaluate_population.m
%%   Local Helper Function 2: update_solutions.m
%%   Local Helper Function 3: apply_bounds.m
%%   Local Helper Function 4: calculate_kinematics.m
%
%  Description:
%    RLMO mimics cinematic physical motion principles (position, velocity, 
%    acceleration, and time delay) to locate optimal solutions within a 
%    bounded continuous search space.
%
%  Inputs:
%    obj_fun  - Function handle to objective cost function: cost = obj_fun(x)
%    dim      - Dimension of the decision variable vector
%    lb       - Lower boundary constraints (scalar or 1 x dim vector)
%    ub       - Upper boundary constraints (scalar or 1 x dim vector)
%    max_iter - Maximum number of allowed movement iterations
%    init_pop - (Optional) Initial candidate population matrix [pop_size x dim]
%
%  Outputs:
%    best_pos  - Optimal position vector located by the algorithm (1 x dim)
%    best_cost - Minimal cost value associated with best_pos
%    nfe       - Total number of function evaluations performed
%
