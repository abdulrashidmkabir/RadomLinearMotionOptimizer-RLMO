# RadomLinearMotionOptimizer-RLMO
%% Random Linear Motion Optimizer (RLMO) Algorithm
%
%  Syntax:
%    [best_pos, best_cost, nfe] = RLMOA(obj_fun, dim, lb, ub, max_iter)
%    [best_pos, best_cost, nfe] = RLMOA(obj_fun, dim, lb, ub, max_iter, init_pop)
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
