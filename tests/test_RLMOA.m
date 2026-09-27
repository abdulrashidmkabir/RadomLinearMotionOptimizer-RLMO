function tests = test_RLMOA
    tests = functiontests(localfunctions);
end

%% Test 1: Validate Search Space Bounds
function testBoundaryConstraints(testCase)
    dim = 5;
    lb = -10;
    ub = 10;
    max_iter = 10;
    obj_fun = @(x) sum(x.^2);
    
    [best_pos, ~, ~] = RLMOA(obj_fun, dim, lb, ub, max_iter);
    
    % Verify returned best position is within specified bounds
    verifyTrue(testCase, all(best_pos >= lb) && all(best_pos <= ub), ...
        'Best position vector exceeded specified boundary constraints.');
end

%% Test 2: Basic Convergence on Convex Function (Sphere)
function testSphereConvergence(testCase)
    dim = 3;
    lb = -5.12;
    ub = 5.12;
    max_iter = 30;
    obj_fun = @(x) sum(x.^2); % Global minimum at x = [0,0,0], cost = 0
    
    [~, best_cost, nfe] = RLMOA(obj_fun, dim, lb, ub, max_iter);
    
    % Verify cost decreases and NFE is tracked
    verifyLessThan(testCase, best_cost, 1.0, 'RLMOA failed to converge near optimum on Sphere function.');
    verifyGreaterThan(testCase, nfe, 0, 'Function evaluation counter (NFE) must be greater than zero.');
end

%% Test 3: Custom Initial Population Input
function testCustomInitialPopulation(testCase)
    dim = 4;
    lb = -5;
    ub = 5;
    max_iter = 5;
    obj_fun = @(x) sum(x.^2);
    
    % Supply a specific 1 x dim initial solution
    init_pop = [1.0, 2.0, -1.0, 0.5];
    
    [best_pos, best_cost, nfe] = RLMOA(obj_fun, dim, lb, ub, max_iter, init_pop);
    
    verifyEqual(testCase, length(best_pos), dim, 'Output position dimension does not match input dim.');
    verifyClass(testCase, best_cost, 'double', 'Best cost output must be a double precision scalar.');
    verifyGreaterThan(testCase, nfe, 0, 'NFE count failed to populate.');
end

%% Test 4: Asymmetric Boundary Vector Handling
function testAsymmetricBounds(testCase)
    dim = 3;
    lb = [-10, -5, -2];
    ub = [ 10,  5,  2];
    max_iter = 10;
    obj_fun = @(x) sum(x.^2);
    
    [best_pos, ~, ~] = RLMOA(obj_fun, dim, lb, ub, max_iter);
    
    verifyTrue(testCase, all(best_pos >= lb) && all(best_pos <= ub), ...
        'RLMOA failed to respect vector-valued lower and upper boundaries.');
end
