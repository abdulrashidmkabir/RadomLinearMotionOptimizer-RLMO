---
title: 'RandomLinearMotionOptimizer-RLMO: A novel heuristic optimizer for numerical functions'
tags:
  - MATLAB
  - optimization
  - heuristic search
  - metaheuristics
authors:
  - name: Kabir Abdulrashid Muhammad
    orcid: 0000-0002-5482-0640
    affiliation: 1
affiliations:
 - name: Department of Electrical and Electronics Engineering, Kebbi State University of Science and Technology Aliero, Nigeria
   index: 1
date: 27 September 2026
bibliography: paper.bib
---

# Summary
RandomLinearMotionOptimizer-RLMO is an open-source heuristic optimization algorithm designed to solve continuous and multi-variable optimization problems. It employs adaptive parameter updates to balance exploration and exploitation phases during search iterations.

# Statement of need
Engineers and researchers frequently face high-dimensional search spaces where traditional gradient-based methods fail or stall at local optima. While algorithms such as Particle Swarm Optimization (PSO) or Grey Wolf Optimizer (GWO) exist, NameOfSoftware improves convergence rates and robustness for constrained engineering design problems.

# Key Functionality
The core execution routine is encapsulated in `main.m`. Users provide an objective function handle, search bounds, and population settings. Built-in stopping criteria include maximum iterations and function evaluation budgets. The RLMO algorithm is achieved by 
the following function components.   

## Overall Function Components:
##   Main Function : main.m
##   Local Helper Function 0: simulate_random_linear_motion.m
##   Local Helper Function 1: evaluate_population.m
##   Local Helper Function 2: update_solutions.m
##   Local Helper Function 3: apply_bounds.m
##   Local Helper Function 4: calculate_kinematics.m

# Mathematics & Mechanics
The parameter updates rely on the following Equations of Linear Motion.
The RLMO employs the position 's' to model its population/parameter exploiting the three possible computation/estimation of 's'

# Kinematic Motion Formulation
The Random Linear Motion Optimizer (RLMO) algorithm leverages classical kinematic mechanics to govern search agent dynamics across the solution space. The four fundamental equations of linear motion with constant acceleration are defined as follows:

$$v = u + a t$$

$$s = u t + \frac{1}{2} a t^2$$

$$v^2 = u^2 + 2 a s \quad \implies \quad s = \frac{v^2 - u^2}{2 a}$$

$$s = \left(\frac{u + v}{2}\right) t$$

where:
- $s$ is the spatial displacement (distance) traversed by the candidate solution $[ \text{m} ]$,
- $u$ is the initial velocity vector of the agent at the start of the time step $[ \text{m/s} ]$,
- $v$ is the final velocity vector of the agent at time $t$ $[ \text{m/s} ]$,
- $a$ is the constant acceleration parameter controlling search step scale $[ \text{m/s}^2 ]$,
- $t$ is the effective movement time duration (iteration step) $[ \text{s} ]$.

These fundamental kinematics principles follow classical Newtonian mechanics [@halliday2013fundamentals; @serway2018physics] and have been adapted for stochastic population exploration in metaheuristic frameworks [@gandomi2013cuckoo].

where $\alpha$ represents the adaptive decay parameter calculated per iteration.

# References
References also attached separately...
