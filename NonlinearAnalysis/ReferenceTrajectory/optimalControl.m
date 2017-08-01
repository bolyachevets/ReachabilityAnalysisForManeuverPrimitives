% Optimization problem to find reference trajectory for the quadrotor
% motion starting at the center of initial set and ending as close as
% possible to the target location, while satisfying state constraints
% (hence nonlinear constraint passed as an additional argument to the
% optimizer)

function u = optimalControl(IC, ulb, uub, u0, delta_t, N, StateConstr, xf)

    % control weights
    % gamma = 1;

    obj_fun = @(u) norm(eulers_solution(IC, u, delta_t, N) - xf, 1);         %+ gamma*norm(u, 1);

    % set up a vector of bounds/guesses for multipoint trajectory optimization
    lb = repmat(ulb, N, 1);
    ub = repmat(uub, N, 1);
    u0 = repmat(u0, N, 1);

    % linear constraints
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    u = fmincon(obj_fun,u0,A,b,Aeq,beq,lb,ub,@(u)confun(IC, StateConstr, u, delta_t, N));
    %u = fmincon(obj_fun,u0,A,b,Aeq,beq,lb,ub);

end 
