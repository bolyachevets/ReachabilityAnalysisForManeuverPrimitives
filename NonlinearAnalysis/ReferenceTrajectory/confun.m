% constraints on state variables
function [c, ceq] = confun(IC, StateConstr, u, delta_t, N)

% upper/lower bounds on state variables
uB = supremum(StateConstr);
lB = infimum(StateConstr);

% Nonlinear inequality constraints
c = [];
for i=1:N
    c = vertcat(c, eulers_solution(IC, u, delta_t, N) - uB);
    c = vertcat(c, lB - eulers_solution(IC, u, delta_t, N));
end

% Nonlinear equality constraints
ceq = [];


