function [X_reach_temp] = quadrotor_linearized_dynamics(A_aug_d, B_d, IC, u_Z, u_Z_generators, ...
    StateConstr, number_steps, xf, u1_bar, x5_bar, timeStep)

IC_Z = zonotope(IC); 

IC_Z_generators = get(IC_Z, 'Z');
% generator matrix for the initial conditions zonotope -
% dropped the first column: center of the zonotope
IC_Z_generators = IC_Z_generators(:, 2:length(IC_Z_generators));


% main loop
cvx_begin
        % pairs of alpha_cx entries
        % give control policy for the center of initial condition zonotope
        % in the form of coefficients for the control zonotope generators
        variable alpha_cx(2*number_steps)
        % pairs of alpha_cx entries
        % give control policy for the generators of initial condition zonotope
        % in the form of coefficients for the control zonotope generators
        variable alpha_gx1(2*number_steps)
        variable alpha_gx2(2*number_steps)
        variable alpha_gx3(2*number_steps)
        variable alpha_gx4(2*number_steps)
        variable alpha_gx5(2*number_steps)
        variable alpha_gx6(2*number_steps)
                % equation (12) in Shurmann and Althoff, 2017c
                minimize(norm([
                    % difference between the center of the reachable set
                    % and desired final state: the origin/ or center of initial set
                    center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), number_steps) ...
                    + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_cx), 2)  - xf;
                    % length of generators of the reachable set
                    (A_aug_d^number_steps)*IC_Z_generators(:,1) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx1), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,2) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx2), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,3) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx3), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,4) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx4), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,5) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx5), 2); 
                    (A_aug_d^number_steps)*IC_Z_generators(:,6) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx6), 2)], 1))
                
                subject to
                % control generator weights constraint, equation (13) in Shurmann and Althoff, 2017c
                    abs(alpha_cx) + abs(alpha_gx1) + abs(alpha_gx2) + abs(alpha_gx3) ...
                    + abs(alpha_gx4) + abs(alpha_gx5) + abs(alpha_gx6) <= 1
                % state constraints:
                % equations (11) and (12) in Schurmann and Althoff, 2017b
                for i=1:number_steps
                
                     center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
                    + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_cx(1:2*i)), 2) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,1) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx1(1:2*i)), 2)) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,2) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx2(1:2*i)), 2)) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,3) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx3(1:2*i)), 2)) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,4) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx4(1:2*i)), 2)) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,5) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx5(1:2*i)), 2)) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,6) ...
                    + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx6(1:2*i)), 2)) <= supremum(StateConstr)

                      center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
                    + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_cx(1:2*i)), 2) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,1) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx1(1:2*i)), 2)) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,2) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx2(1:2*i)), 2)) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,3) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx3(1:2*i)), 2)) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,4) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx4(1:2*i)), 2)) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,5) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx5(1:2*i)), 2)) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,6) ...
                    - sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx6(1:2*i)), 2)) >= infimum(StateConstr)
                
                
                end             
cvx_end

  % reconstruct the optimized reachable set using the optimal control weights
   X_reach_temp = [];
    for i=1:number_steps
        X_new_center = center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
                        + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_cx(1:2*i)), 2);
        X_new_gx1 = (A_aug_d^i)*IC_Z_generators(:,1) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx1(1:2*i)), 2);
        X_new_gx2 = (A_aug_d^i)*IC_Z_generators(:,2) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx2(1:2*i)), 2);
        X_new_gx3 = (A_aug_d^i)*IC_Z_generators(:,3) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx3(1:2*i)), 2);
        X_new_gx4 = (A_aug_d^i)*IC_Z_generators(:,4) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx4(1:2*i)), 2);
        X_new_gx5 = (A_aug_d^i)*IC_Z_generators(:,5) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx5(1:2*i)), 2);
        X_new_gx6 = (A_aug_d^i)*IC_Z_generators(:,6) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, i, alpha_gx6(1:2*i)), 2);
        X_new_zonotope_mat = horzcat(X_new_center, X_new_gx1, X_new_gx2, X_new_gx3, X_new_gx4, X_new_gx5, X_new_gx6);
        X_new_zonotope = zonotope(X_new_zonotope_mat);
        
        % compute max values of second derivatives for each equation in the ODE
        [x3_H, x4_H] = max2Der(u1_bar, x5_bar, X_new_zonotope);
        
        % add error bound vector to the reachable set       
        X_new_zonotope_mat = horzcat(X_new_zonotope_mat, computeErrorBound(computeGamma(X_new_center, x5_bar, X_new_zonotope_mat, ...
            center(u_Z), u1_bar, u_Z_generators), x3_H, x4_H));
        
        X_new_zonotope = zonotope(X_new_zonotope_mat);

        X_reach_temp = horzcat(X_reach_temp, X_new_zonotope);   
    end    

end
  
