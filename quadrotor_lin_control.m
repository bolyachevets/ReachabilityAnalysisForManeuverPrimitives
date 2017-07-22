function quadrotor_lin_control()

% quadrotor longitudinal model:

K = 0.89;
m = 1.4;
n0 = 55;
d0 = 70;
d1 = 17;
g = 9.81;

% base thrust
u_0 = 0;

% linearize around u_0, u1_bar in Fig. 3 in Mitchell et al, 2016
u_0 = u_0 + g*m/K;

% linearize about roll angle theta_0 (rad), x5_bar in Fig. 3
theta_0 = 0;

A = [0 0 1 0   0                      0;
     0 0 0 1   0                      0;
     0 0 0 0 K/m * u_0*cos(theta_0)   0;
     0 0 0 0 -K/m * u_0*sin(theta_0)   0;
     0 0 0 0   0                      1;
     0 0 0 0 -d0                    -d1; ];

B = [0                                              0;
     0                                              0;
     K/m * sin(theta_0)                             0;
     K/m * cos(theta_0)                             0;
     0                                              0;
     0                                             n0;
     0                                              0] ;
 
 const = [ 0;
           0;
          -K/m * u_0 * theta_0 * cos(theta_0);
           K/m * u_0 * theta_0 * sin(theta_0) - g;
           0;
           0 ];
      
% augmented system dynamics matrix that incorporates the constant term       
A_aug_c = [A, const; zeros(1, 7)];

% potentially will need to get rid of the similar fields in the options
tStart=0; %start time
tFinal=1; %final time
timeStep=0.1; %time step size for reachable set computation
number_steps = (tFinal-tStart)/timeStep;

% apply matrix transformation from continuous to discrete dynamics
% Schuermann 2017c, section 3.1
A_aug_d = expm(A_aug_c*timeStep);

integrandB = @(tau) expm(A_aug_c*tau);
B_d = integral(integrandB, 0, timeStep, 'ArrayValued', true)*B;
                    
% had to add an extra state variable set to 1 to accomodate the augmented A
% matrix that includes the constant term: x_dot = [A c; 0]*[x; 1]'
IC = interval([-1.2; 0.5; -0.5; -0.8; -0.1; -0.3; 1], [1.2; 1.7; 0.5; 0.8; 0.1; 0.3; 1]);
% use this for experimentation with rescaling IC intervals
% scaling = 0.4;
% IC_length = supremum(IC) - infimum(IC);
% IC = interval(infimum(IC)+scaling*IC_length, supremum(IC)-scaling*IC_length);

IC_Z = zonotope(IC);

% state constraints to check during reachability analysis
% the last entry is for the dummy dimension in lieu of augmented matrix A:
% it does not matter what those values are
StateConstr = interval([-1.7; 0.3; -0.8; -1.0; -0.15; -pi/2; -1], [1.7; 2.0; 0.8; 1.0; 0.15; pi/2; 1]);

% use this for experimentation with rescaling StateConstr intervals
scaling = 1;
StateConstr_length = supremum(StateConstr) - infimum(StateConstr);
StateConstr = interval(infimum(StateConstr)-scaling*StateConstr_length, supremum(StateConstr)+scaling*StateConstr_length);

u_control = interval([-0.5 + u_0; -pi/16], [0.5 + u_0; pi/16]);
u_Z = zonotope(u_control);
u_Z_generators = get(u_Z, 'Z');
% generator matrix for the control policy zonotope
% dropped the first column: center of the zonotope
u_Z_generators = u_Z_generators(:, 2:length(u_Z_generators));

%specify continuous dynamics-----------------------------------------------
quadrotor_lin_control=linearSys('quadrotor_lin_control', A_aug_d, B_d); %initialize system
%--------------------------------------------------------------------------

%set options --------------------------------------------------------------
dim=7;
options.tStart=tStart; %start time
options.tFinal=timeStep; %final time
options.R0=IC_Z; %initial state for reachability analysis
options.U=u_Z;
options.timeStep=timeStep; %time step size for reachable set computation
options.taylorTerms=4; %number of taylor terms for reachable sets
options.zonotopeOrder=200; %zonotope order
options.originContained=0;
options.reductionTechnique='girard';
options.path = [coraroot '/contDynamics/stateSpaceModels'];
% not sure if this is OK
options.uTrans=center(u_Z);

%------ from the reach.m method, otherwise initReach complains
%obtain factors for initial state and input solution
for i=1:(options.taylorTerms+1)
    %time step
    r = options.timeStep;
    %compute initial state factor
    options.factor(i)= r^(i)/factorial(i);    
end
%-------------------------------------------------------------

% dilate the initial reach set to encompass potential trajectory curvature
[x_0, ~] = initReach(quadrotor_lin_control, IC_Z, options);

% reduce number of generators in the first reach set
x_0 = reduce(x_0.ti, 'girard', 1);

IC_Z_generators = get(x_0, 'Z');

% generator matrix for the initial conditions zonotope -
% dropped the first column: center of the zonotope
IC_Z_generators = IC_Z_generators(:, 2:length(IC_Z_generators));
% reset the initial conditions zonotope to the overapproximated reach set
IC_Z = x_0;

<<<<<<< HEAD
=======
X_trajectory = horzcat(X_trajectory, x_0);

% store control coefficients here
Alpha_Cx = [];
Alpha_Gx1 = [];
Alpha_Gx2 = [];
Alpha_Gx3 = [];
Alpha_Gx4 = [];
Alpha_Gx5 = [];
Alpha_Gx6 = [];


>>>>>>> origin/master
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
        
                minimize(norm([
                    % difference between the center of the reachable set
<<<<<<< HEAD
                    % and desired final state: the origin/ or center of
                    % initial set
                    center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), number_steps) ...
                    + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_cx), 2) - center(IC_Z);
=======
                    % and desired final state: center of the first reachable set/ initial conditions
                    center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
                    + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Cx(:); alpha_cx] - center(IC_Z);
>>>>>>> origin/master
                    % length of generators of the reachable set
                    (A_aug_d^number_steps)*IC_Z_generators(:,1) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx1), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,2) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx2), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,3) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx3), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,4) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx4), 2);
                    (A_aug_d^number_steps)*IC_Z_generators(:,5) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx5), 2); 
                    (A_aug_d^number_steps)*IC_Z_generators(:,6) + sum(generator_matrix(A_aug_d, B_d, u_Z_generators, number_steps, alpha_gx6), 2)], 1))
                
                subject to
                    abs(alpha_cx) + abs(alpha_gx1) + abs(alpha_gx2) + abs(alpha_gx3) ...
                    + abs(alpha_gx4) + abs(alpha_gx5) + abs(alpha_gx6) <= 1
                   
                for i=1:number_steps
                
                     center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
<<<<<<< HEAD
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
=======
                    + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Cx(:); alpha_cx] ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,1) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx1(:); alpha_gx1]) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,2) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx2(:); alpha_gx2]) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,3) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx3(:); alpha_gx3]) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,4) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx4(:); alpha_gx4]) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,5) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx5(:); alpha_gx5]) ...
                    + abs((A_aug_d^i)*IC_Z_generators(:,6) ... 
                    + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx6(:); alpha_gx6]) <= supremum(StateConstr)

                    (center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
                    + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Cx(:); alpha_cx] ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,1) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx1(:); alpha_gx1]) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,2) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx2(:); alpha_gx2]) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,3) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx3(:); alpha_gx3]) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,4) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx4(:); alpha_gx4]) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,5) + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx5(:); alpha_gx5]) ...
                    - abs((A_aug_d^i)*IC_Z_generators(:,6) ...
                    + generator_matrix(A_aug_d, B_d, u_Z_generators, i)*[Alpha_Gx6(:); alpha_gx6])) >= infimum(StateConstr)

>>>>>>> origin/master
    cvx_end

    alpha_cx
    alpha_gx1
    alpha_gx2
    alpha_gx3
    alpha_gx4
    alpha_gx5
    alpha_gx6
    %abs(alpha_cx) + abs(alpha_gx1) + abs(alpha_gx2) + abs(alpha_gx3) + abs(alpha_gx4) + abs(alpha_gx5) + abs(alpha_gx6)

  % reconstruct the optimized reachable set using the optimal control weights
    X_reach = [];
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

        X_reach = horzcat(X_new_zonotope, X_reach);   
    end
    X_reach = horzcat(x_0, X_reach);
    
for plotRun=1:3
    % plot different projections
    if plotRun==1
        projectedDimensions=[1 2];
    elseif plotRun==2
        projectedDimensions=[3 4]; 
    elseif plotRun==3
        projectedDimensions=[5 6]; 
    end 
    
    figure;
    hold on
    
    for i=1:length(X_reach)
        %plot reachable sets 
        plotFilled(X_reach(:,i),projectedDimensions,[.8 .8 .8],'EdgeColor','none');
        %plot reachable set contours
        plot(X_reach(:, i),projectedDimensions,'r-','lineWidth',2);
    end
    
    %plot final reachable set contour
    plot(X_reach(:, length(X_reach)),projectedDimensions,'b-','lineWidth',4);
    
    %plot initial reachable set
    plot(x_0,projectedDimensions,'b-','lineWidth',2);
    
    %plot constraint set
    plot(StateConstr,projectedDimensions,'g-','lineWidth',2);
    
    %label plot
    xlabel(['x_{',num2str(projectedDimensions(1)),'}']);
    ylabel(['x_{',num2str(projectedDimensions(2)),'}']);

end
  
end
