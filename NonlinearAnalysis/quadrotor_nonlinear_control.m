function quadrotor_nonlinear_control()
global K m g
K = 0.89;
m = 1.4;
g = 9.81;

% TIME
%----------------------------------------------------------------------
% potentially will need to get rid of the similar fields in the options
tStart=0; %start time
tFinal=5; %final time
timeStep=0.5; %time step size for reachable set computation
% number of linearization points
number_steps = (tFinal-tStart)/timeStep;
% number of steps at each linearization
number_small_steps = 1;
%----------------------------------------------------------------------

% INITIAL CONDITIONS
%----------------------------------------------------------------------
% had to add an extra state variable set to 1 to accomodate the augmented A
% matrix that includes the constant term: x_dot = [A c; 0]*[x; 1]'
IC = interval([-1.2; 0.5; -0.5; -0.8; -0.1; -0.3], [1.2; 1.7; 0.5; 0.8; 0.1; 0.3]);
% use this for experimentation with rescaling IC intervals
scaling = 0;
IC_length = supremum(IC) - infimum(IC);
IC = interval(infimum(IC)+scaling*IC_length, supremum(IC)-scaling*IC_length);
IC_Z = zonotope(IC);
%----------------------------------------------------------------------


% CONTROL SET
%----------------------------------------------------------------------
% base thrust
u_0 = g*m/K;
ulb = [-0.5 + u_0; -pi/16];
uub = [0.5 + u_0; pi/16];
u_control = interval(ulb, uub);
u_Z = zonotope(u_control);
u_Z_generators = get(u_Z, 'Z');
% generator matrix for the control policy zonotope
% dropped the first column: center of the zonotope
u_Z_generators = u_Z_generators(:, 2:length(u_Z_generators));
%----------------------------------------------------------------------

% REFERENCE TRAJECTORY
%----------------------------------------------------------------------
% optimization problem interior solution guess
u0 = [u_0; 0];
% testing target location to steer the center of IC towards
%xf = center(IC_Z);
xf = [-1; 1; 0.6; -0.2; 0; 0];
StateConstr = interval([-1.7; 0.3; -0.8; -1.0; -0.15; -pi/2], [1.7; 2.0; 0.8; 1.0; 0.15; pi/2]);
% controls for reference trajectory
u_ref = optimalControl(center(IC_Z),ulb, uub, u0, timeStep, number_steps, StateConstr, xf);
%----------------------------------------------------------------------


% AUGMENT STATE SPACE TO INCORPORATE CONSTANT TERM
%----------------------------------------------------------------------
% after reference trajectory has been calculated, augment the dynamics to
% add a dummy dimension in order to incorporate a constant term
IC_augmented = interval([-1.2; 0.5; -0.5; -0.8; -0.1; -0.3; 1], [1.2; 1.7; 0.5; 0.8; 0.1; 0.3; 1]);
IC_Z_augmented = zonotope(IC_augmented);

% state constraints to check during reachability analysis
% the last entry is for the dummy dimension in lieu of augmented matrix A:
% it does not matter what those values are
StateConstr = interval([-1.7; 0.3; -0.8; -1.0; -0.15; -pi/2; -1], [1.7; 2.0; 0.8; 1.0; 0.15; pi/2; 1]);
% use this for experimentation with rescaling StateConstr intervals
scaling = 1;
StateConstr_length = supremum(StateConstr) - infimum(StateConstr);
StateConstr = interval(infimum(StateConstr)-scaling*StateConstr_length, supremum(StateConstr)+scaling*StateConstr_length);
%----------------------------------------------------------------------

% REACH SET
%----------------------------------------------------------------------
 % initialize storage for reachable set trajectory
 X_reach = [];
 X_reach = horzcat(IC_Z_augmented, X_reach);
 
 % auxiallary index for control vector
 j=1;
for i = 1:number_steps
   % intermediate linearization values 
   X_int_lin = eulers_solution(center(IC_Z), u_ref, timeStep, i-1);
   % we are linearize in the middle of the subinterval
   X_int_lin = eulers_solution(X_int_lin, u_ref(j:j+1), timeStep/2, 1);

   % update transition matrices at the new linearization points
   A = computeA(u_ref(j), X_int_lin(5));
   B = computeB(X_int_lin(5));
   C = computeC(u_ref(j), X_int_lin(5));
   
   % augmented system dynamics matrix that incorporates the constant term       
   A_aug_c = [A, C; zeros(1, 7)];
   
   % apply matrix transformation from continuous to discrete dynamics
   % Schuermann 2017c, section 3.1
   A_aug_d = expm(A_aug_c*(timeStep/number_small_steps));

   integrandB = @(tau) expm(A_aug_c*tau);
   B_d = integral(integrandB, 0, (timeStep/number_small_steps), 'ArrayValued', true)*B;
   
   % intermediate target xf
   xf = eulers_solution(center(IC_Z), u_ref, timeStep, i);
   % add extra dimension because dynamimcs matrices have been augmented
   xf = vertcat(xf, 1);
               
   X_reach_temp = quadrotor_linearized_dynamics(A_aug_d, B_d, IC_Z_augmented, ...
       u_Z, u_Z_generators, StateConstr, number_small_steps, xf, ...
       u_ref(j), X_int_lin(5), timeStep);
   
   X_reach = horzcat(X_reach, X_reach_temp);
   % set up initial conditions for the next linearization interval
   IC_Z_augmented = X_reach_temp(:, length(X_reach_temp));
   j = j+2;
end 
%----------------------------------------------------------------------
 
% PLOTS
%----------------------------------------------------------------------
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
        %plotFilled(X_reach(:,i),projectedDimensions,[.8 .8 .8],'EdgeColor','none');
        %plot reachable set contours
        plot(X_reach(:, i),projectedDimensions,'r-','lineWidth',2);
    end
    
    %plot final reachable set contour
    plot(X_reach(:, length(X_reach)),projectedDimensions,'o-','lineWidth',2);
    
    
    %plot initial reachable set
    plot(IC_Z,projectedDimensions,'b-','lineWidth',2);
    
    %plot constraint set
    plot(StateConstr,projectedDimensions,'g-','lineWidth',2);
    
    %label plot
    xlabel(['x_{',num2str(projectedDimensions(1)),'}']);
    ylabel(['x_{',num2str(projectedDimensions(2)),'}']);

end
%----------------------------------------------------------------------

end