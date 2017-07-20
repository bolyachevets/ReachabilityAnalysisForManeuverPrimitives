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
tFinal=2; %final time
timeStep=0.1; %time step size for reachable set computation
number_steps = (tFinal-tStart)/timeStep;

% apply matrix transformation from continuous to discrete dynamics
% Schuermann 2017c, section 3.1
A_aug_d = expm(A_aug_c*timeStep);

integrandB = @(tau) expm(A_aug_c*tau);
B_d = integral(integrandB, 0, timeStep, 'ArrayValued', true)*B;
                    
% had to add an extra state variable set to 0 to accomodate the augmented A
% matrix that includes the constant term
IC = interval([-1.2; 0.5; -0.5; -0.8; -0.1; -0.3; 0],[1.2; 1.7; 0.5; 0.8; 0.1; 0.3; 0]);
IC_Z = zonotope(IC);
IC_Z_generators = get(IC_Z, 'Z');
% generator matrix for the initial conditions zonotope -
% dropped the first column: center of the zonotope
IC_Z_generators = IC_Z_generators(:, 2:length(IC_Z_generators));

% state constraints to check during reachability analysis
StateConstr = interval([-1.7; 0.3; -0.8; -1.0; -0.15; -pi/2; 0], [1.7; 2.0; 0.8; 1.0; 0.15; pi/2; 0]);

u_control = interval([-0.5 + u_0; -pi/16], [0.5 + u_0; pi/16]);
u_Z = zonotope(u_control);
u_Z_generators = get(u_Z, 'Z');
% generator matrix for the control policy zonotope
% dropped the first column: center of the zonotope
u_Z_generators = u_Z_generators(:, 2:length(u_Z_generators));

% set up a generator matrix of the reachable set to be used in the optimization problem
% only for 1 step ahead
GR = generator_matrix(A_aug_d, B_d, u_Z_generators, 1);
% set up a vector for the the reachable point of the initial zonotope
% center only for 1 step ahead
cR = center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), 1);

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

% store trajectory of reachable sets here
X_trajectory = [];

% dilate the initial reach set to encompass potential trajectory curvature
[x_0, options] = initReach(quadrotor_lin_control, IC_Z, options);

X_trajectory = horzcat(X_trajectory, x_0.ti);
% extract 

% store control coefficients here
Alpha_Cx = [];
Alpha_Gx1 = [];
Alpha_Gx2 = [];
Alpha_Gx3 = [];
Alpha_Gx4 = [];
Alpha_Gx5 = [];
Alpha_Gx6 = [];

% main loop
for i=1:number_steps
    cvx_begin;
        % pairs of alpha_cx entries
        % give control policy for the center of initial condition zonotope
        % in the form of coefficients for the control zonotope generators
        variable alpha_cx(2);
        % pairs of alpha_cx entries
        % give control policy for the generators of initial condition zonotope
        % in the form of coefficients for the control zonotope generators
        variable alpha_gx1(2);
        variable alpha_gx2(2);
        variable alpha_gx3(2);
        variable alpha_gx4(2);
        variable alpha_gx5(2);
        variable alpha_gx6(2);
        
                minimize(norm([center_reach(A_aug_d, B_d, center(IC_Z), center(u_Z), i) ...
                    + generator_matrix(A_aug_d, B_d, u_Z_generators, 1)*alpha_cx;
                    (A_aug_d^i)*IC_Z_generators(:,1) + GR*alpha_gx1;
                    (A_aug_d^i)*IC_Z_generators(:,2) + GR*alpha_gx2;
                    (A_aug_d^i)*IC_Z_generators(:,3) + GR*alpha_gx3;
                    (A_aug_d^i)*IC_Z_generators(:,4) + GR*alpha_gx4;
                    (A_aug_d^i)*IC_Z_generators(:,5) + GR*alpha_gx5; 
                    (A_aug_d^i)*IC_Z_generators(:,6) + GR*alpha_gx6], 1)); 
                
                subject to
                    abs(alpha_cx(1)) + abs(alpha_cx(2)) + abs(alpha_gx1(1)) + abs(alpha_gx1(2)) ...
                    + abs(alpha_gx2(1)) + abs(alpha_gx2(2)) + abs(alpha_gx3(1)) + abs(alpha_gx3(2)) ...
                    + abs(alpha_gx4(1))  + abs(alpha_gx4(2))+ abs(alpha_gx5(1)) + abs(alpha_gx5(2)) ...
                    + abs(alpha_gx6(1)) + abs(alpha_gx6(2)) <= 1;
                    
                % need to figure out how to construct controls from the
                % coefficients
                    sumLeft(A_aug_d*X_trajectory(:, i) + B_d*(center(u_Z) + u_Z*(alpha_cx ...
                   + alpha_gx1 + alpha_gx2 + alpha_gx3 ... 
                   + alpha_gx4 + alpha_gx5 + alpha_gx6))) >= infimum(StateConstr);
                
                    sumRight(A_aug_d*X_trajectory(:, i) + B_d*(center(u_Z) + u_Z*(alpha_cx ...
                   + alpha_gx1 + alpha_gx2 + alpha_gx3 ... 
                   + alpha_gx4 + alpha_gx5 + alpha_gx6))) <= supremum(StateConstr);
    cvx_end;
  
    Alpha_Cx = horzcat(Alpha_Cx, alpha_cx);
    Alpha_Gx1 = horzcat(Alpha_Gx1, alpha_gx1);
    Alpha_Gx2 = horzcat(Alpha_Gx2, alpha_gx2);
    Alpha_Gx3 = horzcat(Alpha_Gx3, alpha_gx3);
    Alpha_Gx4 = horzcat(Alpha_Gx4, alpha_gx4);
    Alpha_Gx5 = horzcat(Alpha_Gx5, alpha_gx5);
    Alpha_Gx6 = horzcat(Alpha_Gx6, alpha_gx6);
    
    X_trajectory = horzcat(X_trajectory, A_aug_d*X_trajectory(:, i) + B_d*(center(u_Z) + u_Z_generators*alpha_cx ...
                   + (u_Z_generators*alpha_gx1 + u_Z_generators*alpha_gx2 + u_Z_generators*alpha_gx3 ... 
                   + u_Z_generators*alpha_gx4 + u_Z_generators*alpha_gx5 + u_Z_generators*alpha_gx6)));
    options.tStart=options.tFinal; %start time
    options.tFinal=(i+1)*timeStep; %final time  
    
end

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
    
    %plot reachable sets 
    for i=1:length(X_trajectory)
        plotFilled(X_trajectory(1,i),projectedDimensions,[.8 .8 .8],'EdgeColor','none');
    end
    
    %label plot
    xlabel(['x_{',num2str(projectedDimensions(1)),'}']);
    ylabel(['x_{',num2str(projectedDimensions(2)),'}']);

end
  
end
