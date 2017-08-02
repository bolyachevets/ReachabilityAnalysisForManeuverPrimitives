% compute the vector of maximum second order derivatives
% x = [u1; x5]
function [x3_H, x4_H] = max2Der(u1_bar, x5_bar, z_x, z_u)

    % get over approximating bounds on u1 and x5
    IH_X_new = interval(z_x);
    lb_x = infimum(IH_X_new) - abs([0;0;0;0;x5_bar;0;0]);
    lb_x5 = lb_x(5);
    ub_x = supremum(IH_X_new) + abs([0;0;0;0;x5_bar;0;0]);
    ub_x5 = ub_x(5);
    
    IH_U_new = interval(z_u);
    abs([u1_bar;0]);
    lb_u = infimum(IH_U_new);
    lb_u1 = lb_u(1);
    ub_u = supremum(IH_U_new);
    ub_u1 = ub_u(1);
    
    lb = [lb_u1;lb_x5];
    ub = [ub_u1;ub_x5];    
    
    obj_funX3_x5x5 = @(x) -abs(lin_errorX3_x5x5(x));        
    obj_funX3_u1x5 = @(x) -abs(lin_errorX3_u1x5(x));
    obj_funX4_x5x5 = @(x) -abs(lin_errorX4_x5x5(x));
    obj_funX4_u1x5 = @(x) -abs(lin_errorX3_u1x5(x));
    
    % linear constraints
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    x0 = [u1_bar; x5_bar];
    
    x3_max_x5x5 = fmincon(obj_funX3_x5x5,x0,A,b,Aeq,beq,lb,ub);
    x3_max_u1x5 = fmincon(obj_funX3_u1x5,x0,A,b,Aeq,beq,lb,ub);
    x4_max_x5x5 = fmincon(obj_funX4_x5x5,x0,A,b,Aeq,beq,lb,ub);
    x4_max_u1x5 = fmincon(obj_funX4_u1x5,x0,A,b,Aeq,beq,lb,ub);

    x3_H = [0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, lin_errorX3_x5x5(x3_max_x5x5), 0, 0, lin_errorX3_u1x5(x3_max_u1x5), 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, lin_errorX3_u1x5(x3_max_u1x5), 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;];
           
    x4_H = [0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, lin_errorX4_x5x5(x4_max_x5x5), 0, 0, lin_errorX4_u1x5(x4_max_u1x5), 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, lin_errorX4_u1x5(x4_max_u1x5), 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0;];

end