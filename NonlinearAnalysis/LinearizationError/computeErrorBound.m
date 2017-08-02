% overapproximation for  absolute values of the Lagrange remainder

function errBound = computeErrorBound(u1_bar, x5_bar, z_x, z_u)

   u_Z_generators = get(z_u, 'Z');
    u_Z_generators = u_Z_generators(:, 2:length(u_Z_generators));
  
   X_new_zonotope_mat = get(z_x, 'Z');
   X_new_center = center(z_x);
   
    gamma = computeGamma(X_new_center, x5_bar, X_new_zonotope_mat, ...
            center(z_u), u1_bar, u_Z_generators);
        
    [x3_H, x4_H] = max2Der(u1_bar, x5_bar, z_x, z_u);

    x3_e = 0.5*gamma'*abs(x3_H)*gamma;
    x4_e = 0.5*gamma'*abs(x4_H)*gamma;
    
    errBound = [0;0;x3_e;x4_e;0;0;0];
   
end