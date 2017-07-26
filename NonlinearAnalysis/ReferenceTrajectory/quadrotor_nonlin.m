% nonlinear quadrotor longitudinal model

function [dynamics] = quadrotor_nonlin(u, y)

%parameters
global K m n0 d0 d1 g
K = 0.89;
m = 1.4;
n0 = 55;
d0 = 70;
d1 = 17;
g = 9.81;    

dynamics = [y(3); 
            y(4);
            (K*u(1)/m )*sin(y(5)); 
            -g + (K*u(1)/m)*cos(y(5)); 
             y(6); 
             n0*u(2)-d0*y(5)-d1*y(6)];

end