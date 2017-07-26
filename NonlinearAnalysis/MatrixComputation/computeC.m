function [C] = computeC(u_0, theta_0)
global K m g

         C = [ 0;
               0;
              -K/m * u_0 * theta_0 * cos(theta_0);
               K/m * u_0 * theta_0 * sin(theta_0) - g;
               0;
               0 ];

end