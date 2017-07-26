function [B] = computeB(theta_0)
global K m n0

        B = [0                                              0;
             0                                              0;
             K/m * sin(theta_0)                             0;
             K/m * cos(theta_0)                             0;
             0                                              0;
             0                                             n0;
             0                                              0] ;
                          
end