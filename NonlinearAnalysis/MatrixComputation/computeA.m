function [A] = computeA(u_0, theta_0)
global K m d0 d1

        A = [0 0 1 0   0                      0;
             0 0 0 1   0                      0;
             0 0 0 0 K/m * u_0*cos(theta_0)   0;
             0 0 0 0 -K/m * u_0*sin(theta_0)   0;
             0 0 0 0   0                      1;
             0 0 0 0 -d0                    -d1; ];
end