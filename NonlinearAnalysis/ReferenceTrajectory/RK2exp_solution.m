% applies explicit 2-stage Runge-Kutta method for a N time steps of size delta_t

function y = RK2exp_solution(IC, u, delta_t, N)
   
   % if number of time steps N = 0, then exit the function prematurely
   if N==0
       y = IC;
       return
   end
  
   % auxillary variables
   y1 = IC;   
   j=1;
   
   for i=1:N
        f1 = quadrotor_nonlin(u(j:j+1), y1);
        y2 = y1 + delta_t*f1;
        f2 = quadrotor_nonlin(u(j:j+1), y2);
        y = y1 + 0.5*delta_t*(f1 + f2);
        y1 = y;
        j=j+2;
   end
 
end