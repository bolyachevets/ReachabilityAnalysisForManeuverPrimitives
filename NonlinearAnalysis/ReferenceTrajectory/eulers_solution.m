% applies Euler's method for a N time steps of size delta_t
% don't check for constraint satisfaction here, as the
% control for reference trajectory has already been optimized

function y = eulers_solution(IC, u, delta_t, N)
   
   % if number of time steps N = 0, then exit the function prematurely
   if N==0
       y = IC;
       return
   end
   
   % auxillary variables
   y_temp = IC;   
   j=1;
   
   for i=1:N
        y = y_temp + delta_t*quadrotor_nonlin(u(j:j+1), y_temp);
        y_temp = y;
        j=j+2;
   end

end