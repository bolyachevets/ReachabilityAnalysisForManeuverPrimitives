% applies Euler's method for a N time steps of size delta_t

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
    
    
%     for i=1:N
%         u =  Gu*alpha(j:j+1);
%         y = y_temp + delta_t*quadrotor_nonlin(y_temp, u);
%         y_temp = y;
%         j=j+2;
%     end
   
%     y0 = double(IC);
%     
%     dynamics = @(t, u, y)[y(3); 
%                          y(4);
%                          (K*u(1)/m )*sin(y(5)); 
%                          -g + (K*u(1)/m)*cos(y(5)); 
%                           y(6); 
%                           n0*u(2)-d0*y(5)-d1*y(6)];
%     
%         j=1;
%     
%         for i=1:N
%         u =  double(Gu*alpha(j:j+1));
%         tspan = double([(i-1)*delta_t, i*delta_t]);
%         [~, y] = ode45(@(t, y) dynamics(t, u, y), tspan, y0);
%         y0 = y(length(y),:);
%         j=j+2;
%         end
%     
%      y = y(length(y),:);
 
end