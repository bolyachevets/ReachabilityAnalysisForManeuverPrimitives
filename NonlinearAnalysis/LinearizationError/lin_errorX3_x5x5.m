function x3_x5x5 = lin_errorX3_x5x5(x)
   
global K m

   x3_x5x5 = -(K/m)*x(1)*sin(x(2));
    
end