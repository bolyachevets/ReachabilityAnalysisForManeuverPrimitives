% overapproximation for  absolute values of the Lagrange remainder

function errBound = computeErrorBound(gamma, x3_H, x4_H)

    x3_e = 0.5*gamma'*abs(x3_H)*gamma;
    x4_e = 0.5*gamma'*abs(x4_H)*gamma;
    
    errBound = [0;0;x3_e;x4_e;0;0;0]
   
end