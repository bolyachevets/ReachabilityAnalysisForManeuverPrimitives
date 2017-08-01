function gamma = computeGamma(centerX, x5_bar, Gx, centerU, u1_bar, Gu)
% need to make an augmented state space consisting of both state and
% control variables, hence all the vertcat commands
    z_star = [0;0;0;0;x5_bar;0;0;u1_bar;0];
    center = vertcat(centerX, centerU);

    gamma = abs(center - z_star) + vertcat(sum(Gx,2),0,0) + vertcat(0,0,0,0,0,0,0,sum(Gu,2));   
end