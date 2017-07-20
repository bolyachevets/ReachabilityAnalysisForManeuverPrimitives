% creates a center for the reachable set

function cntr_reach = center_reach(A, B, cx, cu, N)
    cntr_reach = (A^N)*cx;
    while N > 0
        cntr_reach = cntr_reach + A^(N-1)*B*cu;
        N = N-1;
    end
end

