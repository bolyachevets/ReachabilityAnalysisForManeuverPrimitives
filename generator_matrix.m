% creates a generator matrix for the reachable set

function gen_mat = generator_matrix(A, B, Gu, N)
gen_mat = [];
    while N > 0
        gen_mat = horzcat(gen_mat, (A^(N-1))*B*Gu);
        N = N-1;
    end
end

