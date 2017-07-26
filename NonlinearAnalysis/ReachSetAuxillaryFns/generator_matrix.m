% creates a generator matrix for the reachable set and multiplies its
% columns by the corresponding vectors of control weights

function gen_mat = generator_matrix(A, B, Gu, N, alpha)
gen_mat = [];
%     while N > 0
%         gen_mat = horzcat(gen_mat, (A^(N-1))*B*Gu*[alpha(2*N-1); alpha(2*N)]);
%         N = N-1;
%     end
    j = 1;
    for i=1:N
          gen_mat = horzcat(gen_mat, (A^(N-i))*B*Gu*[alpha(j); alpha(j+1)]);
          j = j+2;
    end
end

