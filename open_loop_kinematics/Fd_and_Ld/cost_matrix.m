function [blk_Q] = cost_matrix(np, nv, tf, weights)
%cost_matrix Cost matrix for the jerk optimization problem
%   Detailed explanation goes here

n_coeff = np+1;
if (isempty(weights) || sum(weights == 0))
    weights = 1;
end
weights = weights/sum(weights); % normalize weights

% Cost function matrix
Q = zeros(n_coeff, n_coeff);
for i = 1:np
    for j = 1:np
        Q(i+1,j+1) = i*j*tf^(i+j-1)/(i+j-1);
    end
end

Qc = cell(nv, 1);
for veh = 1:nv
    Qc{veh} = weights(veh)*Q;
end
blk_Q = blkdiag(Qc{:});



end

