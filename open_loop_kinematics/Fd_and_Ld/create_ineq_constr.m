function [Aineq, bineq] = create_ineq_constr(np, tf, v0, a_bounds, jerk_bounds, v_bounds)
%create_ineq_constr Create matrix and vector corresponding to inequality
%constraints
%   Detailed explanation goes here

% We create a 3D matrix - each 2D matrix represents the constraints for a
% single sampled time instant. Afterwards we transform this in a single 2D
% matrix to properly pass it to the optimizer
n_coeff = np+1;
nv = length(v0);

n = 0:np;
nineq_const = 3; % acceleration, jerk and velocity
delta = 0.1; % maybe using the original sampling time creates too many constraints
tk = 0:delta:tf;
K = length(tk);
Tk = zeros(nineq_const, n_coeff, K);
for k = 1:K
    Tk(1, :, k) = tk(k).^n;
    Tk(2, :, k) = (n).*tk(k).^(n-1);
    Tk(3, :, k) = tk(k).^(n+1)./(n+1);
end
Tk = permute(Tk, [1 3 2]);
Tk = reshape(Tk, 3*K, n_coeff);
Tk_rep = repmat(Tk, 1, nv);
Tk_cell = mat2cell(Tk_rep, size(Tk, 1), repmat(size(Tk, 2), 1, nv));
Tk_blk = blkdiag(Tk_cell{:});
Aineq = [Tk_blk; -Tk_blk];

upper_bounds = zeros(nineq_const, nv);
lower_bounds = zeros(nineq_const, nv);
for veh = 1:nv
    lower_bounds(:, veh) = [a_bounds(1); jerk_bounds(1); v_bounds(1)-v0(veh)];
    upper_bounds(:, veh) = [a_bounds(2); jerk_bounds(2); v_bounds(2)-v0(veh)];
end

lower_bounds = repmat(lower_bounds(:), K, 1);
upper_bounds = repmat(upper_bounds(:), K, 1);

bineq = [upper_bounds; -lower_bounds];
end

