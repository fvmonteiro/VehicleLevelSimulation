function [Aeq, beq] = create_eq_constr(np, tf, d, v0, vf, a0, af, jerk0, jerkf)
%create_eq_constr Create matrix and vector corresponding to equality
%constraints
%   Detailed explanation goes here

n_coeff = np+1; % number of coefficients = polynomial order + 1
neq_const = 6; % the number of constraints is fixed
nv = length(v0); % number of vehicles

Tf = zeros(neq_const, n_coeff);
n = 0:np;
Tf(1, 1) = 1;
Tf(2, 2) = 1;
Tf(3, :) = tf.^(n);
Tf(4, :) = n.*tf.^(n-1);
Tf(5, :) = tf.^(n+1)./(n+1);
Tf(6, :) = tf.^(n+2)./((n+1).*(n+2));

beq = zeros(neq_const-1, nv);
for veh = 1:nv
    beq(:, veh) = [a0(veh); jerk0(veh); af(veh); jerkf(veh); vf(veh) - v0(veh)];
end
beq = [beq(:);d(:)];

switch nv
    case 1
        Aeq = [blkdiag(Tf(1:5, :));
            -Tf(6, :)];
    case 2
        Aeq = [blkdiag(Tf(1:5, :), Tf(1:5, :));
            -Tf(6, :), Tf(6, :)];
    case 3
        Aeq = [blkdiag(Tf(1:5, :), Tf(1:5, :), Tf(1:5, :));
            -Tf(6, :), Tf(6, :), zeros(1, n_coeff);
            zeros(1, n_coeff), Tf(6, :),  -Tf(6, :)];
end

end

