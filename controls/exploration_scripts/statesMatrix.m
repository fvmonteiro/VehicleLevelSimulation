function A = statesMatrix(vx, params)
% warning('old state space realization')
A = [0, 1, vx, 0; 
    0, -params(1), 0, -vx-params(2); 
    0, 0, 0, 1; 
    0, -params(3), 0, -params(4)];
end