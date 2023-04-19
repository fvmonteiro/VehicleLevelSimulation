function [fx] = classKappaFunction(type, x, parameters)
%classKappaFunction Returns the value of a class Kappa function for value x
% Possible types are:
% - linear: alpha=parameters
% - exponential: exponent = parameters(1); multiplier = parameters(2)

switch type
    case 'linear'
        alpha = parameters;
        fx = alpha*x;
    case 'exponential'
        rho = parameters(1);
        gamma = parameters(2);
        fx = gamma * sign(x) * abs(x)^rho;
    otherwise
        error('Unknown class kappa function');
end
end