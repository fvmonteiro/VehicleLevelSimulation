function dydt = vanDerPolWithFailure(t, y, vdpParams, failureType, p, approximatorParams, refWithTime, ftc)
%vanDerPolWithFailure ODE for van der pol oscilator with failure

% States 1,2 are the real values; states 3,4 are the estimated values
% y = [x1, x2, x1_hat, x2_hat, theta]'

omega = vdpParams(1);
psi = vdpParams(2);
mu = vdpParams(3);
gamma = approximatorParams(1);
c = approximatorParams(2:end-1);
sigma = approximatorParams(end);

% Online approximator values
rbf = exp(-(y(1)-c').^2/sigma^2);
theta = y(5:end);
fHat = theta'*rbf;

% If there is a reference, we apply control
if isempty(refWithTime)
        u = 0;
else
    tyR = refWithTime(:, 1);
    idx(1) = find(tyR<=t, 1, 'last');
    idx(2) = find(tyR>=t, 1);
    if idx(1)==idx(2)
        ref = refWithTime(idx(1), 2);
    else
        ref = interp1(tyR(idx), refWithTime(idx, 2), t);
    end
    u = (2*omega*psi*(mu*y(1)^2 - 1) - 4)*y(2) + (omega^2-4)*y(1) + ref;
    if ftc % accommodation
        u = u - fHat;
    end
end



% Real values
dydtModel = vanDerPol(t, y, omega, psi, mu, u);

switch lower(failureType)
    case 'none'
        f = 0;
    case 'sin'
        f = simpleStep(t)*sin(y(1));
    case 'y2'
        f = simpleStep(t)*5*y(1)^2;
    case 't*sin'
        f = simpleStep(t)*(t-10)*sin(y(1));
    otherwise
        error('Unknown failure type')
end

dydt  = dydtModel+ [0; f];

% Estimator dynamics and error
dydt(3:4) = dydtModel(1:2) + [0; fHat] - p*(y(3:4)-y(1:2));

e = y(1:2) - y(3:4);

% Estimated online approximator learning 
Z = [zeros(length(c), 1), rbf]; % Z = d(fHat)/d(theta)
Gamma = gamma*eye(length(c));
dydt(5:5+length(c)-1) = Gamma*Z*e;


    function x = simpleStep(t)
        x = t>=10;
    end

end

