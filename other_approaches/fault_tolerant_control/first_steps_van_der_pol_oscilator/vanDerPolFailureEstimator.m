function dydt = vanDerPolFailureEstimator(t, y, omega, sigma, mu, u, vdpStates, fHat, p)
%vanDerPolFailureEstimator ODE to estimate van der pol oscilator with 
%failure

warning('not being used')

dydt = vanDerPol(t, vdpStates, omega, sigma, mu, u);

dydt = dydt + [0; fHat] - p*(y-vdpStates);

end