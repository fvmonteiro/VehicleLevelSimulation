function dydt = vanDerPol(t, y, omega, sigma, mu, u)
%vanDerPol ODE for van der pol oscilator

dydt = zeros(2, 1);
dydt(1) = y(2);
dydt(2) = 2*omega*sigma*(1-mu*y(1)^2)*y(2)-omega^2*y(1) + u;

end

