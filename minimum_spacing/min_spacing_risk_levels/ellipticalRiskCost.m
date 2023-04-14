function H = ellipticalRiskCost(distX, distY, vel, invCovMatrix, alpha, beta)

% Faster to code with matrix operation, but having issues today...
% Create the points grid
% distMatrix = combvec(distX, distY);
% 
% gaussianExp = (distMatrix'.^2*diag(covMatrix)).^beta;
% gaussianPeak = exp(-gaussianExp);
% H = gaussianPeak'./(1+exp(-alpha*vel'*distMatrix));

H = zeros(length(distY), length(distX));

for i = 1:length(distY)
    for j = 1:length(distX)
        dist = [distX(j);distY(i)];
        H(i,j) = exp(-(dist'*invCovMatrix*dist)^beta)/(1+exp(-alpha*vel'*dist));
    end
end

end