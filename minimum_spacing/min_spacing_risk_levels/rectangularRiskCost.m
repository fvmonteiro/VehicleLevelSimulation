function H = rectangularRiskCost(distX, distY, vel, invCovMatrix, alpha, beta)

% Faster to code with matrix operation, but having issues today...
% Create the points grid
% distMatrix = combvec(distX, distY);
% 
% gaussianExp = (distMatrix'.^2*diag(covMatrix)).^beta;
% gaussianPeak = exp(-gaussianExp);
% H = gaussianPeak'./(1+exp(-alpha*vel'*distMatrix));

sigmaX2 = 1/invCovMatrix(1,1);
sigmaY2 = 1/invCovMatrix(2,2);
H = zeros(length(distY), length(distX));

for i = 1:length(distY)
    for j = 1:length(distX)
        dist = [distX(j);distY(i)];
        H(i,j) = exp(-(distX(j)^2/sigmaX2)^beta - (distY(i)^2/sigmaY2)^beta)/(1+exp(-alpha*vel'*dist));
    end
end

end