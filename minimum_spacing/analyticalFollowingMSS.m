function [gSafe, specialIdx] = analyticalFollowingMSS(ego, other)
%computeFollowingMSS Computes minimum following distance analytically
% specialIdx output is used only for checking min gaps that don't happen
% when vehicles achieve full stop

% We need to know if we're computing safe distance from main to leader or
% from follower to main
if strncmpi(other.location, 'F', 1)
    leader = ego;
    follower = other;
else
    leader = other;
    follower = ego;
end

% Renaming variables (just for easier coding)
delay = follower.reactionTime;
vL0vector = leader.v0;
vF0vector = follower.v0;
aFmax = follower.maxAccel;
bFmax = follower.maxBrake;
bLmax = leader.maxBrake;
jFmax = follower.maxJerk;
[h, d0] = follower.safeHeadwayParams();


gSafe = zeros(length(vL0vector), length(vF0vector));
specialIdx = zeros(length(vL0vector), length(vF0vector));

for nL = 1:length(vL0vector)
    vL0 = vL0vector(nL);
    for nF = 1:length(vF0vector)
        vF0 = vF0vector(nF);
        if bLmax>=bFmax
            % Case 1: leader can brake at least as hard as follower
            gSafe(nL, nF) = vF0^2/2/bFmax - vL0^2/2/bLmax + h*vF0 + d0;
        else
            % Case 2: leader cannot brake as hard as follower
            tc = (bLmax + aFmax)/jFmax; % t when aF(t) = bmaxL
            t1 = (bFmax + aFmax)/jFmax; % t when aF(t) = bmaxF;
            
            alpha = jFmax*(1/2*tc^2 + tc*delay);
            beta = jFmax*(1/2*t1^2 + t1*delay);
            
            if vF0 >= vL0-alpha && vF0 < vL0-alpha+1/2*(bFmax-bLmax)^2/jFmax
                tmax = delay + tc + sqrt(tc^2+2*tc*delay - 2*(vL0-vF0)/jFmax);
                gSafe(nL, nF) = (vF0-vL0)*tmax + (aFmax+bLmax)*tmax^2/2 - jFmax*(tmax-delay)^3/6;
                
                specialIdx(nL, nF) = 1;

            elseif  vF0 >= vL0-alpha+1/2*(bFmax-bLmax)^2/jFmax && vF0 < bFmax/bLmax*vL0-beta
                tmax = (vF0-vL0 + beta)/(bFmax-bLmax);
                deltaSF = vF0*tmax + aFmax*(delay+t1)*(tmax-(delay+t1)/2) + ...
                    jFmax*t1^2*(delay/2+t1/3-tmax/2) - 1/2*bFmax*(tmax-delay-t1)^2;
                deltaSL = vL0*tmax-1/2*bLmax*tmax^2;
                gSafe(nL, nF) = deltaSF-deltaSL;
                
                specialIdx(nL, nF) = 2;
            else
                gSafe(nL, nF) = vF0^2/2/bFmax - vL0^2/2/bLmax + h*vF0 + d0;
            end
        end
        gSafe(nL, nF) = max(gSafe(nL, nF), h*follower.v0(nF) + d0);
    end
end

% For consistency, always return a column vector if gSafe is a 1D array
if nL==1
    gSafe = gSafe';
end


end