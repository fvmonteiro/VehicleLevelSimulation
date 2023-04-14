function [egoVeh, otherVehs] = createVehicles(vehIdx, paramNames, paramValues)
%createVehicle Returns an ego vehicle and an array of other vehicles. If no
%specific parameters are given, they're all created with arbitrary initial 
%values.
%   The two optional parameters must be cells with same length. The first
%   specifies which parameter to set and the second contains the parameters
%   values
% Names:
% F - follower
% L - leader
% d - destination lane
% o - original lane

% vehWidth = 1.8; % [m] vehicle width
reactionTime = 0.2; % [s] time a vehicle takes to notice its leader is braking

% Vehicle indices
foIdx = vehIdx.fo;
loIdx = vehIdx.lo;
fdIdx = vehIdx.fd;
ldIdx = vehIdx.ld;

% Locations
locVeh(foIdx) = "Fo";
locVeh(loIdx) = "Lo";
locVeh(fdIdx) = "Fd";
locVeh(ldIdx) = "Ld";

% Types: P -  passenger vehicle, B - bus, T - truck
typeEgo = "P";
typeVeh(foIdx) = "P";
typeVeh(loIdx) = "T";
typeVeh(fdIdx) = "T";
typeVeh(ldIdx) = "P";

% Initial conditions 
% Longitudinal positions [m] (front bumper)
xEgo = 50;
x(foIdx) = 30;
x(loIdx) = 90;
x(fdIdx) = 10;
x(ldIdx) = 90;

% Lateral positions [m] (mid point of the front bumper)
yEgo = 0; %0.5*laneWidth; 
y(foIdx) = 0; %0.5*laneWidth;
y(loIdx) = 0; %0.5*laneWidth;
y(fdIdx) = 0; %1.5*laneWidth;
y(ldIdx) = 0; %1.5*laneWidth;

% Longitudinal speeds [m/s]
% speedRange = 10:0.2:30;
vEgo0 = 20;
v0(foIdx) =  22;
v0(loIdx) =  18;
v0(fdIdx) =  22;
v0(ldIdx) =  18;

if nargin > 1 % if there are specific parameters...
    for n = 1:length(paramNames)
        switch char(paramNames{n})
            case 'y'
                userY = paramValues{n};
                yEgo = userY.ego;
                y(foIdx) = userY.others(foIdx); %0.5*laneWidth;
                y(loIdx) = userY.others(loIdx);
                y(fdIdx) = userY.others(fdIdx);
                y(ldIdx) = userY.others(ldIdx);
            case 'vEgo'
                vEgo0 = paramValues{n};
            case 'loc'
                userLoc = paramValues{n};
                locVeh(foIdx) = userLoc(foIdx); 
                locVeh(loIdx) = userLoc(loIdx); 
                locVeh(fdIdx) = userLoc(fdIdx); 
                locVeh(ldIdx) = userLoc(ldIdx); 
        end
                
    end
end

egoVeh = Vehicle(typeEgo, 'ego', xEgo, yEgo, vEgo0, reactionTime);

otherVehs = Vehicle.empty(length(typeVeh), 0);
for k = 1:length(typeVeh)
    otherVehs(k) = Vehicle(typeVeh(k), locVeh(k), x(k), y(k), v0(k), reactionTime);
end

end