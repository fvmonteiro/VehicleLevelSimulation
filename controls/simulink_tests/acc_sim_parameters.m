clearvars;


% Vehicle parameters (initially, the same for both)
m       = 1575;     % Total mass of vehicle                          (kg)
Iz      = 2875;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.2;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.6;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)
tau     = 0.5;      % Longitudinal time constant                     (N/A)
maxAcc = 3; % [m/s^2]
minAcc = -3; % [m/s^2]
maxJerk = 4; % [m/s^3]
minJerk = -4; % [m/s^3]

% Follower initial state
follInitPos = 0;
follInitVel = 23; % [m/s]

% ACC parameters
g0 = 2; % [m] desired gap at standstill position
timeGap = 1; % [s] headway time gap
Ks = 2; % ACC position gain
Kv = 1; % ACC velocity gain
initMinGap = g0 + timeGap*follInitVel;

% Leader initial state
leaderInitPos = initMinGap;
leaderInitVel = 20; % [m/s]