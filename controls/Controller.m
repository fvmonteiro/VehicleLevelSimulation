classdef Controller < handle
    properties
        type
        vehicle
        K % veh following gains: (Ki), Kp, Kv
        Ka % gains for acceleration feedback when there is communication: 
        % Ka, Kd
        Kvel % velocity control gains: Ki, Kp (Kd)
        Klat % lat control gains (state feedback)
        Llat % gains for lateral states observer
        openLoopControl
%         openLoopIterCounter = 1;
        % Specific to Matlab integral controller
        intGapError = 0 % integral of gap error
        intVelError = 0 % integral of velocity error
        Kantiwindup
        windupFactor = 0

        % ACC stuff
        % To avoid high inputs when we start following a new vehicle
        maxPosError = 10; %[m] - could be dependent on speed
        minPosError = -100; %[m]
        accelSaturationFilter
        accLeaderVelFilter
        accLeaderPosFilter
        
    end
    
%     methods (Abstract)
%         setGains(obj, controlParams)
%     end

    methods
        function obj = Controller(vehicle, controlType)
            if nargin>0
                if isa(vehicle, 'Vehicle')
                    obj.vehicle = vehicle;
                    obj.type = controlType;
                else
                    error('Vehicle must be an object of the Vehicle class.')
                end
            end
        end
        
        function [bool] = checkType(obj, requestedType)
            bool = strcmpi(obj.type, requestedType);
        end
        
        function [] = setInput(obj, inputs)
            if strcmp(obj.type, 'OpenLoop')
                obj.openLoopControl = inputs;
            end
        end
        
        function [] = setGains(obj, controlParams)
%             obj.type = controlType;
            veh = obj.vehicle;
            switch obj.type
%                 case 'OpenLoop'
%                     obj.openLoopIterCounter = 1;
                case 'FullStateFeedback'
                    poles = controlParams;
                    obj.K = place(veh.A, veh.B, poles);
                case 'SimpleLQR'
                    Q = controlParams{1};
                    R = controlParams{2};
                    obj.K = lqr(veh.A, veh.B, Q, R);
                case 'ACC PID'
                    obj.Ka = [0, 0];

                    poles = controlParams;
                    p1 = poles(1);
                    p2 = poles(2);
                    p3 = poles(3);
                    C = [1 veh.h 0; 0 1 veh.h; 0 0 1];
                    d = [sum(poles); p1*p2+p1*p3+p2*p3; prod(poles)];
                    A = [-2 -veh.h 0];
                    b = -2/veh.h;
                    lb = zeros(length(poles), 1);
                    ub = Inf*ones(length(poles), 1);
                    options = optimoptions('lsqlin','Display','off');
                    x = lsqlin(C, d, A, b, [], [], lb, ub, [], options);
                    % x = [Kd; Kp; Ki];
                    obj.K = flip(x');
                    obj.checkStringStability();
                    obj.Kantiwindup = sqrt(obj.K(1)/obj.K(3)); %sqrt(Ki/Kd)
                case 'ACC PD'
                    poles = controlParams;
                    C = [1 veh.h; 0 1];
                    d = [sum(poles); prod(poles)];
                    A = [-2 -veh.h];
                    b = -2/veh.h;
                    lb = zeros(length(poles), 1);
                    ub = Inf*ones(length(poles), 1);
                    options = optimoptions('lsqlin','Display','off');
                    x = lsqlin(C, d, A, b, [], [], lb, ub, [], options);
                    % x = [Kd Kp]
                    obj.K = flip(x');
                case 'CACC PD'
                    % Following: Semi-autonomous adaptive cruise control 
                    % systems by Rajamani and Zhu, 2002
                    
%                     k1 = controlParams(1);
                    k = controlParams;
                    h = veh.h;
                    
                    % Sanity checks
                    if veh.tau==0
                        warning(['CACC (which has accel feedback) should only be ' ...
                            'applied to vehicles with actuator dynamics'])
                    end
                    
                    % Assign gains that guarantee string stability
                    obj.Ka(1) = 1.1*veh.tau/h;
                    obj.Ka(2) = obj.Ka(1)*k; %old formulation -k1*(1+k2*h);
                    
                    Kd = 1/h; %old formulation k1*k2+1/h;
                    Kp = k/h;
                    obj.K = [Kp, Kd];
                    
                    obj.checkStringStability();
                case 'CACC PID'
%                     k1 = controlParams(1);
                    k = controlParams;

                    h = veh.h;
                    tau = veh.tau;
                    
                    % Sanity checks
                    if veh.tau==0
                        warning(['CACC (which has accel feedback) should only be ' ...
                            'applied to vehicles with actuator dynamics'])
                    end                 
                    % Assign gains that guarantee string stability
                    obj.Ka(1) = 1.1*tau/h;
%                     obj.Ka(1) = k1;
                    obj.Ka(2) = obj.Ka(1)*k; %old formulation -k1*(1+k2*h);
                    upperBound = k^2/2/(h*(obj.Ka(1)*k*h+obj.Ka(1)+1)-tau);
                    k3 = upperBound*0.9;
                    
                    Kd = 1/h; %old formulation k1*k2+1/h;
                    Kp = k/h;
                    Ki = k3;
                    obj.K = [Ki, Kp, Kd];
                    
                    obj.checkStringStability();
                    
                case 'Queue LQR'
                    w = controlParams{1};
                    R = controlParams{2};
                    
                    nv = veh.nf+1;
                    % Cost matrices
                    F11 = diag([1; zeros(nv-1,1)]);
                    F12 = zeros(nv);
                    F21 = veh.A(nv+1:end,1:nv); %A21;
                    F22 = veh.A(nv+1:end,nv+1:end); %A22;
                    F22(1,1) = 1;
                    F = [F11 F12; F21 F22];
                    Q = F'*diag(w)*F;
                    lqrK = lqr(veh.A, veh.B, Q, R);
                    obj.K = lqrK;
                otherwise
                    error('Unknown controller type')
            end
        end
        
        function [] = setVelControlGains(obj, velCtrlPoles)
            % Velocity controller is a PI controller if no actuator lag and
            % a PID if there is actuator lag
            
            veh = obj.vehicle;
            if veh.tau > 0 % vehicle has actuator dynamics
                Avel = [0 1 0; 0 0 1; 0 0 -1/veh.tau];
                Bvel = [0; 0; -1/veh.tau];
                obj.Kvel = -place(Avel, Bvel, velCtrlPoles);
            else
                obj.Kvel = [prod(velCtrlPoles), -sum(velCtrlPoles), 0];
            end
            
        end
        
        function [] = setLatControlGains(obj, latParams)
            veh = obj.vehicle;
            
            if isnumeric(latParams) % poles for pole placement come in an array
                obj.Klat = place(veh.A, veh.B, latParams);
                obj.Llat = place(veh.A', veh.C', latParams*3)';
            elseif iscell(latParams) % weights for LQR come in a cell
                Q = latParams{1};
                r = latParams{2};
                obj.Klat = lqr(veh.A, veh.B, Q, r);
                % We find the resulting poles and pass them times a
                % constant to the observer
%                 closedLoopA = veh.A - veh.B*obj.Klat;
%                 poles = eig(closedLoopA);
                obj.Llat = place(veh.A', veh.C', -[0.5, 1, 2, 3]*5)'; %place(veh.A', veh.C', poles*3)';
            else
                error('Unknown parameter type for lateral control')
            end
             
        end
        
        function [] = checkStringStability(obj)
            %checkStringStability: Numerical checks for string stability
            %Not really necessary given the analytical checks already in
            %place but good practice
            
            hLC = obj.vehicle.hLC;
            h = obj.vehicle.h;
            
            if isempty(obj.vehicle.leader)
                return
            end
            
            if ~strcmpi(obj.vehicle.type, obj.vehicle.leader.type)
                % We are not checking for string stability of heterogeneous
                % strings
                return
            end
            
            switch obj.type 
                case 'ACC PID'
                    Ki = obj.K(1); Kp = obj.K(2); Kd = obj.K(3);                    
                    G = tf([Kd Kp Ki], [1 h*Kp+Kd h*Ki+Kp Ki]);
                    GLC = tf([Kd Kp Ki], [1 hLC*Kp+Kd hLC*Ki+Kp Ki]);
                    
                case 'CACC PD'
                    h = obj.vehicle.h;
                    hLC = obj.vehicle.hLC;
                    tau = obj.vehicle.tau;
                    k1 = obj.Ka(1); % leader accel gain
                    k2 = obj.K(2)*h; % "k5" from Rajamani and Zhu, 2002
                    
                    G = tf([k1*h, 1+k1*k2*h, k2], ...
                        [tau*h, h*(1+k1+k1*k2*h), 1+k1*k2*h+k2*h, k2]);
                    GLC = tf([k1*hLC, 1+k1*k2*hLC, k2], ...
                        [tau*hLC, hLC*(1+k1+k1*k2*hLC), ...
                        1+k1*k2*hLC+k2*hLC, k2]);
                    
                case 'CACC PID'
                    h = obj.vehicle.h;
                    hLC = obj.vehicle.hLC;
                    tau = obj.vehicle.tau;
                    k1 = obj.Ka(1); % leader accel gain
                    k2 = obj.K(2)*h; % "k5" from Rajamani and Zhu, 2002
                    k3 = obj.K(1); % integrator gain
                    
                    G = tf([k1*h, 1+k1*k2*h, k2, k3*h], ...
                        [tau*h, h*(1+k1+k1*k2*h), 1+k1*k2*h+k2*h, ...
                        k2+k3*h^2, k3*h]);
                    GLC = tf([k1*hLC, 1+k1*k2*hLC, k2 k3*hLC], ...
                        [tau*hLC, hLC*(1+k1+k1*k2*hLC), ...
                        1+k1*k2*hLC+k2*hLC, k2+k3*hLC^2, k3*hLC]);
                    
                otherwise
                    warning('String stability check not written for this controller')
            end
                        
            [mag, ~, wout] = bode(G);
            if any(mag>1)
                [maxMag, maxIdx] = max(mag);
                warning('%s\n |G(jw)|=%g at w = %g', ...
                    'Longitudinal controller is not string stable.', ...
                    maxMag, wout(maxIdx));
            end
            [y, ~] = impulse(G);
            if any(y<-0.01)
                warning('%s\n There is a t such that g(t)=%g', ...
                    'Longitudinal controller might create error oscilations.',... 
                    min(y));
            end
            
            [mag, ~, wout] = bode(GLC);
            if any(mag>1)
                [maxMag, maxIdx] = max(mag);
                warning('%s\n |G(jw)|=%g at w = %g', ...
                    ['Longitudinal controller is not string stable during '...
                    'longitudinal adjustment.'], ...
                    maxMag, wout(maxIdx));
            end
            [y, ~] = impulse(GLC);
            if any(y<-0.01)
                warning('%s\n There is a t such that g(t)=%g', ...
                    ['Longitudinal controller might create error '...
                    'oscilations during longitudinal adjustment.'],... 
                    min(y));
            end
            
        end
        
        function [u] = singleStepInput(obj, controlParams)
            veh = obj.vehicle;
            deltaT = veh.simTime(veh.iterCounter+1) ...
                - veh.simTime(veh.iterCounter);
            switch obj.type
                case 'OpenLoop'
                    u = obj.openLoopControl(veh.iterCounter);
%                     obj.openLoopIterCounter = ...
%                         obj.openLoopIterCounter + 1;
                case {'FullStateFeedback', 'SimpleLQR'}
                    q0 = veh.currentState;
                           
                    if nargin<3 % no provided reference
                        ref = [veh.position; veh.desiredSpeed];
                    else
                        if ~isempty(veh.leader)
                            desiredGap = controlParams;
                            positionRef = veh.leader.position;
                            velocityRef = veh.leader.velocity;
                            ref = [positionRef; velocityRef] - [desiredGap+veh.len; 0];
                        else
                            velocityRef = detailedRef;
                            ref = [veh.positoin; velocityRef];
                        end
                    end
                    u = obj.K*(ref-q0);
                    
                case 'Queue LQR' % only used by a cooperative platoon leader
                    q0 = veh.currentState;
                    
                    desiredGap = controlParams;
                    leaderPosition = veh.leader.position;
                    desiredPosition = leaderPosition - desiredGap - veh.len;
                    desiredSpeed = veh.leader.velocity;
                    
                    if ~isempty(veh.followers)
                        desiredGaps = [veh.followers.minACCGap] + [veh.followers.len];
                        ref = [desiredPosition; desiredGaps'; desiredSpeed*ones(veh.nf+1,1)];
                    else
                        ref = [desiredPosition; desiredSpeed];
                    end
                    
                    stateError = ref - q0';
                    u = obj.K*stateError;
            
                case 'ACC PID'
                    q0 = veh.currentState;
                    
                    leader = veh.leader;
                    if ~isempty(leader)
                        leaderPosition = leader.position;
                        leaderVelocity= leader.velocity;
                        positionRef = leaderPosition-veh.minACCGap-veh.len;
                        velocityRef = leaderVelocity;
                    else
                        positionRef = q0(veh.xIdx);
                        velocityRef = veh.desiredVelocity;
                    end
                    
                    posError = max(min(positionRef - q0(veh.xIdx), obj.maxPosError), obj.minPosError);
                    filteredVelRef = obj.accLeaderVelFilter.runFilter(velocityRef, deltaT);
                    velError = filteredVelRef-q0(veh.vxIdx);
                    
                    % Integrator stuff
                    obj.intGapError = obj.intGapError ...
                        + (posError*obj.K(1) ...
                        - obj.windupFactor*obj.Kantiwindup)*deltaT;
                    
                    % Compute input
                    stateError = [posError; velError];
                    u = obj.K(2:end)*stateError + obj.intGapError;
                    % No need to closely follow the leader if you're already at
                    % your desired speed
                    if u>0 && q0(veh.vxIdx)>(veh.desiredVelocity)
                        u = 0;
                    end
                    
                case 'ACC PD'
                    q0 = veh.currentState;
                    
                    leader = veh.leader;
                    if ~isempty(leader)
                        leaderPosition = leader.position;
                        leaderVelocity= leader.velocity;
                        positionRef = leaderPosition-veh.minACCGap-veh.len;
                        velocityRef = leaderVelocity;
                    else
                        positionRef = q0(veh.xIdx);
                        velocityRef = veh.desiredVelocity;
                    end
                    
                    posError = max(min(positionRef - q0(veh.xIdx), ...
                        obj.maxPosError), obj.minPosError);
                    filteredVelRef = obj.accLeaderVelFilter.runFilter(...
                        velocityRef, deltaT);
                    velError = filteredVelRef-q0(veh.vxIdx);

                    stateError = [posError; velError];
                    u = obj.K*stateError;
                    % No need to closely follow the leader if you're already at
                    % your desired speed
                    if u>0 && (q0(veh.vxIdx)>=(veh.desiredVelocity) ...
                            || posError>=veh.minACCGap)
                        u = 0;
                    end
                    
                case 'CACC PD'
                    if ~isempty(veh.leader)
                        % Vehicle following controller
                        errorVector = obj.computeVehFollowingError();
                        u = [obj.K(1), obj.Ka(2), obj.K(2), obj.Ka(1)] ...
                            * errorVector;
                    else
                        % Velocity controller
                        errorVector = obj.computeVelocityError();
                        u = obj.Kvel * errorVector;
                    end
                case 'CACC PID'
                    error(['ACC PID controller not finished for coded' ...
                        'vehicle models (Simulink use only)'])
                otherwise
                    error('Unknown controller type')
            end
            
            obj.windupFactor = u;
            u = obj.accelSaturationFilter.runFilter(u, deltaT);
%             % 'Cheating' on the nonlinearities
%             if u<0 && veh.velocity<=0
%                 u = 0;
%             end
            obj.windupFactor = obj.windupFactor - u; %  u - sat(u)
        end
        
        function[error] = computeVehFollowingError(obj)
            %Computes gap, gap derivative, velocity and acceleration errors
            veh = obj.vehicle;
            leader = veh.leader;
            deltaT = veh.simTime(veh.iterCounter + 1) ...
                - veh.simTime(veh.iterCounter);
            
            q = veh.currentState;
            qL = leader.currentState;
            
            xL = qL(leader.statesIdx.x);
            gap = xL - q(veh.statesIdx.x) - leader.len;
            gapError = gap - veh.minVehFollGap;
            gapError = max(min(gapError, obj.maxPosError), obj.minPosError);
            
            vL = qL(leader.statesIdx.vx);
            filteredVL = obj.accLeaderVelFilter.runFilter(vL, deltaT);
            velocityError = filteredVL - q(veh.statesIdx.vx);
            
            if obj.vehicle.nStates == 3
                gapErrorDerivative = velocityError ...
                    - veh.h*q(veh.statesIdx.ax);
                aL = qL(leader.statesIdx.ax);
                accelerationError = aL - q(veh.statesIdx.ax);
                error = [gapError; gapErrorDerivative; ...
                    velocityError; accelerationError];
            else
                error = [gapError; velocityError];
            end
        end
        
        function[error] = computeVelocityError(obj)
            %Computes gap, gap derivative, velocity and acceleration errors
            veh = obj.vehicle;
            deltaT = veh.simTime(veh.iterCounter + 1) ...
                - veh.simTime(veh.iterCounter);
            q = veh.currentState;
            
            velocityError = veh.desiredVelocity - q(veh.statesIdx.vx);
            obj.intVelError = obj.intVelError + velocityError*deltaT;
            
            if obj.vehicle.nStates == 3
                accelerationError = - q(veh.statesIdx.ax);
                error = [obj.intVelError; velocityError; accelerationError];
            else
                error = [obj.intVelError; velocityError];
            end
        end        
        
        function [] = createRefVelFilter(obj)
            %createRefVelFilter Creates the filter used to avoid high
            %oscilation when tracking leader speed
            veh = obj.vehicle;
            coeff = 1;
            obj.accLeaderVelFilter = SimpleFilter('variationFilter', ...
                [veh.velocity, veh.accelBounds(1), veh.accelBounds(2), coeff]);
        end
        
        function [] = createAccelSaturationFilter(obj)
            %createAccelSaturationFilter Creates the filter used to limit
            %maximum acceleration and maximum acceleration variation
            veh = obj.vehicle;
            obj.accelSaturationFilter = SimpleFilter('valueAndRateSaturation', ...
                [0, veh.accelBounds(1), veh.accelBounds(2), ...
                veh.jerkBounds(1), veh.jerkBounds(2)]);
        end

        %%% OPEN-LOOP GAP GENERATION CONTROL INPUTS %%%
        function [minTime] = minTimeWithComfortConstraints(obj, desiredGap, gamma, vf, t)
            %minTimeWithComfortConstraints Vehicle brakes to a minimum
            %speed and stays there until gap is generated. Acceleration and
            %jerk constraints are considered
            
            veh = obj.vehicle;
            jc = abs(min(veh.comfJerkBounds));
            ac = max(veh.comfAccelBounds);
            bc = abs(min(veh.comfAccelBounds));
%             x0 = veh.position;
            v0 = veh.velocity;
            vL = veh.leader.velocity;
            
            vMin = veh.desiredVelocity*gamma;
            if vMin>vf
                error('Gap generating min speed greater than final desired speed.')
            end
            
            
            currentGap = veh.leader.position - veh.position - veh.len;
            deltaGap = desiredGap - currentGap;
            
            jerk = [-jc, 0, jc, 0, jc, 0, -jc]; 
            nIntervals = length(jerk)+1;
            
            delta = zeros(1, nIntervals);
            delta(1) = bc/jc;
            delta(2) = (veh.velocity-vMin)/bc - delta(1);
            delta(3) = delta(1);
            % Solution obtained through Wolfram Mathematica
%             delta(4) = (bc*jc*(vf + vMin - 2*vL)*(vf - vMin) + ac^2*bc*(-2*vL + vMin + vf) + ...
%                 ac*(bc^2*(v0 - 2*vL + vMin) + jc*(v0 - vMin)*(v0 - 2*vL + vMin) + ...
%                 2*bc*jc*(deltaGap - initGap)))/(2*ac*bc*jc*(vL - vMin));
            delta(4) = (bc*jc*(2*vL - vMin - vf)*(vMin - vf) + ac^2*bc*(-2*vL + vMin + vf) + ...
                ac*(bc^2*(v0 - 2*vL + vMin) + jc*(v0 - vMin)*(v0 - 2*vL + vMin) + ...
                2*bc*jc*(deltaGap)))/(2*ac*bc*jc*(vL - vMin));
            delta(5) = ac/jc;
            delta(6) = (vf-vMin)/ac - delta(5);
            delta(7) = delta(5);
            
            transitionTimes = cumsum([0, delta]);
            
            accel = zeros(1, nIntervals);
            u = zeros(length(t), 1);
            for n = 1:nIntervals-1
                interval = t>=transitionTimes(n) & t<transitionTimes(n+1);
                u(interval) = accel(n)+jerk(n)*(t(interval)-transitionTimes(n));
                accel(n+1) = accel(n) + jerk(n)*delta(n);
            end
            
            obj.openLoopControl = u;
            
            minIdx = length(t) - find(flip(u), 1) + 2;
            minTime = t(minIdx);            
                  
        end
        
        function [minTime] = minimum_time_control(obj, gmin, t)
            %minimum_time Minimum time control given final state
            
            % Input bounds
            u_bounds = obj.vehicle.accelBounds;
            
            % Initial error
            x0 = obj.vehicle.position;
            v0 = obj.vehicle.velocity;
            xr0 = obj.vehicle.leader.position;
            vr0 = obj.vehicle.leader.velocity;
            ll = obj.vehicle.leader.len;
            e0 = [x0-(xr0-ll-gmin); v0-vr0]; % initial error
            ef = zeros(length(e0), 1); % desired final error
            
            % Minimum time and initial input
            [u0, t1, tf, ~] = obj.minimum_time(e0, ef, u_bounds);
            uf = u_bounds(u_bounds~=u0);
            
            % Feedfoward Control and system response
            tc_idx = find(t>t1, 1);
            tf_idx = find(t>tf, 1);
            u = [u0*ones(tc_idx, 1); uf*ones(tf_idx-tc_idx,1); zeros(length(t)-tf_idx,1)];
            
            obj.openLoopControl = u;
            minTime = tf;
            
        end
        
        function [minTime] = min_time_and_power(obj, gmin, r, t)
            %minimum_time_and_u2 Optimizes for time and power (related to u^2)
            %(only for 2 states for now)
            
            % Input bounds
            u_bounds = obj.vehicle.accelBounds;
            
            % Initial error
            x0 = obj.vehicle.position;
            v0 = obj.vehicle.velocity;
            xr0 = obj.vehicle.leader.position;
            vr0 = obj.vehicle.leader.velocity;
            ll = obj.vehicle.leader.len;
            e0 = [x0-(xr0-ll-gmin); v0-vr0]; % initial error
            ef = zeros(length(e0), 1); % desired final error
            
            % Obtain minimum time
            [u0, ~, tf, min_time_coeffs] = obj.minimum_time(e0, ef, u_bounds);
            uf = u_bounds(u_bounds~=u0);
            
            % Initial guess
            hyp = 1; % first we test the hypothesis that the constraints are not violated
            c1 = min_time_coeffs(1);
            c2 = min_time_coeffs(2);
            constraints_satisfied = 0;
            counter = 0;
            while ~constraints_satisfied && counter<4
                fprintf('Testing hypothesis %d \n', hyp);
                fun = @(x)obj.fun_1_plus_ru2([u0, uf, r, e0', ef'], x, hyp);
                x0 = [c1, c2, tf];
                [sol, ~, exitflag, ~] = fsolve(fun, x0);
                if (exitflag>0)
                    c1 = sol(1);
                    c2 = sol(2);
                    tf = sol(3);
                    t0_constr = -sign(u0)*c2 < abs(u0)*2*r; % underlying assumption: umin < 0; umax > 0
                    tf_constr = -sign(u0)*(c1*tf-c2) < abs(uf)*2*r;
                    if (t0_constr || (hyp~=1 && hyp~=3)) && (tf_constr || (hyp~=1 && hyp~=2))
                        constraints_satisfied = 1;
                    elseif tf_constr
                        hyp = 2;
                    elseif t0_constr
                        hyp = 3;
                    else
                        hyp = 4;
                    end
                else
                    hyp = rem(hyp, 4)+1; % not the smartest solution - should come up with something that
                    % guarantees trying a not yet tested hypothesis.
                end
                counter = counter + 1;
            end
            
            if ~constraints_satisfied
                error('[min_time_and_power] Solution not found.')
            end
            
            tf_idx = find(t>tf, 1);
            switch hyp
                case 1
                    t1_idx = 0;
                    t2_idx = tf_idx;
                case 2
                    t1 = (c2 + 2*r*u0)/c1;
                    t1_idx = find(t>t1, 1);
                    t2_idx = tf_idx;
                case 3
                    t1_idx = 0;
                    t2 = (c2+2*r*uf)/c1;
                    t2_idx = find(t>t2, 1);
                case 4
                    t1 = (c2 + 2*r*u0)/c1;
                    t2 = (c2+2*r*uf)/c1;
                    t1_idx = find(t>t1, 1);
                    t2_idx = find(t>t2, 1);
                otherwise
                    fprintf('No solution found\n');
            end
            
            u = [u0*ones(t1_idx, 1);
                (c1*t(t1_idx+1:t2_idx)'-c2)/(2*r);
                uf*ones(tf_idx-t2_idx,1);
                zeros(length(t)-tf_idx,1)];
            
            obj.openLoopControl = u;
            minTime = tf;
        end
        
        function [minTime] = min_time_and_fuel(obj, gmin, r, t)
            %minimum_time_and_u1 Optimizes for time and fuel (related to |u|)
            %(only for 2 states for now)
            
            % Input bounds
            u_bounds = obj.vehicle.accelBounds;
            
            % Initial error
            x0 = obj.vehicle.position;
            v0 = obj.vehicle.velocity;
            xr0 = obj.vehicle.leader.position;
            vr0 = obj.vehicle.leader.velocity;
            ll = obj.vehicle.leader.len;
            e0 = [x0-(xr0-ll-gmin); v0-vr0]; % initial error
            ef = zeros(length(e0), 1); % desired final error
            
            % Obtain minimum time
            [u0, ~, tf, min_time_coeffs] = obj.minimum_time(e0, ef, u_bounds);
            uf = u_bounds(u_bounds~=u0);
            
            fun = @(x)obj.fun_1_plus_ru1([u0, uf, r, e0', ef'], x);
            x0 = [min_time_coeffs(1), min_time_coeffs(2), tf]; % min time solution is the initial guess
            [sol, ~, exitflag, ~] = fsolve(fun, x0);
            if (exitflag<=0)
                fprintf('[Min time and fuel] No solution found.')
%                 u = 0;
                return;
            end
            
            c1 = sol(1);
            c2 = sol(2);
            tf = sol(3);
            
            t1 = (sign(u0)*r + c2)/c1;
            t2 = (-sign(u0)*r + c2)/c1;
            tf_idx = find(t>tf, 1);
            t1_idx = find(t>t1, 1);
            t2_idx = find(t>t2, 1);
            
            u = [u0*ones(t1_idx, 1);
                zeros(t2_idx-t1_idx,1);
                uf*ones(tf_idx-t2_idx,1);
                zeros(length(t)-tf_idx,1)];
            
            obj.openLoopControl = u;
            minTime = tf;
            
        end
        
        function [u] = minimum_time_throttle_control(obj, gmin, t)
            % TODO 1: better treatment of cases with ant without switches;
            % not sure the current discontinuous form of system of
            % equations in min_time_throttle is a good idea
            % TODO 2: find a way to initialize the solution - currently x0
            % is set to work with a specific case.
            
            % 4 cases:
            % 1. u0 = u_min and no cross
            % 2. u0 = u_min and cross
            % 3. u0 = u_max and no cross
            % 4. u0 = u_max and cross
            
            % Input bounds
            u_bounds = obj.vehicle.accelBounds;
            
            % Initial error
            xf0 = obj.vehicle.x0;
            xr0 = obj.vehicle.leader.x0;
            ll = obj.vehicle.leader.veh_length;
            e0 = [xf0(1)-(xr0(1)-ll-gmin); xf0(2:end)-xr0(2:end)]; % initial error
            ef = zeros(length(e0), 1); % desired final error
            
            u0 = min(u_bounds);
            uf = max(u_bounds);
            
            fun = @(x)obj.min_time_throttle([u0, uf, e0', ef'], x);
            alpha = obj.vehicle.alpha;
            x0 = [1; 1.1*1*alpha; 5]; % TODO
            %[min_time_coeffs(1), min_time_coeffs(2), tf]; % min time solution is the initial guess
            [sol, ~, exitflag, ~] = fsolve(fun, x0);
            if (exitflag<=0)
                fprintf('[Min time and fuel] No solution found.')
                u = 0;
                return;
            end
            c1 = sol(1);
            c2 = sol(2);
            tf = sol(3);
            
            % Repeated code from the min_time_throttle function
            if c2>0
                u0 = u_bounds(1);
                if c2<c1/alpha
                    t1 = log(c1/(c1-alpha*c2))/alpha;
                    uf = u_bounds(2);
                else
                    t1 = 0;
                    uf = u0;
                end
            else
                u0 = u_bounds(2);
                if c2>c1/alpha
                    t1 = log(c1/(c1-alpha*c2))/alpha;
                    uf = u_bounds(1);
                else
                    t1 = 0;
                    uf = u0;
                end
            end
            
            tf_idx = find(t>tf, 1);
            t1_idx = find(t>t1, 1);
            
            u = [u0*ones(t1_idx, 1);
                uf*ones(tf_idx-t1_idx,1);
                zeros(length(t)-tf_idx,1)];
        end
        
    end
    
    methods (Access = private)
        function [u0, t1, tf, coeffs] = minimum_time(~, x0, xf, u_bounds)
            %minimum_time Computes minimum necessary time along with initial control
            %input and costate coefficients for a bang-bang (minimum time) controller
            
            % Obtain minimum time
            u0 = u_bounds';
            uf = u_bounds(end:-1:1)';
            a = u0.*uf;
            b = 2*(x0(2)*uf-xf(2)*u0);
            c = (xf(2)-x0(2))^2 - 2*(uf-u0)*(xf(1)-x0(1));
            delta = b.^2-4*a.*c;
            idx = delta>0;
            if sum(idx)~=1
                warning('[Minimum time] More than one candidate polynomial')
            end
            
            tf = zeros(2, sum(idx));
            for n = 1:sum(idx)
                pol = [a(n), b(n), c(n)];
                tf(:, n) = roots(pol);
            end
            idx2 = tf>0;
            tf = tf(idx2);
            
            if length(tf)~=1
                warning('[Minimum time] %d candidate times; for now let''s assume u0 = u_min', length(tf));
                k = 2; %1; % TODO: still testing which value should be forced
                tf = tf(k);
                u0 = u0(k);
                uf = uf(k);
            else
                [~, col] = find(idx2);
                u0 = u0(col);
                uf = uf(col);
            end
            % Testing new approach
            M = [x0(2) u0;
                xf(2)-tf*uf, uf];
            y = [-1; -1];
            coeffs = M\y;
            t1 = coeffs(2)/coeffs(1);
            
        end
        
        function [F] = fun_1_plus_ru2(~, param, var, hyp)
            %fun_1_plus_ru2 Non-linear system of equations describing times and costate
            %constants in the L = 1 + r*u^2 case. Returns costate constants and final
            %time.
            % Input hyp describes hypothesis: 1 - control input is always within
            % bounds, 2 - control input begins at boundary, 3 - control input ends at
            % boundary, 4 - control input begins and ends at boundaries
            u0 = param(1);
            uf = param(2);
            r = param(3);
            x01 = param(4);
            x02 = param(5);
            xf1 = param(6);
            xf2 = param(7);
            
            c1 = var(1);
            c2 = var(2);
            tf = var(3);
            
            switch hyp
                case 1 % umin < u(t) < umax
                    t1 = 0;
                    t2 = tf;
                    %                     F(1) = max((c2 + [1; -1]*2*sqrt(r*(1+c1*xf2)))/c1 - tf); %TODO: check correctness
                case 2 % u(0) = {umin, umax}
                    t1 = (c2 + 2*r*u0)/c1;
                    t2 = tf;
                    %                     F(1) = max((c2 + [1; -1]*2*sqrt(r*(1+c1*xf2)))/c1 - tf); %TODO: check correctness
                case 3 % u(tf) = {umin, umax}
                    t1 = 0;
                    t2 = (c2+2*r*uf)/c1;
                    %                     F(1) = (c2 + (c1*xf2+1+r*uf^2)/uf)/c1 - tf;
                case 4 % u(0) = {umin, umax} and u(tf) = {umin, umax}
                    t1 = (c2 + 2*r*u0)/c1;
                    t2 = (c2+2*r*uf)/c1;
                    %                     F(1) = (c2 + (c1*xf2+1+r*uf^2)/uf)/c1 - tf;
            end
            
            F(1) = (c2 + (c1*xf2+1+r*uf^2)/uf)/c1 - tf; % tf
            F(2) = x02-xf2 + u0*t1 + (c1*(t2^2-t1^2)/2 - c2*(t2-t1))/(2*r) + uf*(tf-t2); % x2
            first_interval = (tf*t1-t1^2/2)*u0;
            second_interval = - (c1*(t2^3-t1^3)/3 - (t2^2-t1^2)*(c1*tf+c2)/2 + tf*c2*(t2-t1))/(2*r);
            third_interval = (tf - t2)^2*uf/2;
            F(3) = x01-xf1 + x02*tf +  first_interval + second_interval + third_interval; % x1
        end
        
        function [F] = fun_1_plus_ru1(~, param, var)
            %fun_1_plus_ru2 Non-linear system of equations describing times and costate
            %constants in the L = 1 + r*|u| case. Returns costate constants and total
            %time.
            
            u0 = param(1);
            uf = param(2);
            r = param(3);
            x01 = param(4);
            x02 = param(5);
            xf1 = param(6);
            xf2 = param(7);
            
            c1 = var(1);
            c2 = var(2);
            tf = var(3);
            
            t1 = (sign(u0)*r + c2)/c1;
            t2 = (-sign(u0)*r + c2)/c1;
            
            F(1) = (c2 + (1+r*abs(uf)+c1*xf2)/uf)/c1 - tf;
            F(2) = x02-xf2 + u0*t1 + uf*(tf-t2);
            first_interval = (tf*t1-t1^2/2)*u0;
            second_interval = 0;
            third_interval = (tf - t2)^2*uf/2;
            F(3) = x01-xf1 + x02*tf +  first_interval + second_interval + third_interval;
        end
        
        function [F] = min_time_throttle(obj, param, var)
            %min_time_throttle Non-linear system of equations to find the
            %minimum time solution for the system with throttle as input
            
            alpha = obj.vehicle.alpha;
            beta = obj.vehicle.beta;
            u_bounds = param(1:2);
            %uf = param(2);
            x01 = param(3);
            x02 = param(4);
            xf1 = param(5);
            xf2 = param(6);
            
            c1 = var(1);
            c2 = var(2);
            tf = var(3);
            
            if c2>0
                u0 = u_bounds(1);
                if c2<c1/alpha
                    t1 = log(c1/(c1-alpha*c2))/alpha;
                    uf = u_bounds(2);
                else
                    t1 = 0;
                    uf = u0;
                end
            else
                u0 = u_bounds(2);
                if c2>c1/alpha
                    t1 = log(c1/(c1-alpha*c2))/alpha;
                    uf = u_bounds(1);
                else
                    t1 = 0;
                    uf = u0;
                end
            end
            
            % Variables to improve readability
            r1 = beta/alpha;
            exp1 = exp(-alpha*tf);
            exp2 = exp(-alpha*(tf-t1));
            % Equation for tf
            F(1) = log((1+r1*uf)/((alpha*c2-c1)*(x02 - r1*uf)))/alpha - tf;
            % Equation for x2
            x2_zero_t1 = r1*u0*(exp2 - exp1);
            x2_t1_tf = r1*uf*(1 - exp2);
            F(2) = x02*exp1 + x2_zero_t1 + x2_t1_tf - xf2;
            % Equation for x1
            x1_zero_t1 = r1*u0*(t1 - (exp2 - exp1)/alpha);
            x1_t1_tf = r1*uf*(tf - t1 - (1 - exp2)/alpha);
            F(3) = x01 + (1-exp1)*x02/alpha + x1_zero_t1 + x1_t1_tf - xf1;
        end
        
    end
end