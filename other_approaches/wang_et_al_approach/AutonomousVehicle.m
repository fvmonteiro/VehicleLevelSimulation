classdef AutonomousVehicle < Vehicle
    %AutonomousVehicle Class contains vehicle parameters and solves control
    %problem
    %   Detailed explanation goes here
    
    properties
        scenario
    end
    
    properties (SetAccess = private)
        % Inherits id, x0 and len properties from Vehicle Class
        b_safe
        b_eq
        b_cntr
        b_eff
        b_route
        b_pref
        b_switch
        vd
        amax
        amin
        tlc
        tinter
    end
    
    methods
        % Inherits get.id, get.x0 and veh_dynamics methods
        function [obj] = AutonomousVehicle(id, x0, length, betas, vd, amax,...
                amin, tlc, tinter)
            obj@Vehicle(id, x0, length);
            % Cost weights
            obj.b_safe = betas(1);
            obj.b_eq = betas(2);
            obj.b_cntr = betas(3);
            obj.b_eff = betas(4);
            obj.b_route = betas(5);
            obj.b_pref = betas(6);
            obj.b_switch = betas(7);
            % Desired speed
            obj.vd = vd;
            % Acceleration bounds
            obj.amax = amax;
            obj.amin = amin;
            % Lane change time and inter lane change time (waiting time)
            obj.tlc = tlc;
            obj.tinter = tinter;
        end
        
        function [u] = opt_acceleration(obj, t, td, s0, d0, dend, path)
            %opt_acceleration Solving the opt controller for the 2015 paper
            
            % System:
            % x = [s; v],
            % where s and v are the position and speed of the vehicles
            % u = acc
            % dx/dt = Ax + Bu; A = [0 1;0 0], B = [0;1].
            
            % Cost
            % L = too long - check paper
            % H = L + lambda*(Ax+Bu)
            % Optimal control
            % u = -lambda(2)/(2.beta_ctr) - note: lambda of the respective v_i
            
            % Parameters (not given in 2012 paper)
            tolerance = 0.01; % total guess
            alpha = 0.01; % 0<alpha<1 - has to change according to problem
            iter_limit = 10000;
            
            % Other parameters
            dr = 1000; % 1km
            
            % Leader kinematics (depends on the vehicle's lane)
            sampling_interval = t(2)-t(1);
            vl = repelem(obj.scenario.lane_by_id(path).speed(), 1/sampling_interval)';
            % TODO: find better way of coding following loop
            sl = zeros(1, length(t));
            for tk = 1:length(path)
                interval_ind = (tk-1)/sampling_interval+1:tk/sampling_interval;
                current_lane = obj.scenario.lane_by_id(path(tk));
                if isempty(current_lane.veh)
                    sl(interval_ind) = 100000; % anything big enough to not make a difference
                else
                    veh_pos = current_lane.veh.xt(1, :);
                    sl(interval_ind) = veh_pos(interval_ind);
                end
            end
            % Just testing
            %             vl = obj.scenario.lane_by_id(path(1)).speed();
            %             sl = obj.scenario.lane_by_id(path(1)).veh.init_position() + ...
            %                 t*obj.scenario.lane_by_id(path(1)).speed();
            
            lambdaT = [0;0];
            
            Lambda = [zeros(1, length(t)); zeros(1, length(t))];
            lambda = [zeros(1, length(t)); zeros(1, length(t))];
            k=1;
            error = [tolerance+1 zeros(1, iter_limit-1)];
            % Stopping criteria not well specified in paper...
            while (error(k) > tolerance) && (k<iter_limit)
                % Optimal controller
                u = -Lambda(2, :)/(2*obj.b_cntr);
                
                % Test 1: analytical plus numerical integration
                % Solve the state dynamic equation forward
                x = obj.veh_dynamics(u, t);
                
                % Solve the costate dynamic equation backward
                % first we do it "dum" and right; then improve algebra
                delta_v = vl - x(2,:);
                gap = sl - x(1,:)-obj.len;
                
                safety = obj.b_safe./(gap.^2) .* delta_v.^2 .* obj.theta(-delta_v);
                v_eq = obj.compute_v_eq(gap, obj.vd, td, s0);
                eq = 2*obj.b_eq*(v_eq - x(2,:))/td;
                route = obj.b_route*d0/(dend.^2).*exp(d0/dend).*(dend<dr);
                dHds = route + safety - eq;
                lambda(1, :) = lambdaT(1) + trapz(t, dHds) - cumtrapz(t,dHds);
                
                dHdv = -2*obj.b_safe./gap.*delta_v.*obj.theta(-delta_v) + 2*obj.b_eq*(x(2, :)-v_eq) + lambda(1,:);
                lambda(2, :) = lambdaT(2) + trapz(t, dHdv) - cumtrapz(t, dHdv);
                
                % Test 2: TODO using ode45
                
                % Update Lambda
                Lambda = (1-alpha)*Lambda + alpha*lambda;
                
                % Update iteration index
                k = k + 1;
                
                % New error
                % error(k) = sum((Lambda(:) - lambda(:)).^2);
                error(k) = max((Lambda(:) - lambda(:)).^2);
            end
            
        end
        
        function [all_paths, new_path] = generate_paths(obj, t, new_path, all_paths)
            %generate_paths Function to generate all possible lane change decisions
            % There's probably a more efficient (or at least more elegant) way to write
            % this function, but this will work for now.
            
            T = obj.scenario.Tp;
            max_lane = length(obj.scenario.lanes);
            
            if t>T-obj.tlc
                new_path = pad_path_array(new_path); % make the vector more "readable" for future functions
                all_paths = [all_paths; new_path];
                return
            end
            
            path = new_path;
            for psi = [1, 0, -1]
                if t==0
                    updated_path = path(1,end)+psi;
                else
                    updated_path = [path path(1,end)+psi];
                end
                if (path(1,end)+psi>0) && (path(1,end)+psi<=max_lane)
                    if psi == 0
                        new_t = t + 1;
                    else
                        new_t = t + obj.tlc + obj.tinter;
                    end
                    [all_paths, new_path] = obj.generate_paths(new_t, updated_path, all_paths);
                end
                
            end
            
            function proper_array = pad_path_array(path_array)
                max_t = length(path_array);
                proper_array = [path_array zeros(1, T-max_t)];
                proper_array(1, max_t:end) = proper_array(1,max_t);
            end
            
        end
        
        function [L, individual_costs] = running_cost(obj, sampling_interval, ...
                td, s0, d0, dend, acc, path)
            %running_cost Running cost proposed by Wang et al
            
            % Other parameters
            dr = 1000; % 1km
            total_lanes = length(obj.scenario.lanes);
                        
            my_lane = repelem(path, 1/sampling_interval);
            % line below only works if other vehicles have const speed
            v_lane = obj.scenario.lane_by_id(my_lane).speed()';
            delta_v = v_lane - obj.xt(2, :);
            % TODO: recode this gap part
%             gap_lane = zeros(length(obj.scenario.lanes), length(delta_v));
            gap = zeros(1, length(delta_v));
            for l = 1:length(obj.scenario.lanes)
                veh_in_lane = obj.scenario.lane_by_id(l).veh;
                if isempty(veh_in_lane)
                    gap(my_lane == l) = obj.vd*td + s0 + 1; % enough to make v_eq = vd
                else
                    other_veh_pos = veh_in_lane.xt(1, :) - obj.xt(1, :) - veh_in_lane.len;
                    gap(my_lane == l) = other_veh_pos(my_lane==l);
                end
            end
            
            % Longitutdinal costs
            safety_cost = obj.b_safe./gap .* delta_v.^2 .* obj.theta(-delta_v);
            v_eq = obj.compute_v_eq(gap, obj.vd, td, s0);
            eq_cost = obj.b_eq*(v_eq - obj.xt(2,:)).^2;
            cntr_cost = obj.b_cntr*acc.^2;
            % Lane changing costs - these are constant during each time interval
            ones_vector = ones(1, length(safety_cost));
            eff_cost = obj.b_eff*(obj.vd - v_lane).^2 .* (obj.vd>v_lane);
            route_cost = obj.b_route*exp(d0/dend)*(dend<dr) * ones_vector; % TO CHANGE
            pref_cost = obj.b_pref*repelem(h(path), 1/sampling_interval);
            lane_switch = diff([obj.scenario.lane0 path])~=0;
            switch_ind = find(lane_switch);
            switch_cost = zeros(1, length(path));
            for i = switch_ind
                switch_cost(i:end) = switch_cost(i:end)+1;
            end
            switch_cost = obj.b_switch*repelem(switch_cost, 1/sampling_interval);
%             switch_cost = obj.b_switch*repelem(lane_switch, 1/sampling_interval);
            % Note on switch_cost: paper definition is not clear enough. It
            % looks like this cost should not be part of the running cost,
            % but actually a fixed value for every lane change. The way it
            % is coded here only works because of the 1s lane change freq.
            
            individual_costs = [safety_cost; eq_cost; eff_cost; route_cost; pref_cost; switch_cost; cntr_cost];
            L = safety_cost + eq_cost + cntr_cost + eff_cost + route_cost + pref_cost + switch_cost;
            
            figure; plot(L);
                        
            % Function h expresses the keep right directive (lane number increases
            % from left to right)
            function [lane_cost] = h(lane_number)
                lane_cost = total_lanes - lane_number;
            end
            
        end
        
        
        
    end
    
    methods (Access = private)
        function [v_eq] = compute_v_eq(~, gap, vd, td, s0)
            %v_eq Computes local equilibrium speed
            sf = vd*td + s0;
            idx = gap>sf;
            v_eq = vd*(idx) + (gap-s0)/td.*(~idx);
        end
        
        function [b] = theta(~, arg)
            %theta Checks for non-negativity (created only to match paper
            %notation)
            b = arg>=0;
        end
    end
    
end

