classdef TestScenarios
    %TestScenarios Class to facilitate scenario creation and testing
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        % For parameter definitions check paramter_defs.m
        vd
        td
        s0
        tlc
        tinter
        p
        Tp
        betas
        veh_l
        d0
        vmax
        amax
        amin
        sampling_interval = 0.001;
    end
    
    methods
        function obj = TestScenarios()
            %TestScenarios Construct an instance of this class
            params = load('sim_parameters');
            obj.vd = params.vd;
            obj.td = params.td;
            obj.s0 = params.s0;
            obj.tlc = params.tlc;
            obj.tinter = params.tinter;
            obj.p = params.p;
            obj.Tp = params.Tp;
            obj.betas = params.betas;
            obj.veh_l = params.l;
            obj.d0 = params.d0;
            obj.vmax = params.v_max;
            obj.amax = params.a_max;
            obj.amin = params.a_min;
            
        end
        
        function [total_cost, total_individual_costs] = solve_scenario1(obj)
            % Simulation time
            t = 0:obj.sampling_interval:(obj.Tp-obj.sampling_interval);
            
            % Surrounding vehicle
            v0 = 25;
            gap0 = 50;
            xl0 = [gap0 + obj.veh_l, v0];
            veh = Vehicle(1, xl0, obj.veh_l);
            xl = veh.veh_dynamics(zeros(1, length(t)), t); % the leading vehicle
            % has constant speed during all simulation time
            
            % Creating lanes
            n_lanes = 2;
            vlim = 40;
            lanes = Lane.empty(n_lanes, 0);
            for nl = 1:n_lanes
                lanes(nl, 1) = Lane(nl, vlim);
            end
            lanes(2).add_veh(veh);
            
            % Scenario
            scenario = Scenario(lanes, veh, obj.Tp);
            
            % Autonomous vehicle
            xf0 = [0 25];
            auto_veh = AutonomousVehicle(2, xf0, obj.veh_l, obj.betas, obj.vd,...
                obj.amax, obj.amin, obj.tlc, obj.tinter);
            auto_veh.scenario = scenario;
            dend = 2000; % TODO: vary with lane and distance travelled
%             [xi, psi] = auto_veh.enumerate_paths();
            paths = auto_veh.generate_paths(0, scenario.lane0, []);
            tested_path = paths(1, :);
            
            acc = auto_veh.opt_acceleration(t, obj.td, obj.s0, obj.d0, dend, tested_path);
            
            xf = auto_veh.veh_dynamics(acc, t);
            X = zeros(size(xf,1), size(xf,2), 2);
            X(:,:,1) = xf;
            X(:,:,2) = xl;
            plot_kinematics(t, X, 'ACC', {'Follower', 'Leader'});
            
            % Compute costs           
            [L, individual_costs] = auto_veh.running_cost(obj.sampling_interval, ...
                obj.td, obj.s0, obj.d0, dend, acc, tested_path);
            total_individual_costs = trapz(t, individual_costs');
            total_cost = trapz(t, L);
        end
        
    end
end

