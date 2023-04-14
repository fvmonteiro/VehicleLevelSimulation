classdef CoopPlatoonLeader < LongitudinalVehicleModel
    %CoopPlatoonLeader: class for a platoon leader which brakes taking into
    %consideration its predicted effect on its followers
   properties
       nf % number of followers
       followers = LongitudinalVehicleModel()
   end
     
   methods
       function obj = CoopPlatoonLeader(simTime, followers)
           
           obj.name = 'Coop leader';
           obj.simTime = simTime;
           
           obj.nf = length(followers);
           obj.followers = followers;
           leaderStateIdx = [1, (obj.nf+2)];
           obj.xIdx = leaderStateIdx(1);
           obj.vxIdx = leaderStateIdx(2);
           
           obj.safeHeadwayParams();
           % For now, stantard values. Consider computing and changing
%            obj.h = 1;
%            obj.d0 = 2;
                
%            % The cooperative platoon leader knows or estimates the state of
%            % the whole platoon
%            obj.followers = followers;
%            if ~isempty(obj.followers)
%                 gaps = -diff([q0(1), obj.followers.position]) - [obj.followers.len];
%                 obj.initialState = [q0(1); gaps'; q0(2); [obj.followers.velocity]']; % initial input is zero
%            else
%                 obj.initialState = q0;
%            end
%            
%            obj.states = zeros(length(simTime), obj.nStates);
%            obj.states(1, :) = obj.initialState;
%            obj.inputs = zeros(length(simTime), 1);
%            
%            leaderStateIdx = 1:(obj.nf+1):obj.nStates;
%            obj.xIdx = leaderStateIdx(1);
%            obj.vxIdx = leaderStateIdx(2);
%            
%            % Proper model matrices
%            nv = obj.nf+1;
%            A11 = zeros(nv);
%            A12 = diag([1; -ones(obj.nf,1)]) + diag(ones(obj.nf,1),-1);
%            A21 = diag([0; Ks*ones(obj.nf,1)]);
%            if ~isempty(obj.followers) % in case the vehicle is alone
%                A22 = diag([0, -(Ks*[obj.followers.h]+Kv)]) + diag(Kv*ones(obj.nf,1),-1);
%            else
%                A22 = 0;
%            end
%            obj.A = [A11 A12; A21 A22];
%            obj.B = zeros(2*nv, 1);
%            obj.B(nv+1) = 1;
%            obj.C = eye(obj.nStates);
%            obj.D = zeros(obj.nInputs);
       end
       
       function [] = includeInSimulation(obj, q0, Ks, Kv)
           % The cooperative platoon leader knows or estimates the state of
           % the whole platoon
           
           if ~isempty(obj.followers)
                gaps = -diff([q0(1), obj.followers.position]) - [obj.followers.len];
                obj.initialState = [q0(1); gaps'; q0(2); [obj.followers.velocity]']; % initial input is zero
           else
                obj.initialState = q0;
           end
           
           obj.states = zeros(length(obj.simTime), obj.nStates);
           obj.states(1, :) = obj.initialState;
           obj.inputs = zeros(length(obj.simTime), 1);
           
           % Proper model matrices
           nv = obj.nf+1;
           A11 = zeros(nv);
           A12 = diag([1; -ones(obj.nf,1)]) + diag(ones(obj.nf,1),-1);
           A21 = diag([0; Ks*ones(obj.nf,1)]);
           if ~isempty(obj.followers) % in case the vehicle is alone
               A22 = diag([0, -(Ks*[obj.followers.h]+Kv)]) + diag(Kv*ones(obj.nf,1),-1);
           else
               A22 = 0;
           end
           obj.A = [A11 A12; A21 A22];
           obj.B = zeros(2*nv, 1);
           obj.B(nv+1) = 1;
           obj.C = eye(obj.nStates);
           obj.D = zeros(obj.nInputs);
       end
       
       function longVehObj = LongitudinalVehicleModel(obj)
           longVehObj = LongitudinalVehicleModel(obj.name, obj.simTime, obj.initialState([obj.xIdx, obj.vxIdx]));
           longVehObj.states = obj.states(:, [obj.xIdx, obj.vxIdx]);
           longVehObj.inputs = obj.inputs;
       end
              
%        function [accel] = stateFeedbackAccel(obj, desiredGap)
%             
% %             leaderPosition = obj.leader.position;
% %             desiredPosition = leaderPosition - desiredGap - obj.len;
% %             desiredSpeed = obj.leader.velocity;
% %             
% %             if ~isempty(obj.followers)
% %                 desiredGaps = [obj.followers.minACCGap] + [obj.followers.len];
% %                 ref = [desiredPosition; desiredGaps'; desiredSpeed*ones(obj.nf+1,1)];
% %             else
% %                 ref = [desiredPosition; desiredSpeed];
% %             end
%             
% %             accel = stateFeedbackAccel@LongitudinalVehicleModel(obj, [], ref);
% 
%        end
        
              
   end
   
   
   
end