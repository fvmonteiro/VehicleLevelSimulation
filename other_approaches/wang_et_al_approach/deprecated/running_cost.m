function [L, individual_costs] = running_cost(betas, td, s0, d0, dend, xf, xl, vd, l, v_lanei, ...
    acc, total_lanes, lane_number, lane_switch)
%running_cost Running cost proposed by Wang et al
%   Detailed explanation goes here

% TO DO list:
% should work...

% Cost weights
b_safe = betas(1);
b_eq = betas(2);
b_cntr = betas(3);
b_eff = betas(4);
b_route = betas(5);
b_pref = betas(6);
b_switch = betas(7);

% Other parameters
dr = 1000; % 1km

delta_v = xl(2, :) - xf(2, :);
gap = xl(1, :) - xf(1, :) - l;

% Longitutdinal costs
safety_cost = b_safe./gap .* delta_v.^2 .* theta(-delta_v);
eq_cost = b_eq*(v_eq(gap) - xf(2, :)).^2;
cntr_cost = b_cntr*acc.^2;
% Lane changing costs - these are constant during each time interval
ones_vector = ones(1, length(safety_cost));
eff_cost = b_eff*(vd - v_lanei)^2 * ones_vector;
route_cost = b_route*exp(d0/dend)*(dend<dr) * ones_vector;
pref_cost = b_pref*h * ones_vector;
switch_cost = b_switch*lane_switch * ones_vector;

individual_costs = [safety_cost; eq_cost; eff_cost; route_cost; pref_cost; switch_cost; cntr_cost];
L = safety_cost + eq_cost + cntr_cost + eff_cost + route_cost + pref_cost + switch_cost;

% Function theta is not defined in the paper
% My assumption is: output 1 if the vehicle is approaching its leader
    function [b] = theta(arg)
        b = arg>=0; 
    end

% Local equilibrium speed
    function [output_v] = v_eq(gap)
        sf = vd*td + s0;
        idx = gap>sf;
        output_v = vd*(idx) + (gap-s0)/td.*(~idx);
%         if gap > sf
%             output_v = vd;
%         else
%             output_v = (gap-s0)/td;
%         end
    end

% Function h expresses the keep right directive (lane number increases 
% from left to right)
    function [lane_cost] = h
        lane_cost = total_lanes - lane_number;
    end

end

