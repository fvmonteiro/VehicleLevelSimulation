function [all_paths, new_path] = generate_paths(t, T, new_path, all_paths, max_lane)
%generate_paths Function to generate all possible lane change decisions
% There's probably a more efficient (or at least more elegant) way to write
% this function, but this will work for now.

tlc = 5;
tinter = 2;

if t>T-tlc
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
            new_t = t + tlc + tinter;
        end
        [all_paths, new_path] = generate_paths(new_t, T, updated_path, all_paths, max_lane);      
    end
    
end

    function proper_array = pad_path_array(path_array)
        max_t = length(path_array);
        proper_array = [path_array zeros(1, T-max_t)];
        proper_array(1, max_t:end) = proper_array(1,max_t);
    end

end