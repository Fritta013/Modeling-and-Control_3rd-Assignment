function [q] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration
len=length(q)
    % Updating q
 q = (q+(q_dot*ts))
    % Saturating the joint velocities 
    %making sure to respect the joints limits, min and max 

     for i = 1: len
       
        if (q(i) > q_max(i)) %respecting upper limit
            q(i)= q_max(i)
        elseif (q(i) < q_min(i)) %respecting lower limit 
            q(i)= q_min(i)
        end
    end

end