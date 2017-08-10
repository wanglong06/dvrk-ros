function [x_dot_des,x_dot_des_disp]= ...
    compute_desired_twist(position_error_norm,n_unit_vec,Omega_unit_vec,theta_error,epsilon_vec,v_limits,scale_eplison,enable)
%% compute_twist :
%   Description:    This embedded matlab function computes the desired twist
%                   of the end-effector to follow a straight line through p_curr
%                   and p_des and bring the R_curr to R_des using angle/axis
%                   representation
%
%   References:     Siciliano, Sciavicco, Villani, Oriolo, Robotics: Modelling, Planning and Control
%                   1. angle/axis pg 52
%                   2. trajectories pg 161
%
%                   Nabil's notes on Resolved Rate Algorithm
%
%   Inputs:         1. p_des is the desired end effector position
%                   2. R_des is the desired end effectror orientation
%                   3. p_curr is the current position of the robot's operational point
%                   5. R_curr is the current orientation of the robot's tool
%   Outputs:        1. x_des is the desired twist of the robot end-effector
%                   2. theta is the orientation error (to be removed in final version)
%--------------------------------------------------------------------------
if enable == 1  % if motion is enabled
    Vmin = v_limits(1,1); % [m/s]
    Vmax = v_limits(2,1); % [m/s]
    OmegaMin = v_limits(3,1);   %rad/s
    OmegaMax = v_limits(4,1);   %rad/s
    
    position_epsilon=epsilon_vec(1);
    orientation_epsilon=epsilon_vec(2);

    
    %%  check for convergence
%     lin_vel_ratio=Vmax/Vmin;
    lin_vel_ratio = scale_eplison;
    ang_vel_ratio=OmegaMax/OmegaMin;
    %     if abs(position_error) < position_epsilon && abs(theta_error) < orientation_epsilon
    %         x_dot_des = zeros(6,1);
    %         %%if not converged calculate target speed
    %
    %     else
    
    if abs(position_error_norm)>position_epsilon*lin_vel_ratio;
%         alpha_translation_t=1; 
        LinVelMag = Vmax; %go at maximal speed
    elseif abs(position_error_norm)<position_epsilon;
%         alpha_translation_t=0;
        LinVelMag = 0;
    else
        alpha_translation_t=...
            (position_error_norm - position_epsilon)/...
            (lin_vel_ratio*position_epsilon - position_epsilon);
        LinVelMag = (Vmax - Vmin)*alpha_translation_t + Vmin;
%         safety check
        if (LinVelMag >Vmax) 
            LinVelMag =Vmax;
        end
    end
    
       
    
    
    if abs(theta_error)>orientation_epsilon*ang_vel_ratio
        alpha_rotation_t=1; %go at maximal speed
    elseif abs(theta_error)<orientation_epsilon,
        alpha_rotation_t=0;
    else
        alpha_rotation_t=theta_error/(ang_vel_ratio*orientation_epsilon);
    end
    
    AngVelMag=alpha_rotation_t*OmegaMax;
    
    % compute desired twist
    x_dot_des = [LinVelMag*n_unit_vec
        AngVelMag*Omega_unit_vec];
    % end
else  %enable=0
    x_dot_des = zeros(6,1);
end
    x_dot_des_disp = x_dot_des;
% safety by Long Wang
if (sum(isnan(x_dot_des))~=0)
    x_dot_des = zeros(6,1);
end
