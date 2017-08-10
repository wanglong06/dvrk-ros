% Create ROBOT
% Create SENSOR
% Create TRAJECTORY
% Create force LOG
% Create IDX = 0

% While ROBOT.position != TRAJECTORY.end:
% 	POS = ROBOT.position
% 	FORCE = SENSOR.force
% 	LOG.record(FORCE, POS, time)
% 	if TRAJECTORY[IDX] ~= POS:
% 		IDX ++
% 	NEXTPOS = TRAJECTORY[idx]
% 	NEXTROT = makeRot(FORCE)
% 	ROBOT.move(NEXTPOS, NEXTROT);
% 	

robot = psm('PSM1');
sensor = force_sensor('/dvrk/PSM1_FT/raw_wrench');
trajectory = [0 0 0; 0 1 0];
rot_desired = eye(3);
log = [];
trajectory_idx = 1;
epsilon = 1e-3;
% resolved rates algorithm parameters
epsilon_vec = 0;
    Vmin = 0.005; % [m/s]
    Vmax = 0.05; % [m/s]
    OmegaMin = 10/180*pi;   %rad/s
    OmegaMax = 120/180*pi;   %rad/s
v_limits = [Vmin;Vmax;OmegaMin;OmegaMax];
scale_epsilon = 10;
enable = true;
% force controller parameters
force_avg_window_size = 50;
force_control_dir = [0 0 1];
force_dir_buffer = repmat(force_control_dir,force_avg_window_size,1);
force_data_buffer_idx = 1;
f_des_mag = 0.5; % [N]
force_admittance_gain = diag([1 1 1]);
%%  Main loop
while trajectory_idx<size(trajectory,1)
    %%  Position and orientaion control
    % get current position and orientation
    [frame_cur, timestamp] = robot.get_position_current();
    pos = frame_cur(1:3,4)';
    R_cur = frame_cur(1:3,1:3);
    % get desired position and orientaion
    pos_des = trajectory(trajectory_idx,:);
    R_des = rot_desired;
    % calculate position error
    [position_error_mag,n_unit_vec] = get_position_err(pos_des',pos');
    R_err  = get_orientation_err(R_des,R_cur);
    [n_unit_vec, theta_error] = get_axis_angle(R_err);
    % compute desired twist
    [x_dot_des_position,~]= compute_desired_twist(...
        position_error_mag,n_unit_vec,omega_unit_vec,theta_error,...
        epsilon_vec,v_limits,scale_epsilon,enable);
    %%  Force control
    % calculate force error
    [wrench, timestamp]= sensor.get_wrench_current;
    f_cur = wrench(1:3)';  
    force_dir_buffer(force_data_buffer_idx,:) = normr(f_cur); 
    f_des = force_control_dir*f_des_mag;
    f_err = f_des - f_cur;
    p_dot_des_force = - force_admittance_gain*f_err(:);
    %%  Hybrid force/position
    proj_mat_force = force_control_dir(:)* force_control_dir(:)';
    proj_mat_position = eye(3) - proj_mat_force;
    p_dot_des = proj_mat_position*x_dot_des_position(1:3) + ...
        proj_mat_force*p_dot_des_force;
    %   Updating force control direction
    force_control_dir = mean(force_dir_buffer);
    %%  update index of force buffer and trajectory
    if norm(pos - trajectory(trajectory_idx,:))<epsilon
        trajectory_idx = trajectory_idx + 1;
    end
    force_data_buffer_idx = ...
        mod(force_data_buffer_idx + 1,force_avg_window_size);
end
