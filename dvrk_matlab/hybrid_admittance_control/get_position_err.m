function [position_error_norm,n_unit_vec] = get_position_err(p_des,p_cur)
% Long Wang 2013/06/02
p_des = p_des(:); % make sure column vector
p_cur = p_cur(:);
position_error_vec = p_des-p_cur;
position_error_norm = norm(position_error_vec);
%% compute linear velocity direction
if position_error_norm==0
    n_unit_vec = [1;0;0];
else
    n_unit_vec = position_error_vec/position_error_norm;
end