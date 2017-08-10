function R  = get_orientation_err(R_des,R_cur)
% Long Wang
% 2013/06/02
% orientation error R_des=R*R_cur is a rotayionn sequence in world frame
R = R_des * (R_cur');
