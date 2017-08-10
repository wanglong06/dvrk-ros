function [n_vec, theta] = get_axis_angle(R)
%#codegen
% Jason Pile
%axis_angle(R) gives the rotation axis and the rotation angle extracted
%from rotation matrix R
%   n_vec=unit vector along rotation axis
%   theta=rotation angle about rotation axis
theta=acos(1/2*(trace(R)-1));
if abs(theta)<(10^-4)
    n_vec=[1;0;0];
elseif abs(theta-pi)<(10^-4)
    n_vec=[-1;0;0];
else
    n_cross=1/(2*sin(theta))*(R-R');
    n_vec=[-n_cross(2,3);n_cross(1,3);-n_cross(1,2)];
    n_vec=n_vec/norm(n_vec);
end
end