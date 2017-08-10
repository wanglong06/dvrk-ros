function R_hat = r_norm(R)
%#codegen
rx = R(:,1);
rx = rx/norm(rx);
ry = R(:,2);
ry = ry/norm(ry);
rz = cross(rx,ry);
rz = rz/norm(rz);
R_hat = [rx,ry,rz];
end