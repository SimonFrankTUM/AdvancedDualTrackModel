function A_z = f_zrot(ang)
% Rotation matrix for angle ang about Z axis
A_z = [ cos(ang)   -sin(ang)    0;
        sin(ang)    cos(ang)    0;
        0           0           1 ];
end