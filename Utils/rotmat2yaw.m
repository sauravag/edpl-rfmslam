function yaw = rotmat2yaw(R)

costheta = R(1,1);
sintheta = R(2,1);

yaw = atan2(sintheta, costheta);

end