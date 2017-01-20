function yaw = dcm2yaw(C)

costheta = C(1,1);
sintheta = C(1,2);

yaw = atan2(sintheta, costheta);

end