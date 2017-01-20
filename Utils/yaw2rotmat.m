function R = yaw2rotmat(yaw)

R = [cos(yaw) -sin(yaw);...
    sin(yaw) cos(yaw)];

end