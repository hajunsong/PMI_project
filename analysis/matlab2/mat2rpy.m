function rpy = mat2rpy(mat)
    % Rotation matrix to roll-pitch-yaw
    % pitch = -asin(mat(3,1));
    pitch = atan2(-mat(3,1), sqrt(mat(1,1)^2 + mat(2,1)^2));
    roll = atan2(mat(3,2), mat(3,3));
    yaw = atan2(mat(2,1), mat(1,1));
    rpy = [roll; pitch; yaw];
end
