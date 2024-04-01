function [Q,q_best] = getConfig(points,start_ori)
    %points    n,3
    alpha = 0;
    beta = 0;
    gamma = 0;
    
    % Rotation matrix
    R = [cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
        sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
                  -sin(beta),                                  cos(beta)*sin(gamma),                                 cos(beta)*cos(gamma)];
    
    T_mat = [R,points;0,0,0,1];
    [Q,q_best] = IK(T_mat,start_ori);
    
end
    
