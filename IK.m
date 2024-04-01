% Code of Inverse Kinematics 
% Copyrights : HILS lab.
%
% Contributor:- Krushang Gabani ,Amir Memar , Sri Sadan ,Ehsan T. Esfahani
% Do not share this code file with anyone without permission.

% Input = Transportation matrix and Last Joint Position
% Output = 8 Different solution and best solution based on current joint position


function [Q,q_best] = IK(T_mat,joint_pos_now)

% T_mat = [n_e s_e a_e p_e
%           0   0   0   1 ];
% if joint_pos_now=[] then all the solutions
% if joint_pos_now~=[] then closest solution

n_e = T_mat(1:3,1);
s_e = T_mat(1:3,2);
a_e = T_mat(1:3,3);
p_e = T_mat(1:3,4);

Q = zeros(8,6); % 8 different solutions

% SCHUNK link lengths
a1 = .205;
a2 = .350;
a3 = .305;
d6 = .075;

% Solution of Anthropomorphic Arm
p_w = p_e - d6*a_e;
p_wx = p_w(1);
p_wy = p_w(2);
p_wz = p_w(3)-a1;

eps = 1e-4;
if (abs(p_wx)<=eps) 
    p_wx=0; 
end
if (abs(p_wy)<=eps) 
    p_wy=0; 
end

if (p_wx==0) && (p_wy==0)
    disp('There are infinity solutions');
    if isempty(joint_pos_now)
        return
    else
        q1_i = joint_pos_now(1);
        q1_ii = q1_i;
    end    
else
    q1_i = atan2(p_wy,p_wx);
    if p_wy>=0
        q1_ii = q1_i-pi;
    else
        q1_ii = q1_i+pi;
    end
end

if (sqrt(p_wx^2+p_wy^2+p_wz^2)>=abs(a2-a3)) && (sqrt(p_wx^2+p_wy^2+p_wz^2)<=a2+a3)
%     disp('The wrist point is inside of the workspace');
else
    disp('Error: The wrist point is outside of the workspace');
    Q = [];
    q_best = [];
    return
end

c3 = (p_wx^2+p_wy^2+p_wz^2-a2^2-a3^2)/(2*a2*a3);
s3 = sqrt(1-c3^2);
q3_i = atan2(s3,c3);
q3_ii = -q3_i;

s2_a = (a2+a3*c3)*p_wz;
s2_b = a3*s3*sqrt(p_wx^2+p_wy^2);
c2_a = (a2+a3*c3)*sqrt(p_wx^2+p_wy^2);
c2_b = a3*s3*p_wz;
q2_i =   atan2( (s2_a-s2_b),(c2_a+c2_b) );
q2_ii =  atan2( (s2_a+s2_b),(-c2_a+c2_b) );
q2_iii = atan2( (s2_a+s2_b),(c2_a-c2_b) );
q2_iv =  atan2( (s2_a-s2_b),(-c2_a-c2_b) );

Q(1,1:3) = [q1_i, q2_i, q3_i];
Q(2,1:3) = [q1_i, q2_iii, q3_ii];
Q(3,1:3) = [q1_ii, q2_ii, q3_i];
Q(4,1:3) = [q1_ii, q2_iv, q3_ii];
Q(5:8,1:3) = Q(1:4,1:3);

% SCHUNK 0 position effect using siciliano 2.95 to 2.97
for k=1:8 
    if Q(k,2)<-pi/2
        Q(k,2) =  Q(k,2) + 3*pi/2;
    else
        Q(k,2) =  Q(k,2) - pi/2;
    end
end
Q(:,3) = - Q(:,3);

% Solution of spherical wrist
R_6_0 = [n_e , s_e , a_e];
for k=1:4
    q1 = Q(k,1);
    q2 = Q(k,2);
    q3 = Q(k,3);
    R_3_0 = [ cos(q2 - q3)*cos(q1), -sin(q1), -sin(q2 - q3)*cos(q1);
        cos(q2 - q3)*sin(q1),  cos(q1), -sin(q2 - q3)*sin(q1);
        sin(q2 - q3),        0,          cos(q2 - q3)];
    
    R_6_3 = R_3_0'*R_6_0;
    a_x = R_6_3(1,3);
    a_y = R_6_3(2,3);
    a_z = R_6_3(3,3);
    s_z = R_6_3(3,2);
    n_z = R_6_3(3,1);
    
    if (abs(s_z)<eps) && (abs(n_z)<eps)
        disp('Wrist is singular.');
        if isempty(joint_pos_now)
            return
        else
            q6_i = joint_pos_now(6);
            q6_ii = q6_i;
        end
    else
        q6_i = atan2(s_z , -n_z);
        q6_ii = atan2(-s_z , n_z);
    end
    
    q4_i = atan2(a_y,a_x);
    q5_i = atan2(sqrt(a_x^2+a_y^2) , a_z);
    Q(k,4:6) = [q4_i , q5_i , q6_i];
    
    q4_ii = atan2(-a_y,-a_x);
    q5_ii = atan2(-sqrt(a_x^2+a_y^2) , a_z);
    Q(k+4,4:6) = [q4_ii , q5_ii , q6_ii];
end

out_range = [];
for i=1:8
    for j=1:6
        if (2*pi-abs(Q(i,j)) < 0.01)
            Q(i,j) = 0;
        end
        if abs(Q(i,j)) > (160*pi/180)
            out_range = [out_range;i];
            break;
        end
    end
end
Q(out_range,:) = [];

q_best = [];
if ~isempty(joint_pos_now)
    joint_pos_now = reshape(joint_pos_now,1,6); %make sure of the shape of q_now
    Q_now = repmat(joint_pos_now,size(Q,1),1);
    norms = sqrt(sum((Q-Q_now).^2,2));
%     norms = sqrt(sum((Q(:,1:3)-Q_now(:,1:3)).^2,2));    % is changed for test of singularity
    [C,I] = min(norms);
    q_best = Q(I,:);
end

end