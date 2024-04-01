clear all
[vrep,clientID,joint_handles] = Vrep_Powerball();

% Set the obstacle location
obs_x = 0  ;  obs_y = 0.3  ; obs_z = 0.5;
obs_pos = [obs_x,obs_y,obs_z];
[ball_return,ball_handles] = vrep.simxGetObjectHandle(clientID,'Sphere_obs',vrep.simx_opmode_oneshot_wait);
[floor_return,floor_handles] = vrep.simxGetObjectHandle(clientID,'base_link_respondable',vrep.simx_opmode_oneshot_wait);
[returnCode]=vrep.simxSetObjectPosition(clientID,ball_handles,floor_handles,[-obs_x,obs_y,obs_z+0.25],vrep.simx_opmode_oneshot);

% Set the start and goal point
start_x = -0.5  ;  start_y = 0  ; start_z = 0.5;
start_point = [start_x,start_y,start_z];
[ball_return,ball_handles] = vrep.simxGetObjectHandle(clientID,'Sphere_start',vrep.simx_opmode_oneshot_wait);
[floor_return,floor_handles] = vrep.simxGetObjectHandle(clientID,'base_link_respondable',vrep.simx_opmode_oneshot_wait);
[returnCode]=vrep.simxSetObjectPosition(clientID,ball_handles,floor_handles,[-start_x,start_y,start_z+0.25],vrep.simx_opmode_oneshot);

goal_x = 0.4  ;  goal_y = 0.4  ; goal_z = 0.5;
goal_point = [goal_x,goal_y,goal_z];
[ball_return,ball_handles] = vrep.simxGetObjectHandle(clientID,'Sphere_end',vrep.simx_opmode_oneshot_wait);
[floor_return,floor_handles] = vrep.simxGetObjectHandle(clientID,'base_link_respondable',vrep.simx_opmode_oneshot_wait);
[returnCode]=vrep.simxSetObjectPosition(clientID,ball_handles,floor_handles,[-goal_x,goal_y,goal_z+0.25],vrep.simx_opmode_oneshot);

% Configuration space
range1 = [-2.9 5.9];
range2 = [-1.9 3.8];
range3 = [-2.7 5.4];
range4 = [-2.9 5.9];
range5 = [-2.4 4.8];
range6 = [-2.9 5.9];

% Generate random configuration

for i = 1:10000
    joint1 = rand(1,1) .* (range1(2) - range1(1)) + range1(1);
    joint2 = rand(1,1) .* (range2(2) - range2(1)) + range2(1);
    joint3 = rand(1,1) .* (range3(2) - range3(1)) + range3(1);
    joint4 = rand(1,1) .* (range4(2) - range4(1)) + range4(1);
    joint5 = rand(1,1) .* (range5(2) - range5(1)) + range5(1);
    joint6 = rand(1,1) .* (range6(2) - range6(1)) + range6(1);
    q = [joint1, joint2, joint3, joint4, joint5, joint6];

    random_angles(:,i) = q';

    % Validate the configuration
    [condition(i),j_pos] = collision_check(q,obs_pos,0.2);
    j(:,i) = j_pos(:,end);

end

idx = find(condition==0);
joint_angles = random_angles(:,idx);
safe_points = j(:,idx);
% Start and Goal Configurations

% orientation of the coordinate system at waypoints
alpha = 0;
beta = 0;
gamma = 0;

% Rotation matrix
R = [cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
    sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
              -sin(beta),                                  cos(beta)*sin(gamma),                                 cos(beta)*cos(gamma)];

T = [0.5;0.3;0.3];
waypoints = [start_point;goal_point];
T_mat = [R,T;0,0,0,1];

% Initial angles
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
q = [q1 q2 q3 q4 q5 q6];

% iteration for performing IK at all waypoints
for point = 1:2
    T_mat = [R,waypoints(point,:)';0,0,0,1];
    [Q,q_best(point,:)] = IK(T_mat,q);
end

% Start configuration
s_config = q_best(1,:);
[s_cond,start_pos] = collision_check(s_config,obs_pos,0.2);
% Goal configuration
g_config = q_best(2,:); 
[g_cond,goal_pos] = collision_check(g_config,obs_pos,0.2);

% Adding start and goal config to the list of joint angles generated before
joint_angles = [s_config',joint_angles,g_config'];
end_effectors = [start_pos(:,end),safe_points,goal_pos(:,end)];

% Generate tree
start_node = start_pos(:,end);
goal_node = goal_pos(:,end);

safe_points = [start_node,safe_points,goal_node];
query_points = safe_points';

% Distance at which to find neighbors
k = 0.3;

[indices,D] = rangesearch(query_points, query_points, k);

% Delete repeating nodes
for i = 1:length(indices)
    del_values = i;
    ind_del = ismember(indices{i},del_values);
    indices{i}(ind_del) = [];
    D{i}(ind_del) = [];
end

% A* search

% Calculate heuristic cost
for i = 1:length(query_points)
    temp = query_points(i,:)';
    hu_cost(i) = norm(temp - goal_node);
end

% Calculate total cost
for i = 1:length(hu_cost)
    temp1 = indices{i};
    h = hu_cost(temp1);
    total_cost{i} = D{i} + h;
end
total_cost = total_cost';


% Calculate path
visited_set = [1]; % Shortes path
count = 0;
while count==0
    index = visited_set(end);
    open_set = indices{index};
    open_set_cost = total_cost{index};
    temp2 = ismember(open_set,visited_set);
    open_set(temp2) = [];
    open_set_cost(temp2) = [];
    temp3 = find(open_set_cost==min(open_set_cost));
    next_node = open_set(temp3);
    visited_set(end+1) = next_node;
    if next_node==length(indices)
        count=1;
    end
end

% Generate trajectory
intial = 1;
final = 2;

path_points = end_effectors(:,visited_set);

% Get points between waypoints/path_points
pnts = get_points(path_points');
pnts = pnts';
pnts(:,1) = start_point';
pnts(:,end) = goal_point';
start_ori = [0,0,0,0,0,0];

for p = 1:length(pnts)
    % Perform inverse kinematics at each point generated
    [Q,q_best] = getConfig(pnts(:,p),start_ori);

    % Sometimes there are no angles found by IK algorithm in that case we
    % move forward to next point
    co = length(q_best);
    if co>0
       % Check for collision
        [condition,j_pos] = collision_check(q_best',obs_pos,0.1);
        if condition==1
            disp("stop")
            disp("Failed to reach the goal")
            return
        else
            % Send the angles to simulator
            for i = 1:6
                vrep.simxSetJointPosition(clientID,joint_handles(i),q_best(i),vrep.simx_opmode_oneshot_wait);
            end
        end
    end
    start_ori = q_best;
end

plot_path(pnts',query_points,'Interpolated points')

plot_path(path_points',query_points,'PRM Path using A* search')
hold on;
scatter3(obs_x,obs_y,obs_z,[1000,1000,1000],'filled')

