function [condition,j_pos] = collision_check(q,obs_pos,threshold)

a       =   [   0     0.350     0       0       0       0    ];
d       =   [   0.205 0         0       0.305   0       0.075];  
alpha   =   [   pi/2    pi   pi/2       -pi/2      pi/2   0 ];
theta   =   [   q(1)     q(2)+pi/2  q(3)+pi/2 q(4)      q(5)      q(6)   ];

% Perform forward kinametics
T01 = transformation(a(1), d(1), theta(1), alpha(1));
T12 = transformation(a(2), d(2), theta(2), alpha(2));
T23 = transformation(a(3), d(3), theta(3), alpha(3));
T34 = transformation(a(4), d(4), theta(4), alpha(4));
T45 = transformation(a(5), d(5), theta(5), alpha(5));
T56 = transformation(a(6), d(6), theta(6), alpha(6));

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;
% Get position of each joint
j_pos(:,1) = T01(1:3,4);
j_pos(:,2) = T02(1:3,4);
j_pos(:,3) = T03(1:3,4);
j_pos(:,4) = T04(1:3,4);
j_pos(:,5) = T05(1:3,4);
j_pos(:,6) = T06(1:3,4);

% Create points between joints referencing as links
jpos = j_pos';
n = 8;
cnt = 1;
pnts = zeros(n*6,3); % points on links
for i1=1:6
    p1 = jpos(i1);
    p2 = jpos(i1+1);
    spacing = norm(p2-p1)/(n+1);
    displacement = (p2-p1)/(norm(p2-p1)*spacing);

    for i2 = 1:n
        pnts(cnt,:)= p1 + i2*displacement;
        cnt = cnt+1;
    end
end

% Concatenate joint positions and points on links
j_poses = cat(2,j_pos,pnts');

% Check for collision
dist = [];
condition = 0; % collision condition 0 if there is no collision
for j = 1:length(j_poses)
    dist(j,:) = norm(j_poses(:,j)' - obs_pos(1:end));
    if dist(j,:)<threshold
        condition=1;
        break
    end
end