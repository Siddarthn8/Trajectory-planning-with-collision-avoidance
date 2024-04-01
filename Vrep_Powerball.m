% Copyrights : HILS lab.
%
% Contributor:- Krushang Gabani ,Amir Memar , Sri Sadan ,Ehsan T. Esfahani
% Do not share this code file with anyone without permission.
% 
% Output = VREP Library functions , client ID of Vrep Connections, Requires
% Joint Handles.
% 

function [vrep,clientID,joint_handles] = Vrep_Powerball()
%% Add path of kinematics files
addpath(genpath(pwd))
%% Vrep remote api communication with matlab
vrep = remApi('remoteApi');
vrep.simxFinish(-1);

%% create a client id
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);


if clientID > -1
    disp('%----------Connection Successful----------%');
%     vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('%----------Connection failed----------%');
%     return
end
vrep.simxSynchronous(clientID,true);



%% Get joint handles
joint_return = zeros(1,6);
joint_handles = zeros(1,6);
for i = 1:6
   name = strcat('arm_',num2str(i),'_joint');
   [joint_return(i),joint_handles(i)] = vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_oneshot_wait); 
end


%% Setting each joint torque limits
torque = 2*[15 15 10 10 5 5];
N_joints = 6;

for j = 1:N_joints
    vrep.simxSetJointForce(clientID,joint_handles(j),torque(j),vrep.simx_opmode_oneshot_wait);
end

%% Starting simulation 
returnCode=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
disp('simulation started.')

end
