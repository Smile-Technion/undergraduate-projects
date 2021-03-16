
function qpos = CalcPath(Pos,Rot)

useHardware = false;
if ~exist('Rot')
    Rot = [pi 0 0];
end

%% Initialize simulation
% -> This code has been tested on the UR5 and UR10 systems. Minor 
% adjustments may be required to use it with the UR3.
endEffector_X_offset = 0;
endEffector_Y_offset = 0;
endEffector_Z_offset = 0;

if ~exist('simObj')
    % Create object
    simObj = URsim;
    simObj.Visualize = false;
    % Initialize simulation
    simObj.Initialize;
    % Set a tool frame offset (e.g. for Robotiq end-effector not shown in
    % visualization)
    simObj.FrameT = Tx(endEffector_X_offset)*Ty(endEffector_Y_offset)*Tz(endEffector_Z_offset);
    
    % Hide frames
    frames = '0123456E';
    for i = 1:numel(frames)
        hideTriad(simObj.(sprintf('hFrame%s',frames(i))));
    end
end



%% Create path
% -> NOTE: This is provided for demonstration only. Perfect execution of
% this path will require infinite accelerations at the initial and final
% waypoints meaning the robot will not track perfectly.



% t_s_len_full = t_s_len*2
% ar_size = size(linspace(0,1,t_s_len))
% s_x = [linspace(0,1,t_s_len), ones(ar_size)]
% s_y = [zeros(ar_size), linspace(0,1,t_s_len)]
% s = linspace(0,ts,length(s_x))
% % Define circle radius (mm)
% start = 200;
% stroke = 400;
% % Define position data
% w = 3;
% X = [];
% X(1,:) = start+stroke*s_x; % x-position
% X(2,:) =  -1.5*start+stroke*s_y;    % y-position
% X(3,:) =  50+50*sin(2*pi*w*s); % ?-position
% X(4,:) = 1;             % Append 1 (homogeneous coordinate)

X(1,:) =  Pos(1,:); % x-position
X(2,:) =  Pos(2,:);    % y-position
X(3,:) =  Pos(3,:); % ?-position
% X(4,:) =  1;             % Append 1 (homogeneous coordinate)
s = 1;
% Transform coordinates into the workspace of the robot
% Rotation = Rx(pi);
Rotation = Rx(Rot(1,:))*Ry(Rot(1,:))*Rz(Rot(1,:));

% X = X;

% Plot waypoints
% plt_Waypoints = plot3(simObj.Axes,X(1,:),X(2,:),X(3,:),'.m');
qpos = [];
%% Animate simulation and move the robot to test
% Home simulation
% simObj.Home;


% Move through waypoints
for i = 1:numel(s)
    % Define pose from waypoint
    H_cur = Tx(X(1,i))*Ty(X(2,i))*Tz(X(3,i))*Rotation;
    
    % Set simulation toolpose to waypoint pose
    simObj.ToolPose = H_cur;
    
    % Move robot to match simulation
    q = simObj.Joints;
    qpos(:,i) = q;
    % Allow plot to update
  if isDrawing
      drawnow;
  end
  
end

% save("C:\Users\crazy\OneDrive - Technion\Semester 7\Final Project\UR5 Simulation\UR5_Sim\OldGripper\rec_qjoints.mat",'qpos')

end

