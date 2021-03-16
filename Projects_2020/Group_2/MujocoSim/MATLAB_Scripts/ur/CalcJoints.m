
function qpos = CalcJoints(CurrentJoints,Pos,Rot)

load simObj

useHardware = false;
if ~exist('Rot')
    Rot = [pi 0 0];
end

%% Initialize simulation
% -> This code has been tested on the UR5 and UR10 systems. Minor 
% adjustments may be required to use it with the UR3.
endEffector_X_offset = 0;
endEffector_Y_offset = 0;
endEffector_Z_offset = 100;

%{
if ~exist('simObj')
    % Create object
    simObj = URsim;
    simObj.Visualize = false;
    % Initialize simulation
    simObj.Initialize;
    % Set a tool frame offset (e.g. for Robotiq end-effector not shown in
    % visualization)
    simObj.FrameT = Tx(endEffector_X_offset)*Ty(endEffector_Y_offset)*Tz(endEffector_Z_offset);
    %simObj.FrameT = Tz(100);
    % Hide frames
    frames = '0123456E';
    for i = 1:numel(frames)
        hideTriad(simObj.(sprintf('hFrame%s',frames(i))));
    end
end
%}
display('test')

%% Create path
% -> NOTE: This is provided for demonstration only. Perfect execution of
% this path will require infinite accelerations at the initial and final
% waypoints meaning the robot will not track perfectly.

% Define dependent variable

isDrawing = false;

X(1,1) =  -Pos(1); % x-position
X(2,1) =  -Pos(2);    % y-position
X(3,1) =  Pos(3); % Z-position
s = 1;
% Transform coordinates into the workspace of the robot
Rotation = Rx(Rot(1))*Ry(Rot(2))*Rz(Rot(3));

qpos = [];
%% Animate simulation and move the robot to test
% Home simulation
% simObj.Home;
simObj.Joints = CurrentJoints;

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


end

