classdef UR_Wrapper < handle

    properties
        q_current = [];
        q_desired = [];
        % Our default ee offsets
        ee_offset_x = 0;
        ee_offset_y = 0;
        ee_offset_z = 100;
        simObj = [];
    end
    
    methods
        function Object = UR_Wrapper()
            Object.simObj = URsim;
            Object.simObj.Visualize = false;
            % Initialize simulation
            Object.simObj.Initialize;
            % Set a tool frame offset (e.g. for Robotiq end-effector not shown in
            % visualization)
            Object.simObj.FrameT = Tx(Object.ee_offset_x)*Ty(Object.ee_offset_y)*Tz(Object.ee_offset_z);
        end
        
        function q_desired = calcJoints(Object,q_current, Pos, Rot)
            X(1) =  -Pos(1); % x-position
            X(2) =  -Pos(2); % y-position
            X(3) =  Pos(3);  % z-position
            Rotation = Rx(Rot(1))*Ry(Rot(2))*Rz(Rot(3));
            H_cur = Tx(X(1))*Ty(X(2))*Tz(X(3))*Rotation;
            % Set simulation toolpose to waypoint pose
            Object.simObj.Joints = q_current;
            Object.simObj.ToolPose = H_cur;
            % Move robot to match simulation
            q_desired = Object.simObj.Joints;
            Object.q_desired = q_desired;
        end
        
    end
end

