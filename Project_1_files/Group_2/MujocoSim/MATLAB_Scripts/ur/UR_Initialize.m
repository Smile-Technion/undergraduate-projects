function s = UR_Initialize()
    endEffector_X_offset = 0;
    endEffector_Y_offset = 0;
    endEffector_Z_offset = 100;

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
    s = 1
end
