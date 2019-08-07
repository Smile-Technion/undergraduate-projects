%  USAGE: playlog(data); 
%    MuJoCo data log player; data must be loaded with readlog()
%    MuJoCo HAPTIX must be running, with the same model loaded and paused

function playlog(data)

    % make sure we are connected
    if ~mj_connected,
        mj_connect;
        if ~mj_connected,
            error('Could not connect to simulator');
        end
    end

    % make sure model loaded in simulator is compatible
    info = mj_info;
    if data.nq~=info.nq || data.nv~=info.nv || data.nu~=info.nu || ...
       data.nmocap~=info.nmocap || data.nsensordata~=info.nsensordata,
        error('Incompatible model loaded in simulator');
    end

    % prepare simulator data
    state = mj_get_state;
    control = mj_get_control;
    mocap = mj_get_mocap;
 
    % data ranges
    fmax = length(data.time);
    tmin = min(data.time);
    tmax = max(data.time);
    
    % prepare player figure
    figure(1);
    clf;
    set(gcf,'MenuBar','none','Name','MuJoCo Player','Units','normalized',...
        'WindowButtonDownFcn',@mousedown,'WindowButtonUpFcn',@mouseup,'KeyPressFcn',@keypress);
    timeplot = subplot('Position', [.1 .3 .8 .6]);
    plot(data.time,'linewidth',1,'color','b');
    hold on;
    lineh = line([1 fmax], [tmin tmin], 'linewidth',1,'color','r','linestyle',':');
    linev = line([1 1], [tmin tmax], 'linewidth',1,'color','r','linestyle',':');
    xlabel('frame number');
    ylabel('simulation time');
    title('select frame with mouse or arrow keys');
    axis tight;
    
    % insert controls
    panel = uipanel('Units','normalized','Position',[.1 .05 .8 .1]);
    play = uicontrol('Parent',panel,'Style','togglebutton','Units','normalized','Position',[.02 .1 .15 .8],...
        'String','play','fontsize',12,'Callback',@playbutton);
    uicontrol('Parent',panel,'Style','text','Units','normalized','Position',[.2 .02 .15 .8],...
        'String','speed','fontsize',12);
    speed = uicontrol('Parent',panel,'Style','edit','Units','normalized','Position',[.34 .1 .14 .8],...
        'String','1','fontsize',10,'Callback',@gotoframe);
    uicontrol('Parent',panel,'Style','text','Units','normalized','Position',[.5 .02 .15 .8],...
        'String','frame','fontsize',12);
    frame = uicontrol('Parent',panel,'Style','edit','Units','normalized','Position',[.64 .1 .14 .8],...
        'String','','fontsize',10,'Callback',@gotoframe);
    uicontrol('Parent',panel,'Style','pushbutton','Units','normalized','Position',[.83 .1 .15 .8],...
        'String','close','fontsize',12,'Callback','close(gcf);');
    
    % initial rendering
    lastframe = 1;
    tstart = tic;
    lasttm = toc(tstart);
    render(1);
    
    
    %------------------- callbacks
    
    % play button
    function playbutton(hObject, callbackdata)
        
        % start
        if hObject.Value,
            t = timer('ExecutionMode','fixedRate','Period',0.02,'TimerFcn',@playtimer);
            hObject.UserData = t;
            lasttm = toc(tstart);
            start(t);
            
        % stop
        else
            t = hObject.UserData;
            stop(t);
            delete(t);
        end
    end
    

    % function called by play timer
    function playtimer(hObject, callbackdata)
        
        % compute next frame
        sp = min(10, max(0.01, str2num(speed.String)));
        desiredInterval = (toc(tstart) - lasttm) * sp;
        f = lastframe;
        while (f<fmax) && (data.time(f)-data.time(lastframe)<desiredInterval) ...
                && (data.time(f)>=data.time(lastframe)),
            f = f+1;
        end
        
        % go back one frame if closer approximation
        if (abs(data.time(f)-data.time(lastframe)-desiredInterval) > ...
            abs(data.time(f-1)-data.time(lastframe)-desiredInterval)) && ...
            data.time(f)>=data.time(lastframe),
           f = f-1;
        end
        
        % render frame
        render(f);
    end

    
    % key press
    function keypress(hObject, callbackdata)
        
        % process key
        if strcmp(callbackdata.Key, 'leftarrow'),
            f = max(lastframe-1, 1);
        elseif strcmp(callbackdata.Key, 'rightarrow'),
            f = min(lastframe+1, fmax);
        elseif strcmp(callbackdata.Key, 'uparrow'),
            f = fmax;
        elseif strcmp(callbackdata.Key, 'downarrow'),
            f = 1;
        else
            f = -1;
        end
        
        % render
        if f>0,
            render(f);
        end
    end
    
    
    % mouse down
    function mousedown(hObject, callbackdata)
        
        % get cursor point in plot coordinates
        p = timeplot.CurrentPoint;
        f = round(p(1,1));
        t = p(1,2);
        
        % return if point is not inside axis
        if f<1 || f>fmax || t<tmin || t>tmax,
            return;
        end
        
        % install mouse motion callback
        set(gcf,'WindowButtonMotionFcn',@mousemove);
        
        % render frame
        render(f);
    end


    % mouse up
    function mouseup(hObject, callbackdata)

        % un-install mouse motion callback
        set(gcf,'WindowButtonMotionFcn','');
    end


    % mouse move
    function mousemove(hObject, callbackdata)
        
        % get cursor point in plot coordinates
        p = timeplot.CurrentPoint;
        f = round(p(1,1));
        
        % return if frame is not inside axis
        if f<1 || f>fmax,
            return;
        end
        
        % render frame
        render(f);
    end


    % go to frame edit box
    function gotoframe(hObject, callbackdata)
        
        % get frame number
        f = round(str2num(frame.String));
        
        % render frame
        render(f);
    end


    % render frame
    function render(f)
        
        % enforce range
        if f<1,
            f = 1;
        elseif f>fmax,
            f = fmax;
        end
        
        % show frame number
        frame.String = sprintf('%d', f);
        
        % skip repetition
        if f==lastframe,
            return;
        end
        
        % send to simulator
        state.time = data.time(f);
        state.qpos = data.qpos(:,f);
        state.qvel = data.qvel(:,f);
        mj_set_state(state);

        control.time = data.time(f);
        control.ctrl = data.ctrl(:,f);
        mj_set_control(control);

        mocap.time = data.time(f);
        mocap.pos = reshape(data.mocap_pos(:,f), [3 mocap.nmocap])';
        mocap.quat = reshape(data.mocap_quat(:,f), [4 mocap.nmocap])';
        mj_set_mocap(mocap);
        
        mj_message(sprintf('time %.3f', data.time(f)));
        
        % record render frame and cpu time
        lastframe = f;
        lasttm = toc(tstart);
        
        % plot selection
        lineh.XData = [1 fmax];
        lineh.YData = [state.time state.time];
        linev.XData = [f f];
        linev.YData = [tmin tmax];
        drawnow;
    end   

end
