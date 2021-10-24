%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the acrobot after the MAIN script has been run.
%
%   Matthew Sheen, 2014
%
%   Note: data is passed from this to the callback functions in the figure
%   object's UserData field.
%   For compatibility with 2014a and earlier, I use set/get instead of the
%   object.field notation.
%   Can also be done with global vars as in the backup version. I hate
%   global variables so, this version is the result.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function PID_Plotter(p)
close all

% show and implement wall
wall = p.wall;

% show inverse kinematics solution
show_solution = p.showsolution;
%Name the whole window and define the mouse callback function
f = figure;
set(f,'WindowButtonMotionFcn','','WindowButtonDownFcn',@ClickDown,'WindowButtonUpFcn',@ClickUp,'KeyPressFc',@KeyPress);


figData.Fx = [];
figData.Fy = [];
figData.xend = [];
figData.yend = [];
figData.fig = f;
figData.tarControl = true;

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%
figData.simArea = subplot(1,1,1); %Eliminated other subplots, but left this for syntax consistency.
axis equal
hold on
grid on




%Plot trajectory :
p.traj = [p.traj; p.traj(1,:)];
p.xtarget = p.traj(1,1);
p.ytarget = p.traj(1,2);
plot(p.traj(:,1), p.traj(:,2),'r--');

% Create solution object for inverse kinematics
if show_solution
%Create solution link1 object:
sol_width1 = p.l1*0.05;
sol_xdat1 = 0.5*sol_width1*[-1 1 1 -1];
sol_ydat1 = p.l1*[0 0 1 1];
sol_link1 = patch(sol_xdat1,sol_ydat1, [0 0 0 0], 'LineStyle', '--');

%Create solution link2 object:
sol_width2 = p.l2*0.05;
sol_xdat2 = 0.5*sol_width2*[-1 1 1 -1];
sol_ydat2 = p.l2*[0 0 1 1];
sol_link2 = patch(sol_xdat2,sol_ydat2, [0 0 0 0], 'LineStyle', '--');
end

% Create wall object
if wall
wall_left = p.wallleft;
wall_right = p.wallright;
wall_stiffness = p.wallstiffness;
wall_x = [wall_left wall_right wall_right wall_left];
wall_y = [-3 -3 3 3];
patch(wall_x,wall_y,'red','FaceAlpha',.3)
end

%Create pendulum link1 object:
width1 = p.l1*0.05;
xdat1 = 0.5*width1*[-1 1 1 -1];
ydat1 = p.l1*[0 0 1 1];
link1 = patch(xdat1,ydat1, [0 0 0 0],'r');

%Create pendulum link2 object:
width2 = p.l2*0.05;
xdat2 = 0.5*width2*[-1 1 1 -1];
ydat2 = p.l2*[0 0 1 1];
link2 = patch(xdat2,ydat2, [0 0 0 0],'b');
axis([-3.5 3.5 -3.6 3.6]);

%Dots for the hinges:
h1 = plot(0,0,'.k','MarkerSize',40); %First link anchor
h2 = plot(0,0,'.k','MarkerSize',40); %link1 -> link2 hinge

% Traces for the trajectory:
trace = plot(0,0,'.g','MarkerSize',10); %First link anchor

%Timer label:
timer = text(-3.2,-3.2,'0.00','FontSize',28);

%Std of effector - x
std_x = text(-3.2,-0.2,'0.00','FontSize',28);

%Torque meters on screen
tmeter1 = text(0.6,-3.2,'0.00','FontSize',22,'Color', 'r');
tmeter2 = text(2.2,-3.2,'0.00','FontSize',22,'Color', 'b');
tau = [0, 0];
%Target Pt.
targetPt = plot(p.xtarget,p.ytarget,'xb','MarkerSize',30);

hold off

%Make the whole window big for handy viewing:
set(f, 'units', 'inches', 'position', [5 5 10 9])
set(f,'Color',[1,1,1]);

%Animation plot loop -- Includes symplectic integration now.
z1 = p.init;

set(f,'UserData',figData);

% current_time is where physics take place. the controller frequency
% differs from the frequency of the physics.
current_time = 0;
physics_freq = 100;
controller_freq = 99;
freq_ratio = controller_freq/physics_freq;
dt_phy = 1/physics_freq;
time_passed = 0; % for PID
controller_counter = 0; % to track when the controller frequency hit
if freq_ratio<=0 || freq_ratio>=1
error("controller frequency is not valid")
end

% trajectory planner
traj = Trajectory_planner(p);
iter = 0;
iterlen = length(traj);


%End efftor movement and joint values
EndEff_x = zeros(1,iterlen);
EndEff_y = zeros(1,iterlen);
traj_x = zeros(1,iterlen);
traj_y = zeros(1,iterlen);
q1_ideal = zeros(1,iterlen);
q2_ideal = zeros(1,iterlen);
q1_real = zeros(1,iterlen);
q2_real = zeros(1,iterlen);
round_counter_1 = 0;
round_counter_2 = 0;


movegui('center');    
while (ishandle(f))
    figData = get(f,'UserData');
    
    %%%% INTEGRATION %%%%%%%%%
    %Old velocity and position
    xold = [z1(1),z1(3)];
    vold = [z1(2),z1(4)];
   
    %Call RHS given old state
    zdot1 = Physics(z1,p,tau);
    ainter = [zdot1(2),zdot1(4)];
    vinter = vold + ainter*dt_phy; %Update velocity based on old RHS call
    
    %Update position.
    xnew = xold + vinter*dt_phy;
    vnew = (xnew-xold)/dt_phy;
    z1 = [xnew(1) vnew(1) xnew(2) vnew(2)];
    %%%%%%%%%%%%%%%%%%%%
    
    
    %%%%CONTROLLER%%%%%%%%%%
    if controller_counter<1   % do nothing
        controller_counter = controller_counter + freq_ratio;
        %disp("torque hasn't changed");
    elseif controller_counter >=1   % change tau
        tau = PIDController(z1,p);
        %disp("torque has been altered");
        controller_counter = controller_counter -1;
        time_passed = 0;
    end  
    %%%%%%%%%%%%%%%%%%%%
    
    % trajectory follower
     if iter < iterlen
        iter = iter+1;
    else
        iter = 1;
    end
    p.xtarget = traj(iter,1);
    p.ytarget = traj(iter,2);
    set(targetPt,'xData',p.xtarget); %Change the target point graphically.
    set(targetPt,'yData',p.ytarget);
   
    

    ra_e = ForwardKin(p.l1,p.l2,z1(1),z1(3));
    figData.xend = ra_e(1);
    figData.yend = ra_e(2);
    set(f,'UserData',figData);
    
    %When you hit a key, it changes to force mode, where the mouse will
    %pull things.
    if ~isempty(figData.Fx)
    p.Fx = figData.Fx;
    end
    if ~isempty(figData.Fy)
    p.Fy = figData.Fy;
    end
    
    
       % wall apply force
    if wall 
          if figData.xend> wall_left && figData.xend<wall_right
              p.Fx = -wall_stiffness*(figData.xend - wall_left);

          else
              p.Fx = 0;
          end
    end 
   
    %On screen timer.
    set(timer,'string',strcat(num2str(current_time,'%.2f'),'s'))
    
%     On screen std of x
%     set(std_x,'string',strcat(num2str(current_time,'%.2f'),'s'))
%     On screen std of y
%     On screen average error of q1
%     On screen average error of q2 

    %Draw the inverse kinematics solution
    if show_solution
    [q1_sol, q2_sol] = InverseKin(p.l1, p.l2, p.xtarget, p.ytarget);
    sol_num = p.invKsol; % 1 = lower arm, 2 = upper arm
    q1_sol =  q1_sol(sol_num)-pi/2;
    q2_sol =  q2_sol(sol_num);
    rot1_sol = [cos(q1_sol), -sin(q1_sol); sin(q1_sol),cos(q1_sol)]*[sol_xdat1;sol_ydat1];
    rot2_sol = [cos(q2_sol+q1_sol), -sin(q2_sol+q1_sol); sin(q2_sol+q1_sol),cos(q2_sol+q1_sol)]*[sol_xdat2;sol_ydat2];
    set(sol_link1,'xData',rot1_sol(1,:));
    set(sol_link1,'yData',rot1_sol(2,:));
    set(sol_link2,'xData',rot2_sol(1,:)+(rot1_sol(1,3)+rot1_sol(1,4))/2);
    set(sol_link2,'yData',rot2_sol(2,:)+(rot1_sol(2,3)+rot1_sol(2,4))/2);
    end
    
    
    
    
    %For the record
    traj_x(iter) = traj(iter,1);
    traj_y(iter) = traj(iter,2);
    EndEff_x(iter) = ra_e(1);
    EndEff_y(iter) = ra_e(2);
    q1_real(iter) = z1(1);
    q2_real(iter) = z1(3);
    if iter>1
        if q1_ideal(iter-1) - (q1_sol + 2*pi*round_counter_1) >= pi
            round_counter_1 = round_counter_1 + 1;
        elseif q1_ideal(iter-1) - (q1_sol + 2*pi*round_counter_1) <= -pi
                 round_counter_1 = round_counter_1 -1;
        end
        if q2_ideal(iter-1) - (q2_sol+ 2*pi*round_counter_2) >= pi
            round_counter_2 = round_counter_2 + 1;
        elseif q2_ideal(iter-1) - (q2_sol+ 2*pi*round_counter_2) <= -pi
                 round_counter_2 = round_counter_2 -1;
        end
    end
    q1_ideal(iter) = q1_sol + 2*pi*round_counter_1;
    q2_ideal(iter) = q2_sol + 2*pi*round_counter_2;
    %Position & Trajectory Record for further analysis
    if iterlen == iter
        save('End_Effector_data.mat','EndEff_x','EndEff_y','traj_x','traj_y','q1_ideal','q2_ideal','q1_real','q2_real','dt_phy');
        disp('New data is saved to End_effector_data.mat. Open Difference_Analysis.m to see the plot') 
    end     
    
    
    
    %Rotation matrices to manipulate the vertices of the patch objects
    %using theta1 and theta2 from the output state vector.
    rot1 = [cos(z1(1)), -sin(z1(1)); sin(z1(1)),cos(z1(1))]*[xdat1;ydat1];
    set(link1,'xData',rot1(1,:))
    set(link1,'yData',rot1(2,:))
    
    rot2 = [cos(z1(3)+z1(1)), -sin(z1(3)+z1(1)); sin(z1(3)+z1(1)),cos(z1(3)+z1(1))]*[xdat2;ydat2];
    
    set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2); %We want to add the midpoint of the far edge of the first link to all points in link 2.
    set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2);
    
    %Change the hinge dot location
    set(h2,'xData',(rot1(1,3)+rot1(1,4))/2);
    set(h2,'yData',(rot1(2,3)+rot1(2,4))/2);
    
    %Show torques on screen (text only atm) update for time series later.
    set(tmeter1,'string',strcat(num2str(tau(1),2),' Nm'));
    set(tmeter2,'string',strcat(num2str(tau(2),2),' Nm'));
        
    %Keep the trace drawing
    set(trace,'xData',rot2(1,3)+(rot1(1,3)+rot1(1,4))/2)
    set(trace,'yData',rot2(2,3)+(rot1(2,3)+rot1(2,4))/2)

    drawnow;
    current_time = current_time + dt_phy;
    time_passed = time_passed + dt_phy; %for the pid controller
end
end

%%%% BEGIN CALLBACKS FOR MOUSE AND KEYBOARD STUFF %%%%%

% When click-up occurs, disable the mouse motion detecting callback
function ClickUp(varargin)
    figData = get(varargin{1},'UserData');
    set(figData.fig,'WindowButtonMotionFcn','');
    figData.Fx = 0;
    figData.Fy = 0;
    set(varargin{1},'UserData',figData);
end

% When click-down occurs, enable the mouse motion detecting callback
function ClickDown(varargin)
    figData = get(varargin{1},'UserData');
    figData.Fx = 0;
    figData.Fy = 0;

    set(figData.fig,'WindowButtonMotionFcn',@MousePos);
    set(varargin{1},'UserData',figData);
end

% any keypress switches from dragging the setpoint to applying a
% disturbance.
function KeyPress(hObject, eventdata, handles)

figData = get(hObject,'UserData');

figData.tarControl = ~figData.tarControl;

    if figData.tarControl
       disp('Mouse will change the target point of the end effector.')
    else
       disp('Mouse will apply a force on end effector.') 
    end
set(hObject,'UserData',figData);
end

% Checks mouse position and sends it back up.
function MousePos(varargin)
    figData = get(varargin{1},'UserData');

    mousePos = get(figData.simArea,'CurrentPoint');
    if figData.tarControl
        
    else
        figData.Fx = 10*(mousePos(1,1)-figData.xend);
        figData.Fy = 10*(mousePos(1,2)-figData.yend);
    end
     set(varargin{1},'UserData',figData);
end