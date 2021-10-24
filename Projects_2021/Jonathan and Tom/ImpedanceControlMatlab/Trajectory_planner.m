function [traj] =Trajectory_planner(p)
%Return the [x,y] array for the given trajectory points
% traj = [];
% traj_follow_rate = 100;
% for traj_counter = 1: length(p.traj)-1
%     traj_lin_max = ceil((pdist(p.traj(traj_counter:traj_counter+1,:))*traj_follow_rate));
%     temp_array1 = linspace(p.traj(traj_counter,1),p.traj(traj_counter+1,1),traj_lin_max);
%     temp_array2 = linspace(p.traj(traj_counter,2),p.traj(traj_counter+1,2),traj_lin_max); 
%     for traj_lin_counter = 1: traj_lin_max
%             traj = [traj; temp_array1(traj_lin_counter), temp_array2(traj_lin_counter)];
%     end
% end

traj = [];
traj_follow_rate = p.trajfollowrate; %lower the rate faster it tracks
for traj_counter = 1: length(p.traj)-1
    traj_time = ceil(pdist(p.traj(traj_counter:traj_counter+1,:))*traj_follow_rate);
    tmax = traj_time/100;
    t = linspace(0, tmax,traj_time+1);
    
    
    x_t = (p.traj(traj_counter+1,1)-p.traj(traj_counter,1))/(tmax^3);
    y_t = (p.traj(traj_counter+1,2)-p.traj(traj_counter,2))/(tmax^3); 
    
    temp1 = (6/tmax^2)*t.^5 - (15/tmax)*t.^4 + 10*t.^3;
    temp2 = (6/tmax^2)*t.^5 - (15/tmax)*t.^4 + 10*t.^3;
    x_t = x_t .* temp1 + p.traj(traj_counter,1)*ones(1,length(temp1)) ;
    y_t = y_t .* temp2 + p.traj(traj_counter,2);
    
    traj = [traj; x_t', y_t'];
end

end

