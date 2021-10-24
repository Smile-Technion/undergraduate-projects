clc
clear
close all

load('End_Effector_data.mat');

Traj = [traj_x(1,:)',traj_y(1,:)'];
End_Efftor = [EndEff_x(1,:)',EndEff_y(1,:)'];

diff_x = EndEff_x - traj_x;
diff_y = EndEff_y - traj_y;
RMS_x = rms(diff_x);
RMS_y = rms(diff_y);
RMS = rms([diff_x, diff_y]);

diff_q1 = q1_real - q1_ideal;
diff_q2 = q2_real - q2_ideal;
RMS_q1 = rms(diff_q1);
RMS_q2 = rms(diff_q2);

figure(1);
movegui('north');    
subplot(2,1,1);
hold on
title("Value of x position in Cartasian");
plot(EndEff_x,'r');
plot(traj_x,'b');
legend("End Effector","Required");
xlabel("time lapse");
ylabel("Value of x");


subplot(2,1,2);
hold on
title("Value of y position in Cartasian");
plot(EndEff_y,'r');
plot(traj_y,'b');
legend("End Effector","Required");
xlabel("time lapse");
ylabel("Value of y");


figure(2);
movegui('northwest');    
title("trajectory comparison");
hold on


plot(End_Efftor(:,1),End_Efftor(:,2),'r');
plot(Traj(:,1),Traj(:,2),'b');
xlabel("x [m]");
ylabel("y [m]");
legend("End Effector","Required");

%%%%%%%%% For writing report%%%%%%%
xlim([0 3]);
ylim([-2 2]);
if  wall
    patch(wall_x,wall_y,'red','FaceAlpha',.3)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(3);
movegui('northeast');    
subplot(2,1,1);
hold on
title("Joint Value");
plot(q1_real,'r');
plot(q1_ideal,'b');

legend("End effector","Required");
xlabel("time lapse");
ylabel("Joint value of q1 [rad]");


subplot(2,1,2);
hold on
plot(q2_real,'r');
plot(q2_ideal,'b');
legend("End effector","Required");
xlabel("time lapse");
ylabel("Joint value of q2 [rad]");



figure(4);
movegui('south');    
subplot(2,1,1);
hold on
grid on
title("Difference of x position in Cartasian");
plot(diff_x,'r');
xlabel("time lapse");
ylabel("Difference of x [m]");
str_rmsx = {'RMS of x: ',RMS_x};
ylim=get(gca,'ylim');
xlim=get(gca,'xlim');
text(xlim(1),ylim(2),str_rmsx);

subplot(2,1,2);
hold on
grid on
title("Difference of y position in Cartasian");
plot(diff_y,'r');
xlabel("time lapse");
ylabel("Difference of y [m]");
str_rmsy = {'RMS of y: ',RMS_y};
ylim=get(gca,'ylim');
xlim=get(gca,'xlim');
text(xlim(1),ylim(2),str_rmsy);

figure(5);
movegui('southeast');    
subplot(2,1,1);
hold on
grid on
title("Difference of q2 to desired value");
plot(diff_q1,'r');
xlabel("time lapse");
ylabel("Difference of q1 [rad]");
str_rmsx = {'RMS of q1: ',RMS_q1};
ylim=get(gca,'ylim');
xlim=get(gca,'xlim');
text(xlim(1),ylim(2),str_rmsx);

subplot(2,1,2);
hold on
grid on
title("Difference of q1 to desired value");
plot(diff_q2,'r');
xlabel("time lapse");
ylabel("Difference of q2 [rad]");
str_rmsy = {'RMS of q2: ',RMS_q2};
ylim=get(gca,'ylim');
xlim=get(gca,'xlim');
text(xlim(1),ylim(2),str_rmsy);

figure(6);
movegui('center');    
subplot(2,1,1);
hold on
title("Value of velocity in x direction");
plot(diff(EndEff_x)/dt_phy,'r');
plot(diff(traj_x)/dt_phy,'b');
legend("End Effector","Required");
xlabel("time");
ylabel("v_x");


subplot(2,1,2);
hold on
title("Value of velocity in y direction");
plot(diff(EndEff_y)/dt_phy,'r');
plot(diff(traj_y)/dt_phy,'b');
legend("End Effector","Required");
xlabel("time");
ylabel("v_y");

if exist('x_m_record','var')
    
figure(7);
movegui('southwest');    
title(" model comparison");
hold on
plot(x_m_record,y_m_record,'r*');
plot(Traj(:,1),Traj(:,2),'b');
xlabel("x [m]");
ylabel("y [m]");
legend("x_m","Required");

end
