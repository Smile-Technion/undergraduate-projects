
close all
try
    delete path.csv
end 
copyfile totalpath.csv path.csv
load path.csv

try
    delete travel.csv
end
copyfile totaltravel.csv travel.csv
load travel.csv

try
    delete xmv.csv
end
copyfile xms.csv xmv.csv
load xmv.csv

try
    delete force.csv
end
copyfile totalforce.csv force.csv
load force.csv

dt=0.002;
npts=dot(size(travel),[1,0]);
tvec=linspace(0,npts*dt,npts);

x = path(1:10:end*2.6/4,1)';
y = path(1:10:end*2.6/4,3)';
phi =path(1:10:end*2.6/4,5)';

x_travel = travel(1:10:end,1)';
y_travel = travel(1:10:end,2)';
phi_travel =travel(1:10:end,3)';

xm = xmv(1:10:end,1)';
ym = xmv(1:10:end,2)';
phim =xmv(1:10:end,3)';

%  
figure
hold on
p1=scatter(x,y ,'r','*')
p2=scatter(x_travel,y_travel,'g')
p3=scatter(xm,ym , 'y')

line([0.10 0.40],[1.5 1.5])
line([0.10 0.40],[2 2])
line([0.10 0.10],[1.5 2])
line([0.40 0.40],[1.5 2])

line([0.18 0.32],[1.5 1.5])
line([0.18 0.32],[1.735 1.735])
line([0.32 0.32],[1.50 1.735])
line([0.18 0.18],[1.50 1.735])

legend([p1 p2 p3],{'Planned trajectory','Actual trajectory', 'Impedance ODE xm'})
legend show
grid on
axis([0 0.6 1 2])
xlabel('x[m]')
ylabel('y[m]')
axis equal
hold off


figure
fx=plot(tvec,force(:,1))
xlabel('time [t]')
ylabel('end effector x - axis force [N]')

saveas(fx,'fx.bmp');

figure
fy=plot(tvec,force(:,2))
xlabel('time [t]')
ylabel('end effector y - axis force [N]')

saveas(fy,'fy.bmp');

figure
tz=plot(tvec,force(:,6))
xlabel('time [t]')
ylabel('end effector z - axis external torque [N]')

saveas(tz,'tz.bmp');


y_travel_full = travel(1:end,2)';
ym_full = xmv(1:end,2)';
path_y_full = path(1:end,3)';

figure
hold on
p2=plot(tvec,y_travel_full)
p3=plot(tvec,ym_full)
p1=plot(tvec,path_y_full)
xlabel('time [t]')
ylabel('y axsis trajectory[m]')
legend([p1 p2 p3],{'Planned trajectory','Actual trajectory', 'Impedance ODE xm'})
grid on
