delete path.csv
copyfile current_location_tot.csv path.csv
load path.csv
x = path(1:end,1)';
y = path(1:end,2)';
phi =path(1:end,3)';

hold on
scatter(path(1,1),path(1,2))

plot(x,y)
scatter(0,0.26,'r')
xlim([-0.3 0.3])
xlim([-0.3 0.3])