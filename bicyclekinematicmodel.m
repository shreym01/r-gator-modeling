clear;
fismatrix  = readfis("knockofffis.fis")

v = 5.0; %inital velocity, m/s
caf = 2.787e-4; %cornering stiffness of front tire
car = 2.787e-4; % ''     '' of rear tire
lf = 0.5; %cog to front axle, m
lr = 0.5; %cog to rear axle, m
m = 12; %bicycle weight, kg
theta = pi/2; %initial steering angle, radians


a11 = (-1/v)*((caf*lf^2)+(car*lr^2)/theta);
a12 = -1*((caf*lf)-(car*lr))/theta;
a21 = -1-(1/v^2)*((caf*lf)-(car*lr))/m;
a22 = (-1/v)*((caf+car)/m);

b11 = (caf*lf)/theta;
b12 = (1/v)*caf/m;

A = [a11, a12; 
    a21, a22];

B = [b11;
    b12;];

C = [1,0;0,1];

D = [0;0;];
syms s
% sys = ss(A,B,C,D);
% Define the simulation time span
time = 0:0.1:60; % Adjust the time span as needed


[b,a] = ss2tf(A,B,C,D)
newsys = tf(b(1,:),a)
newsys2 = tf(b(2,:),a)


b1 = b(1,:);
b2 = b(2,:);

 [num,den] = tfdata(newsys);
 G_sym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s)
 step(newsys)
 Y_lap_sym = G_sym/s; % U(s) = 1/s for the unit step
 y_time_sym = ilaplace(Y_lap_sym);



% Define arrays to hold the 1001 waypoints for a figure-8 path
    
% Calculate the waypoints along the figure-8 path
waypoints_x = [0, 1, 2, 3, 4];
waypoints_y = [0, 1, 3, 4, 5];

save('gatorwaypoints.mat', 'waypoints_x', 'waypoints_y');
load('gatorwaypoints.mat');

