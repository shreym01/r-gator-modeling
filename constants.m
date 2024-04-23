% Bicycle parameters
lf = 0.5;           % Distance from cog to front [m]
lr = 0.5;           % Distance from cog to back [m]
m = 1000;           % Mass [kg]
Iz = 0;             % Moment of inertia [kg*m^2]
c = 2.787e-4        % cornering stiffness of tire
v = 5.0;            %inital velocity, [m/s]
theta0 = pi/2;       %initial steering angle, [radians]

% Laplace transforms of initial conditions
x0 = 0;             
y0 = 0;
theta0 = theta0;
vx0 = v;
vy0 = 0;
omega0 = 0;

% Constants
g = 9.81;           % Acceleration due to gravity (m/s^2)
r = 0.33;           % Tire radius (m)
J = (pi/2) * (r^4); % Moment of inertia of the wheel (kg*m^2)
L = lf + lr;        % Wheelbase (m)
lambda = 0.05;      % Relaxation length (s)
B = 10;             % Longitudinal stiffness factor
C = c;              % Cornering stiffness factor

