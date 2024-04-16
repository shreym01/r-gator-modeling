clear;
fismatrix  = readfis("knockofffis.fis")
syms s t X Y Theta

%from old model
%v = 5.0; %inital velocity, m/s
% caf = 2.787e-4; %cornering stiffness of front tire
% car = 2.787e-4; % cornering stiffness of rear tire
%lf = 0.5; %cog to front axle, m
%lr = 0.5; %cog to rear axle, m
%m = 12; %bicycle weight, kg
%theta = pi/2; %initial steering angle, radians

% Bicycle parameters
lf = 0.5;           % Distance from cog to front [m]
lr = 0.5;           % Distance from cog to back [m]
m = 1000;           % Mass [kg]
Iz = 0;             % Moment of inertia [kg*m^2]
c = 2.787e-4        % cornering stiffness of tire
v = 5.0;            %inital velocity, [m/s]
theta = pi/2;       %initial steering angle, [radians]

% Constants
m = m;              % Mass of the bike (kg)
g = 9.81;           % Acceleration due to gravity (m/s^2)
r = 0.33;           % Tire radius (m)
J = (pi/2) * (r^4); % Moment of inertia of the wheel (kg*m^2)
L = lf + lr;        % Wheelbase (m)
lambda = 0.05;      % Relaxation length (s)
B = 10;             % Longitudinal stiffness factor
C = c;              % Cornering stiffness factor

% Laplace transforms of initial conditions
x0 = 0;             
y0 = 0;
theta0 = theta;
vx0 = v;
vy0 = 0;
omega0 = 0;

% Laplace transforms of velocities
eq1 = s^2 * X - s * x0 - vx0 == B/m * (s*(s+r*Theta)/(s+lambda/2) - r*theta0);
eq2 = s^2 * Y - s * y0 - vy0 == C/m * (s/(s+lambda/2));
eq3 = s^2 * Theta - s * theta0 - omega0 == C*L/J * (s/(s+lambda/2));

% Solve equations
sol = solve([eq1, eq2, eq3], [X, Y, Theta]);

% Inverse Laplace transform
x = ilaplace(sol.X, s, t);
y = ilaplace(sol.Y, s, t);
theta = ilaplace(sol.Theta, s, t);

time = [0,60];

% Plot results
fplot(x, time, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Longitudinal Position (m)');
title('Longitudinal Position of the bike over time');
grid on;

figure;
fplot(y, time, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Lateral Position (m)');
title('Lateral Position of the bike over time');
grid on;

figure;
fplot(theta, time, 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Orientation of the bike over time');
grid on;

%% EQUATIONS
%%
% $
% s2X(s)−sx(0)−x˙(0)\=\\frac{m}{B}​(s+2λ​s(s+r⋅θ)​−r⋅θ(0))
% 
% s2Y(s)−sy(0)−y˙(0)\=Cm(ss+λ2)s^2 Y(s) - s y(0) - \\dot{y}(0) = \\frac{C}{m} \\left( \\frac{s}{s + \\frac{\\lambda}{2}} \\right)s2Y(s)−sy(0)−y˙​(0)\=mC​(s+2λ​s​)
% 
% s2Θ(s)−sθ(0)−θ˙(0)\=C⋅LJ(ss+λ2)s^2 \\Theta(s) - s \\theta(0) - \\dot{\\theta}(0) = \\frac{C \\cdot L}{J} \\left( \\frac{s}{s + \\frac{\\lambda}{2}} \\right)s2Θ(s)−sθ(0)−θ˙(0)\=JC⋅L​(s+2λ​s​)
% $ 
