clear;
fismatrix  = readfis("knockofffis.fis")

%from old model
%v = 5.0; %inital velocity, m/s
caf = 2.787e-4; %cornering stiffness of front tire
car = 2.787e-4; % ''     '' of rear tire
%lf = 0.5; %cog to front axle, m
%lr = 0.5; %cog to rear axle, m
%m = 12; %bicycle weight, kg
theta = pi/2; %initial steering angle, radians

% Bicycle parameters
lf = 0.5; % Distance from cog to front [m]
lr = 0,5; % Distance from cog to back [m]
m = 12; % Mass [kg]
Iz = 25; % Moment of inertia [kg*m^2]

% Initial conditions
x0 = 7; % Initial x-position [m]
y0 = 7; % Initial y-position [m]
v0 = 5.0; % Initial velocity [m/s]
psi0 = 6; % Initial yaw angle [rad]
psiDot0 = 5; % Initial yaw rate [rad/s]

% State vector
x0 = [x0; y0; v0; psi0; psiDot0];

% Tire parameters
B = 2.787e-4; % Stiffness factor
C = 1.65; % Shape factor
D = 9; % Peak value of lateral force
E = 10; % Curvature factor
mu = .9; % Friction

% Pacejka Function
Fy = @(alpha, Fz) D*sin(C*atan(B*alpha - E*(B*alpha - atan(B*alpha))))*mu*Fz;


% Time
dt = 0.01; % Time step [s]
tspan = 0:dt:60; % Time span for simulation [s]

% Initialize state vector
X = zeros(5, length(tspan));
X(:,1) = x0;


% bicycle model with Pacejka
for i = 2:length(tspan)
    % Extract states
    x = X(1, i-1);
    y = X(2, i-1);
    v = X(3, i-1);
    psi = X(4, i-1);
    psiDot = X(5, i-1);

    delta = pi/2; % Steering angle [rad]

    alpha =  atan2((v + lf*psiDot), v); % tire slip angle [rad]
    
    fy = Fy(alpha, m * 9.81); %F lateral [N]
    
    % Update states via Euler 
    X(1, i) = x + v*cos(psi) * dt;
    X(2, i) = y + v*sin(psi) * dt;
    X(3, i) = v + (fy/m - psiDot*v) * dt;
    X(4, i) = psi + psiDot * dt;
    X(5, i) = psiDot + (fy*lf/Iz - psiDot^2) * dt;
end
plot(X(5,:))