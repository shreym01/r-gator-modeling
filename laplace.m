clear;
syms s x0 vx0 B m r Theta theta0 lambda L J y0 vy0 omega0 C X Y

% Laplace transforms of velocities
eq1 = s^2 * X - s * x0 - vx0 == B/m * (s*(s+r*Theta)/(s+lambda/2) - r*theta0);
eq2 = s^2 * Y - s * y0 - vy0 == C/m * (s/(s+lambda/2));
eq3 = s^2 * Theta - s * theta0 - omega0 == C*L/J * (s/(s+lambda/2));

% Solve equations
sol = solve([eq1, eq2, eq3], [X, Y, Theta]);

disp(sol);

Xq = simplify(sol.X);
Yq = simplify(sol.Y);
Thetaq = simplify(sol.Theta);

disp(Xq);
disp(Yq);
disp(Thetaq);

dX = 4*B*J*s^4 + 2*B*J*lambda*s^3 + 4*J*m*s^3*vx0 + 4*J*m*s^4*x0 + 4*B*C*L*r*s + J*lambda^2*m*s^2*x0 + 2*B*J*lambda*omega0*r;
dX2 = 4*B*J*omega0*r*s + 4*B*J*r*s^2*theta0 - 4*B*J*r*s^3*theta0 + 4*J*lambda*m*s^2*vx0 + J*lambda^2*m*s*vx0 + 4*J*lambda*m*s^3*x0 - 4*B*J*lambda*r*s^2*theta0 - B*J*lambda^2*r*s*theta0 + 2*B*J*lambda*r*s*theta0;
dX = coeffs(dX, s, 'All');
disp(dX)

dXs = (J*m*s^3*(lambda + 2*s)^2);

disp(coeffs(dXs, s, 'All'))

dY = (2*C*s + 2*m*s^2*y0 + lambda*m*vy0 + 2*m*s*vy0 + lambda*m*s*y0);
dYs = (m*s^3*(lambda + 2*s));

disp(coeffs(dY, s, 'All'))
disp(coeffs(dYs, s, 'All'))

dT = (2*C*L*s + J*lambda*omega0 + 2*J*omega0*s + 2*J*s^2*theta0 + J*lambda*s*theta0);
dTs = (J*s^3*(lambda + 2*s));
disp(coeffs(dT, s, 'All'))
disp(coeffs(dTs, s, 'All'))
